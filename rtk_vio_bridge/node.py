"""Conservative ROS adapter. No TF publication or fabricated covariance."""
import json
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .core import Bridge, ENU, Pose, rotation, quaternion, yaw


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec*1e-9


def xyz(v):
    return np.array([v.x, v.y, v.z])


def xyzw(q):
    return np.array([q.x, q.y, q.z, q.w])


class BridgeNode(Node):
    def __init__(self):
        super().__init__('rtk_vio_bridge')
        defaults = dict(vio_topic='/ov_msckf/odomimu', fix_topic='/gps/fix',
                        quality_topic='/gps/diagnostics', heading_topic='/rtk/heading_enu',
                        heading_quality_topic='/rtk/heading_diagnostics',
                        origin_configured=False, origin_lat=0., origin_lon=0., origin_alt=0.,
                        extrinsics_configured=False, imu_from_body_xyzw=[0.,0.,0.,1.],
                        imu_to_body_in_imu_m=[0.,0.,0.], antenna_in_body_m=[0.,0.,0.],
                        manual_alignment_enabled=False, field_from_vio_yaw_deg=0.,
                        rtk_timeout=1., vio_timeout=.5, quality_timeout=1.5,
                        heading_timeout=.3, sync_gap=.1, recover_count=5, recover_seconds=1.,
                        max_recovery_m=2., max_recovery_yaw_deg=20.,
                        reset_step_m=2., reset_step_deg=45.,
                        field_frame='field_enu', body_frame='base_link',
                        expected_vio_frame='global', expected_vio_child='imu',
                        diagnostic_name='RTK GPS', heading_diagnostic_name='Dual antenna heading')
        self.cfg = {k: self.declare_parameter(k,v).value for k,v in defaults.items()}
        c = self.cfg
        if not c['origin_configured'] or not c['extrinsics_configured']:
            raise ValueError('Set origin_configured and extrinsics_configured only after defining origin and body/antenna transforms; see README')
        for key in ('rtk_timeout','vio_timeout','quality_timeout','heading_timeout','sync_gap',
                    'recover_seconds','max_recovery_m','max_recovery_yaw_deg','reset_step_m','reset_step_deg'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError('positive finite parameter required: '+key)
        if c['recover_count'] < 1:
            raise ValueError('recover_count must be >= 1')
        if not math.isfinite(c['field_from_vio_yaw_deg']):
            raise ValueError('field_from_vio_yaw_deg must be finite')
        self.core = Bridge(manual_yaw=math.radians(c['field_from_vio_yaw_deg']) if c['manual_alignment_enabled'] else None,
                           **{k:c[k] for k in ('rtk_timeout','vio_timeout','sync_gap','recover_count','recover_seconds','max_recovery_m','reset_step_m')},
                           max_recovery_yaw=math.radians(c['max_recovery_yaw_deg']), reset_step_rad=math.radians(c['reset_step_deg']))
        self.enu = ENU(c['origin_lat'], c['origin_lon'], c['origin_alt'])
        self.r_ib = rotation(c['imu_from_body_xyzw'])
        self.p_ib = np.asarray(c['imu_to_body_in_imu_m'])
        self.lever = np.asarray(c['antenna_in_body_m'])
        for v in (self.p_ib, self.lever):
            if v.shape != (3,) or not np.all(np.isfinite(v)):
                raise ValueError('lever arms must be three finite numbers')
        self.quality, self.heading_quality, self.headings = deque(maxlen=100), deque(maxlen=100), deque(maxlen=200)
        self.pending = deque(maxlen=100)
        self.last_received = dict(vio=None, fix=None, quality=None, heading=None, heading_quality=None)
        self.last_clock = None
        self.reason = 'waiting for input'
        self.pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.event_pub = self.create_publisher(String, '~/events', 100)
        self.source_pub = self.create_publisher(String, '~/source', 10)
        for cls, key, cb in [(Odometry,'vio_topic',self.on_vio), (NavSatFix,'fix_topic',self.on_fix),
                             (DiagnosticArray,'quality_topic',self.on_quality),
                             (QuaternionStamped,'heading_topic',self.on_heading),
                             (DiagnosticArray,'heading_quality_topic',self.on_heading_quality)]:
            self.create_subscription(cls, c[key], cb, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset_vio', self.reset_service)
        self.create_timer(.1, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def reset_inputs(self, reason):
        self.core.reset(reason, self.now_s())
        for b in (self.pending, self.quality, self.heading_quality, self.headings):
            b.clear()
        self.last_received = dict.fromkeys(self.last_received)

    def reset_service(self, request, response):
        self.reset_inputs('explicit reset service')
        response.success, response.message = True, 'Alignment invalidated; fresh RTK alignment required.'
        return response

    def on_vio(self, m):
        try:
            if m.header.frame_id != self.cfg['expected_vio_frame'] or m.child_frame_id != self.cfg['expected_vio_child']:
                raise ValueError('unexpected VIO frames')
            t = stamp(m)
            if t > self.now_s()+.05:
                raise ValueError('VIO timestamp ahead of ROS clock; check use_sim_time / time domains')
            r = rotation(xyzw(m.pose.pose.orientation))
            p = xyz(m.pose.pose.position)+r@self.p_ib
            epoch = self.core.epoch
            added = self.core.add_pose(Pose(t, p, quaternion(r@self.r_ib)))
            if self.core.epoch != epoch:
                self.pending.clear()
                self.headings.clear()
            self.last_received['vio'] = t
            self.process_pending()
            out = self.core.output(self.now_s())
            if out is not None and added:
                msg = PoseStamped()
                msg.header.stamp = m.header.stamp
                msg.header.frame_id = self.cfg['field_frame']
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float,out.p)
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float,out.q)
                self.pub.publish(msg)
        except ValueError as e:
            self.reason = str(e)
            self.core.reset(self.reason, self.now_s())

    def on_quality(self, m):
        t, good = stamp(m), False
        if t > self.now_s()+.05 or self.now_s()-t > self.cfg['quality_timeout']:
            self.reason = 'quality timestamp outside clock/freshness window'
            return
        for s in m.status:
            if s.name == self.cfg['diagnostic_name']:
                kv = {v.key:v.value for v in s.values}
                good = s.level == b'\x00' and kv.get('Fix Quality') == '4' and kv.get('Status') == 'RTK_FIXED'
        if self.quality and t <= self.quality[-1][0]:
            return
        self.quality.append((t,good))
        self.last_received['quality'] = t
        if not good:
            self.core.degrade()
        self.process_pending()

    def on_heading_quality(self, m):
        good = any(s.name == self.cfg['heading_diagnostic_name'] and s.level == b'\x00' and
                   any(v.key == 'Heading Valid' and v.value.lower() == 'true' for v in s.values)
                   for s in m.status)
        t = stamp(m)
        if t > self.now_s()+.05 or self.now_s()-t > self.cfg['heading_timeout']:
            return
        if self.heading_quality and t <= self.heading_quality[-1][0]:
            return
        self.heading_quality.append((t,good))
        self.last_received['heading_quality'] = t

    def on_heading(self, m):
        try:
            if m.header.frame_id != self.cfg['field_frame']:
                raise ValueError('heading frame must be field_enu')
            t, h = stamp(m), yaw(rotation(xyzw(m.quaternion)))
            if t > self.now_s()+.05 or self.now_s()-t > self.cfg['heading_timeout']:
                return
            if self.headings and t <= self.headings[-1][0]:
                return
            self.headings.append((t,h))
            self.last_received['heading'] = t
        except ValueError as e:
            self.reason = str(e)

    @staticmethod
    def past(buffer, t, age):
        return next((value for ts,value in reversed(buffer) if 0 <= t-ts <= age), None)

    def heading_at(self, t):
        age = self.cfg['heading_timeout']
        if self.past(self.heading_quality,t,age) is not True:
            return None
        for ts,h in reversed(self.headings):
            if 0 <= t-ts <= age:
                # A newly valid quality flag must not retroactively validate an old heading.
                return h if self.past(self.heading_quality,ts,age) is True else None
        return None

    def on_fix(self, m):
        t = stamp(m)
        if t > self.now_s()+.05 or self.now_s()-t > self.cfg['rtk_timeout']:
            self.reason = 'fix timestamp outside clock/freshness window'
            return
        if self.last_received['fix'] is not None and t <= self.last_received['fix']:
            return
        self.last_received['fix'] = t
        if m.status.status < 0:
            self.core.degrade()
            return
        try:
            p = self.enu.convert(m.latitude,m.longitude,m.altitude)
        except ValueError as e:
            self.reason = str(e)
            self.core.degrade()
            return
        self.pending.append((t,p))
        self.process_pending()

    def process_pending(self):
        now = self.now_s()
        while self.pending:
            t,p = self.pending[0]
            if now-t > self.cfg['rtk_timeout'] or t > now+.05:
                self.pending.popleft()
                self.reason = 'fix expired or future timestamp'
                continue
            # /clock and sensor topics can arrive in different callback order.
            # Keep a slightly-ahead fix until the clock catches up, not discard it.
            if t > now:
                break
            good = self.past(self.quality, t, self.cfg['quality_timeout'])
            if good is None or not self.core.poses or self.core.poses[-1].t < t:
                break
            self.pending.popleft()
            if not good:
                self.core.degrade()
                self.reason = 'RTK not fixed'
                continue
            h = self.heading_at(t)
            ok = self.core.accept_fix(t,p,self.lever,h,now)
            self.reason = 'RTK accepted' if ok else 'waiting alignment / recovery gate / time match'

    def tick(self):
        now = self.now_s()
        if self.last_clock is not None and now < self.last_clock:
            self.reset_inputs('ROS clock moved backward')
        self.last_clock = now
        if self.past(self.quality,now,self.cfg['quality_timeout']) is not True:
            self.core.degrade()
        self.process_pending()
        state = self.core.state(now)
        ages = {k: None if t is None else now-t for k,t in self.last_received.items()}
        status = dict(stamp=now, source=state, available=state not in ('UNAVAILABLE','WAIT_ALIGNMENT'),
                      body_frame=self.cfg['body_frame'], field_frame=self.cfg['field_frame'],
                      epoch=self.core.epoch, yaw_alignment_source=self.core.yaw_source,
                      heading_available=self.heading_at(now) is not None,
                      rtk_fixed_available=self.past(self.quality,now,self.cfg['quality_timeout']) is True and
                      ages['fix'] is not None and 0 <= ages['fix'] <= self.cfg['rtk_timeout'],
                      ages_s=ages, reason=self.reason, covariance_available=False,
                      last_accepted_rtk_age_s=None if self.core.last_accepted is None else now-self.core.last_accepted,
                      last_raw_residual=self.core.last_residual)
        self.status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
        self.source_pub.publish(String(data=state))
        for event in self.core.events:
            self.event_pub.publish(String(data=json.dumps(event,allow_nan=False)))
            if event['event'] != 'rtk_residual':
                self.get_logger().info(json.dumps(event))
        self.core.events.clear()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
