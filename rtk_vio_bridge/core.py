"""ROS-independent, timestamped yaw+translation alignment. All rotations Hamilton."""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(r):
    # Stable matrix -> xyzw including 180-degree rotations.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]]) / 3
    _, v = np.linalg.eigh(k)
    return v[:, -1]


def yaw(r):
    if math.hypot(r[0, 0], r[1, 0]) < 1e-4:
        raise ValueError('body forward nearly vertical; heading undefined')
    return math.atan2(r[1, 0], r[0, 0])


def slerp(a, b, u):
    a, b = a/np.linalg.norm(a), b/np.linalg.norm(b)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > .9995:
        q = a + u*(b-a)
        return q/np.linalg.norm(q)
    angle = math.acos(np.clip(d, -1, 1))
    return (math.sin((1-u)*angle)*a + math.sin(u*angle)*b)/math.sin(angle)


def ecef(lat, lon, alt):
    lat, lon = math.radians(lat), math.radians(lon)
    n = 6378137. / math.sqrt(1-6.69437999014e-3*math.sin(lat)**2)
    return np.array([(n+alt)*math.cos(lat)*math.cos(lon),
                     (n+alt)*math.cos(lat)*math.sin(lon),
                     (n*(1-6.69437999014e-3)+alt)*math.sin(lat)])


class ENU:
    def __init__(self, lat, lon, alt):
        if not (-90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(alt)):
            raise ValueError('invalid ENU origin')
        self.origin = ecef(lat, lon, alt)
        p, l = math.radians(lat), math.radians(lon)
        self.r = np.array([[-math.sin(l), math.cos(l), 0],
                           [-math.sin(p)*math.cos(l), -math.sin(p)*math.sin(l), math.cos(p)],
                           [math.cos(p)*math.cos(l), math.cos(p)*math.sin(l), math.sin(p)]])

    def convert(self, lat, lon, alt):
        if not (-90 <= lat <= 90 and -180 <= lon <= 180 and math.isfinite(alt)):
            raise ValueError('invalid fix')
        return self.r @ (ecef(lat, lon, alt)-self.origin)


@dataclass
class Pose:
    t: float
    p: np.ndarray
    q: np.ndarray


class Bridge:
    def __init__(self, manual_yaw=None, rtk_timeout=1., vio_timeout=.5,
                 sync_gap=.1, recover_count=5, recover_seconds=1.,
                 max_recovery_m=2., max_recovery_yaw=.35,
                 reset_step_m=2., reset_step_rad=.8):
        self.manual_yaw = manual_yaw
        self.rtk_timeout, self.vio_timeout, self.sync_gap = rtk_timeout, vio_timeout, sync_gap
        self.recover_count, self.recover_seconds = recover_count, recover_seconds
        self.max_recovery_m, self.max_recovery_yaw = max_recovery_m, max_recovery_yaw
        self.reset_step_m, self.reset_step_rad = reset_step_m, reset_step_rad
        self.poses = deque(maxlen=4000)
        self.a = self.b = None
        self.last_fix = None
        self.last_accepted = None
        self.count = 0
        self.since = None
        self.rtk_good = False
        self.epoch = 0
        self.events = []
        self.yaw_source = 'unavailable'
        self.last_residual = None

    def reset(self, reason, t):
        # A manually measured yaw belongs to this VIO epoch only.
        if self.poses or self.a is not None:
            self.manual_yaw = None
        self.poses.clear()
        self.a = self.b = None
        self.last_fix = self.last_accepted = None
        self.count, self.since, self.rtk_good = 0, None, False
        self.yaw_source = 'unavailable'
        self.epoch += 1
        self.last_residual = None
        self.events.append(dict(event='vio_reset', stamp=t, reason=reason, epoch=self.epoch))

    def add_pose(self, pose):
        if not math.isfinite(pose.t) or not np.all(np.isfinite(pose.p)):
            raise ValueError('invalid pose')
        rotation(pose.q)
        if self.poses:
            prev = self.poses[-1]
            dt = pose.t-prev.t
            if dt == 0:
                return False
            angle = 2*math.acos(np.clip(abs(np.dot(prev.q/np.linalg.norm(prev.q),
                                                   pose.q/np.linalg.norm(pose.q))), 0, 1))
            if dt < 0 or dt > self.vio_timeout or np.linalg.norm(pose.p-prev.p) > self.reset_step_m or angle > self.reset_step_rad:
                self.reset('timestamp/gap/discontinuity', pose.t)
        self.poses.append(pose)
        return True

    def interpolate(self, t):
        for p in reversed(self.poses):
            if abs(p.t-t) < 1e-7:
                return p
        for a, b in zip(self.poses, list(self.poses)[1:]):
            if a.t < t < b.t and b.t-a.t <= self.sync_gap:
                u = (t-a.t)/(b.t-a.t)
                return Pose(t, a.p+u*(b.p-a.p), slerp(a.q, b.q, u))
        return None

    def degrade(self):
        self.rtk_good = False
        self.count, self.since = 0, None

    def accept_fix(self, t, antenna_enu, lever_body, heading=None, now=None):
        now = t if now is None else now
        if not -1e-6 <= now-t <= self.rtk_timeout or (self.last_fix is not None and t <= self.last_fix):
            return False
        p = self.interpolate(t)
        if p is None:
            return False
        if self.last_fix is not None and t-self.last_fix > self.rtk_timeout:
            self.degrade()
        self.last_fix = t
        if self.rtk_good and self.last_accepted is not None and t-self.last_accepted > self.rtk_timeout:
            self.degrade()
        old_a = self.a
        if heading is not None:
            a, ys = wrap(heading-yaw(rotation(p.q))), 'dual_antenna'
        elif self.a is not None:
            a, ys = self.a, self.yaw_source
        elif self.manual_yaw is not None:
            a, ys = self.manual_yaw, 'manual_alignment'
        else:
            return False
        target = np.asarray(antenna_enu)-rz(a)@rotation(p.q)@np.asarray(lever_body)
        b = target-rz(a)@p.p
        recovering = not self.rtk_good
        if old_a is not None:
            delta = target-(rz(old_a)@p.p+self.b)
            dyaw = wrap(a-old_a) if heading is not None else None
            self.last_residual = dict(event='rtk_residual', stamp=t, epoch=self.epoch,
                                      delta_enu_m=delta.tolist(), delta_yaw_rad=dyaw,
                                      recovering=recovering)
            self.events.append(self.last_residual.copy())
            if recovering and (np.linalg.norm(delta) > self.max_recovery_m or
                               (dyaw is not None and abs(dyaw) > self.max_recovery_yaw)):
                self.count, self.since = 0, None
                return False
        if recovering:
            self.count += 1
            if self.since is None:
                self.since = t
            if self.count < self.recover_count or t-self.since < self.recover_seconds:
                return False
            self.events.append(dict(event='rtk_acquired' if old_a is None else 'rtk_recovered',
                                    stamp=t, epoch=self.epoch, residual=self.last_residual))
        self.a, self.b, self.yaw_source = a, b, ys
        self.last_accepted, self.rtk_good = t, True
        return True

    def state(self, now):
        if self.rtk_good and self.last_accepted is not None and now-self.last_accepted > self.rtk_timeout:
            self.degrade()
        # Allow the same 50 ms scheduling skew as the ROS adapter. In rosbag
        # playback /clock may arrive a callback later than a sensor message.
        if not self.poses or not -.05 <= now-self.poses[-1].t <= self.vio_timeout:
            return 'UNAVAILABLE'
        if self.a is None:
            return 'WAIT_ALIGNMENT'
        return 'RTK_ANCHORED' if self.rtk_good else ('RECOVERING' if self.count else 'VIO_CONTINUATION')

    def output(self, now):
        state = self.state(now)
        if state in ('UNAVAILABLE', 'WAIT_ALIGNMENT'):
            return None
        p = self.poses[-1]
        return Pose(p.t, rz(self.a)@p.p+self.b, quaternion(rz(self.a)@rotation(p.q)))
