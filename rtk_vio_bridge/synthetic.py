"""RTK generated from VIO: logic test ONLY, never a ground truth accuracy test."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import QuaternionStamped
from .node import stamp, xyz, xyzw
from .core import rz, rotation, quaternion


def geodetic(x):
    lon=math.atan2(x[1],x[0]); p=math.hypot(x[0],x[1]); lat=math.atan2(x[2],p*(1-6.69437999014e-3))
    for _ in range(10):
        n=6378137/math.sqrt(1-6.69437999014e-3*math.sin(lat)**2)
        alt=p/math.cos(lat)-n
        lat=math.atan2(x[2],p*(1-6.69437999014e-3*n/(n+alt)))
    return math.degrees(lat),math.degrees(lon),alt


class Synthetic(Node):
    def __init__(self):
        super().__init__('synthetic_rtk')
        from .core import ENU
        self.enu=ENU(25.,121.,30.)
        self.start=None; self.last=-math.inf
        self.fix=self.create_publisher(NavSatFix,'/test/gps/fix',10)
        self.quality=self.create_publisher(DiagnosticArray,'/test/gps/diagnostics',10)
        self.heading=self.create_publisher(QuaternionStamped,'/test/heading',10)
        self.hquality=self.create_publisher(DiagnosticArray,'/test/heading_diagnostics',10)
        self.create_subscription(Odometry,'/ov_msckf/odomimu',self.on_vio,qos_profile_sensor_data)
        self.get_logger().warning('SYNTHETIC RTK from VIO, NOT independent ground truth. Identity extrinsics only.')

    def on_vio(self,m):
        t=stamp(m)
        if self.start is None or t < self.last: self.start=t
        if t-self.last < .1: return
        self.last=t; elapsed=t-self.start
        # Repeat: fixed 0-10, timeout 10-15, float 15-20, fixed recovery 20-40.
        phase=elapsed%40
        if 10 <= phase < 15: return
        good=not 15 <= phase < 20
        diag=DiagnosticArray(); diag.header=m.header
        s=DiagnosticStatus(); s.name='RTK GPS'; s.level=b'\x00' if good else b'\x01'
        s.message='RTK_FIXED' if good else 'RTK_FLOAT'
        s.values=[KeyValue(key='Fix Quality',value='4' if good else '5'),KeyValue(key='Status',value=s.message)]
        diag.status=[s]; self.quality.publish(diag)
        hd=DiagnosticArray(); hd.header=m.header
        hs=DiagnosticStatus(); hs.name='Dual antenna heading'
        hs.values=[KeyValue(key='Heading Valid',value='true')]; hd.status=[hs]; self.hquality.publish(hd)
        h=QuaternionStamped(); h.header.stamp=m.header.stamp; h.header.frame_id='field_enu'
        q=quaternion(rz(.4)@rotation(xyzw(m.pose.pose.orientation)))
        h.quaternion.x,h.quaternion.y,h.quaternion.z,h.quaternion.w=map(float,q); self.heading.publish(h)
        p=rz(.4)@xyz(m.pose.pose.position)+np.array([10.,20.,0.])
        if phase >= 20: p+=np.array([.4,0,0])
        f=NavSatFix(); f.header.stamp=m.header.stamp; f.header.frame_id='gps'
        f.status.status=2; f.latitude,f.longitude,f.altitude=geodetic(self.enu.origin+self.enu.r.T@p)
        f.position_covariance_type=2; f.position_covariance=[.0004,0.,0.,0.,.0004,0.,0.,0.,.0016]
        self.fix.publish(f)


def main(args=None):
    rclpy.init(args=args); n=Synthetic()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
