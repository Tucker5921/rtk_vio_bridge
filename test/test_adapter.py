"""Callback contract tests without a ROS graph; source ROS setup to enable."""
import unittest
from collections import deque
from types import SimpleNamespace
try:
    from rtk_vio_bridge.node import BridgeNode
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    ROS_AVAILABLE=True
except ImportError:
    ROS_AVAILABLE=False
from rtk_vio_bridge.core import Bridge


@unittest.skipUnless(ROS_AVAILABLE,'source /opt/ros/humble/setup.bash for adapter tests')
class TestAdapter(unittest.TestCase):
    def fake(self):
        f=SimpleNamespace(cfg=dict(quality_timeout=1.5,heading_timeout=.3,rtk_timeout=1.,
                                  diagnostic_name='RTK GPS'),now_s=lambda:10.,
                          quality=deque(),heading_quality=deque(),headings=deque(),
                          pending=deque(),core=Bridge(),last_received={},reason='')
        f.process_pending=lambda:None
        f.past=BridgeNode.past
        f.heading_at=lambda t:BridgeNode.heading_at(f,t)
        return f

    def diag(self,quality='4',status='RTK_FIXED',level=b'\x00',t=10):
        d=DiagnosticArray(); d.header.stamp.sec=t
        s=DiagnosticStatus(); s.name='RTK GPS'; s.level=level
        s.values=[KeyValue(key='Fix Quality',value=quality),KeyValue(key='Status',value=status)]
        d.status=[s]; return d

    def test_fixed_float_quality(self):
        f=self.fake(); BridgeNode.on_quality(f,self.diag(t=9))
        self.assertEqual(f.quality[-1],(9.,True))
        f.core.rtk_good=True
        BridgeNode.on_quality(f,self.diag('5','RTK_FLOAT'))
        self.assertFalse(f.quality[-1][1]); self.assertFalse(f.core.rtk_good)

    def test_error_level_not_trusted(self):
        f=self.fake(); BridgeNode.on_quality(f,self.diag(level=b'\x02'))
        self.assertFalse(f.quality[-1][1])

    def test_stale_future_quality_not_used(self):
        f=self.fake()
        BridgeNode.on_quality(f,self.diag(t=1)); BridgeNode.on_quality(f,self.diag(t=11))
        self.assertFalse(f.quality)

    def test_quality_never_from_future(self):
        self.assertIsNone(BridgeNode.past([(10,True)],9.9,1.5))

    def test_heading_not_retroactively_validated(self):
        f=self.fake(); f.heading_quality.extend([(9.8,False),(10.,True)])
        f.headings.append((9.9,.4))
        self.assertIsNone(f.heading_at(10.))
        f.headings.append((10.,.5))
        self.assertEqual(f.heading_at(10.),.5)

    def test_clock_ahead_fix_retained(self):
        f=self.fake(); f.pending.append((10.01,[0,0,0]))
        BridgeNode.process_pending(f)
        self.assertEqual(len(f.pending),1)


if __name__=='__main__': unittest.main()
