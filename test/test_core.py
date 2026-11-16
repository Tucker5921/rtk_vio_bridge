import math
import unittest
import numpy as np
from rtk_vio_bridge.core import Bridge, Pose, ENU, rz, rotation, quaternion


def pose(t, x=None, angle=0):
    return Pose(t,np.array([t if x is None else x,0.,0.]),quaternion(rz(angle)))


class TestBridge(unittest.TestCase):
    def bridge(self, **kw):
        return Bridge(manual_yaw=math.pi/2,recover_count=2,recover_seconds=.1,**kw)

    def anchor(self,b):
        for t in (0., .1, .2):
            b.add_pose(pose(t))
            b.accept_fix(t,np.array([10.,20.+t,0.]),[0,0,0])

    def test_rotation(self):
        for a in (0,.2,math.pi,-2.):
            np.testing.assert_allclose(rotation(quaternion(rz(a))),rz(a),atol=1e-12)

    def test_enu(self):
        e=ENU(25.,121.,30.)
        np.testing.assert_allclose(e.convert(25.,121.,30.),0)
        self.assertGreater(e.convert(25.,121.00001,30.)[0],0)
        self.assertGreater(e.convert(25.00001,121.,30.)[1],0)

    def test_alignment_continuation_and_timeout(self):
        b=self.bridge(); self.anchor(b)
        np.testing.assert_allclose(b.output(.2).p,[10,20.2,0])
        for t in np.arange(.3,1.6,.1): b.add_pose(pose(t))
        self.assertEqual(b.state(1.5),'VIO_CONTINUATION')
        np.testing.assert_allclose(b.output(1.5).p,[10,21.5,0],atol=1e-9)
        self.assertEqual(b.state(2.1),'UNAVAILABLE')

    def test_recovery_preserves_raw_residual(self):
        b=self.bridge(); self.anchor(b); b.degrade()
        b.add_pose(pose(.3)); b.accept_fix(.3,[10.4,20.3,0],[0,0,0])
        self.assertEqual(b.state(.3),'RECOVERING')
        np.testing.assert_allclose(b.output(.3).p,[10,20.3,0])
        b.add_pose(pose(.5)); self.assertTrue(b.accept_fix(.5,[10.4,20.5,0],[0,0,0]))
        np.testing.assert_allclose(b.last_residual['delta_enu_m'],[.4,0,0],atol=1e-10)

    def test_recovery_after_actual_timeout(self):
        b=self.bridge(); self.anchor(b)
        for t in np.arange(.3,1.6,.1): b.add_pose(pose(t))
        b.state(1.5)
        for t in (1.6,1.8):
            b.add_pose(pose(t)); b.accept_fix(t,[10,20+t,0],[0,0,0]); b.state(t)
        self.assertEqual(b.state(1.8),'RTK_ANCHORED')

    def test_large_recovery_rejected(self):
        b=self.bridge(); self.anchor(b); b.degrade()
        for t in (.3,.5,.7):
            b.add_pose(pose(t)); b.accept_fix(t,[30,20+t,0],[0,0,0])
        self.assertFalse(b.rtk_good)
        self.assertGreater(b.last_residual['delta_enu_m'][0],19)

    def test_reset_and_restart(self):
        b=self.bridge(); self.anchor(b)
        b.add_pose(pose(.3,100))
        self.assertEqual(b.state(.3),'WAIT_ALIGNMENT')
        self.assertEqual(b.epoch,1)
        b.add_pose(pose(.1,0))
        self.assertEqual(b.epoch,2)

    def test_interpolation_and_lever(self):
        b=self.bridge(sync_gap=.3)
        b.add_pose(pose(0)); b.add_pose(pose(.2))
        np.testing.assert_allclose(b.interpolate(.1).p,[.1,0,0])
        self.assertIsNone(b.interpolate(.3))
        b.accept_fix(0,[10,21,0],[1,0,0],math.pi/2)
        b.accept_fix(.2,[10,21.2,0],[1,0,0],math.pi/2)
        np.testing.assert_allclose(b.output(.2).p,[10,20.2,0])

    def test_heading_required(self):
        b=Bridge(recover_count=1,recover_seconds=0)
        b.add_pose(pose(0))
        self.assertFalse(b.accept_fix(0,[0,0,0],[0,0,0]))
        self.assertEqual(b.state(0),'WAIT_ALIGNMENT')

    def test_old_future_and_duplicate_fix(self):
        b=self.bridge(); self.anchor(b)
        self.assertFalse(b.accept_fix(.2,[100,0,0],[0,0,0]))
        self.assertFalse(b.accept_fix(.3,[100,0,0],[0,0,0],now=.2))
        self.assertFalse(b.accept_fix(.3,[100,0,0],[0,0,0],now=2))

    def test_reset_requires_new_heading_and_reanchors_new_origin(self):
        b=self.bridge(); self.anchor(b)
        b.add_pose(pose(.3,100))
        self.assertIsNone(b.manual_yaw)
        self.assertFalse(b.accept_fix(.3,[10,20.3,0],[0,0,0]))
        for t in (.4,.6):
            b.add_pose(pose(t,100+t-.3))
            b.accept_fix(t,[10,20+t,0],[0,0,0],math.pi/2)
        self.assertEqual(b.state(.6),'RTK_ANCHORED')
        np.testing.assert_allclose(b.output(.6).p,[10,20.6,0],atol=1e-9)

    def test_heading_recovery_gate(self):
        b=self.bridge(); self.anchor(b); b.degrade()
        for t in (.3,.5):
            b.add_pose(pose(t))
            b.accept_fix(t,[10,20+t,0],[0,0,0],math.pi)
        self.assertFalse(b.rtk_good)
        self.assertAlmostEqual(b.last_residual['delta_yaw_rad'],math.pi/2)

    def test_nonfinite_pose_rejected(self):
        b=self.bridge()
        with self.assertRaises(ValueError): b.add_pose(pose(float('nan')))
        with self.assertRaises(ValueError): rotation([0,0,0,0])

    def test_clock_callback_jitter_does_not_toggle_availability(self):
        b=self.bridge(); self.anchor(b)
        self.assertEqual(b.state(.195),'RTK_ANCHORED')
        self.assertEqual(b.state(.1),'UNAVAILABLE')


if __name__=='__main__': unittest.main()
