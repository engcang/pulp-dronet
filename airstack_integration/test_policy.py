"""Control-interface checks; no simulator, OpenCV or checkpoint required."""
import math
import unittest
import numpy as np
from airstack_integration.policy import Policy, HeadingReference, trajectory


class PolicyTests(unittest.TestCase):
    def test_network_outputs_preserve_upstream_sign_and_scaling(self):
        policy=Policy(maximum_speed=.35,yaw_gain=120,smoothing=0)
        speed,yaw=policy.step(.5,.2)
        self.assertAlmostEqual(speed,.28)
        self.assertAlmostEqual(yaw,math.pi/3)
        with self.assertRaises(ValueError):policy.step(float('nan'),.2)

    def test_optical_forward_and_positive_left_turn(self):
        pose=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,1.2],[0,0,0,1]])
        points=trajectory(pose,.2,.3)
        self.assertGreater(points[-1,0],0)
        self.assertGreater(points[-1,1],0)
        self.assertGreater(points[-1,3],0)
        np.testing.assert_allclose(points[:,2],1.2)

    def test_reference_integrates_sim_time_and_bounds_tracking_lag(self):
        reference=HeadingReference()
        reference.update(0,.4,100)
        for i in range(1,11):value=reference.update(0,.4,100+i*.1)
        self.assertAlmostEqual(value,.4)
        self.assertAlmostEqual(reference.update(0,.4,101),value)
        for i in range(11,100):value=reference.update(0,.4,100+i*.1)
        self.assertAlmostEqual(value,math.pi/4)


if __name__=='__main__':unittest.main()
