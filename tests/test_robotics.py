"""Analytic robotics correctness gates independent of GPU and physics runtime."""
import math
import unittest

from dsiq_native.robotics import (
    forward_kinematics, inverse_kinematics, translation_jacobian,
    JOINT_LIMITS_DEG, check_angles,
)


class KinematicsTests(unittest.TestCase):
    def test_zero_configuration_is_metric_and_reproducible(self):
        zero = [0.0] * 6
        a = forward_kinematics(zero)
        b = forward_kinematics(zero)
        self.assertEqual(a, b)
        for actual, target in zip(a.end_effector_m, (1.2, 0., 0.5)):
            self.assertAlmostEqual(actual, target, places=12)
        self.assertEqual(len(a.joint_origins_m), 6)

    def test_rotation_is_orthonormal_and_right_handed(self):
        p = forward_kinematics((0.4, -0.5, 0.2, 0.6, -0.3, 0.1))
        R = p.orientation
        for i in range(3):
            for j in range(3):
                dot = sum(R[k][i] * R[k][j] for k in range(3))
                self.assertAlmostEqual(dot, 1. if i == j else 0., places=10)
        det = (
            R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1]) -
            R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0]) +
            R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0])
        )
        self.assertAlmostEqual(det, 1., places=10)

    def test_analytic_jacobian_matches_finite_difference(self):
        angles = [0.25, -0.4, 0.5, 0.2, -0.3, 0.1]
        J = translation_jacobian(angles)
        eps = 1e-6
        for i in range(6):
            hi = list(angles)
            lo = list(angles)
            hi[i] += eps
            lo[i] -= eps
            a = forward_kinematics(hi).end_effector_m
            b = forward_kinematics(lo).end_effector_m
            for row in range(3):
                self.assertAlmostEqual((a[row]-b[row])/(2*eps), J[row][i], places=6)

    def test_limits_and_invalid_values_fail_closed(self):
        with self.assertRaises(ValueError):
            check_angles((1.0, 2.0))
        with self.assertRaises(ValueError):
            check_angles((math.nan,) + (0.,)*5)
        with self.assertRaises(ValueError):
            check_angles((0., math.radians(130), 0., 0., 0., 0.))
        for lo, hi in JOINT_LIMITS_DEG:
            self.assertLess(lo, hi)

    def test_ik_handles_home_and_unreachable_targets(self):
        home = inverse_kinematics((1.2, 0., .5))
        self.assertTrue(home.converged)
        self.assertLess(home.position_error_m, 1e-9)
        self.assertFalse(inverse_kinematics((20.,20.,20.), max_iterations=80).converged)

    def test_ik_converges_on_reachable_reference(self):
        reference = (.22, -.45, .40, .20, -.18, .08)
        target = forward_kinematics(reference).end_effector_m
        solved = inverse_kinematics(target, tolerance_m=0.002)
        self.assertTrue(solved.converged, f"Residual {solved.position_error_m:.6f} meters")
        self.assertLess(solved.position_error_m, 0.002)
        self.assertLessEqual(solved.iterations, 250)
        check_angles(solved.angles_rad)

    def test_ik_invalid_api_arguments_are_rejected(self):
        with self.assertRaises(ValueError):
            inverse_kinematics((1., 2.))
        with self.assertRaises(ValueError):
            inverse_kinematics((1., 2., math.inf))
        with self.assertRaises(ValueError):
            inverse_kinematics((1., 2., 3.), damping=0.)
        with self.assertRaises(ValueError):
            inverse_kinematics((1., 2., 3.), max_iterations=0)


if __name__ == "__main__":
    unittest.main()
