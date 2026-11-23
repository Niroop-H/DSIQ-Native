"""Principal moment triangle inequalities must hold in ANY inertial frame."""
import math
import unittest

from dsiq_native.urdf_bridge import _verify_inertia_tensor,_principal_moments


def tensor(xx=1.,yy=1.,zz=1.,xy=0.,xz=0.,yz=0.):
    return {"ixx":xx,"iyy":yy,"izz":zz,
            "ixy":xy,"ixz":xz,"iyz":yz}


class PrincipalMomentTests(unittest.TestCase):
    def test_strong_cross_inertia_is_spd_but_not_physically_feasible(self):
        impossible=tensor(xz=.95)
        values=_principal_moments(impossible)
        self.assertAlmostEqual(values[0],.05,places=9)
        self.assertAlmostEqual(values[1],1.,places=9)
        self.assertAlmostEqual(values[2],1.95,places=9)
        with self.assertRaisesRegex(ValueError,"principal inertia moments"):
            _verify_inertia_tensor(impossible,"bad_hardware")

    def test_valid_rotated_inertia_passes_and_principal_invariants_match(self):
        source=tensor(xx=1.,yy=2.,zz=2.5)
        rotated=tensor(xx=1.5,yy=1.5,zz=2.5,xy=-.5)
        for actual,expected in zip(
            _principal_moments(rotated),_principal_moments(source)
        ):
            self.assertAlmostEqual(actual,expected,places=9)
        _verify_inertia_tensor(rotated,"calibrated_reference_shape")

    def test_feasible_positive_non_diagonal_source_passes(self):
        case=tensor(xz=.4)
        _verify_inertia_tensor(case,"symmetric_feasible")
        vals=_principal_moments(case)
        self.assertGreaterEqual(vals[0]+vals[1],vals[2]-1e-10)

    def test_negative_or_bad_tensor_fails(self):
        with self.assertRaisesRegex(ValueError,"non-positive-definite"):
            _verify_inertia_tensor(tensor(xy=1.1),"nonSPD")
        with self.assertRaisesRegex(ValueError,"moment inequalities"):
            _verify_inertia_tensor(tensor(xx=.1,yy=.1,zz=5.),"not_real")

if __name__=="__main__":
    unittest.main()
