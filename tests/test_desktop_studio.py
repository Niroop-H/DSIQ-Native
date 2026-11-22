"""Desktop studio quality checks that run on headless Windows/Linux CI."""
import math
import unittest

from dsiq_native.desktop_studio import PALETTES, StudioConfig, orbit_eye


class NativeDesktopContractTests(unittest.TestCase):
    def test_pure_black_and_light_themes_have_full_contrast(self):
        self.assertEqual(PALETTES["black"]["background"], "#000000")
        self.assertEqual(PALETTES["light"]["background"], "#FFFFFF")
        self.assertNotEqual(PALETTES["black"]["text"], PALETTES["light"]["text"])
        self.assertTrue(all(key in PALETTES["black"]
                            for key in ("panel","input","text","muted","accent","button")))
        for palette in PALETTES.values():
            for color in palette.values():
                self.assertRegex(color, r"^#[0-9A-Fa-f]{6}$")

    def test_12fps_small_cpu_camera_is_admitted(self):
        StudioConfig().validate()
        StudioConfig(theme="light", boxes=40, camera_width=400,
                     camera_height=300, fps_limit=15).validate()

    def test_invalid_scene_budget_is_rejected(self):
        for values in (
            {"theme": "midnight"},
            {"boxes": 100},
            {"camera_width": 1280},
            {"camera_height": 920},
            {"fps_limit": 0},
            {"fps_limit": 120},
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                StudioConfig(**values).validate()

    def test_orbit_geometry_is_finite_and_metric(self):
        eye = orbit_eye(0, 0, 3., center_m=(0.,0.,.5))
        self.assertEqual(eye, (3.,0.,.5))
        center = (0.1,0.,.7)
        p = orbit_eye(.6,.3,2.5,center)
        self.assertAlmostEqual(math.dist(p, center), 2.5, places=10)
        self.assertTrue(p[2] > center[2])

    def test_orbit_limits_prevent_numerical_pathology(self):
        with self.assertRaises(ValueError):
            orbit_eye(float("nan"),0,2.)
        with self.assertRaises(ValueError):
            orbit_eye(0,0,.5)
        with self.assertRaises(ValueError):
            orbit_eye(0,math.pi/2,2.)


if __name__ == "__main__":
    unittest.main()
