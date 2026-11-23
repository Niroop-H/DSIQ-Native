"""Real measured performance figures: CPU only, all supported reference scenes."""
import importlib.util
import math
import unittest
from dsiq_native.render_benchmark import benchmark_render_presets

class RenderValidationTests(unittest.TestCase):
    def test_bounded_arguments(self):
        for args in (
            {"frames":0},{"frames":100},
            {"width":99999},{"height":600},
            {"presets":("invalid",)},
            {"presets":("robotics_cell","robotics_cell")},
        ):
            with self.subTest(args=args),self.assertRaises(ValueError):
                benchmark_render_presets(**args)

@unittest.skipUnless(importlib.util.find_spec("pybullet"),"Requires Bullet")
class ActualRenderBenchmarkTests(unittest.TestCase):
    def test_actual_reference_environments_render_on_cpu(self):
        report=benchmark_render_presets(
            frames=2,width=64,height=48,
            presets=("robotics_cell","cleanroom_bay","logistics_bay"),
        )
        self.assertEqual(len(report["reports"]),3)
        self.assertFalse(report["qualifies_rtx3050"])
        self.assertIsNone(report["rtx_gpu_fps"])
        for scene in report["reports"]:
            self.assertEqual(scene["captured_frames"],2)
            self.assertEqual(scene["renderer"],"BULLET_TINY_RENDERER_CPU")
            self.assertTrue(math.isfinite(scene["render_p50_ms"]))
            self.assertGreater(scene["render_p50_ms"],0.)
            self.assertGreaterEqual(scene["render_p95_ms"],
                                    scene["render_p50_ms"])
            self.assertTrue(all(
                len(f["rgba_sha256"])==64 and
                f["distinct_rgb_colors"]>1 and
                f["depth_min_m"]>0 and
                f["depth_max_m"]>=f["depth_min_m"]
                for f in scene["frames"]
            ))
            self.assertFalse(scene["photorealistic"])
            self.assertFalse(scene["gpu_allocations_measured"])

if __name__=="__main__":
    unittest.main()
