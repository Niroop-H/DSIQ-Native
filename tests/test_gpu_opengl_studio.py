"""Real GL3D viewport / physics CAD geometry smoke in Xvfb/Mesa or local NVIDIA.

Only the separately provisioned GPU CI job executes GL graphics; pure
contract jobs avoid a GPU/Panda dependency. No simulated FPS values.
"""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.machine_designer import MachineDesign,save_project

HAS_GL=(bool(os.environ.get("DSIQ_GPU_CI")) and
        importlib.util.find_spec("panda3d") is not None and
        importlib.util.find_spec("pybullet") is not None)


@unittest.skipUnless(HAS_GL,"Use xvfb-run, Panda3D and DSIQ_GPU_CI=1")
class OpenGLWorkcellStudioAcceptance(unittest.TestCase):
    def test_opengl_source_geometry_hierarchy_transform_and_screenshot(self):
        from dsiq_native.gpu_studio import GPUStudio
        with TemporaryDirectory() as tmp:
            a=save_project(
                MachineDesign(asset_id="opengl_first_robot"),Path(tmp)/"a"
            )
            b=save_project(
                MachineDesign(asset_id="opengl_second_robot"),Path(tmp)/"b"
            )
            image=Path(tmp)/"gpu.png"
            app=GPUStudio(headless=True,width=640,height=480)
            try:
                first=app.add_verified_machine(a["manifest"])
                first_cache=app.render_resource_report()
                second=app.add_verified_machine(
                    b["manifest"],base_position_m=(2.,1.,0.)
                )
                second_cache=app.render_resource_report()
                self.assertEqual(
                    second_cache["unique_geom_buffer_objects"],
                    first_cache["unique_geom_buffer_objects"],
                )
                self.assertEqual(
                    second_cache["unique_geom_buffer_triangles"],
                    first_cache["unique_geom_buffer_triangles"],
                )
                self.assertEqual(
                    second_cache["total_authored_triangle_instances"],
                    2*first_cache["total_authored_triangle_instances"],
                )
                self.assertGreater(second_cache["reused_triangle_instances"],0)
                self.assertEqual(second_cache["verified_source_visual_nodes"],34)
                self.assertEqual(second_cache["distinct_bullet_assemblies"],2)
                self.assertEqual(
                    len({x["body_id"] for x in app.world.imported_robots.values()}),2
                )
                self.assertFalse(second_cache["gpu_vram_saving_verified"])
                self.assertEqual(first["parts"],17)
                self.assertEqual(second["parts"],17)
                self.assertGreater(first["triangles"],200)
                self.assertEqual(len(app._objects),2)
                self.assertGreater(app._triangles,400)
                app._selected="opengl_first_robot"
                before=next(x for x in app.editor.entities()
                            if x["asset_id"]==app._selected)["position_m"]
                moved=app.move_selected(.05,0.,0.)
                self.assertTrue(moved["changed"])
                self.assertAlmostEqual(moved["position_m"][0],before[0]+.05)
                app.undo()
                restored=next(x for x in app.editor.entities()
                              if x["asset_id"]==app._selected)["position_m"]
                self.assertAlmostEqual(restored[0],before[0])
                app.redo()
                before_grid=next(x for x in app.editor.entities()
                                 if x["asset_id"]==app._selected)["position_m"]
                grid=app.set_grid_snap(.005)
                self.assertEqual(grid["grid_snap_m"],.005)
                snap=app.move_selected(.025,0,0)
                self.assertAlmostEqual(snap["position_m"][0],before_grid[0]+.005)
                app.undo()
                after_grid=next(x for x in app.editor.entities()
                                if x["asset_id"]==app._selected)["position_m"]
                self.assertAlmostEqual(after_grid[0],before_grid[0])
                self.assertEqual(app.cycle_grid_snap()["grid_snap_m"],.001)
                with self.assertRaises(ValueError):
                    app.set_grid_snap(float("nan"))
                app.set_grid_snap(0)
                frame=app.benchmark(frames=24,screenshot=image)
                self.assertEqual(frame["renderer"],"Panda3D_OpenGL_rasterization")
                self.assertEqual(frame["referenced_assemblies"],2)
                self.assertEqual(
                    frame["render_resource_reuse"]["unique_geom_buffer_objects"],
                    first_cache["unique_geom_buffer_objects"],
                )
                self.assertEqual(
                    frame["render_resource_reuse"]["reused_triangle_instances"],
                    second_cache["reused_triangle_instances"],
                )
                self.assertGreater(frame["triangles_in_gpu_scene"],400)
                self.assertGreater(frame["elapsed_wall_seconds"],0.)
                self.assertGreater(frame["observed_fps"],0.)
                self.assertFalse(frame["rtx_path_tracing_implemented"])
                self.assertFalse(frame["nvidia_rtx_hardware_qualified"])
                self.assertTrue(image.is_file())
                self.assertGreater(image.stat().st_size,1000)
            finally:
                app.close()

@unittest.skipUnless(HAS_GL,"Use xvfb-run, Panda3D and DSIQ_GPU_CI=1")
class MultiAssetRenderReuseAcceptance(unittest.TestCase):
    def test_multiple_real_gl_machines_share_only_exact_source_geometry(self):
        from dsiq_native.gpu_studio import benchmark_exact_geometry_instances
        report=benchmark_exact_geometry_instances(
            copies=3,frames=18,width=480,height=360
        )
        resources=report["source_render_cache"]
        self.assertEqual(report["actual_distinct_Bullet_bodies"],3)
        self.assertEqual(resources["distinct_bullet_assemblies"],3)
        self.assertEqual(resources["verified_source_visual_nodes"],51)
        self.assertGreater(resources["reused_triangle_instances"],0)
        self.assertLess(
            resources["unique_geom_buffer_triangles"],
            resources["total_authored_triangle_instances"],
        )
        self.assertEqual(
            resources["total_authored_triangle_instances"],
            report["triangles_in_gpu_scene"],
        )
        self.assertEqual(report["referenced_assemblies"],3)
        self.assertFalse(resources["gpu_vram_saving_verified"])
        self.assertFalse(report["hydra_or_vulkan_renderer_implemented"])
        self.assertGreater(report["observed_fps"],0)
        self.assertGreater(report["assembly_import_wall_seconds"],0)

    def test_invalid_instance_budget_fails_before_gpu_initialization(self):
        from dsiq_native.gpu_studio import benchmark_exact_geometry_instances
        for count in (0,1,25,True,2.5):
            with self.subTest(count=count):
                with self.assertRaises(ValueError):
                    benchmark_exact_geometry_instances(copies=count)


if __name__=="__main__":
    unittest.main()
