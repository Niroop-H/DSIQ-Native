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
                second=app.add_verified_machine(
                    b["manifest"],base_position_m=(2.,1.,0.)
                )
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
                self.assertGreater(frame["triangles_in_gpu_scene"],400)
                self.assertGreater(frame["elapsed_wall_seconds"],0.)
                self.assertGreater(frame["observed_fps"],0.)
                self.assertFalse(frame["rtx_path_tracing_implemented"])
                self.assertFalse(frame["nvidia_rtx_hardware_qualified"])
                self.assertTrue(image.is_file())
                self.assertGreater(image.stat().st_size,1000)
            finally:
                app.close()

if __name__=="__main__":
    unittest.main()
