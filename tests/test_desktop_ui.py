"""End-to-end smoke tests for the *real* desktop GUI under Linux Xvfb.

This test is skipped on headless CI jobs without a GUI display. A dedicated
Xvfb job exercises the actual Tk window, Bullet live renderer, controls and
theme switching. It is NOT an RTX3050 hardware-qualification test.
"""
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.desktop_studio import DesktopStudio, StudioConfig


@unittest.skipUnless(bool(os.environ.get("DISPLAY")), "Tk display / Xvfb required")
class DesktopStudioUiSmokeTests(unittest.TestCase):
    def setUp(self):
        self.app = DesktopStudio(StudioConfig(
            theme="black", boxes=1, camera_width=160,
            camera_height=120, fps_limit=8,
        ))
        self.addCleanup(self.app.close)

    def test_live_camera_theme_robot_and_payload_controls(self):
        app = self.app
        app.root.update()
        self.assertEqual(app.root.cget("background"), "#000000")
        self.assertIsNotNone(app._latest)
        self.assertEqual(len(app._latest.rgba), 160*120*4)
        self.assertTrue(app._image is not None)

        app.set_theme("light")
        app.root.update()
        self.assertEqual(app.root.cget("background"), "#FFFFFF")
        app.set_theme("black")
        app.root.update()
        self.assertEqual(app.root.cget("background"), "#000000")

        prev = len(app.world.payloads)
        app._mass.set("2")
        app._side.set("0.18")
        app.add_box()
        self.assertEqual(len(app.world.payloads), prev + 1)

        app._joint_vars[1].set(-15.)
        app._joint_vars[2].set(25.)
        app.apply_joints()
        self.assertNotEqual(app.world.report()["target_radians"][1], 0.)
        app.step_once()
        self.assertEqual(app.world.step_index, 1)

        app.home()
        self.assertTrue(all(abs(x) < 1e-8 for x in app.world.report()["target_radians"]))
        app.lidar_scan()
        self.assertIn("LiDAR", app._status_message)

        app.pause()
        app._render()
        self.assertEqual(app._latest.width, 160)

    def test_image_and_state_file_exports(self):
        app = self.app
        app.root.update()
        with TemporaryDirectory() as tmp:
            target = Path(tmp) / "robot.png"
            app.filedialog = type("StubFileDialog", (), {
                "asksaveasfilename": staticmethod(lambda **kw: str(target))
            })()
            app.save_camera_png()
            self.assertTrue(target.is_file())
            self.assertEqual(target.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            report = Path(tmp) / "state.json"
            app.filedialog = type("StubFileDialog", (), {
                "asksaveasfilename": staticmethod(lambda **kw: str(report))
            })()
            app.export_state()
            self.assertTrue(report.is_file())
            self.assertIn("pybullet_cpu", report.read_text())

    def test_smooth_joint_program_executes_actual_bullet_motor_steps(self):
        app = self.app
        app.root.update()
        app._joint_vars[1].set(-10.)
        app._joint_vars[2].set(14.)
        app.plan_motion()
        self.assertIsNotNone(app._active_plan)
        first = app.world.step_index
        app._advance_plan(12)
        self.assertGreater(app.world.step_index, first)
        self.assertGreater(app._plan_index, 1)
        self.assertGreaterEqual(app._plan_tracking_max, 0.)
        app.pause()
        self.assertFalse(app.running)
        app.root.update()

    def test_free_fly_and_semantic_environments_in_live_viewport(self):
        from types import SimpleNamespace
        app=self.app
        app.root.update()
        app.set_camera_mode("free")
        before=app._free_camera.position_m
        app._camera_key(SimpleNamespace(keysym="w",state=0))
        self.assertNotEqual(before,app._free_camera.position_m)
        app._mouse_down(SimpleNamespace(x=100,y=100))
        yaw=app._free_camera.yaw_rad
        app._mouse_move(SimpleNamespace(x=140,y=98))
        self.assertNotEqual(yaw,app._free_camera.yaw_rad)
        app.set_environment("robotics_cell")
        report=app.environment.report()
        self.assertGreater(report["part_count"],40)
        self.assertGreater(report["collision_parts"],20)
        app.set_viewport_quality("performance")
        app._render()
        self.assertEqual(app._latest.width,192)
        app.set_environment("cleanroom_bay")
        self.assertGreater(app.environment.report()["part_count"],50)
        app.set_environment("logistics_bay")
        self.assertGreater(app.environment.report()["part_count"],30)
        app.set_environment("open")
        self.assertEqual(app.environment.report()["part_count"],0)
        app.set_camera_mode("orbit")

    def test_custom_engineered_gantry_is_imported_and_motors_actuated(self):
        from dsiq_native.machine_designer import MachineDesign,save_project
        app=self.app
        app.root.update()
        with TemporaryDirectory() as folder:
            export=save_project(
                MachineDesign(asset_id="gui_user_test_gantry"),
                Path(folder)/"machine"
            )
            app._use_machine_project(export)
            self.assertIn("gui_user_test_gantry",app.world.imported_robots)
            result=app.world.command_imported_joint(
                "gui_user_test_gantry","actuator_x",.22
            )
            self.assertEqual(result["units"],"meters")
            data=app.world.step(180)
            reported=data["imported_robots"][0]["joint_states"]
            x=next(j for j in reported if j["name"]=="actuator_x")
            self.assertGreater(x["position"],.01)
        self.assertEqual(app._camera_mode,"free")

    def test_real_analytic_physics_verification_accessible_from_studio(self):
        app=self.app
        app.root.update()
        report=app.verify_physics()
        self.assertEqual(report["case_count"],5)
        self.assertTrue(report["passed"],report)
        self.assertIn("VERIFICATION",app._status_message)

    def test_reset_scene_retains_valid_ui(self):
        app = self.app
        app.root.update()
        app.world.step(20)
        self.assertGreater(app.world.step_index, 0)
        app.reset()
        self.assertEqual(app.world.step_index, 0)
        self.assertEqual(len(app.world.payloads), 1)
        app._render()
        self.assertIsNotNone(app._latest)


if __name__ == "__main__":
    unittest.main()
