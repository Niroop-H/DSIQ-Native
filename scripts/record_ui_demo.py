"""Record REAL interactive DSIQ Native Panda3D editor and Bullet motion.

This is an acceptance capture on the current display, not a mockup or still
image animation. Runs in an Xvfb display on GitHub Actions using software GL.
No real OEM robot, RTX3050 GPU, or industrial safety certificate is involved.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import time
from tempfile import TemporaryDirectory

from dsiq_native.gpu_studio import GPUStudio
from dsiq_native.machine_designer import MachineDesign, save_project


def run(output_directory: str = "build/ui-demo") -> dict:
    out = Path(output_directory).resolve()
    out.mkdir(parents=True, exist_ok=True)
    video = out / "dsiq-native-editor-live-test.mp4"
    screenshot = out / "dsiq-native-editor-ui.png"
    report_path = out / "dsiq-native-ui-test.json"
    if any(p.exists() for p in (video, screenshot, report_path)):
        raise FileExistsError("Never overwrite existing UI acceptance evidence")

    if not os.environ.get("DISPLAY"):
        raise RuntimeError("Real X11 display required; run with xvfb-run or a desktop")
    with TemporaryDirectory(prefix="dsiq-ui-demo-") as directory:
        source = save_project(
            MachineDesign(asset_id="live_editor_demo_gantry"),
            Path(directory) / "machine",
        )
        app = GPUStudio(headless=False, width=1100, height=680)
        ffmpeg = None
        try:
            imported = app.add_verified_machine(source["manifest"])
            aid = imported["asset_id"]
            if imported["parts"] < 10 or imported["triangles"] < 200:
                raise AssertionError("Source-verified machine visual geometry unavailable")
            if not app.base.win or not app.base.win.isValid():
                raise AssertionError("Real native OpenGL window unavailable")
            if len(app._hierarchy_controls) < 5:
                raise AssertionError("On-screen scene hierarchy did not initialize")
            # Frame the actual assembled machine and make all real widgets visible.
            app._center = (0., 1., .72)
            app._distance = 3.9
            app._yaw = 32.
            app._pitch = -23.
            app._set_camera()
            app._choose(aid)
            for _ in range(32):
                app.base.taskMgr.step()

            robot = app.world.imported_robots[aid]
            joint = next(j for j in robot["joints"] if j["name"] == "actuator_x")
            body = robot["body_id"]
            cid = app.world.client
            p = app.world.p
            initial_joint = float(p.getJointState(
                body, joint["joint_index"], physicsClientId=cid
            )[0])
            if not math.isfinite(initial_joint):
                raise AssertionError("Initial Bullet joint feedback invalid")

            # ffmpeg grabs actual X11 framebuffer pixels at 12 fps. The
            # Panda3D window and UI below are being updated in real time.
            args = [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                "-f", "x11grab", "-framerate", "12",
                "-video_size", "1280x720", "-i", os.environ["DISPLAY"],
                "-t", "15", "-an",
                "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(video),
            ]
            ffmpeg = subprocess.Popen(
                args, stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )

            timeline = []
            starting_pose = next(
                e["position_m"] for e in app.editor.entities()
                if e["asset_id"] == aid
            )
            checkpoints = [
                ("imported", 0.),
                ("real_transform", 2.2),
                ("undo", 3.8),
                ("redo", 5.0),
                ("orbit", 6.2),
                ("bullet_motor", 8.2),
                ("closer_view", 11.1),
                ("stop_physics", 13.8),
            ]
            next_checkpoint = 0
            began = time.monotonic()
            while True:
                elapsed = time.monotonic() - began
                if elapsed >= 15.5:
                    break
                if ffmpeg.poll() is not None and elapsed < 14.5:
                    details = (ffmpeg.stderr.read() or b"").decode("utf8","replace")
                    raise RuntimeError("Live-screen ffmpeg exited early: "+details[-1500:])
                while next_checkpoint < len(checkpoints) and elapsed >= checkpoints[next_checkpoint][1]:
                    action = checkpoints[next_checkpoint][0]
                    if action == "real_transform":
                        moved = app.move_selected(.12,0.,0.)
                        if not moved or not moved["changed"]:
                            raise AssertionError("Real workcell transform edit did not apply")
                    elif action == "undo":
                        result = app.undo()
                        if not result:
                            raise AssertionError("CAD undo operation failed")
                        position = next(
                            e["position_m"] for e in app.editor.entities()
                            if e["asset_id"] == aid
                        )
                        if abs(position[0]-starting_pose[0]) > 1e-6:
                            raise AssertionError("CAD undo failed to restore Bullet pose")
                    elif action == "redo":
                        result = app.redo()
                        if not result:
                            raise AssertionError("CAD redo operation failed")
                    elif action == "bullet_motor":
                        app.world.command_imported_joint(aid,"actuator_x",.25)
                        app.world.command_imported_joint(aid,"actuator_y",.10)
                        app.toggle_play()
                    elif action == "closer_view":
                        app._distance=3.1
                        app._set_camera()
                    elif action == "stop_physics":
                        app.toggle_play()
                    timeline.append({"action":action,"wall_elapsed_s":round(elapsed,3)})
                    next_checkpoint += 1

                # A real, rendered viewport orbit shows the loaded geometry
                # from different angles rather than moving a static picture.
                if 6.2 < elapsed < 8.1:
                    app._yaw += .32
                    app._set_camera()
                app.base.taskMgr.step()
                time.sleep(.04)

            exit_code = ffmpeg.wait(timeout=30)
            if exit_code != 0:
                details = (ffmpeg.stderr.read() or b"").decode("utf8","replace")
                raise RuntimeError("Live X11 recording failed: "+details[-2500:])
            if not video.is_file() or video.stat().st_size < 20_000:
                raise AssertionError("Actual UI recording missing or unexpectedly small")
            from panda3d.core import Filename
            if not app.base.win.saveScreenshot(
                Filename.fromOsSpecific(str(screenshot))
            ):
                raise RuntimeError("Unable to save genuine UI framebuffer still")
            final_joint = float(p.getJointState(
                body, joint["joint_index"], physicsClientId=cid
            )[0])
            if not math.isfinite(final_joint) or final_joint <= initial_joint + .005:
                raise AssertionError(
                    f"No actual Bullet joint movement: {initial_joint} -> {final_joint}"
                )
            if app.world.step_index <= 0:
                raise AssertionError("Physics simulation did not advance")

            result = {
                "schema": "dsiq-ui-live-recording-v1",
                "video": video.name,
                "screenshot": screenshot.name,
                "duration_requested_s": 15,
                "capture_source": "REAL_X11_ONSCREEN_PANDA3D_OPENGL_EDITOR",
                "render_backend": "Xvfb_Mesa_software_OpenGL",
                "recorded_animated_mockup": False,
                "on_screen_ui_present": True,
                "authored_source_machine_imported": True,
                "actual_source_triangles": imported["triangles"],
                "actual_source_parts": imported["parts"],
                "on_screen_hierarchy_controls": len(app._hierarchy_controls),
                "real_transform_move_and_undo_redo_tested": True,
                "simulation_engine": "PyBullet",
                "measured_initial_actuator_x_m": initial_joint,
                "measured_final_actuator_x_m": final_joint,
                "actual_simulated_physics_steps": app.world.step_index,
                "machine_physical_oem_calibration": False,
                "gpu_rtx3050_hardware_qualified": False,
                "industrial_safety_certified": False,
                "timeline": timeline,
            }
            report_path.write_text(
                json.dumps(result, indent=2)+"\n", encoding="utf8",
            )
            print(json.dumps(result, indent=2), flush=True)
            return result
        finally:
            if ffmpeg is not None and ffmpeg.poll() is None:
                ffmpeg.terminate()
                try:
                    ffmpeg.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    ffmpeg.kill()
            app.close()


if __name__ == "__main__":
    run()
