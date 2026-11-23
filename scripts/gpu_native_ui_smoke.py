"""Exercise real ON-SCREEN Panda3D native editor widgets under virtual X11.

Does not take the place of user testing on Windows/RTX3050, but catches
broken DirectGUI controls or window initialization that an offscreen
framebuffer test alone would miss.
"""
from pathlib import Path
from tempfile import TemporaryDirectory

from dsiq_native.gpu_studio import GPUStudio
from dsiq_native.machine_designer import MachineDesign,save_project


def run():
    with TemporaryDirectory() as path:
        source=save_project(
            MachineDesign(asset_id="onscreen_native_editor_test"),
            Path(path)/"machine",
        )
        app=GPUStudio(headless=False,width=800,height=600)
        try:
            result=app.add_verified_machine(source["manifest"])
            if result["parts"]<10:
                raise AssertionError("Actual source assembly missing geometry")
            if not hasattr(app,"_hierarchy") or len(app._hierarchy_controls)<5:
                raise AssertionError("Native editor scene hierarchy not initialized")
            if "SourceCAD" in str(result):
                raise AssertionError("Source metadata returned unexpected fake data")
            app._choose(result["asset_id"])
            before=next(x for x in app.editor.entities()
                        if x["asset_id"]==result["asset_id"])["position_m"]
            app.move_selected(.025,0.,0.)
            app.undo()
            after=next(x for x in app.editor.entities()
                       if x["asset_id"]==result["asset_id"])["position_m"]
            if abs(after[0]-before[0])>1e-7:
                raise AssertionError("On-screen CAD transform undo did not restore Bullet pose")
            # Actual interactive GUI grid snapping runs against the Bullet base
            # pose, and undo must restore the exact source-geometry placement.
            app.set_grid_snap(.005)
            snapped=app.move_selected(.025,0,0)
            if abs(snapped["position_m"][0]-(before[0]+.005))>1e-7:
                raise AssertionError("Native editor 5 mm grid snapping failed")
            app.undo()
            restored=next(x for x in app.editor.entities()
                          if x["asset_id"]==result["asset_id"])["position_m"]
            if abs(restored[0]-before[0])>1e-7:
                raise AssertionError("Snapped real transform undo did not restore pose")
            app.set_grid_snap(0.)
            for _ in range(18):
                app.base.taskMgr.step()
            state=app.toggle_imu_overlay()
            if not state["enabled"] or state["sensor_calibrated"]:
                raise AssertionError("Real native IMU diagnostic overlay not enabled")
            app.toggle_play()
            for _ in range(28):
                app.base.taskMgr.step()
            app.toggle_play()
            if "SPECIFIC FORCE [m/s2]" not in app._imu_label["text"]:
                raise AssertionError("Actual on-screen physics IMU values did not update")
            app.toggle_imu_overlay()
            if not app.base.win or not app.base.win.isValid():
                raise AssertionError("Actual Panda3D GL editor window invalid")
            print({
                "native_interactive_window":True,
                "real_bullet_link_visualization":True,
                "hierarchy_control_count":len(app._hierarchy_controls),
                "source_triangles":app._triangles,
                "undo_restores_simulated_base_pose":True,
                "native_metric_grid_snap_passed":True,
                "native_live_bullet_imu_overlay_passed":True,
                "RTX3050_hardware_test":False,
            })
        finally:
            app.close()


if __name__=="__main__":
    run()
