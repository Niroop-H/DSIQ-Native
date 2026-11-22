"""A real Kit-hosted USD stage inspector. Rendering/physics are provided by Kit.

No fabricated geometry or unsupported GPU settings. This extension works only
when loaded by a separately licensed/installed compatible Omniverse Kit app.
"""
import omni.ext
import omni.ui as ui
import omni.usd

from pxr import UsdGeom, UsdPhysics


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window(
            "DSIQ Native | USD Scene Diagnostics", width=520, height=390
        )
        with self._window.frame:
            with ui.VStack(spacing=9, height=0):
                ui.Label("DSIQ Native / Scene Assurance", height=28)
                ui.Label(
                    "Current OpenUSD stage, asset identity and physics-schema diagnostics. "
                    "No factory certification is inferred.",
                    word_wrap=True,
                    height=52,
                )
                ui.Button("Inspect active OpenUSD stage", clicked_fn=self._inspect)
                self._status = ui.Label(
                    "Open a USD stage in Kit, then select Inspect.",
                    word_wrap=True,
                    height=270,
                )

    def _inspect(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._status.text = "No USD stage is open. Load a project through Kit's File menu."
            return
        counts = {
            "prims": 0,
            "meshes": 0,
            "cameras": 0,
            "physics_scenes": 0,
            "rigid_bodies": 0,
        }
        try:
            for prim in stage.Traverse():
                counts["prims"] += 1
                if prim.IsA(UsdGeom.Mesh):
                    counts["meshes"] += 1
                if prim.IsA(UsdGeom.Camera):
                    counts["cameras"] += 1
                if prim.IsA(UsdPhysics.Scene):
                    counts["physics_scenes"] += 1
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    counts["rigid_bodies"] += 1
            units = UsdGeom.GetStageMetersPerUnit(stage)
            axis = UsdGeom.GetStageUpAxis(stage)
            valid_frame = abs(float(units) - 1.0) < 1e-9 and str(axis) == "Z"
            self._status.text = (
                f"Stage: {stage.GetRootLayer().identifier}\n"
                f"Units: {units} meter(s)/unit | Up axis: {axis}\n"
                f"Canonical DSIQ coordinate frame: {'YES' if valid_frame else 'NO'}\n\n"
                f"Prims: {counts['prims']}\n"
                f"Meshes: {counts['meshes']}\n"
                f"Cameras: {counts['cameras']}\n"
                f"Physics scenes: {counts['physics_scenes']}\n"
                f"Rigid bodies: {counts['rigid_bodies']}\n\n"
                "These checks do not measure geometric fidelity, joint dynamics, "
                "collision correctness, GPU residency, or OEM acceptance."
            )
        except Exception as exc:
            self._status.text = f"Scene inspection failed: {exc}"

    def on_shutdown(self):
        self._status = None
        self._window = None
