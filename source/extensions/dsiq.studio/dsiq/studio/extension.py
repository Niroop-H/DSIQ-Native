"""DSIQ Native Kit Studio: working USD robotics-playground UI and scene assurance.

Hosts inside a separately installed, compatible NVIDIA Kit application.
UI controls operate real authored USD schemas and Kit timeline state. They do
NOT silently claim an RTX renderer, PhysX extension, or OEM-calibrated dynamics
where the host cannot provide those capabilities.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import traceback

import omni.ext
import omni.ui as ui
import omni.usd
import omni.timeline
from omni.ui import color as cl
from pxr import UsdGeom, UsdPhysics

# Keep the new software independent of the pre-existing browser repository.
# In a local checkout, the extension is five parents below the Git repository.
_CHECKOUT = Path(__file__).resolve().parents[5]
if (_CHECKOUT / "dsiq_native").is_dir() and str(_CHECKOUT) not in sys.path:
    sys.path.insert(0, str(_CHECKOUT))

from dsiq_native.playground import (  # noqa: E402
    PlaygroundSettings, create_playground, playground_report, spawn_box,
    set_reference_joint_targets,
)
from dsiq_native.robotics import (  # noqa: E402
    JOINT_LIMITS_DEG, forward_kinematics, inverse_kinematics,
)

THEMES = {
    "black": {
        "background": "#000000",
        "panel": "#101318",
        "button": "#1C2633",
        "text": "#FFFFFF",
        "muted": "#C2CBD5",
        "accent": "#87BBFF",
        "border": "#2B3645",
    },
    "light": {
        "background": "#FFFFFF",
        "panel": "#F1F4F8",
        "button": "#E5EBF3",
        "text": "#111827",
        "muted": "#334155",
        "accent": "#1155A6",
        "border": "#C5CFDA",
    },
}


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._theme = "black"
        self._status_message = (
            "DSIQ Native is a development preview. Open a metric Z-up USD scene; "
            "press Create Playground. RTX/PhysX use depends on your installed Kit."
        )
        self._data = {
            "mass": ui.SimpleFloatModel(1.0),
            "side": ui.SimpleFloatModel(0.16),
            "gravity": ui.SimpleFloatModel(9.81),
            "x": ui.SimpleFloatModel(-0.5),
            "y": ui.SimpleFloatModel(0.75),
            "z": ui.SimpleFloatModel(1.1),
            "target_x": ui.SimpleFloatModel(0.75),
            "target_y": ui.SimpleFloatModel(0.20),
            "target_z": ui.SimpleFloatModel(0.70),
        }
        self._joints = [ui.SimpleFloatModel(0.0) for _ in range(6)]
        self._window = ui.Window(
            "DSIQ Native | Engineering & Robotics Studio",
            width=510, height=850,
        )
        self._window.frame.set_build_fn(self._build_ui)
        self._window.frame.rebuild()
        self._status("DSIQ Native loaded. Create or open a USD scene in Kit.")

    def _set_theme(self, mode):
        if mode not in THEMES:
            return
        self._theme = mode
        self._window.frame.rebuild()
        self._status(f"{mode.upper()} theme active. Kit viewport rendering is unaffected.")

    def _build_ui(self):
        palette = THEMES[self._theme]
        self._window.frame.set_style({
            "Window": {
                "background_color": cl(palette["background"]),
                "border_color": cl(palette["border"]),
                "border_width": 1,
            }
        })
        widget_style = {
            "Label": {"color": cl(palette["text"])},
            "Button": {
                "background_color": cl(palette["button"]),
                "color": cl(palette["text"]),
                "border_color": cl(palette["border"]),
                "border_width": 1,
            },
            "Button:hovered": {"background_color": cl(palette["accent"])},
            "FloatField": {
                "background_color": cl(palette["panel"]),
                "color": cl(palette["text"]),
            },
        }
        with ui.ScrollingFrame(
            style={"ScrollingFrame": {"background_color": cl(palette["background"])}}
        ):
            with ui.VStack(spacing=9, style=widget_style, width=0):
                ui.Label("DSIQ  /  NATIVE ENGINEERING STUDIO", height=25)
                ui.Label(
                    "OpenUSD  ·  Robotics  ·  Physics  ·  Scene Assurance",
                    height=22,
                )
                with ui.HStack(height=30, spacing=8):
                    ui.Button("PURE BLACK #000000",
                              clicked_fn=lambda: self._set_theme("black"))
                    ui.Button("LIGHT #FFFFFF",
                              clicked_fn=lambda: self._set_theme("light"))
                ui.Separator()
                ui.Label("01 / SCENE CONTROLS", height=22)
                with ui.HStack(height=32, spacing=8):
                    ui.Button("Inspect Active Stage", clicked_fn=self._inspect)
                    ui.Button("Save USD Layer", clicked_fn=self._save_stage)
                with ui.HStack(height=32, spacing=8):
                    ui.Button("Create Playground", clicked_fn=lambda:
                              self._make_playground(False))
                    ui.Button("RESET PLAYGROUND", clicked_fn=lambda:
                              self._make_playground(True))
                ui.Label(
                    "Reset replaces only /World/Playground. It does not delete "
                    "your other USD stage assets. Reset cannot be undone here.",
                    word_wrap=True, height=45,
                )
                ui.Separator()
                ui.Label("02 / PHYSICS TIMELINE", height=22)
                with ui.HStack(height=32, spacing=8):
                    ui.Button("PLAY", clicked_fn=lambda: self._timeline("play"))
                    ui.Button("PAUSE", clicked_fn=lambda: self._timeline("pause"))
                    ui.Button("STOP", clicked_fn=lambda: self._timeline("stop"))
                ui.Label(
                    "Play/pause/stop control Kit's actual timeline. PhysX "
                    "must be installed/enabled to simulate USD rigid bodies.",
                    word_wrap=True, height=42,
                )
                ui.Separator()
                ui.Label("03 / PAYLOAD GENERATOR", height=22)
                self._field("Payload mass (kg)", "mass", 80)
                self._field("Payload cube side (m)", "side", 80)
                with ui.HStack(height=29, spacing=5):
                    for name in ("x", "y", "z"):
                        self._field(name.upper() + " (m)", name, 40)
                ui.Button("SPAWN DYNAMIC PAYLOAD",
                          clicked_fn=self._spawn, height=32)
                ui.Separator()
                ui.Label("04 / ROBOT REFERENCE-6R", height=22)
                ui.Label(
                    "6 revolute axes · target angles in degrees · model is NOT OEM CAD",
                    word_wrap=True, height=36,
                )
                for i in range(6):
                    with ui.HStack(height=25, spacing=5):
                        lo, hi = JOINT_LIMITS_DEG[i]
                        ui.Label(f"J{i+1}  [{lo:.0f}, {hi:.0f}] deg", width=185)
                        ui.FloatField(model=self._joints[i])
                with ui.HStack(height=32, spacing=8):
                    ui.Button("Apply Joint Targets", clicked_fn=self._apply_joints)
                    ui.Button("Home Robot", clicked_fn=self._home)
                ui.Label("05 / CARTESIAN TARGET — POSITION-ONLY IK", height=22)
                with ui.HStack(height=30, spacing=5):
                    for axis in ("x", "y", "z"):
                        self._field(axis.upper() + " (m)", f"target_{axis}", 40)
                ui.Button("SOLVE + APPLY XYZ", clicked_fn=self._solve_ik, height=32)
                ui.Label(
                    "IK checks joint limits and displays residual. It does "
                    "NOT solve orientation, obstacle avoidance or motor torque.",
                    word_wrap=True, height=42,
                )
                ui.Separator()
                ui.Label("06 / GRAVITY & SETTINGS", height=22)
                self._field("Gravity magnitude (m/s²)", "gravity", 95)
                ui.Button("Apply Scene Gravity", clicked_fn=self._gravity, height=30)
                ui.Separator()
                ui.Label("ENGINEERING DIAGNOSTICS", height=22)
                self._status_label = ui.Label(
                    self._status_message,
                    word_wrap=True,
                    height=230,
                )
                ui.Label(
                    "REAL LIMITS: RTX 3050 is below published Isaac Sim "
                    "requirements. No OEM certification. No hidden surrogate "
                    "geometry. Save before destructive operations.",
                    word_wrap=True, height=60,
                )

    def _field(self, label, key, width):
        with ui.VStack(spacing=2, width=0):
            ui.Label(label, height=17)
            ui.FloatField(model=self._data[key], height=25, width=width)

    def _status(self, message):
        self._status_message = str(message)
        if getattr(self, "_status_label", None) is not None:
            self._status_label.text = self._status_message

    def _stage(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError(
                "No active USD stage. Use Kit File > New or File > Open "
                "before using DSIQ playground controls."
            )
        return stage

    def _run(self, callback):
        try:
            callback()
        except Exception as exc:
            self._status("ACTION FAILED (not silently ignored): " + str(exc))

    def _inspect(self):
        def action():
            stage = self._stage()
            result = playground_report(stage)
            count = {
                "prims": 0, "meshes": 0, "cameras": 0,
                "rigid_bodies": 0, "physics_scenes": 0,
            }
            for prim in stage.Traverse():
                count["prims"] += 1
                if prim.IsA(UsdGeom.Mesh):
                    count["meshes"] += 1
                if prim.IsA(UsdGeom.Camera):
                    count["cameras"] += 1
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    count["rigid_bodies"] += 1
                if prim.IsA(UsdPhysics.Scene):
                    count["physics_scenes"] += 1
                if count["prims"] > 50000:
                    raise RuntimeError("Stage diagnostic limit reached (50,000 prims)")
            self._status(
                f"USD: {stage.GetRootLayer().identifier}\n"
                f"Units: {UsdGeom.GetStageMetersPerUnit(stage)} m/unit\n"
                f"Axis: {UsdGeom.GetStageUpAxis(stage)}\n"
                f"Prims: {count['prims']} | Meshes: {count['meshes']}\n"
                f"Rigid bodies: {count['rigid_bodies']}\n"
                f"Physics scenes: {count['physics_scenes']}\n"
                f"Playground: {result}\n"
                "Physics calibration: NOT QUALIFIED"
            )
        self._run(action)

    def _save_stage(self):
        def action():
            stage = self._stage()
            root = stage.GetRootLayer()
            if root.anonymous:
                raise RuntimeError("Anonymous layer. Use Kit File > Save As instead.")
            if not root.Save():
                raise RuntimeError("OpenUSD root layer Save() returned failure")
            self._status("USD layer saved: " + root.identifier)
        self._run(action)

    def _timeline(self, method):
        def action():
            # Kit can provide a timeline without the PhysX extension. We
            # warn explicitly rather than claiming physically correct motion.
            import omni.kit.app
            manager = omni.kit.app.get_app().get_extension_manager()
            physics_available = manager.is_extension_enabled("omni.physx")
            timeline = omni.timeline.get_timeline_interface()
            getattr(timeline, method)()
            self._status(
                f"Kit timeline: {method.upper()}\n"
                f"omni.physx enabled: {physics_available}\n"
                + ("Scene physics runtime not available; timeline does not "
                   "prove collision/robot dynamics." if not physics_available
                   else "Runtime present; actual dynamics still require acceptance tests.")
            )
        self._run(action)

    def _make_playground(self, reset):
        def action():
            stage = self._stage()
            result = create_playground(stage, PlaygroundSettings(), replace=reset)
            self._status(
                ("Playground RESET" if reset else "Playground CREATED")
                + "\nRobot: " + str(result["robot"]["model"])
                + "\nPayloads: " + str(len(result["payloads"]))
                + "\n" + result["simulation_engine"]
                + "\nUse Play only with an enabled USD Physics runtime."
            )
        self._run(action)

    def _spawn(self):
        def action():
            stage = self._stage()
            path = spawn_box(
                stage,
                mass_kg=self._data["mass"].get_value_as_float(),
                side_m=self._data["side"].get_value_as_float(),
                position_m=tuple(self._data[c].get_value_as_float()
                                 for c in ("x", "y", "z")),
            )
            self._status(
                f"Created real USD rigid body: {path}\n"
                "Sandbox cube is explicitly primitive, not OEM equipment."
            )
        self._run(action)

    def _apply_joints(self):
        def action():
            stage = self._stage()
            degrees = [v.get_value_as_float() for v in self._joints]
            targets = set_reference_joint_targets(
                stage, [math.radians(q) for q in degrees]
            )
            pose = forward_kinematics([math.radians(q) for q in degrees])
            self._status(
                "Authored six USD angular motor targets (deg):\n"
                + ", ".join(f"{x:.2f}" for x in targets)
                + "\nAnalytic FK tip XYZ (m): "
                + ", ".join(f"{x:.4f}" for x in pose.end_effector_m)
                + "\nActual position will differ during acceleration/contacts."
            )
        self._run(action)

    def _home(self):
        for model in self._joints:
            model.set_value(0.0)
        self._apply_joints()

    def _solve_ik(self):
        def action():
            target = [self._data[f"target_{c}"].get_value_as_float()
                      for c in ("x", "y", "z")]
            seed = [math.radians(j.get_value_as_float()) for j in self._joints]
            solution = inverse_kinematics(target, seed_rad=seed)
            if not solution.converged:
                self._status(
                    f"IK did not converge. Residual = {solution.position_error_m:.5f} m "
                    f"after {solution.iterations} iterations. No joint drive was applied."
                )
                return
            stage = self._stage()
            set_reference_joint_targets(stage, solution.angles_rad)
            for j, angle in zip(self._joints, solution.angles_rad):
                j.set_value(math.degrees(angle))
            self._status(
                f"IK CONVERGED in {solution.iterations} iterations.\n"
                f"Position residual: {solution.position_error_m:.6f} m.\n"
                "Six USD joint-drive targets applied. Orientation and "
                "collision avoidance NOT solved."
            )
        self._run(action)

    def _gravity(self):
        def action():
            stage = self._stage()
            magnitude = self._data["gravity"].get_value_as_float()
            if not math.isfinite(magnitude) or not 0.0 <= magnitude <= 30.0:
                raise ValueError("Gravity magnitude must be between 0 and 30 m/s²")
            scene = UsdPhysics.Scene.Get(stage, "/World/Physics")
            if not scene:
                raise RuntimeError("No /World/Physics schema; create a DSIQ scene")
            scene.GetGravityMagnitudeAttr().Set(float(magnitude))
            self._status(f"Authored USD gravity magnitude {magnitude:.3f} m/s²")
        self._run(action)

    def on_shutdown(self):
        if getattr(self, "_window", None):
            self._window.visible = False
        self._window = None
        self._status_label = None
        self._data = {}
        self._joints = []
