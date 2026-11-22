"""DSIQ Native CPU Robotics Studio — genuine standalone desktop GUI.

This is an independent, lightweight native Tk desktop application, not a
renderer embedded in NVIDIA Kit. It shows LIVE Bullet TinyRenderer imagery and
controls a LIVE articulated Bullet physics model. PURE BLACK (#000000) and
LIGHT (#FFFFFF) are actual selectable visual themes.

Every visible button is functional. No stock image, screenshot or simulated
'live' data is presented. OEM CAD, RTX and PhysX require separate qualification.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Literal

from .robotics import JOINT_LIMITS_DEG, inverse_kinematics, forward_kinematics
from .bullet_runtime import BulletPlayground, BulletSettings
from .sensors import BulletSensors, CameraFrame


PALETTES = {
    "black": {
        "background": "#000000",
        "panel": "#10151E",
        "input": "#1B2533",
        "text": "#F8FAFC",
        "muted": "#BBC8D9",
        "accent": "#8DC4FF",
        "accent_text": "#06111D",
        "border": "#354458",
        "status": "#17212F",
        "button": "#23344B",
    },
    "light": {
        "background": "#FFFFFF",
        "panel": "#F2F5F9",
        "input": "#FFFFFF",
        "text": "#101827",
        "muted": "#43516A",
        "accent": "#14589C",
        "accent_text": "#FFFFFF",
        "border": "#C1CBD9",
        "status": "#E7EFF7",
        "button": "#E0E8F2",
    },
}


@dataclass(frozen=True)
class StudioConfig:
    theme: Literal["black", "light"] = "black"
    boxes: int = 4
    camera_width: int = 480
    camera_height: int = 360
    fps_limit: int = 12

    def validate(self) -> None:
        if self.theme not in PALETTES:
            raise ValueError("Theme must be black or light")
        if type(self.boxes) is not int or not 0 <= self.boxes <= 40:
            raise ValueError("boxes must be an integer in [0,40]")
        if type(self.camera_width) is not int or type(self.camera_height) is not int:
            raise ValueError("camera dimensions must be integers")
        if (not 128 <= self.camera_width <= 640
                or not 96 <= self.camera_height <= 480
                or self.camera_width * self.camera_height > 200_000):
            raise ValueError("Camera resolution must fit <=200k pixels and minimum width/height")
        if type(self.fps_limit) is not int or not 1 <= self.fps_limit <= 20:
            raise ValueError("fps_limit must be in [1,20]")


def orbit_eye(
    yaw_rad: float,
    pitch_rad: float,
    distance_m: float,
    center_m=(0.0, 0.0, 0.65),
) -> tuple[float, float, float]:
    """Bounded, deterministic orbit camera; Z-up scene."""
    values = (yaw_rad, pitch_rad, distance_m, *center_m)
    if not all(isinstance(v, (float, int)) and math.isfinite(v) for v in values):
        raise ValueError("Orbit camera requires finite parameters")
    if not 1.0 <= distance_m <= 20.0:
        raise ValueError("Orbit distance must be in [1,20] meters")
    if not -1.45 <= pitch_rad <= 1.45:
        raise ValueError("Orbit pitch must be within +/-1.45 radians")
    return (
        center_m[0] + distance_m * math.cos(pitch_rad) * math.cos(yaw_rad),
        center_m[1] + distance_m * math.cos(pitch_rad) * math.sin(yaw_rad),
        center_m[2] + distance_m * math.sin(pitch_rad),
    )


class DesktopStudio:
    def __init__(self, config: StudioConfig):
        config.validate()
        try:
            import tkinter as tk
            from tkinter import filedialog, messagebox
        except ImportError as exc:
            raise RuntimeError(
                "Tkinter unavailable. Install the standard Python 3.11 Windows distribution "
                "with Tcl/Tk, or run the CPU physics CLI in headless mode."
            ) from exc
        self.tk = tk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.config = config
        self.theme = config.theme
        self.palette = PALETTES[self.theme]
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(
                "Native desktop display unavailable. Use bullet-run in headless mode."
            ) from exc
        self.root.title("DSIQ NATIVE  |  Robotics Engineering Studio")
        self.root.geometry("1410x870")
        self.root.minsize(1070, 670)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.world = BulletPlayground(BulletSettings(payload_count=config.boxes))
        self.sensors = BulletSensors(self.world)
        self.running = False
        self._closed = False
        self._tick_id = None
        self._latest: CameraFrame | None = None
        self._image = None
        self._orbit_yaw = -0.8
        self._orbit_pitch = 0.35
        self._orbit_distance = 2.9
        self._orbit_center = (0.1, 0.0, 0.55)
        self._drag = None
        self._last_tick = time.perf_counter()
        self._status_message = "Ready. Physics is genuine Bullet CPU, not calibrated OEM dynamics."
        self._mass = tk.StringVar(value="1.0")
        self._side = tk.StringVar(value="0.16")
        self._x = tk.StringVar(value="-0.7")
        self._y = tk.StringVar(value="1.0")
        self._z = tk.StringVar(value="1.1")
        self._target_xyz = [
            tk.StringVar(value=val) for val in ("0.7", "0.2", "0.7")
        ]
        self._joint_vars = [tk.DoubleVar(value=0.) for _ in range(6)]
        self._build()
        self._tick()

    def _button(self, parent, label, action, *, accent=False):
        palette = self.palette
        return self.tk.Button(
            parent, text=label, command=lambda: self._safe(action),
            background=palette["accent"] if accent else palette["button"],
            foreground=palette["accent_text"] if accent else palette["text"],
            activebackground=palette["accent"],
            activeforeground=palette["accent_text"],
            relief="flat", bd=0, padx=12, pady=9,
            font=("Segoe UI", 10, "bold" if accent else "normal"),
            cursor="hand2",
        )

    def _label(self, parent, text, *, size=10, bold=False, muted=False, **kwargs):
        return self.tk.Label(
            parent, text=text, background=self.palette["panel"],
            foreground=self.palette["muted" if muted else "text"],
            font=("Segoe UI", size, "bold" if bold else "normal"),
            justify="left", anchor="w", **kwargs,
        )

    def _field(self, parent, title, variable):
        host = self.tk.Frame(parent, background=self.palette["panel"])
        host.pack(fill="x", pady=(0, 6))
        self._label(host, title, size=9, muted=True).pack(anchor="w")
        field = self.tk.Entry(
            host, textvariable=variable, font=("Consolas", 11),
            background=self.palette["input"], foreground=self.palette["text"],
            insertbackground=self.palette["text"], relief="flat", bd=0,
            highlightbackground=self.palette["border"], highlightthickness=1,
        )
        field.pack(fill="x", ipady=5)
        return field

    def _section(self, parent, title):
        self._label(parent, title, size=10, bold=True).pack(
            anchor="w", pady=(13, 7)
        )

    def _build(self):
        tk = self.tk
        p = self.palette
        for child in self.root.winfo_children():
            child.destroy()
        self.root.configure(background=p["background"])
        self.root.option_add("*Font", "Segoe UI 10")
        self._menu()
        header = tk.Frame(self.root, background=p["background"], height=63)
        header.pack(fill="x", padx=22, pady=(12, 0))
        tk.Label(
            header, text="DSIQ  /  NATIVE", background=p["background"],
            foreground=p["text"], font=("Segoe UI", 19, "bold")
        ).pack(side="left")
        tk.Label(
            header, text="ROBOTICS   •   OPENUSD   •   PHYSICS LAB",
            background=p["background"], foreground=p["muted"],
            font=("Segoe UI", 10),
        ).pack(side="left", padx=(24, 0))
        theme_label = self._button(
            header, "☀ Light" if self.theme == "black" else "● Pure black",
            lambda: self.set_theme("light" if self.theme == "black" else "black"),
        )
        theme_label.pack(side="right", padx=4)

        body = tk.Frame(self.root, background=p["background"])
        body.pack(fill="both", expand=True, padx=18, pady=(7,12))
        left = tk.Frame(body, background=p["panel"], width=270, padx=16, pady=14)
        left.pack(side="left", fill="y", padx=(0,9))
        left.pack_propagate(False)
        middle = tk.Frame(body, background=p["panel"], padx=12, pady=12)
        middle.pack(side="left", fill="both", expand=True)
        right = tk.Frame(body, background=p["panel"], width=292, padx=16, pady=14)
        right.pack(side="right", fill="y", padx=(9,0))
        right.pack_propagate(False)

        self._section(left, "SCENE / PLAYGROUND")
        self._button(left, "Reset reference workcell", self.reset).pack(fill="x", pady=3)
        self._button(left, "Export reference OpenUSD", self.export_usd).pack(fill="x", pady=3)
        self._section(left, "SIMULATION")
        playback = tk.Frame(left, background=p["panel"])
        playback.pack(fill="x")
        self._button(playback, "▶ Play", self.play, accent=True).pack(
            side="left", expand=True, fill="x", padx=(0, 3)
        )
        self._button(playback, "Ⅱ Pause", self.pause).pack(
            side="left", expand=True, fill="x", padx=3
        )
        self._button(playback, "→ Step", self.step_once).pack(
            side="left", expand=True, fill="x", padx=(3,0)
        )
        self._section(left, "ADD A DYNAMIC PAYLOAD")
        for title, v in (
            ("Mass (kg)", self._mass), ("Box side length (m)", self._side),
            ("X (m)", self._x), ("Y (m)", self._y), ("Z (m)", self._z),
        ):
            self._field(left, title, v)
        self._button(left, "Add physical payload", self.add_box, accent=True).pack(
            fill="x", pady=(6,8)
        )
        self._section(left, "SENSORS")
        self._button(left, "Scan 72-ray LiDAR", self.lidar_scan).pack(fill="x", pady=3)
        self._button(left, "Save RGB screenshot", self.save_camera_png).pack(fill="x", pady=3)
        self._button(left, "Export state JSON", self.export_state).pack(fill="x", pady=3)
        self._label(
            left,
            "NO OEM CLAIMS\n\nLive Bullet rigid bodies and motor physics; "
            "sandbox shapes and reference robot only. "
            "Calibration / sensor realism require independent validation.",
            muted=True, wraplength=225
        ).pack(fill="x", side="bottom", pady=(0,8))

        toolbar = tk.Frame(middle, background=p["panel"])
        toolbar.pack(fill="x", pady=(0,10))
        self._label(
            toolbar, "LIVE SOFTWARE-RENDERED 3D VIEW",
            size=10, bold=True
        ).pack(side="left")
        self._button(toolbar, "Frame all", self.frame_all).pack(side="right")
        self._viewport = tk.Label(
            middle, background="#101720", foreground=p["text"],
            text="Initializing live Bullet camera...",
            font=("Consolas", 12), compound="center",
        )
        self._viewport.pack(fill="both", expand=True)
        self._viewport.bind("<ButtonPress-1>", self._mouse_down)
        self._viewport.bind("<B1-Motion>", self._mouse_move)
        self._viewport.bind("<MouseWheel>", self._mouse_scroll)
        self._viewport.bind("<Button-4>", lambda e: self._zoom(0.88))
        self._viewport.bind("<Button-5>", lambda e: self._zoom(1.12))
        self._label(
            middle, "Drag to orbit  ·  Wheel to zoom  ·  Z-up metric scene",
            muted=True, size=9
        ).pack(fill="x", pady=(9,4))
        self._metrics = self._label(
            middle, "Starting solver...",
            muted=True, size=9, wraplength=600,
        )
        self._metrics.pack(fill="x")
        self._status_display = self._label(
            middle, self._status_message,
            size=10, wraplength=640, height=3,
        )
        self._status_display.pack(fill="x", pady=(10,0))

        self._section(right, "REFERENCE SIX-AXIS ROBOT")
        self._label(
            right, "Original 6R kinematics and motor controls. "
            "Simplified reference geometry; not an OEM machine.",
            muted=True, wraplength=255,
        ).pack(fill="x", pady=(0,10))
        for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG):
            panel = tk.Frame(right, background=p["panel"])
            panel.pack(fill="x", pady=(0,6))
            self._label(
                panel, f"J{i+1}   [{lo:.0f}°, {hi:.0f}°]",
                muted=True, size=9,
            ).pack(fill="x")
            scale = tk.Scale(
                panel, from_=lo, to=hi, orient="horizontal",
                variable=self._joint_vars[i], resolution=0.5,
                showvalue=True, length=238,
                background=p["panel"], foreground=p["text"],
                troughcolor=p["border"], activebackground=p["accent"],
                highlightthickness=0, bd=0,
                font=("Segoe UI", 8),
            )
            scale.pack(fill="x")
        row = tk.Frame(right, background=p["panel"])
        row.pack(fill="x", pady=(4,12))
        self._button(row, "Apply joints", self.apply_joints, accent=True).pack(
            side="left", fill="x", expand=True, padx=(0,4)
        )
        self._button(row, "Home", self.home).pack(
            side="left", fill="x", expand=True, padx=(4,0)
        )
        self._section(right, "CARTESIAN TARGET / IK")
        for label, value in zip(("X target (m)","Y target (m)","Z target (m)"),
                                self._target_xyz):
            self._field(right, label, value)
        self._button(right, "Solve XYZ and apply motors", self.solve_ik).pack(
            fill="x", pady=(5,10)
        )
        self._label(
            right, "Position-only damped IK: no end-effector orientation "
            "or collision-aware planning yet.",
            muted=True, wraplength=260,
        ).pack(fill="x")

    def _menu(self):
        tk = self.tk
        p = self.palette
        menu = tk.Menu(
            self.root, tearoff=False, background=p["panel"],
            foreground=p["text"], activebackground=p["accent"],
            activeforeground=p["accent_text"],
        )
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="Reset reference workcell", command=lambda: self._safe(self.reset))
        file_menu.add_command(label="Export OpenUSD reference scene", command=lambda: self._safe(self.export_usd))
        file_menu.add_command(label="Export physics JSON", command=lambda: self._safe(self.export_state))
        file_menu.add_command(label="Save current RGB image", command=lambda: self._safe(self.save_camera_png))
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.close)
        menu.add_cascade(label="File", menu=file_menu)
        view_menu = tk.Menu(menu, tearoff=False)
        view_menu.add_command(label="Pure black (#000000)", command=lambda: self.set_theme("black"))
        view_menu.add_command(label="Light (#FFFFFF)", command=lambda: self.set_theme("light"))
        view_menu.add_command(label="Frame all", command=lambda: self._safe(self.frame_all))
        menu.add_cascade(label="View", menu=view_menu)
        sim_menu = tk.Menu(menu, tearoff=False)
        sim_menu.add_command(label="Play", command=lambda: self._safe(self.play))
        sim_menu.add_command(label="Pause", command=lambda: self._safe(self.pause))
        sim_menu.add_command(label="Step", command=lambda: self._safe(self.step_once))
        sim_menu.add_command(label="Reset", command=lambda: self._safe(self.reset))
        menu.add_cascade(label="Simulation", menu=sim_menu)
        self.root.config(menu=menu)

    def set_theme(self, theme):
        if theme not in PALETTES:
            raise ValueError("Invalid theme")
        self.theme = theme
        self.palette = PALETTES[theme]
        self._build()
        self._status(f"{theme.upper()} theme active. Viewport retains actual sensor imagery.")

    def _status(self, text):
        self._status_message = str(text)
        if getattr(self, "_status_display", None):
            self._status_display.config(text=self._status_message)

    def _safe(self, action):
        try:
            action()
        except Exception as exc:
            self._status(f"ACTION FAILED: {exc}")

    def play(self):
        self.running = True
        self._status("LIVE Bullet physics running, fixed timestep 1/240 seconds.")

    def pause(self):
        self.running = False
        self._status("Bullet physics paused. Camera and scene controls remain active.")

    def step_once(self):
        self.running = False
        self.world.step(1)
        self._status("Advanced exactly one 1/240-second Bullet physics step.")

    def reset(self):
        self.running = False
        self.world.close()
        self.world = BulletPlayground(BulletSettings(payload_count=self.config.boxes))
        self.sensors = BulletSensors(self.world)
        for joint in self._joint_vars:
            joint.set(0.0)
        self._status("New local Bullet scene created; previous sandbox state discarded.")

    def add_box(self):
        body = self.world.spawn_box(
            mass_kg=float(self._mass.get()),
            side_m=float(self._side.get()),
            position_m=(float(self._x.get()),float(self._y.get()),float(self._z.get())),
        )
        self._status(
            f"Created Bullet dynamic payload body #{body}; collisions, "
            f"mass and gravity are handled by the physics solver."
        )

    def apply_joints(self):
        values = tuple(math.radians(x.get()) for x in self._joint_vars)
        self.world.command_joints(values)
        target = forward_kinematics(values).end_effector_m
        self._status(
            "Motor targets issued. Analytic target tool position (m): "
            + ", ".join(f"{x:.3f}" for x in target)
            + ". Real state will be read from Bullet feedback."
        )

    def home(self):
        for model in self._joint_vars:
            model.set(0.0)
        self.apply_joints()

    def solve_ik(self):
        xyz = tuple(float(x.get()) for x in self._target_xyz)
        seed = self.world.report()["joint_radians"]
        result = inverse_kinematics(xyz, seed_rad=seed, max_iterations=350)
        if not result.converged:
            self._status(
                f"IK FAILED (unreachable/singular/local optimum). "
                f"Residual: {result.position_error_m:.4f} m; motors NOT updated."
            )
            return
        for var, angle in zip(self._joint_vars, result.angles_rad):
            var.set(math.degrees(angle))
        self.world.command_joints(result.angles_rad)
        self._status(
            f"IK CONVERGED. Position residual {result.position_error_m:.5f} m, "
            f"iterations={result.iterations}. Motor commands issued."
        )

    def lidar_scan(self):
        scan = self.sensors.lidar_2d(
            origin_m=(-1.5,0.0,0.7), count=72, max_range_m=5.0,
        )
        hits = [m for m in scan["measurements"] if m["hit"]]
        if hits:
            nearest = min(hits, key=lambda m: m["range_m"])
            self._status(
                f"ACTUAL Bullet LiDAR: {len(hits)}/{len(scan['measurements'])} "
                f"rays hit, nearest={nearest['range_m']:.3f}m, "
                f"body ID={nearest['body_id']}. Ideal geometric sensor, not calibrated."
            )
        else:
            self._status("LiDAR scan: no geometric hits within 5m.")

    def export_state(self):
        path = self.filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")], title="Export actual Bullet physics state",
        )
        if not path:
            return
        result = self.world.report()
        target = Path(path)
        if target.exists():
            raise FileExistsError("Refusing to overwrite existing physics report")
        target.write_text(json.dumps(result, indent=2), encoding="utf-8")
        self._status("Saved real physics report to " + str(target))

    def export_usd(self):
        path = self.filedialog.asksaveasfilename(
            defaultextension=".usda",
            filetypes=[("OpenUSD ASCII", "*.usda")],
            title="Export reference USD schemas (not Bullet dynamic state)",
        )
        if not path:
            return
        from .usd_stage import new_stage, _load
        from .playground import create_playground, PlaygroundSettings, set_reference_joint_targets
        destination = Path(path)
        if destination.exists():
            raise FileExistsError("Refusing to overwrite existing OpenUSD scene")
        new_stage(destination)
        stage = _load(destination)
        create_playground(stage, PlaygroundSettings(
            payload_count=len(self.world.payloads),
            payload_mass_kg=float(self._mass.get()),
        ))
        set_reference_joint_targets(stage, self.world.report()["target_radians"])
        stage.GetRootLayer().Save()
        self._status(
            "Saved real OpenUSD reference scene to " + str(destination)
            + ". CAUTION: contains reference USD physics schemas and motor targets, "
            "not measured Bullet dynamic transforms or real OEM assets."
        )

    def save_camera_png(self):
        if self._latest is None:
            raise RuntimeError("No camera frame has been rendered yet")
        path = self.filedialog.asksaveasfilename(
            defaultextension=".png", filetypes=[("PNG image", "*.png")],
            title="Save current actual Bullet TinyRenderer image",
        )
        if path:
            self._latest.save_png(path)
            self._status("Saved actual CPU Bullet camera frame: " + path)

    def frame_all(self):
        self._orbit_center = (0.,0.,.55)
        self._orbit_yaw = -.8
        self._orbit_pitch = .35
        self._orbit_distance = 3.0

    def _mouse_down(self, event):
        self._drag = (event.x, event.y)

    def _mouse_move(self, event):
        if self._drag is None:
            return
        dx, dy = event.x-self._drag[0], event.y-self._drag[1]
        self._drag = (event.x,event.y)
        self._orbit_yaw -= .009*dx
        self._orbit_pitch = max(-1.35,min(1.35,self._orbit_pitch+.009*dy))

    def _zoom(self, factor):
        self._orbit_distance = max(1.2,min(12.,self._orbit_distance*factor))

    def _mouse_scroll(self, event):
        self._zoom(.88 if event.delta>0 else 1.12)

    def _render(self):
        eye = orbit_eye(
            self._orbit_yaw, self._orbit_pitch,
            self._orbit_distance, self._orbit_center,
        )
        self._latest = self.sensors.camera_rgbd(
            eye_m=eye, look_at_m=self._orbit_center,
            width=self.config.camera_width, height=self.config.camera_height,
        )
        data = base64.b64encode(self._latest.to_png_bytes()).decode("ascii")
        self._image = self.tk.PhotoImage(data=data)
        self._viewport.configure(image=self._image, text="")
        result = self.world.report()
        self._metrics.configure(
            text=(
                f"ENGINE  Bullet CPU     "
                f"SIM {result['elapsed_sim_seconds']:.3f}s     "
                f"BODIES {len(result['payload_positions_m'])+6}     "
                f"CONTACTS {result['num_contacts']}     "
                f"FK Δ {result['bullet_analytic_position_difference_m']*1000:.2f} mm\n"
                "RENDERER  CPU TinyRenderer   |   NOT RTX   |   "
                "MODEL  Reference6R, not OEM qualified"
            ),
        )

    def _tick(self):
        if self._closed:
            return
        try:
            now = time.perf_counter()
            elapsed = now - self._last_tick
            self._last_tick = now
            if self.running:
                # Bounded catch-up prevents overwhelming the UI if CPU
                # software rendering slows. Simulation time is always
                # (steps / 240), and is not misreported as real wall time.
                n = min(24, max(1, int(elapsed*240)))
                self.world.step(n)
            self._render()
        except Exception as exc:
            self.running = False
            self._status("RENDER/SIM ERROR; simulation paused: " + str(exc))
        self._tick_id = self.root.after(
            max(50, int(1000/self.config.fps_limit)), self._tick
        )

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.close()

    def close(self):
        if getattr(self, "_closed", True):
            return
        self._closed = True
        if self._tick_id:
            try:
                self.root.after_cancel(self._tick_id)
            except Exception:
                pass
        self.world.close()
        try:
            self.root.destroy()
        except Exception:
            pass


def launch_studio(
    theme: str = "black", boxes: int = 4,
    camera_width: int = 480, camera_height: int = 360,
    fps_limit: int = 12,
):
    config = StudioConfig(
        theme=theme, boxes=boxes,
        camera_width=camera_width, camera_height=camera_height,
        fps_limit=fps_limit,
    )
    config.validate()
    app = DesktopStudio(config)
    app.run()
    return {
        "launched": True, "closed": True,
        "viewer": "TK_DESKTOP_BULLET_CPU_SOFTWARE_RENDERER",
        "note": "GPU-accelerated RTX/Kit is separate and requires official SDK/hardware qualification",
    }
