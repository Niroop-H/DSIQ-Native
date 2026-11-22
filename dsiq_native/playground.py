"""A real, inspectable OpenUSD rigid-body & 6-axis robotics playground.

The payloads are explicitly primitive *user sandbox objects*. The robot is a
physically parameterized, independently authored **reference manipulator**;
its simplified collision geometry is not manufacturer CAD and its dynamics
are not calibration-qualified. Kit/PhysX executes USD Physics when available.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .robotics import (
    MODEL_ID, HOME_OFFSETS_M, JOINT_AXES, JOINT_LIMITS_DEG,
    LINK_MASSES_KG, check_angles,
)
from .usd_stage import _pxr

PLAYGROUND_PATH = "/World/Playground"
ROBOT_PATH = PLAYGROUND_PATH + "/Reference6R"
COLLIDER_PATH = PLAYGROUND_PATH + "/Ground"
MAX_BOXES = 40


@dataclass(frozen=True)
class PlaygroundSettings:
    gravity_m_s2: float = 9.81
    payload_count: int = 5
    payload_mass_kg: float = 1.0
    payload_side_m: float = 0.16
    ground_width_m: float = 8.0
    include_reference_robot: bool = True

    def validate(self) -> None:
        for name, value, minimum, maximum in (
            ("gravity_m_s2", self.gravity_m_s2, 0.0, 30.0),
            ("payload_mass_kg", self.payload_mass_kg, 0.05, 100.0),
            ("payload_side_m", self.payload_side_m, 0.05, 0.6),
            ("ground_width_m", self.ground_width_m, 2.0, 20.0),
        ):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be numeric")
            if not math.isfinite(value) or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
        if type(self.payload_count) is not int or not 0 <= self.payload_count <= MAX_BOXES:
            raise ValueError(f"payload_count must be integer in [0,{MAX_BOXES}]")
        if type(self.include_reference_robot) is not bool:
            raise ValueError("include_reference_robot must be boolean")


def _xyz(v):
    from pxr import Gf
    return Gf.Vec3d(*[float(x) for x in v])


def _v3(v):
    from pxr import Gf
    return Gf.Vec3f(*[float(x) for x in v])


def _translate(xformable, xyz):
    xformable.AddTranslateOp().Set(_xyz(xyz))


def _scale(xformable, xyz):
    xformable.AddScaleOp().Set(_v3(xyz))


def _color(prim, rgb):
    prim.CreateDisplayColorAttr([_v3(rgb)])


def _cube(stage, path, *, center, dimensions, rgb, collide=False):
    _, _, UsdGeom, UsdPhysics = _pxr()
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    _translate(cube, center)
    _scale(cube, dimensions)
    _color(cube, rgb)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _validate_canonical_stage(stage) -> None:
    _, _, UsdGeom, _ = _pxr()
    if stage is None:
        raise ValueError("OpenUSD stage required")
    if abs(float(UsdGeom.GetStageMetersPerUnit(stage)) - 1.0) > 1e-9:
        raise ValueError("Playground requires metersPerUnit == 1")
    if str(UsdGeom.GetStageUpAxis(stage)) != "Z":
        raise ValueError("Playground requires Z-up")
    if not stage.GetPrimAtPath("/World").IsValid():
        raise ValueError("Expected /World root. Create a DSIQ stage first")


def _rigid_box(stage, path, *, center, side, mass, rgb):
    _, _, UsdGeom, UsdPhysics = _pxr()
    box = _cube(stage, path, center=center, dimensions=(side,side,side),
                rgb=rgb, collide=True)
    body = UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    body.CreateRigidBodyEnabledAttr(True)
    inertia = mass * side * side / 6.0
    m = UsdPhysics.MassAPI.Apply(box.GetPrim())
    m.CreateMassAttr(float(mass))
    m.CreateDiagonalInertiaAttr(_v3((inertia, inertia, inertia)))
    return box


def _reference_arm(stage) -> dict:
    _, _, UsdGeom, UsdPhysics = _pxr()
    robot = UsdGeom.Xform.Define(stage, ROBOT_PATH)
    robot.GetPrim().SetCustomDataByKey("dsiq_model_id", MODEL_ID)
    robot.GetPrim().SetCustomDataByKey(
        "dsiq_fidelity",
        "PARAMETRIC_REFERENCE_NOT_OEM_CAD_NOT_PHYSICALLY_CALIBRATED",
    )
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    UsdGeom.Scope.Define(stage, ROBOT_PATH + "/Links")
    UsdGeom.Scope.Define(stage, ROBOT_PATH + "/Joints")

    # Geometric home pivots in world coordinates. Every rigid body lives in
    # a sibling Xform, never nested under a second rigid body.
    p = (0.0, 0.0, 0.0)
    joints: list[str] = []
    link_paths: list[str] = []
    total_mass = 0.0
    for i, (off, mass, axis) in enumerate(
        zip(HOME_OFFSETS_M, LINK_MASSES_KG, JOINT_AXES)
    ):
        p = tuple(p[j] + off[j] for j in range(3))
        link_path = ROBOT_PATH + f"/Links/Link_{i+1}"
        link = UsdGeom.Xform.Define(stage, link_path)
        _translate(link, p)
        body = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        body.CreateRigidBodyEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(link.GetPrim())
        m.CreateMassAttr(float(mass))
        total_mass += mass
        link.GetPrim().SetCustomDataByKey("dsiq_function", (
            "base_azimuth", "shoulder", "elbow", "wrist_roll",
            "wrist_pitch", "wrist_tool_roll"
        )[i])
        # This is explicit simplified collision *for a reference robot*:
        # a link body with one box collision core and a visual joint housing.
        # It is not a substitute for an OEM collision mesh.
        if i == 0:
            dims = (0.22, 0.22, 0.18)
            center = (0.0, 0.0, 0.09)
        else:
            reach = (0.42, 0.38, 0.16, 0.12, 0.12)[i-1]
            thickness = (0.20, 0.17, 0.13, 0.11, 0.10)[i-1]
            dims = (reach, thickness, thickness)
            center = (reach / 2, 0.0, 0.0)
        core = _cube(
            stage, link_path + "/StructuralCore",
            center=center, dimensions=dims,
            rgb=(0.12, 0.30 + 0.035*i, 0.60),
            collide=True,
        )
        core.GetPrim().SetCustomDataByKey(
            "dsiq_collision_model", "UNVERIFIED_REFERENCE_BOX"
        )
        # Distinct joint housing: visual only, because the simplified reference
        # collision core is deliberately kept separate.
        housing = UsdGeom.Sphere.Define(stage, link_path + "/JointHousing")
        housing.CreateRadiusAttr((0.125, 0.118, 0.107, 0.095, 0.08, 0.07)[i])
        _color(housing, (0.14, 0.16, 0.21))
        flange = UsdGeom.Cylinder.Define(stage, link_path + "/MotorCover")
        flange.CreateRadiusAttr((0.10, 0.095, 0.09, 0.075, 0.065, 0.055)[i])
        flange.CreateHeightAttr(0.07)
        _color(flange, (0.62, 0.69, 0.76))

        # Box approximation diagonal inertia for the specified mass
        # and core dimensions. Used for reference mechanics only.
        dx, dy, dz = dims
        inertia = (
            mass * (dy*dy + dz*dz) / 12,
            mass * (dx*dx + dz*dz) / 12,
            mass * (dx*dx + dy*dy) / 12,
        )
        m.CreateDiagonalInertiaAttr(_v3(inertia))
        m.CreateCenterOfMassAttr(_v3(center))

        joint_path = ROBOT_PATH + f"/Joints/Joint_{i+1}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        joint.CreateAxisAttr(("X" if axis[0] else "Y" if axis[1] else "Z"))
        if i > 0:
            joint.CreateBody0Rel().SetTargets([link_paths[-1]])
            joint.CreateLocalPos0Attr(_v3(off))
        else:
            # World anchored: body0 relation omitted, world-frame pivot.
            joint.CreateLocalPos0Attr(_v3(p))
        joint.CreateBody1Rel().SetTargets([link_path])
        joint.CreateLocalPos1Attr(_v3((0,0,0)))
        lo, hi = JOINT_LIMITS_DEG[i]
        joint.CreateLowerLimitAttr(float(lo))
        joint.CreateUpperLimitAttr(float(hi))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(150.0)
        drive.CreateDampingAttr(25.0)
        drive.CreateMaxForceAttr(450.0)
        joints.append(joint_path)
        link_paths.append(link_path)

    return {
        "model": MODEL_ID,
        "joints": joints,
        "links": link_paths,
        "sum_reference_link_mass_kg": sum(LINK_MASSES_KG),
        "fidelity": "parametric_reference",
        "warning": "This is not a calibrated industrial robot. Collision cores are simplified.",
    }


def spawn_box(
    stage, *, mass_kg: float = 1.0, side_m: float = 0.16,
    position_m: Sequence[float] = (0.0, 1.0, 1.0),
) -> str:
    """Spawn one physically parameterized sandbox box (not OEM equipment)."""
    _validate_canonical_stage(stage)
    values = tuple(float(x) for x in position_m)
    if len(values) != 3 or any(not math.isfinite(x) or abs(x)>20 for x in values):
        raise ValueError("Box position must be 3 finite numbers within +/-20 meters")
    PlaygroundSettings(payload_mass_kg=mass_kg, payload_side_m=side_m).validate()
    if not stage.GetPrimAtPath(PLAYGROUND_PATH).IsValid():
        raise ValueError("Create the playground before spawning a body")
    prefix = PLAYGROUND_PATH + "/Payloads"
    if not stage.GetPrimAtPath(prefix).IsValid():
        _, _, UsdGeom, _ = _pxr()
        UsdGeom.Scope.Define(stage, prefix)
    for i in range(MAX_BOXES):
        path = prefix + f"/Box_{i+1:03d}"
        if not stage.GetPrimAtPath(path).IsValid():
            _rigid_box(
                stage, path, center=values, side=side_m, mass=mass_kg,
                rgb=(0.86, 0.48, 0.15),
            )
            return path
    raise ValueError(f"Playground hard limit reached: {MAX_BOXES} active boxes")


def create_playground(stage, settings: PlaygroundSettings | None = None, *, replace: bool = False) -> dict:
    """Build one reproducible reference-robot & rigid payload scene in real USD."""
    _, _, UsdGeom, UsdPhysics = _pxr()
    _validate_canonical_stage(stage)
    spec = settings or PlaygroundSettings()
    spec.validate()
    if stage.GetPrimAtPath(PLAYGROUND_PATH).IsValid():
        if not replace:
            raise FileExistsError("Playground already exists; use explicit replace to reset it")
        stage.RemovePrim(PLAYGROUND_PATH)

    root = UsdGeom.Xform.Define(stage, PLAYGROUND_PATH)
    root.GetPrim().SetCustomDataByKey(
        "dsiq_scene_scope", "REPRODUCIBLE_ROBOTICS_SANDBOX_NOT_FACTORY_QUALIFIED"
    )
    root.GetPrim().SetCustomDataByKey("dsiq_playground_version", "v1")
    scene = UsdPhysics.Scene.Get(stage, "/World/Physics")
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, "/World/Physics")
    scene.CreateGravityDirectionAttr(_v3((0., 0., -1.)))
    scene.CreateGravityMagnitudeAttr(float(spec.gravity_m_s2))

    # Ground top lies at z=0. Cube uses unit dimension before scaling.
    _cube(
        stage, COLLIDER_PATH,
        center=(0.,0.,-0.10), dimensions=(spec.ground_width_m, spec.ground_width_m, 0.20),
        rgb=(0.18,0.22,0.25), collide=True,
    )
    UsdGeom.Scope.Define(stage, PLAYGROUND_PATH + "/Payloads")
    payloads: list[str] = []
    for i in range(spec.payload_count):
        # Deliberate separation across a row; no seeded randomness needed.
        x = -1.40 + (i % 5) * 0.34
        y = 1.05 + (i // 5) * 0.34
        z = 0.85 + (i // 10) * 0.22
        payloads.append(spawn_box(
            stage, mass_kg=spec.payload_mass_kg,
            side_m=spec.payload_side_m, position_m=(x,y,z)
        ))
    robot = _reference_arm(stage) if spec.include_reference_robot else None
    return {
        "playground_path": PLAYGROUND_PATH,
        "gravity_m_s2": spec.gravity_m_s2,
        "payloads": payloads,
        "robot": robot,
        "simulation_engine": "USD_PHYSICS_SCHEMA_ONLY_UNTIL_KIT_PHYSX_RUNS",
        "rendering": "HOST_DEPENDENT",
    }


def set_reference_joint_targets(stage, angles_rad: Sequence[float]) -> list[float]:
    """Write 6 real USD drive targets (degrees). Simulation must be playing in host."""
    _, _, _, UsdPhysics = _pxr()
    values = check_angles(angles_rad)
    joints = [stage.GetPrimAtPath(ROBOT_PATH + f"/Joints/Joint_{i+1}") for i in range(6)]
    if any(not prim.IsValid() for prim in joints):
        raise ValueError("Reference6R joints missing; create a playground with robot enabled")
    targets = [math.degrees(q) for q in values]
    for prim, angle in zip(joints, targets):
        drive = UsdPhysics.DriveAPI.Get(prim.GetStage(), prim.GetPath(), "angular")
        if not drive:
            raise RuntimeError(f"USD angular drive missing on {prim.GetPath()}")
        drive.GetTargetPositionAttr().Set(float(angle))
    return targets


def playground_report(stage) -> dict:
    _, Usd, UsdGeom, UsdPhysics = _pxr()
    _validate_canonical_stage(stage)
    root = stage.GetPrimAtPath(PLAYGROUND_PATH)
    if not root.IsValid():
        return {"exists": False}
    bodies = 0
    colliders = 0
    revolutes = 0
    for prim in Usd.PrimRange(root):  # noqa: F821 (Usd imported below)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders += 1
        if prim.IsA(UsdPhysics.RevoluteJoint):
            revolutes += 1
    return {
        "exists": True,
        "rigid_bodies": bodies,
        "colliders": colliders,
        "revolute_joints": revolutes,
        "is_robot_present": stage.GetPrimAtPath(ROBOT_PATH).IsValid(),
        "qualified_for_factory": False,
    }
