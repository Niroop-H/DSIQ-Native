"""Executable local Bullet rigid-body/reference-arm playground.

Uses the mature C++ Bullet solver via PyBullet. The GUI uses Bullet's own
native 3D window; headless DIRECT works without NVIDIA Kit or an RTX GPU.
Robotics geometry is an explicitly ORIGINAL training reference, not OEM CAD.
This engine is distinct from the OpenUSD/PhysX backend; parity is *not* assumed.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Iterable, Sequence

from .robotics import (
    HOME_OFFSETS_M, JOINT_AXES, JOINT_LIMITS_DEG,
    LINK_MASSES_KG, MODEL_ID, TOOL_OFFSET_M,
    check_angles, forward_kinematics,
)

PHYSICS_DT = 1.0 / 240.0
MAX_PAYLOADS = 40


def _bullet():
    try:
        import pybullet as p
    except ImportError as exc:
        raise RuntimeError(
            'PyBullet unavailable. Install optional CPU physics runtime: '
            'python -m pip install -e ".[physics]"'
        ) from exc
    return p


def _box_geometry_for_link(i: int):
    """Length/width/height and center for each *reference* mechanical housing."""
    if i == 0:
        return (0.20, 0.20, 0.22), (0.0, 0.0, 0.11)
    length = (0.42, 0.38, 0.16, 0.12, 0.12)[i-1]
    thickness = (0.18, 0.16, 0.13, 0.11, 0.10)[i-1]
    return (length, thickness, thickness), (length / 2, 0.0, 0.0)


@dataclass(frozen=True)
class BulletSettings:
    gui: bool = False
    gravity_m_s2: float = 9.81
    solver_iterations: int = 100
    payload_count: int = 4
    payload_mass_kg: float = 1.0

    def validate(self):
        if type(self.gui) is not bool:
            raise ValueError("gui must be bool")
        for name, number, lower, upper in (
            ("gravity_m_s2", self.gravity_m_s2, 0., 30.),
            ("payload_mass_kg", self.payload_mass_kg, .05, 100.),
        ):
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f"{name} must be numeric")
            if not math.isfinite(number) or not lower <= number <= upper:
                raise ValueError(f"{name} out of supported bounds [{lower},{upper}]")
        if type(self.solver_iterations) is not int or not 10 <= self.solver_iterations <= 500:
            raise ValueError("solver_iterations must be int in [10,500]")
        if type(self.payload_count) is not int or not 0 <= self.payload_count <= MAX_PAYLOADS:
            raise ValueError(f"payload_count must be int in [0,{MAX_PAYLOADS}]")


class BulletPlayground:
    """Owns exactly one isolated Bullet client, never relies on global client 0."""

    def __init__(self, settings: BulletSettings | None = None):
        self.settings = settings or BulletSettings()
        self.settings.validate()
        self.p = _bullet()
        mode = self.p.GUI if self.settings.gui else self.p.DIRECT
        self.client = self.p.connect(mode)
        if self.client < 0:
            raise RuntimeError(
                "Bullet GUI/DIRECT failed to connect; no physics simulation started"
            )
        self.step_index = 0
        self.robot = -1
        self.payloads: list[int] = []
        self.ground = -1
        self._targets = (0.,) * 6
        try:
            self._initialize()
        except Exception:
            self.close()
            raise

    def _initialize(self):
        p = self.p
        cid = self.client
        p.resetSimulation(physicsClientId=cid)
        p.setGravity(0., 0., -self.settings.gravity_m_s2, physicsClientId=cid)
        p.setRealTimeSimulation(0, physicsClientId=cid)
        p.setTimeStep(PHYSICS_DT, physicsClientId=cid)
        p.setPhysicsEngineParameter(
            numSolverIterations=self.settings.solver_iterations,
            deterministicOverlappingPairs=1,
            physicsClientId=cid,
        )
        ground_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[4., 4., .10], physicsClientId=cid
        )
        ground_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[4., 4., .10],
            rgbaColor=[.17,.20,.24,1.], physicsClientId=cid,
        )
        self.ground = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=ground_col,
            baseVisualShapeIndex=ground_visual,
            basePosition=[0.,0.,-.10],
            physicsClientId=cid,
        )
        self.robot = self._build_robot()
        for i in range(self.settings.payload_count):
            x = -1.4 + .34 * (i % 5)
            y = 1.05 + .34 * (i // 5)
            self.spawn_box(mass_kg=self.settings.payload_mass_kg,
                           position_m=(x, y, 1.0 + .20 * (i // 10)))
        if self.settings.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=2.7, cameraYaw=42, cameraPitch=-27,
                cameraTargetPosition=[0.,0.,.5], physicsClientId=cid,
            )
        self.command_joints((0.,)*6)

    def _build_robot(self):
        p = self.p
        cid = self.client
        base_col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=.23, height=.22,
            collisionFramePosition=[0.,0.,.11], physicsClientId=cid,
        )
        base_visual = p.createVisualShape(
            p.GEOM_CYLINDER, radius=.23, length=.22,
            visualFramePosition=[0.,0.,.11],
            rgbaColor=[.20,.23,.28,1.], physicsClientId=cid,
        )
        collisions = []
        visuals = []
        inertial_centers = []
        for i in range(6):
            dimensions, center = _box_geometry_for_link(i)
            halves = [v/2 for v in dimensions]
            collisions.append(p.createCollisionShape(
                p.GEOM_BOX, halfExtents=halves,
                collisionFramePosition=list(center), physicsClientId=cid,
            ))
            visuals.append(p.createVisualShape(
                p.GEOM_BOX, halfExtents=halves,
                visualFramePosition=list(center),
                rgbaColor=[.12, .28 + .045*i, .68 - .055*i, 1.],
                physicsClientId=cid,
            ))
            inertial_centers.append(list(center))
        robot = p.createMultiBody(
            baseMass=0.0,  # World-fixed industrial REFERENCE base
            baseCollisionShapeIndex=base_col,
            baseVisualShapeIndex=base_visual,
            basePosition=[0.,0.,0.],
            baseOrientation=[0.,0.,0.,1.],
            linkMasses=list(LINK_MASSES_KG),
            linkCollisionShapeIndices=collisions,
            linkVisualShapeIndices=visuals,
            linkPositions=[list(offset) for offset in HOME_OFFSETS_M],
            linkOrientations=[[0.,0.,0.,1.]]*6,
            linkInertialFramePositions=inertial_centers,
            linkInertialFrameOrientations=[[0.,0.,0.,1.]]*6,
            linkParentIndices=[0,1,2,3,4,5],
            linkJointTypes=[p.JOINT_REVOLUTE]*6,
            linkJointAxis=[list(axis) for axis in JOINT_AXES],
            physicsClientId=cid,
        )
        if robot < 0 or p.getNumJoints(robot, physicsClientId=cid) != 6:
            raise RuntimeError("Bullet failed to build six-joint reference manipulator")
        for i in range(6):
            p.changeDynamics(
                robot, i,
                lateralFriction=.65,
                angularDamping=.025,
                linearDamping=.015,
                physicsClientId=cid,
            )
        return robot

    def command_joints(self, angles_rad: Sequence[float]):
        targets = check_angles(angles_rad)
        p = self.p
        cid = self.client
        for i, q in enumerate(targets):
            p.setJointMotorControl2(
                self.robot, i,
                p.POSITION_CONTROL,
                targetPosition=q,
                targetVelocity=0.0,
                force=(100., 170., 150., 80., 50., 35.)[i],
                maxVelocity=(2.,2.,2.,3.,3.,3.)[i],
                positionGain=.30,
                velocityGain=1.0,
                physicsClientId=cid,
            )
        self._targets = targets

    def reset_angles(self, angles_rad: Sequence[float]):
        """Teleport only for kinematic validation; NOT a physical trajectory."""
        angles = check_angles(angles_rad)
        for i, q in enumerate(angles):
            self.p.resetJointState(
                self.robot, i, targetValue=q, targetVelocity=0.0,
                physicsClientId=self.client,
            )
        self.command_joints(angles)

    def spawn_box(self, *, mass_kg: float = 1., side_m: float = .16,
                  position_m: Sequence[float] = (0.,1.,1.)):
        if len(self.payloads) >= MAX_PAYLOADS:
            raise ValueError("Active payload limit reached")
        for name, value, lower, upper in (
            ("mass_kg", mass_kg, .05, 100.),
            ("side_m", side_m, .05, .60),
        ):
            if isinstance(value, bool) or not isinstance(value, (float,int)):
                raise ValueError(f"{name} must be numeric")
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"{name} out of supported range")
        xyz = tuple(float(x) for x in position_m)
        if len(xyz) != 3 or any(not math.isfinite(x) or abs(x) > 20 for x in xyz):
            raise ValueError("position_m must have 3 finite coordinates within +/-20m")
        half = side_m / 2
        col = self.p.createCollisionShape(
            self.p.GEOM_BOX, halfExtents=[half]*3, physicsClientId=self.client,
        )
        vis = self.p.createVisualShape(
            self.p.GEOM_BOX, halfExtents=[half]*3,
            rgbaColor=[.90,.49,.17,1.], physicsClientId=self.client,
        )
        body = self.p.createMultiBody(
            baseMass=float(mass_kg),
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=list(xyz), physicsClientId=self.client,
        )
        self.p.changeDynamics(
            body, -1, lateralFriction=.65, restitution=.1,
            physicsClientId=self.client,
        )
        self.payloads.append(body)
        return body

    def step(self, count: int = 1):
        if type(count) is not int or not 1 <= count <= 50000:
            raise ValueError("step count must be integer in [1,50000]")
        for _ in range(count):
            self.p.stepSimulation(physicsClientId=self.client)
        self.step_index += count
        return self.report()

    def tool_pose(self):
        """Bullet's *world link frame*, not COM frame, plus declared flange offset."""
        p = self.p
        link = p.getLinkState(
            self.robot, 5, computeForwardKinematics=True, physicsClientId=self.client,
        )
        position, orientation = p.multiplyTransforms(
            link[4], link[5], TOOL_OFFSET_M, [0.,0.,0.,1.],
        )
        return tuple(position), tuple(orientation)

    def report(self):
        p = self.p
        q = tuple(p.getJointState(self.robot, i, physicsClientId=self.client)[0]
                  for i in range(6))
        tool, quat = self.tool_pose()
        # Comparison with independent analytic FK of the measured joint angles
        # is a meaningful cross-check of the geometric chain, not a
        # certification of dynamic/contact accuracy.
        expected = forward_kinematics(q).end_effector_m
        error = math.dist(tool, expected)
        boxes = [
            tuple(p.getBasePositionAndOrientation(body, physicsClientId=self.client)[0])
            for body in self.payloads
        ]
        return {
            "engine": "pybullet_cpu",
            "model": MODEL_ID,
            "fidelity": "reference_kinematics_and_bullet_dynamics_not_oem_calibrated",
            "mode": "GUI" if self.settings.gui else "DIRECT",
            "step_index": self.step_index,
            "fixed_dt_seconds": PHYSICS_DT,
            "elapsed_sim_seconds": round(self.step_index*PHYSICS_DT, 9),
            "joint_radians": list(q),
            "target_radians": list(self._targets),
            "tool_position_m": list(tool),
            "tool_orientation_xyzw": list(quat),
            "analytic_tool_position_m": list(expected),
            "bullet_analytic_position_difference_m": error,
            "payload_positions_m": [list(v) for v in boxes],
            "num_contacts": len(p.getContactPoints(physicsClientId=self.client)),
            "physically_qualified": False,
        }

    def run_interactive(self):
        if not self.settings.gui:
            raise RuntimeError("Interactive loop requires BulletSettings(gui=True)")
        p = self.p
        sliders = [
            p.addUserDebugParameter(
                f"Joint {i+1} ({JOINT_LIMITS_DEG[i][0]:.0f}..{JOINT_LIMITS_DEG[i][1]:.0f})",
                JOINT_LIMITS_DEG[i][0], JOINT_LIMITS_DEG[i][1], 0.,
                physicsClientId=self.client,
            )
            for i in range(6)
        ]
        p.addUserDebugText(
            "DSIQ Reference6R: original robotics training model, NOT OEM CAD",
            [0.,-.85,1.65], textColorRGB=[.2,.9,.9], textSize=1.2,
            physicsClientId=self.client,
        )
        print(
            "DSIQ Bullet Playground (local CPU). Move six joint sliders. "
            "Use mouse to orbit/pan/zoom; close window or Ctrl+C to stop."
        )
        last = time.perf_counter()
        while p.isConnected(self.client):
            degrees = [
                p.readUserDebugParameter(s, physicsClientId=self.client)
                for s in sliders
            ]
            targets = [math.radians(q) for q in degrees]
            self.command_joints(targets)
            self.step(1)
            last += PHYSICS_DT
            sleep = last - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                last = time.perf_counter()

    def close(self):
        if self.client >= 0:
            try:
                self.p.disconnect(physicsClientId=self.client)
            finally:
                self.client = -1

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
