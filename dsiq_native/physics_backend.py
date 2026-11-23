"""Strict, vendor-neutral boundary for the first executable physics backend.

This is Phase 0 of the DSIQ Native backend-agnostic roadmap. One REAL
implementation is shipped: isolated PyBullet DIRECT for a verified, locally
licensed URDF. Neither PhysX nor MuJoCo/Newton is silently emulated.

Keep USD composition, render quality and safety credentials OUT of solver
settings. This narrow interface is additive; existing apps can migrate one
use case at a time after conformance tests pass.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Protocol, runtime_checkable


@dataclass(frozen=True)
class BackendCapabilities:
    backend_id: str
    source_format: str
    step_mode: str
    fixed_timestep_s: float
    supports_verified_urdf: bool
    supports_usd_physics_import: bool
    supports_joint_position_target: bool
    supports_measured_joint_feedback: bool
    supports_collision_contacts: bool
    supports_gpu_batched_worlds: bool
    supports_hardware_actuation: bool
    independently_qualified: bool

    def as_dict(self) -> dict:
        return asdict(self)


@runtime_checkable
class PhysicsBackendWorld(Protocol):
    """Minimum common contract; a structural API, not solver equivalence."""

    @property
    def capabilities(self) -> BackendCapabilities: ...

    def import_verified_urdf(self, manifest: str | Path) -> dict: ...

    def command_joint(self, asset_id: str, name: str, target: float) -> dict: ...

    def step(self, count: int = 1) -> dict: ...

    def read_joint_states(self, asset_id: str) -> dict: ...

    def read_contacts(self, asset_id: str, *, maximum: int = 32) -> dict: ...

    def close(self) -> None: ...


class BulletPhysicsBackend:
    """One isolated reference world using C++ Bullet through PyBullet.

    A loaded imported robot is required for joint queries. Reference6R is
    constructed internally by BulletPlayground, but NEVER presented as an
    OEM robot or as an asset loaded from the customer's manifest.
    """

    def __init__(self):
        from .bullet_runtime import BulletPlayground, BulletSettings
        self.world = BulletPlayground(BulletSettings(payload_count=0))
        from .bullet_runtime import PHYSICS_DT
        self._caps = BackendCapabilities(
            backend_id="bullet_cpu_direct_v1",
            source_format="verified_local_urdf",
            step_mode="fixed_step_explicit",
            fixed_timestep_s=PHYSICS_DT,
            supports_verified_urdf=True,
            supports_usd_physics_import=False,
            supports_joint_position_target=True,
            supports_measured_joint_feedback=True,
            supports_collision_contacts=True,
            supports_gpu_batched_worlds=False,
            supports_hardware_actuation=False,
            independently_qualified=False,
        )
        self._closed = False

    @property
    def capabilities(self) -> BackendCapabilities:
        return self._caps

    def _require_open(self):
        if self._closed:
            raise RuntimeError("Physics backend is closed")

    def _robot(self, asset_id: str) -> dict:
        self._require_open()
        if not isinstance(asset_id, str) or asset_id not in self.world.imported_robots:
            raise ValueError("Named verified URDF not loaded in this backend world")
        return self.world.imported_robots[asset_id]

    def import_verified_urdf(self, manifest: str | Path) -> dict:
        self._require_open()
        robot = self.world.import_urdf_robot(str(manifest))
        return {
            "asset_id":robot["asset_id"],
            "source_urdf_sha256":robot["source_urdf_sha256"],
            "joint_count":robot["joint_count"],
            "verified_source_imported":True,
            "independently_qualified":False,
        }

    def command_joint(self, asset_id: str, name: str, target: float) -> dict:
        self._robot(asset_id)
        return self.world.command_imported_joint(asset_id, name, target)

    def step(self, count: int = 1) -> dict:
        self._require_open()
        if type(count) is not int or not 1 <= count <= 50000:
            raise ValueError("Backend step count must be integer in [1,50000]")
        self.world.step(count)
        return {
            "backend_id":self.capabilities.backend_id,
            "step_index":self.world.step_index,
            "simulated_time_s":self.world.step_index*self.capabilities.fixed_timestep_s,
            "fixed_timestep_s":self.capabilities.fixed_timestep_s,
            "hardware_control_executed":False,
        }

    def read_joint_states(self, asset_id: str) -> dict:
        robot = self._robot(asset_id)
        p = self.world.p
        cid = self.world.client
        states = []
        for joint in robot["joints"]:
            if joint["lower"] is None:
                continue
            q, velocity, _, effort = p.getJointState(
                robot["body_id"],joint["joint_index"],physicsClientId=cid,
            )
            values=(q, velocity, effort)
            if any(not math.isfinite(float(value)) for value in values):
                raise ArithmeticError("Bullet produced nonfinite joint telemetry")
            states.append({
                "name":joint["name"],
                "index":joint["joint_index"],
                "unit":"m" if joint["type"]==p.JOINT_PRISMATIC else "rad",
                "position":float(q),
                "velocity":float(velocity),
                "solver_reported_effort":float(effort),
                "source_max_velocity":float(joint["max_velocity"]),
                "source_max_effort":float(joint["max_effort"]),
            })
        if not states:
            raise RuntimeError("Imported URDF has no actuated joints")
        return {
            "asset_id":asset_id,
            "backend_id":self.capabilities.backend_id,
            "source_urdf_sha256":robot["source_urdf_sha256"],
            "step_index":self.world.step_index,
            "simulated_time_s":self.world.step_index*self.capabilities.fixed_timestep_s,
            "joints":states,
            "measured_from_solver":True,
            "physically_calibrated":False,
        }

    def read_contacts(self, asset_id: str, *, maximum: int = 32) -> dict:
        robot = self._robot(asset_id)
        if type(maximum) is not int or not 1 <= maximum <= 256:
            raise ValueError("Contact reporting limit must be integer in [1,256]")
        p = self.world.p
        contacts = p.getContactPoints(
            bodyA=robot["body_id"],physicsClientId=self.world.client
        )
        details=[]
        for cp in contacts[:maximum]:
            distance=float(cp[8])
            if not math.isfinite(distance):
                raise ArithmeticError("Bullet produced nonfinite contact distance")
            details.append({
                "other_body_id":int(cp[2]),
                "robot_link_index":int(cp[3]),
                "other_link_index":int(cp[4]),
                "signed_separation_m":distance,
            })
        return {
            "asset_id":asset_id,
            "step_index":self.world.step_index,
            "contact_count":len(contacts),
            "contacts":details,
            "truncated":len(contacts)>maximum,
            "solver_reported_discrete_contacts":True,
            "swept_collision_detection_qualified":False,
            "self_collision_qualified":False,
        }

    def close(self) -> None:
        if not self._closed:
            self.world.close()
            self._closed=True

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


def available_backend_capabilities() -> list[dict]:
    """Supported backends only; do not advertise unimplemented solvers."""
    from .bullet_runtime import PHYSICS_DT
    return [BackendCapabilities(
        "bullet_cpu_direct_v1", "verified_local_urdf", "fixed_step_explicit",
        PHYSICS_DT, True, False, True, True, True, False, False, False,
    ).as_dict()]


def open_backend(name: str = "bullet_cpu_direct_v1") -> PhysicsBackendWorld:
    if name!="bullet_cpu_direct_v1":
        raise ValueError(
            f"Physics backend {name!r} is unavailable. "
            "MuJoCo, Newton and PhysX require independent implementation "
            "and conformance; no silent Bullet fallback."
        )
    return BulletPhysicsBackend()


def run_backend_conformance(name: str = "bullet_cpu_direct_v1") -> dict:
    """Deterministic, real-solver smoke conformance for one fixture only.

    This suite checks source identity, bounds rejection, feedback, timing
    and independently isolated client starts. It is neither a formal
    simulation verification standard nor a cross-solver equivalence test.
    """
    from .machine_designer import MachineDesign, save_project

    with TemporaryDirectory(prefix="dsiq-backend-conformance-") as root:
        files=save_project(
            MachineDesign(asset_id="backend_contract_fixture"),
            Path(root)/"gantry",
        )
        manifest=Path(files["manifest"])
        import json
        doc=json.loads(manifest.read_text(encoding="utf8"))
        expected_sha=doc["geometry"]["sha256"]
        tests=[]
        final_positions=[]
        for trial in range(2):
            with open_backend(name) as backend:
                if not isinstance(backend, PhysicsBackendWorld):
                    raise AssertionError("Backend does not expose declared world API")
                record=backend.import_verified_urdf(manifest)
                if record["source_urdf_sha256"]!=expected_sha:
                    raise AssertionError("Solver did not preserve verified source hash")
                asset=record["asset_id"]
                start=backend.read_joint_states(asset)
                xstart=next(
                    x for x in start["joints"] if x["name"]=="actuator_x"
                )["position"]
                if abs(xstart)>.01:
                    raise AssertionError("Isolated backend did not start at home")
                refused_unknown=refused_excess=False
                try:
                    backend.command_joint(asset,"invented_joint",.2)
                except ValueError:
                    refused_unknown=True
                try:
                    backend.command_joint(asset,"actuator_x",999.)
                except ValueError:
                    refused_excess=True
                if not (refused_unknown and refused_excess):
                    raise AssertionError("Unknown or overtravel command was accepted")
                backend.command_joint(asset,"actuator_x",.20)
                clock=backend.step(100)
                actual=backend.read_joint_states(asset)
                contacts=backend.read_contacts(asset)
                x=next(
                    item for item in actual["joints"]
                    if item["name"]=="actuator_x"
                )
                if not (.005<x["position"]<.30):
                    raise AssertionError("Actuator did not respond within fixture bounds")
                if clock["step_index"]!=100 or abs(
                    clock["simulated_time_s"]-
                    100*backend.capabilities.fixed_timestep_s
                )>1e-12:
                    raise AssertionError("Solver clock contract did not hold")
                if contacts["step_index"]!=100:
                    raise AssertionError("Contact telemetry was not from the current frame")
                final_positions.append(x["position"])
                tests.append({
                    "trial":trial+1,
                    "verified_manifest_hash":True,
                    "disallowed_command_rejections":2,
                    "step_index":clock["step_index"],
                    "measured_x_position_m":x["position"],
                    "measured_x_velocity_m_s":x["velocity"],
                    "contact_count":contacts["contact_count"],
                    "isolated_real_bullet_world":True,
                })
        deviation=abs(final_positions[0]-final_positions[1])
        pass_repeatability=deviation <= 1e-6
        return {
            "schema":"dsiq-physics-backend-conformance-v1",
            "backend_id":name,
            "source_fixture_asset_class":"self_authored_test_fixture",
            "test_case_count":len(tests),
            "tests":tests,
            "repeatability_position_delta_m":deviation,
            "repeatability_threshold_m":1e-6,
            "passed":pass_repeatability,
            "capabilities":available_backend_capabilities(),
            "other_physics_backends_tested":False,
            "usd_physics_import_implemented":False,
            "industrial_safety_or_oem_calibration_qualified":False,
        }
