"""Analytical motion-limit audit for imported quintic robot trajectories.

This is a fail-closed *reference planner* qualification helper. It checks
maxima of the exact quintic polynomial (not only sampled positions), and
ties engineer-supplied joint limits to verified imported-URDF identities.

It deliberately cannot validate torque, collision-free swept motion,
OEM calibration or hardware safety. It cannot issue physical commands.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from .imported_trajectory import ImportedTrajectory

SCHEMA="dsiq-document-controlled-joint-motion-limits-v1"
ACC_FACTOR=10*math.sqrt(3)/3
JERK_FACTOR=60.


def _positive(v,name):
    if type(v) not in (int,float) or not math.isfinite(v) or v<=0:
        raise ValueError(f"{name} must be a positive finite engineering limit")
    return float(v)


def audit_joint_motion(plan: ImportedTrajectory, source_limits: dict) -> dict:
    """Check strict source-declared velocity, acceleration, and jerk envelopes.

    All actuated axes must have bounded, explicit limits. A missing axis
    rejects the entire plan; it is never assigned a pretend default.
    """
    if not isinstance(source_limits,dict) or source_limits.get("schema")!=SCHEMA:
        raise ValueError("Motion-limit document schema missing")
    if source_limits.get("asset_id")!=plan.asset_id:
        raise ValueError("Engineering limits do not apply to this source robot")
    limits=source_limits.get("joints")
    if not isinstance(limits,dict) or set(limits)!=set(plan.names):
        raise ValueError("Engineering limits must cover every imported actuator")
    if not isinstance(plan.duration_s,(float,int)) or not math.isfinite(plan.duration_s) or plan.duration_s<=0:
        raise ValueError("Motion duration must be positive")
    if len(plan.samples)<2 or len(plan.samples)>7200:
        raise ValueError("Motion plan exceeds verified sample bounds")
    if any(len(x)!=len(plan.names) or any(not math.isfinite(v) for v in x)
           for x in plan.samples):
        raise ValueError("Motion plan contains invalid joint states")
    report={}
    all_pass=True
    for name,a,b,source_speed in zip(
        plan.names,plan.start,plan.goal,plan.declared_speed_limits
    ):
        limit=limits[name]
        if not isinstance(limit,dict):
            raise ValueError("Joint limit entry must be an object")
        max_speed=_positive(limit.get("max_speed"),f"{name}.max_speed")
        max_acceleration=_positive(limit.get("max_acceleration"),f"{name}.max_acceleration")
        max_jerk=_positive(limit.get("max_jerk"),f"{name}.max_jerk")
        source_speed=_positive(source_speed,"verified source URDF motor rating")
        if max_speed>source_speed*(1+1e-9):
            raise ValueError("Engineering speed limit exceeds source URDF rating")
        delta=abs(b-a)
        t=float(plan.duration_s)
        actual_v=1.875*delta/t
        actual_acc=ACC_FACTOR*delta/t**2
        actual_jerk=JERK_FACTOR*delta/t**3
        success=(actual_v<=max_speed+1e-10 and
                 actual_acc<=max_acceleration+1e-10 and
                 actual_jerk<=max_jerk+1e-10)
        all_pass=all_pass and success
        report[name]={
            "unit":"m" if limit.get("joint_type")=="prismatic" else "rad",
            "documented_limits":{
                "speed":max_speed,
                "acceleration":max_acceleration,
                "jerk":max_jerk,
            },
            "analytical_maxima":{
                "speed":actual_v,
                "acceleration":actual_acc,
                "jerk":actual_jerk,
            },
            "passes_all_documented_limits":success,
        }
    canonical=json.dumps(source_limits,sort_keys=True,separators=(",",":"),
                         allow_nan=False).encode()
    return {
        "schema":"dsiq-quintic-motion-limit-audit-v1",
        "asset_id":plan.asset_id,
        "limit_document_sha256":hashlib.sha256(canonical).hexdigest(),
        "quintic_analytical_maxima_used":True,
        "all_documented_limits_passed":all_pass,
        "per_joint":report,
        "collision_free_qualified":False,
        "torque_and_force_qualified":False,
        "physical_estop_validated":False,
        "instrumented_real_machine_verified":False,
        "industrial_safety_certified":False,
        "conclusion":(
            "Source-document kinematic envelope checks passed; external "
            "calibration, torque, risk and safety validation remain required."
            if all_pass else
            "The requested motion violates documented kinematic limits."
        ),
    }


def load_joint_limits(path: str|Path) -> dict:
    p=Path(path).resolve()
    if not p.is_file() or p.stat().st_size>262144:
        raise ValueError("Engineering limits file missing or oversized")
    doc=json.loads(p.read_text(encoding="utf8"))
    if not isinstance(doc,dict):
        raise ValueError("Engineering limits JSON must be an object")
    return doc


class SimulationStopLatch:
    """Stateful demonstration of fail-closed gating; never an E-stop device.

    Real emergency stopping depends on separately rated, independently
    tested safety hardware. An in-process Python latch is NOT an emergency
    stop and must NEVER be wired as sole operational protection.
    """
    def __init__(self):
        self.state="STOPPED"
        self.fault_reason=None
        self._documented_clear=False

    def inspect(self):
        return {
            "software_state":self.state,
            "fault_reason":self.fault_reason,
            "motion_permitted_in_simulation":self.state=="READY",
            "hardware_stop_function":False,
            "iso_10218_safety_function":False,
        }

    def arm(self, *, documented_limits_passed=False,
            collision_screen_passed=False):
        if self.state=="FAULT":
            raise RuntimeError("Software stop latched: explicit reset required")
        if not documented_limits_passed or not collision_screen_passed:
            self.trip("Failed simulation movement prerequisites")
            raise RuntimeError("Simulation move refused: prerequisite failed")
        self._documented_clear=True
        self.state="READY"

    def trip(self,reason: str):
        if not isinstance(reason,str) or not reason.strip():
            raise ValueError("A nonempty fault reason must be recorded")
        self.state="FAULT"
        self.fault_reason=reason
        self._documented_clear=False

    def reset(self):
        self.state="STOPPED"
        self.fault_reason=None
        self._documented_clear=False

    def consume_single_move(self):
        if self.state!="READY" or not self._documented_clear:
            raise RuntimeError("Simulation move blocked: stop latch is not READY")
        self.state="STOPPED"
        self._documented_clear=False
        return True
