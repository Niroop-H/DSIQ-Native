"""DSIQ Reference6R joint trajectory planning and Bullet tracking evidence.

Finite, joint-limit checked quintic position/velocity/acceleration trajectories,
sampled at an exact fixed simulation step. These are reference kinematic paths:
they are NOT collision-free industrial plans, torque-feasible trajectories,
jerk-limited servo commands or manufacturer-calibrated controller outputs.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .robotics import JOINT_LIMITS_DEG, MODEL_ID, check_angles

MAX_TRAJECTORY_SAMPLES = 10000
BULLET_STEP_SECONDS = 1.0 / 240.0

# Design-reference, not OEM motor/rated limits (radians/s and radians/s^2).
REFERENCE_VELOCITY_LIMITS = (1.5, 1.2, 1.2, 2.0, 2.0, 2.5)
REFERENCE_ACCEL_LIMITS = (3.0, 2.4, 2.4, 4.0, 4.0, 5.0)
_QUINTIC_MAX_SLOPE = 1.875
_QUINTIC_MAX_CURVATURE = 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class TrajectoryStep:
    time_s: float
    angles_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    accelerations_rad_s2: tuple[float, ...]


@dataclass(frozen=True)
class Trajectory:
    model: str
    duration_s: float
    dt_s: float
    reference_min_duration_s: float
    max_reference_velocity_rad_s: float
    max_reference_acceleration_rad_s2: float
    steps: tuple[TrajectoryStep, ...]

    def summary(self) -> dict:
        return {
            "model": self.model,
            "profile": "QUINTIC_C2_JOINT_SPACE_REFERENCE",
            "duration_s": self.duration_s,
            "reference_min_duration_s": self.reference_min_duration_s,
            "dt_s": self.dt_s,
            "sample_count": len(self.steps),
            "max_reference_velocity_rad_s": self.max_reference_velocity_rad_s,
            "max_reference_acceleration_rad_s2": self.max_reference_acceleration_rad_s2,
            "joint_position_limits_enforced": True,
            "reference_speed_and_acceleration_limits_enforced": True,
            "torque_qualified": False,
            "collision_free": False,
            "model_calibrated": False,
            "note": "A mathematically bounded path is not a collision-free OEM robot program.",
        }


def minimum_duration(
    start_rad: Sequence[float], goal_rad: Sequence[float],
    *, max_velocity_rad_s: Sequence[float] = REFERENCE_VELOCITY_LIMITS,
    max_acceleration_rad_s2: Sequence[float] = REFERENCE_ACCEL_LIMITS,
) -> float:
    start = check_angles(start_rad)
    goal = check_angles(goal_rad)
    if len(max_velocity_rad_s) != 6 or len(max_acceleration_rad_s2) != 6:
        raise ValueError("Exactly six speed and acceleration limits required")
    def positive(values, label):
        if any(
            isinstance(v,bool) or not isinstance(v,(float,int))
            or not math.isfinite(v) or v <= 0
            for v in values
        ):
            raise ValueError(label + " must be six positive finite numbers")
        return tuple(float(v) for v in values)
    vmax = positive(max_velocity_rad_s, "Velocity limits")
    amax = positive(max_acceleration_rad_s2, "Acceleration limits")
    duration = 0.2
    for s, g, v, a in zip(start, goal, vmax, amax):
        delta = abs(g-s)
        duration = max(
            duration,
            _QUINTIC_MAX_SLOPE * delta / v,
            math.sqrt(_QUINTIC_MAX_CURVATURE * delta / a),
        )
    return duration


def plan_joint_trajectory(
    start_rad: Sequence[float], goal_rad: Sequence[float],
    *, duration_s: float | None = None,
    dt_s: float = BULLET_STEP_SECONDS,
    max_velocity_rad_s: Sequence[float] = REFERENCE_VELOCITY_LIMITS,
    max_acceleration_rad_s2: Sequence[float] = REFERENCE_ACCEL_LIMITS,
) -> Trajectory:
    start = check_angles(start_rad)
    goal = check_angles(goal_rad)
    if not math.isfinite(dt_s) or not 0.001 <= dt_s <= 0.05:
        raise ValueError("Trajectory dt must be finite in [0.001,0.05] seconds")
    required = minimum_duration(
        start, goal,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
    )
    if duration_s is None:
        # Small 0.5% safety margin for float roundoff, then quantize to fixed dt.
        duration = required * 1.005
    else:
        if not math.isfinite(duration_s) or duration_s < required - 1e-9:
            raise ValueError(
                f"Requested duration violates reference speed/acceleration limits; "
                f"minimum duration is {required:.6f}s"
            )
        duration = float(duration_s)
    if not 0.0 < duration <= 60.0:
        raise ValueError("Trajectory duration must be <=60 seconds")
    ticks = int(math.ceil(duration/dt_s))
    if ticks < 1 or ticks + 1 > MAX_TRAJECTORY_SAMPLES:
        raise ValueError("Trajectory exceeds bounded memory/sample budget")
    T = ticks * dt_s
    delta = tuple(g-s for s,g in zip(start,goal))
    samples = []
    max_seen_v = 0.0
    max_seen_a = 0.0
    for i in range(ticks+1):
        u = i/ticks
        s = (10*u**3 - 15*u**4 + 6*u**5)
        ds = 30*u*u - 60*u**3 + 30*u**4
        d2s = 60*u - 180*u*u + 120*u**3
        q = tuple(a + diff*s for a,diff in zip(start, delta))
        v = tuple(diff*ds/T for diff in delta)
        accel = tuple(diff*d2s/(T*T) for diff in delta)
        max_seen_v = max(max_seen_v, *(abs(vj) for vj in v))
        max_seen_a = max(max_seen_a, *(abs(aj) for aj in accel))
        samples.append(TrajectoryStep(
            i*dt_s, q, v, accel,
        ))
    # Analytic continuous-time peak bounds (independent of sampling density).
    for diff, v, a in zip(
        delta, max_velocity_rad_s, max_acceleration_rad_s2,
    ):
        if (abs(diff)*_QUINTIC_MAX_SLOPE/T > v + 1e-8 or
            abs(diff)*_QUINTIC_MAX_CURVATURE/(T*T) > a + 1e-8):
            raise ArithmeticError("Computed duration violates requested reference limits")
    if not all(math.isfinite(v) for sample in samples
               for v in (*sample.angles_rad, *sample.velocities_rad_s,
                         *sample.accelerations_rad_s2)):
        raise ArithmeticError("Trajectory produced a non-finite state")
    return Trajectory(
        MODEL_ID, T, dt_s, required, max_seen_v, max_seen_a,
        tuple(samples),
    )


def replay_in_bullet(world, plan: Trajectory, *, record_every: int = 10) -> dict:
    """Apply every planned motor sample to Bullet, record true measured error.

    This evaluates *the dynamics of this local Bullet reference manipulator*
    and does not prove collision avoidance, torque sufficiency, hardware
    control or equivalence with NVIDIA's Isaac Sim.
    """
    if plan.model != MODEL_ID or not math.isclose(plan.dt_s, BULLET_STEP_SECONDS, abs_tol=1e-12):
        raise ValueError("Plan model and fixed dt must match the Bullet reference runtime")
    if not 1 <= record_every <= 1000:
        raise ValueError("record_every must be in [1,1000]")
    initial = world.report()["joint_radians"]
    if max(abs(a-b) for a,b in zip(initial, plan.steps[0].angles_rad)) > 0.03:
        raise ValueError("Bullet joint state differs from planned start; reset or replan")
    residuals: list[float] = []
    recorded = []
    largest = 0.0
    collisions = 0
    for index, point in enumerate(plan.steps[1:], start=1):
        world.command_joints(point.angles_rad)
        state = world.step(1)
        measured = state["joint_radians"]
        error = max(abs(q-ref) for q,ref in zip(measured, point.angles_rad))
        residuals.append(error)
        largest = max(largest, error)
        collisions = max(collisions, int(state["num_contacts"]))
        if index % record_every == 0 or index == len(plan.steps)-1:
            recorded.append({
                "time_s": point.time_s,
                "max_joint_tracking_error_rad": error,
                "reference_angles_rad": list(point.angles_rad),
                "measured_angles_rad": list(measured),
            })
    ordered = sorted(residuals)
    p95 = ordered[min(len(ordered)-1, math.ceil(.95*len(ordered))-1)] if ordered else 0.0
    return {
        "engine": "pybullet_cpu",
        "model": MODEL_ID,
        "duration_s": plan.duration_s,
        "simulated_steps": len(plan.steps)-1,
        "max_abs_joint_tracking_error_rad": largest,
        "p95_max_joint_tracking_error_rad": p95,
        "final_measured_joint_rad": world.report()["joint_radians"],
        "final_reference_joint_rad": list(plan.steps[-1].angles_rad),
        "max_contact_points_in_scene": collisions,
        "samples": recorded,
        "physical_calibration": False,
        "collision_free_trajectory": False,
        "note": "Observed residuals are not an OEM controller acceptance certificate.",
    }
