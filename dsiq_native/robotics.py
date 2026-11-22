"""DSIQ Reference-6 robotics: explicit six-revolute-joint kinematic chain.

A genuinely executable, original *reference* model, not the CAD, masses or
calibration of a real OEM manipulator. All units SI; angle inputs radians.
This is position-only IK: orientation and collision avoidance are NOT solved.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

HOME_OFFSETS_M: tuple[Vec3, ...] = (
    (0.0, 0.0, 0.28),
    (0.0, 0.0, 0.22),
    (0.42, 0.0, 0.0),
    (0.38, 0.0, 0.0),
    (0.16, 0.0, 0.0),
    (0.12, 0.0, 0.0),
)
JOINT_AXES: tuple[Vec3, ...] = (
    (0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)
# Genuine kinematic model parameters, not an OEM machine specification.
JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-175.0, 175.0),
    (-125.0, 125.0),
    (-145.0, 145.0),
    (-180.0, 180.0),
    (-120.0, 120.0),
    (-180.0, 180.0),
)
LINK_MASSES_KG = (8.0, 6.0, 4.0, 1.8, 1.2, 0.8)
TOOL_OFFSET_M: Vec3 = (0.12, 0.0, 0.0)
JOINT_NAMES = tuple(f"joint_{i + 1}" for i in range(6))
MODEL_ID = "dsiq_reference_6r_v1"


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _length(a: Vec3) -> float:
    return math.sqrt(sum(x * x for x in a))


def _identity() -> Mat3:
    return ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def _mv(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _mm(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )  # type: ignore[return-value]


def _rotation(axis: Vec3, angle: float) -> Mat3:
    """Rodrigues' rotation for a verified unit-length joint axis."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return (
        (c + x*x*v, x*y*v - z*s, x*z*v + y*s),
        (y*x*v + z*s, c + y*y*v, y*z*v - x*s),
        (z*x*v - y*s, z*y*v + x*s, c + z*z*v),
    )


def check_angles(angles_rad: Sequence[float], *, within_limits: bool = True) -> tuple[float, ...]:
    if len(angles_rad) != 6:
        raise ValueError("Reference-6R requires exactly six joint angles (radians)")
    values = tuple(float(x) for x in angles_rad)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Joint angles must be finite")
    if within_limits:
        for name, q, (lo, hi) in zip(JOINT_NAMES, values, JOINT_LIMITS_DEG):
            if not math.radians(lo) - 1e-10 <= q <= math.radians(hi) + 1e-10:
                raise ValueError(f"{name} angle exceeds declared mechanical limits")
    return values


@dataclass(frozen=True)
class ForwardPose:
    end_effector_m: Vec3
    orientation: Mat3
    joint_origins_m: tuple[Vec3, ...]
    joint_axes_world: tuple[Vec3, ...]

    def as_dict(self) -> dict:
        return {
            "model": MODEL_ID, "end_effector_m": list(self.end_effector_m),
            "orientation": [list(row) for row in self.orientation],
            "joint_origins_m": [list(v) for v in self.joint_origins_m],
            "joint_axes_world": [list(v) for v in self.joint_axes_world],
        }


def forward_kinematics(angles_rad: Sequence[float]) -> ForwardPose:
    angles = check_angles(angles_rad)
    p: Vec3 = (0.0, 0.0, 0.0)
    R = _identity()
    origins: list[Vec3] = []
    axes: list[Vec3] = []
    for offset, axis, q in zip(HOME_OFFSETS_M, JOINT_AXES, angles):
        p = _add(p, _mv(R, offset))
        origins.append(p)
        axes.append(_mv(R, axis))
        R = _mm(R, _rotation(axis, q))
    tip = _add(p, _mv(R, TOOL_OFFSET_M))
    return ForwardPose(tip, R, tuple(origins), tuple(axes))


def translation_jacobian(angles_rad: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    """Analytic 3x6 Jacobian in meters/radian, not an approximate pose delta."""
    pose = forward_kinematics(angles_rad)
    columns = [
        _cross(axis, _sub(pose.end_effector_m, origin))
        for axis, origin in zip(pose.joint_axes_world, pose.joint_origins_m)
    ]
    return tuple(tuple(column[row] for column in columns) for row in range(3))


def _solve3(A: Sequence[Sequence[float]], b: Sequence[float]) -> Vec3:
    """Small pivoted linear solver for damped least squares; singular matrices fail."""
    a = [[float(A[r][c]) for c in range(3)] + [float(b[r])] for r in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-14:
            raise ArithmeticError("Singular damping matrix")
        a[col], a[pivot] = a[pivot], a[col]
        div = a[col][col]
        for j in range(col, 4):
            a[col][j] /= div
        for r in range(3):
            if r == col:
                continue
            mult = a[r][col]
            for j in range(col, 4):
                a[r][j] -= mult * a[col][j]
    return (a[0][3], a[1][3], a[2][3])


@dataclass(frozen=True)
class InverseResult:
    angles_rad: tuple[float, ...]
    position_error_m: float
    iterations: int
    converged: bool

    def as_dict(self) -> dict:
        return {
            "model": MODEL_ID,
            "angles_rad": list(self.angles_rad),
            "angles_deg": [math.degrees(a) for a in self.angles_rad],
            "position_error_m": self.position_error_m,
            "iterations": self.iterations,
            "converged": self.converged,
            "orientation_solved": False,
            "collision_avoidance": False,
        }


def inverse_kinematics(
    target_m: Sequence[float],
    seed_rad: Sequence[float] = (0., 0., 0., 0., 0., 0.),
    *,
    tolerance_m: float = 0.001,
    max_iterations: int = 250,
    damping: float = 0.12,
) -> InverseResult:
    """Damped least-squares position-only IK with bounded step and joint limits.

    Returns explicit converged=False for unreachable or poorly conditioned cases.
    Does not optimize orientation, joint torques or collision-free motion.
    """
    if len(target_m) != 3:
        raise ValueError("Expected target XYZ in meters")
    target: Vec3 = tuple(float(x) for x in target_m)  # type: ignore[assignment]
    if not all(math.isfinite(v) for v in target):
        raise ValueError("Target coordinates must be finite")
    if not math.isfinite(tolerance_m) or not 1e-8 <= tolerance_m <= 0.1:
        raise ValueError("tolerance_m must be between 1e-8 and 0.1 meters")
    if not isinstance(max_iterations, int) or not 1 <= max_iterations <= 5000:
        raise ValueError("max_iterations must be in [1,5000]")
    if not math.isfinite(damping) or not 1e-5 <= damping <= 100:
        raise ValueError("damping must be finite, positive and bounded")
    q = list(check_angles(seed_rad))
    err = float("inf")
    for iteration in range(max_iterations + 1):
        pose = forward_kinematics(q)
        residual = _sub(target, pose.end_effector_m)
        err = _length(residual)
        if err <= tolerance_m:
            return InverseResult(tuple(q), err, iteration, True)
        if iteration == max_iterations:
            break
        J = translation_jacobian(q)
        A = [
            [
                sum(J[i][k] * J[j][k] for k in range(6))
                + (damping * damping if i == j else 0.0)
                for j in range(3)
            ]
            for i in range(3)
        ]
        d = _solve3(A, residual)
        dq = [sum(J[row][i] * d[row] for row in range(3)) for i in range(6)]
        scale = min(1., 0.15 / max((abs(x) for x in dq), default=1.))
        for i, change in enumerate(dq):
            lo, hi = JOINT_LIMITS_DEG[i]
            q[i] = min(math.radians(hi), max(math.radians(lo), q[i] + scale * change))
    return InverseResult(tuple(q), err, max_iterations, False)
