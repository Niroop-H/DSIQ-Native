"""Independent, measured Bullet reference-physics acceptance suite.

Each metric compares an isolated Bullet C++ world against analytic mechanics,
with explicit tolerances and observed residuals. Passing these narrowly
defined experiments does NOT validate arbitrary industrial physical systems.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from .bullet_runtime import PHYSICS_DT, _bullet

@dataclass(frozen=True)
class PhysicsCase:
    name: str
    observed: dict
    reference: dict
    tolerances: dict
    passed: bool
    explanation: str

    def as_dict(self) -> dict:
        return asdict(self)


def _client(gravity: float):
    p = _bullet()
    cid = p.connect(p.DIRECT)
    if cid < 0:
        raise RuntimeError("Bullet physics client did not initialize")
    try:
        p.setGravity(0, 0, -gravity, physicsClientId=cid)
        p.setTimeStep(PHYSICS_DT, physicsClientId=cid)
        p.setRealTimeSimulation(0, physicsClientId=cid)
        p.setPhysicsEngineParameter(
            numSolverIterations=120, deterministicOverlappingPairs=1,
            physicsClientId=cid,
        )
    except Exception:
        p.disconnect(physicsClientId=cid)
        raise
    return p, cid


def _box(p, cid, *, mass, side, position):
    shape = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[side/2]*3, physicsClientId=cid
    )
    body = p.createMultiBody(
        baseMass=float(mass), baseCollisionShapeIndex=shape,
        basePosition=list(position), physicsClientId=cid,
    )
    if body < 0:
        raise RuntimeError("Bullet failed to create benchmark body")
    p.changeDynamics(
        body, -1, linearDamping=0, angularDamping=0, restitution=0,
        physicsClientId=cid,
    )
    return body


def free_fall(*, duration_steps: int = 72) -> PhysicsCase:
    """Unconstrained translational motion from rest; compare to g*t²/2."""
    if type(duration_steps) is not int or not 24 <= duration_steps <= 120:
        raise ValueError("Free-fall sample count must be in [24,120]")
    p, cid = _client(9.81)
    try:
        body = _box(p, cid, mass=2., side=.1, position=(0.,0.,3.))
        for _ in range(duration_steps):
            p.stepSimulation(physicsClientId=cid)
        position = p.getBasePositionAndOrientation(body, physicsClientId=cid)[0]
        velocity = p.getBaseVelocity(body, physicsClientId=cid)[0]
        t = duration_steps * PHYSICS_DT
        ideal_z = 3. - 9.81 * t*t / 2
        ideal_vz = -9.81*t
        abs_position_error = abs(position[2]-ideal_z)
        abs_speed_error = abs(velocity[2]-ideal_vz)
        return PhysicsCase(
            "unconstrained_free_fall",
            {"z_m":position[2], "vz_m_s":velocity[2],
             "position_absolute_error_m":abs_position_error,
             "velocity_absolute_error_m_s":abs_speed_error,
             "samples":duration_steps, "dt_s":PHYSICS_DT},
            {"z_m":ideal_z, "vz_m_s":ideal_vz},
            {"position_absolute_error_m":.02, "velocity_absolute_error_m_s":.05},
            abs_position_error < .02 and abs_speed_error < .05,
            "First-order fixed-step solver is not expected to be analytically exact.",
        )
    finally:
        p.disconnect(physicsClientId=cid)


def ground_contact() -> PhysicsCase:
    """Drop a cube onto a planar static collider and measure resting depth."""
    p, cid = _client(9.81)
    try:
        ground_shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[3.,3.,.10], physicsClientId=cid
        )
        p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=ground_shape,
            basePosition=[0.,0.,-.10], physicsClientId=cid,
        )
        body = _box(p, cid, mass=1., side=.20, position=(0.,0.,1.5))
        p.changeDynamics(
            body, -1, restitution=0, lateralFriction=.65, physicsClientId=cid
        )
        for _ in range(720):
            p.stepSimulation(physicsClientId=cid)
        z = p.getBasePositionAndOrientation(body, physicsClientId=cid)[0][2]
        vz = p.getBaseVelocity(body, physicsClientId=cid)[0][2]
        contacts = p.getContactPoints(bodyA=body, physicsClientId=cid)
        error = abs(z-.10)
        return PhysicsCase(
            "resting_rigid_contact",
            {"center_height_m":z, "vertical_speed_m_s":vz,
             "contact_points":len(contacts), "height_error_m":error},
            {"center_height_m":.10, "vertical_speed_m_s":0.},
            {"height_error_m":.025, "absolute_speed_m_s":.07, "contact_points_min":1},
            error < .025 and abs(vz) < .07 and len(contacts)>0,
            "Contact solver is compliant/approximate and does not certify precision machine contacts.",
        )
    finally:
        p.disconnect(physicsClientId=cid)


def torque_and_inertia() -> PhysicsCase:
    """Controlled pure yaw torque on a freely rotating cuboid, no gravity."""
    p, cid = _client(0.)
    try:
        mass = 2.
        sx, sy, sz = (.24, .32, .18)
        shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[sx/2,sy/2,sz/2], physicsClientId=cid
        )
        body = p.createMultiBody(
            baseMass=mass, baseCollisionShapeIndex=shape,
            basePosition=[0.,0.,1.], physicsClientId=cid,
        )
        p.changeDynamics(body,-1,linearDamping=0,angularDamping=0,physicsClientId=cid)
        torque = .045
        steps = 96
        for _ in range(steps):
            p.applyExternalTorque(
                body, -1, [0.,0.,torque], p.WORLD_FRAME, physicsClientId=cid
            )
            p.stepSimulation(physicsClientId=cid)
        omega = p.getBaseVelocity(body, physicsClientId=cid)[1][2]
        inertia = mass*(sx*sx+sy*sy)/12
        analytic = torque/inertia*steps*PHYSICS_DT
        error = abs(omega-analytic)
        return PhysicsCase(
            "rigid_body_inertia_yaw",
            {"angular_velocity_z_rad_s":omega,
             "angular_velocity_error_rad_s":error},
            {"angular_velocity_z_rad_s":analytic,"Izz_kg_m2":inertia,
             "torque_Nm":torque},
            {"angular_velocity_error_rad_s":.04},
            error < .04,
            "Analytic reference assumes no damping and constant principal-axis torque.",
        )
    finally:
        p.disconnect(physicsClientId=cid)


def repeatability() -> PhysicsCase:
    """Re-run independent identical clients; measure full rigid-body state drift."""
    def run():
        p, cid = _client(9.81)
        try:
            body = _box(p, cid, mass=1., side=.18, position=(.2,-.3,2.))
            p.resetBaseVelocity(
                body, linearVelocity=[.3,-.1,.25], physicsClientId=cid
            )
            for _ in range(140):
                p.stepSimulation(physicsClientId=cid)
            pos, rot = p.getBasePositionAndOrientation(body, physicsClientId=cid)
            vel, omega = p.getBaseVelocity(body, physicsClientId=cid)
            return tuple(pos)+tuple(rot)+tuple(vel)+tuple(omega)
        finally:
            p.disconnect(physicsClientId=cid)
    a, b = run(), run()
    drift = max(abs(x-y) for x,y in zip(a,b))
    return PhysicsCase(
        "isolated_client_repeatability",
        {"max_absolute_state_delta":drift, "state_dimensions":len(a)},
        {"max_absolute_state_delta":0.},
        {"max_absolute_state_delta":1e-8},
        drift <= 1e-8,
        "Determinism in this one simple fixture does not imply physical accuracy "
        "or bitwise determinism across different CPUs/drivers.",
    )


def run_physics_suite() -> dict:
    cases = [free_fall(), ground_contact(), torque_and_inertia(), repeatability()]
    return {
        "solver":"bullet_cpu",
        "fixed_timestep_s":PHYSICS_DT,
        "cases":[x.as_dict() for x in cases],
        "passed":all(x.passed for x in cases),
        "case_count":len(cases),
        "claims":"Only named analytic reference fixtures pass or fail; no OEM qualification.",
    }
