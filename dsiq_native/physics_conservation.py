"""Quantitative conservation laws checked on actual isolated Bullet states.

No claim of universal energy conservation in impact/friction/motor cases:
the tests are intentionally constrained to systems where the relevant
invariant follows analytically from the declared forces and damping.
"""
from __future__ import annotations

import math

from .physics_benchmarks import PhysicsCase, _box, _client


def gravitational_energy_drift() -> PhysicsCase:
    """Refine integration timestep and measure mechanical energy residual."""
    mass=1.6
    gravity=9.81
    initial_height=3.
    initial_energy=mass*gravity*initial_height
    observations=[]
    for hz in (120,240,480):
        dt=1./hz
        p,cid=_client(gravity,timestep_s=dt)
        try:
            body=_box(
                p,cid,mass=mass,side=.12,position=(0.,0.,initial_height)
            )
            for _ in range(hz//2):
                p.stepSimulation(physicsClientId=cid)
            xyz=p.getBasePositionAndOrientation(
                body,physicsClientId=cid
            )[0]
            velocity=p.getBaseVelocity(body,physicsClientId=cid)[0]
            potential=mass*gravity*xyz[2]
            kinetic=.5*mass*sum(v*v for v in velocity)
            observed=kinetic+potential
            observations.append({
                "frequency_hz":hz,
                "dt_s":dt,
                "energy_joules":observed,
                "potential_joules":potential,
                "kinetic_joules":kinetic,
                "signed_residual_J":observed-initial_energy,
                "abs_error_J":abs(observed-initial_energy),
                "relative_abs_error":abs(observed-initial_energy)/initial_energy,
            })
        finally:
            p.disconnect(physicsClientId=cid)
    err=[x["abs_error_J"] for x in observations]
    improvement=err[0]>err[1]>err[2]
    successful=(improvement and err[2]<.25 and
                all(math.isfinite(x["energy_joules"]) for x in observations))
    return PhysicsCase(
        name="gravity_energy_refinement_no_contact",
        observed={
            "samples":observations,
            "strict_error_reduction":improvement,
            "finest_abs_energy_error_J":err[2],
        },
        reference={
            "initial_total_energy_J":initial_energy,
            "analytic_constant_mechanical_energy":True,
            "energy_definition":"m*g*z + (1/2)*m*|linear_velocity|^2",
        },
        tolerances={"finest_abs_energy_error_J":.25,
                    "error_reduces_on_refinement":True},
        passed=successful,
        explanation=(
            "Tests no-contact free-fall integration energy only. "
            "Semi-implicit Bullet time integration is not analytically exact. "
            "Does not imply energy conservation under damping, friction, "
            "motors, impacts, joints, or arbitrary material forces."
        ),
    )


def force_free_linear_momentum() -> PhysicsCase:
    """Verify isolated linear momentum with zero gravity/damping/contact."""
    p,cid=_client(0.)
    mass=2.5
    start_velocity=(1.2,-.4,.15)
    try:
        body=_box(p,cid,mass=mass,side=.10,position=(0.,0.,2.))
        p.resetBaseVelocity(
            body,linearVelocity=list(start_velocity),physicsClientId=cid
        )
        for _ in range(120):
            p.stepSimulation(physicsClientId=cid)
        velocity=p.getBaseVelocity(
            body,physicsClientId=cid
        )[0]
        expected=tuple(mass*v for v in start_velocity)
        measured=tuple(mass*v for v in velocity)
        residual=math.dist(expected,measured)
        return PhysicsCase(
            name="isolated_linear_momentum_no_external_forces",
            observed={
                "initial_momentum_kg_m_s":list(expected),
                "final_momentum_kg_m_s":list(measured),
                "absolute_momentum_error_kg_m_s":residual,
            },
            reference={
                "linear_momentum_conserved":True,
                "external_gravity_m_s2":0.,
                "linear_damping":0.,
                "contacts":0,
            },
            tolerances={"absolute_momentum_error_kg_m_s":1e-5},
            passed=math.isfinite(residual) and residual<1e-5,
            explanation=(
                "Free rigid body with no gravity, damping or contacts only; "
                "not a general industrial conservation guarantee."
            ),
        )
    finally:
        p.disconnect(physicsClientId=cid)


def run_conservation_suite() -> dict:
    cases=[gravitational_energy_drift(),force_free_linear_momentum()]
    return {
        "schema":"dsiq-bullet-conservation-v1",
        "engine":"pybullet_cpu",
        "cases":[c.as_dict() for c in cases],
        "case_count":len(cases),
        "passed":all(c.passed for c in cases),
        "independent_physical_qualification":False,
    }
