"""Evidence-driven contact, friction and articulated motor physics tests.

These are independent Bullet C++ experiments. They test *bounded reference*
behaviour on isolated scenes, never accuracy of proprietary industrial
equipment or equivalence to NVIDIA PhysX.

All dynamic experiments are run in headless CPU Bullet (DIRECT).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
from tempfile import TemporaryDirectory

from .bullet_runtime import BulletPlayground, BulletSettings
from .machine_designer import MachineDesign, save_project
from .physics_benchmarks import PhysicsCase, _box, _client


def _ground(p, cid, *, friction: float = .7):
    shape = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[3., 3., .1], physicsClientId=cid
    )
    ground = p.createMultiBody(
        baseMass=0., baseCollisionShapeIndex=shape,
        basePosition=[0., 0., -.1], physicsClientId=cid,
    )
    p.changeDynamics(
        ground, -1, lateralFriction=friction, restitution=0.,
        physicsClientId=cid,
    )
    return ground


def _settling_trial(hz: int) -> dict:
    p, cid = _client(9.81, timestep_s=1. / hz)
    try:
        ground = _ground(p, cid)
        body = _box(
            p, cid, mass=1.0, side=.2, position=(0., 0., .8)
        )
        p.changeDynamics(
            body, -1, lateralFriction=.7, restitution=0.,
            physicsClientId=cid,
        )
        for _ in range(2 * hz):
            p.stepSimulation(physicsClientId=cid)
        xyz = p.getBasePositionAndOrientation(
            body, physicsClientId=cid
        )[0]
        linear = p.getBaseVelocity(body, physicsClientId=cid)[0]
        contacts = p.getContactPoints(
            bodyA=body, bodyB=ground, physicsClientId=cid,
        )
        # Bullet's contact tuple reports signed separation and normal force.
        normal_force = sum(max(0., float(cp[9])) for cp in contacts)
        max_penetration = max(
            (max(0., -float(cp[8])) for cp in contacts), default=0.
        )
        return {
            "hz":hz, "final_height_m":float(xyz[2]),
            "height_error_m":abs(float(xyz[2]) - .1),
            "vertical_speed_m_s":float(linear[2]),
            "contacts":len(contacts),
            "max_contact_penetration_m":max_penetration,
            "reported_normal_force_N":normal_force,
        }
    finally:
        p.disconnect(physicsClientId=cid)


def rigid_contact_refinement() -> PhysicsCase:
    """Compare contact stability at 120/240/480 Hz, with no false monotonic claim."""
    samples = [_settling_trial(hz) for hz in (120, 240, 480)]
    errors = [v["height_error_m"] for v in samples]
    heights = [v["final_height_m"] for v in samples]
    velocity = [abs(v["vertical_speed_m_s"]) for v in samples]
    penetration = [v["max_contact_penetration_m"] for v in samples]
    # Contact is nonsmooth; demanding strictly monotonic pointwise
    # convergence would be mathematically unjustified.
    ok = (
        all(v["contacts"] >= 1 for v in samples)
        and max(errors) < .025
        and max(velocity) < .10
        and max(penetration) < .025
        and max(heights) - min(heights) < .025
    )
    return PhysicsCase(
        name="resting_contact_timestep_stability",
        observed={
            "samples":samples,
            "max_height_error_m":max(errors),
            "height_spread_between_timesteps_m":max(heights)-min(heights),
            "max_abs_vertical_speed_m_s":max(velocity),
            "max_penetration_m":max(penetration),
        },
        reference={
            "reference_rest_center_height_m":.1,
            "rigid_support_surface_z_m":0.,
            "gravity_m_s2":9.81,
        },
        tolerances={
            "max_height_error_m":.025,
            "height_spread_between_timesteps_m":.025,
            "max_abs_vertical_speed_m_s":.10,
            "max_penetration_m":.025,
            "minimum_contact_points":1,
        },
        passed=ok,
        explanation=(
            "Quantifies bounded support-contact stability across fixed timesteps. "
            "Nonsmooth contact does not imply monotonic convergence. "
            "Not a proof of accurate contact force, friction, restitution, "
            "stiffness, wear, or real machine safety."
        ),
    )


def _sliding_trial(mu: float) -> dict:
    p, cid = _client(9.81)
    try:
        ground = _ground(p, cid, friction=mu)
        body = _box(
            p, cid, mass=1., side=.2, position=(0., 0., .1005)
        )
        p.changeDynamics(
            body, -1, lateralFriction=mu, restitution=0.,
            physicsClientId=cid,
        )
        p.resetBaseVelocity(
            body, linearVelocity=[2.,0.,0.], physicsClientId=cid,
        )
        for _ in range(240):
            p.stepSimulation(physicsClientId=cid)
        position = p.getBasePositionAndOrientation(
            body, physicsClientId=cid
        )[0]
        velocity = p.getBaseVelocity(
            body, physicsClientId=cid
        )[0]
        contacts = p.getContactPoints(
            bodyA=body, bodyB=ground, physicsClientId=cid,
        )
        return {
            "material_lateral_friction_parameter":mu,
            "x_displacement_m":float(position[0]),
            "final_horizontal_speed_m_s":math.hypot(
                float(velocity[0]),float(velocity[1])
            ),
            "contact_points":len(contacts),
        }
    finally:
        p.disconnect(physicsClientId=cid)


def sliding_friction_response() -> PhysicsCase:
    """Measure, rather than assume, lower travel with higher contact friction."""
    low = _sliding_trial(.03)
    high = _sliding_trial(.90)
    travel_delta = low["x_displacement_m"] - high["x_displacement_m"]
    speed_delta = (
        low["final_horizontal_speed_m_s"]
        - high["final_horizontal_speed_m_s"]
    )
    ok = (
        all(math.isfinite(v) for v in (
            travel_delta, speed_delta, low["x_displacement_m"],
            high["x_displacement_m"]
        ))
        and low["contact_points"] >= 1
        and high["contact_points"] >= 1
        and travel_delta > .08
        and speed_delta > .05
    )
    return PhysicsCase(
        name="sliding_friction_material_sensitivity",
        observed={
            "low_friction":low, "high_friction":high,
            "travel_reduction_m":travel_delta,
            "speed_reduction_m_s":speed_delta,
        },
        reference={
            "same_initial_horizontal_speed_m_s":2.,
            "gravity_m_s2":9.81,
            "expected_direction":"greater friction reduces travel and speed",
        },
        tolerances={
            "minimum_travel_reduction_m":.08,
            "minimum_speed_reduction_m_s":.05,
            "contacts_per_trial_minimum":1,
        },
        passed=ok,
        explanation=(
            "Tests directional material sensitivity in one sliding box-on-floor "
            "fixture; does not calibrate Coulomb coefficient, stiction, or "
            "rolling/anisotropic friction on actual machines."
        ),
    )


def imported_motor_limits() -> PhysicsCase:
    """Actual user-authored prismatic motor respects travel/speed input gates."""
    with TemporaryDirectory() as folder:
        manifest = save_project(
            MachineDesign(asset_id="physics_motor_fixture",
                          x_travel_m=.6,y_travel_m=.5,z_travel_m=.2),
            Path(folder)/"gantry",
        )["manifest"]
        with BulletPlayground(BulletSettings(
            payload_count=0, gravity_m_s2=0.,
        )) as world:
            import_result = world.import_urdf_robot(
                manifest,base_position_m=(0.,2.,0.)
            )
            joint = next(j for j in import_result["joints"]
                         if j["name"]=="actuator_x")
            max_velocity = joint["max_velocity"]
            effort = joint["max_effort"]
            rejection = False
            try:
                world.command_imported_joint(
                    "physics_motor_fixture","actuator_x",.75,
                )
            except ValueError:
                rejection = True
            world.command_imported_joint(
                "physics_motor_fixture","actuator_x",.4,
            )
            state = world.step(60)
            actuator = next(j for j in state["imported_robots"][0]["joint_states"]
                            if j["name"]=="actuator_x")
            position = actuator["position"]
            # Simulated 0.25 s, source URDF vmax = 0.8 m/s.
            # Generous tolerance to admit finite stepping/discrete controllers,
            # but reject instant travel to the 0.4m goal.
            no_teleport = 0. <= position <= max_velocity*.25 + .08
            moved = position > .005
            passed = (
                rejection and moved and no_teleport and
                max_velocity > 0 and effort > 0
            )
            return PhysicsCase(
                name="imported_prismatic_motor_constraints",
                observed={
                    "position_m_after_60_steps":position,
                    "simulated_seconds":60/240.,
                    "out_of_range_command_rejected":rejection,
                    "source_max_velocity_m_s":max_velocity,
                    "source_max_effort_N":effort,
                    "motor_moved":moved,
                },
                reference={
                    "commanded_position_m":.4,
                    "mechanical_x_travel_m":.6,
                    "maximum_source_speed_m_s":max_velocity,
                },
                tolerances={
                    "max_position_m_after_60_steps":max_velocity*.25+.08,
                    "minimum_observed_motion_m":.005,
                },
                passed=passed,
                explanation=(
                    "Uses actual URDF declared motor effort/velocity and Bullet "
                    "joint position. A command gate and short motor response "
                    "check do not validate motor current, torque limits under load, "
                    "joint encoders, controller safety or hardware equivalence."
                ),
            )


def run_extended_physics_suite() -> dict:
    cases = [
        rigid_contact_refinement(),
        sliding_friction_response(),
        imported_motor_limits(),
    ]
    return {
        "schema":"dsiq-physics-extended-v1",
        "solver":"pybullet_cpu",
        "cases":[c.as_dict() for c in cases],
        "case_count":len(cases),
        "passed":all(c.passed for c in cases),
        "independent_hardware_validation":False,
        "note":"Reference-only finite tolerance tests, not industrial physics certification.",
    }
