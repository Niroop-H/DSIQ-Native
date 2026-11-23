"""Joint-space motion for any verified imported URDF with bounded motors.

Works with revolute/prismatic mixed axes and fixed joints, not just
DSIQ's own 6R robot. Quintic C2 trajectory uses source URDF velocity
limits, measured Bullet start states and actual motor feedback.

IMPORTANT: No acceleration/torque limits are inferred from missing URDF
data. Paths have no swept-collision, dynamic load or hardware certification.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from .bullet_runtime import PHYSICS_DT

MAX_STEPS = 7200
_SLOPE = 1.875


@dataclass(frozen=True)
class ImportedTrajectory:
    asset_id: str
    names: tuple[str,...]
    joint_indices: tuple[int,...]
    start: tuple[float,...]
    goal: tuple[float,...]
    duration_s: float
    timestep_s: float
    samples: tuple[tuple[float,...],...]
    declared_speed_limits: tuple[float,...]

    def summary(self) -> dict:
        return {
            "asset_id":self.asset_id,
            "joint_names":list(self.names),
            "start":list(self.start),
            "goal":list(self.goal),
            "duration_s":self.duration_s,
            "dt_s":self.timestep_s,
            "sample_count":len(self.samples),
            "speed_limits_from_source_urdf":list(self.declared_speed_limits),
            "speed_bounded_analytically":True,
            "acceleration_qualified":False,
            "collision_checked":False,
            "torque_qualified":False,
            "physically_calibrated":False,
        }


def plan_imported_trajectory(world, asset_id: str, goals: Mapping[str,float],
                             *, duration_s: float | None = None) -> ImportedTrajectory:
    if not isinstance(goals,Mapping) or not goals:
        raise ValueError("At least one named URDF joint target is required")
    robot=world.imported_robots.get(asset_id)
    if not robot:
        raise ValueError("Imported robot asset not present in this Bullet world")
    actuators=[j for j in robot["joints"] if j["lower"] is not None]
    if not actuators:
        raise ValueError("Imported robot has no actuated joints")
    if set(goals)-{j["name"] for j in actuators}:
        raise ValueError("Target includes an unknown, fixed or ambiguous joint")
    if len(actuators)>63:
        raise ValueError("Imported trajectory exceeds bounded joint budget")

    start=[]
    goal=[]
    speed=[]
    names=[]
    indices=[]
    for joint in actuators:
        q=float(world.p.getJointState(
            robot["body_id"],joint["joint_index"],physicsClientId=world.client
        )[0])
        if not math.isfinite(q):
            raise ValueError("Bullet source joint state is nonfinite")
        name=joint["name"]
        target=goals.get(name,q)
        if type(target) not in (int,float) or not math.isfinite(target):
            raise ValueError(f"Joint {name}: goal must be finite numeric")
        target=float(target)
        lo,hi=joint["lower"],joint["upper"]
        if lo<=hi and not lo-1e-9<=target<=hi+1e-9:
            raise ValueError(f"Joint {name}: goal outside URDF travel limits")
        if abs(target)>8*math.pi:
            raise ValueError(f"Joint {name}: exceeds conservative absolute motor bound")
        v=joint["max_velocity"]
        if not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:
            raise ValueError(f"Joint {name}: missing real URDF speed rating")
        start.append(q)
        goal.append(target)
        speed.append(float(v))
        names.append(name)
        indices.append(int(joint["joint_index"]))
    required=max(
        .2,
        *(_SLOPE*abs(b-a)/v for a,b,v in zip(start,goal,speed)),
    )
    if duration_s is None:
        duration=required*1.02
    else:
        if type(duration_s) not in (int,float) or not math.isfinite(duration_s):
            raise ValueError("Duration must be a finite number")
        duration=float(duration_s)
        if duration<required:
            raise ValueError(
                f"Duration is shorter than the verified URDF velocity-bound minimum ({required:.6f}s)"
            )
    if duration>30.:
        raise ValueError("Imported URDF trajectory exceeds 30s bounded memory budget")
    ticks=int(math.ceil(duration/PHYSICS_DT))
    if ticks<1 or ticks+1>MAX_STEPS:
        raise ValueError("Imported trajectory exceeds bounded 7200-sample budget")
    T=ticks*PHYSICS_DT
    samples=[]
    for i in range(ticks+1):
        u=i/ticks
        fraction=10*u**3-15*u**4+6*u**5
        q=tuple(a+(b-a)*fraction for a,b in zip(start,goal))
        samples.append(q)
    for a,b,v in zip(start,goal,speed):
        if abs(b-a)*_SLOPE/T>v+1e-9:
            raise ArithmeticError("Quintic duration violates the source motor velocity limit")
    return ImportedTrajectory(
        asset_id,tuple(names),tuple(indices),tuple(start),
        tuple(goal),T,PHYSICS_DT,tuple(samples),tuple(speed),
    )



def validate_execution_plan(world, plan: ImportedTrajectory) -> dict:
    """Reject altered or decimated motor programs before issuing ANY commands.

    Screening may use a decimated path; execution may not. The executable
    path must be the original fixed-timestep quintic reference and must
    match the CURRENT imported URDF and live start posture. This is a
    software integrity gate, not a trusted-signature or safety certificate.
    """
    from .motion_collision import _validate_path_against_loaded_robot
    robot = world.imported_robots.get(plan.asset_id)
    if robot is None:
        raise ValueError("Planned URDF robot is no longer available")
    _validate_path_against_loaded_robot(world, robot, plan)
    if not math.isclose(plan.timestep_s, PHYSICS_DT, rel_tol=0., abs_tol=1e-12):
        raise ValueError("Execution plan and Bullet fixed timestep differ")
    n = len(plan.samples) - 1
    if n < 1 or n + 1 > MAX_STEPS or not math.isclose(
        plan.duration_s, n * PHYSICS_DT, rel_tol=0., abs_tol=1e-9
    ):
        raise ValueError("Execution plan sample count, duration and timestep disagree")
    for i, row in enumerate(plan.samples):
        u = i / n
        blend = 10*u**3 - 15*u**4 + 6*u**5
        for q, start, goal in zip(row, plan.start, plan.goal):
            expected = start + (goal-start)*blend
            if abs(q-expected) > 2e-8 * max(1., abs(start), abs(goal)):
                raise ValueError("Execution trajectory differs from authorized quintic samples")
    for start, goal, vmax in zip(
        plan.start, plan.goal, plan.declared_speed_limits
    ):
        if 1.875 * abs(goal-start) / plan.duration_s > vmax + 1e-9:
            raise ValueError("Execution reference path exceeds URDF speed limit")
    return {
        "fixed_step_quintic_integrity_checked": True,
        "all_samples_match_plan": True,
        "actual_robot_start_state_checked": True,
        "source_urdf_velocity_ratings_checked": True,
        "safety_rated": False,
    }


def _request_simulation_brake(world, plan, robot) -> bool:
    """Request zero joint velocity in Bullet after a software fault.

    This is an ordinary physics-motor command. It is NOT a protective stop
    or a hardware E-stop, and the state may still coast or move under load.
    """
    p = world.p
    for name, index in zip(plan.names, plan.joint_indices):
        j = next(j for j in robot["joints"] if j["name"] == name)
        p.setJointMotorControl2(
            robot["body_id"], index, p.VELOCITY_CONTROL,
            targetVelocity=0., force=float(j["max_effort"]),
            physicsClientId=world.client,
        )
    return True


def execute_imported_trajectory(world, plan: ImportedTrajectory,
                                *, record_every: int = 24,
                                runtime_clearance_m: float | None = None,
                                max_tracking_error: float | None = None) -> dict:
    """Execute verified quintic targets with a post-step simulation fault monitor.

    Monitors velocity, effort and hard travel limits from the imported URDF,
    plus optional real-time geometric clearance and user-set tracking error.
    Any violation aborts future target commands and requests a Bullet brake.
    The violating simulation step has ALREADY occurred; this cannot protect
    humans or real actuators, nor provide continuous swept collision safety.
    """
    if type(record_every) is not int or not 1 <= record_every <= 500:
        raise ValueError("Recorded sampling interval must be 1..500")
    if runtime_clearance_m is not None and (
        type(runtime_clearance_m) not in (int,float) or
        not math.isfinite(runtime_clearance_m) or
        not 0. <= runtime_clearance_m <= .05
    ):
        raise ValueError("Runtime clearance must be a finite value in [0,0.05] m")
    if max_tracking_error is not None and (
        type(max_tracking_error) not in (int,float) or
        not math.isfinite(max_tracking_error) or
        not 0 < max_tracking_error <= 1.
    ):
        raise ValueError("Tracking-error threshold must be finite in (0,1]")
    verification = validate_execution_plan(world, plan)
    robot = world.imported_robots[plan.asset_id]
    actual = {
        j["name"]:j for j in robot["joints"] if j["lower"] is not None
    }
    body = robot["body_id"]
    p = world.p
    cid = world.client
    worst = 0.
    errors = []
    records = []
    high_contacts = 0
    violations = []
    max_measured_speed = {name:0. for name in plan.names}
    max_measured_effort = {name:0. for name in plan.names}
    speed_violations = []
    effort_violations = []
    abort_reason = None
    post_step_neutralization_requested = False
    current = tuple(plan.start)
    completed_steps = 0
    monitored_contacts = 0
    for i, targets in enumerate(plan.samples[1:], 1):
        # The preflight above is executed before the first motor command.
        for name, target in zip(plan.names, targets):
            world.command_imported_joint(plan.asset_id, name, target)
        p.stepSimulation(physicsClientId=cid)
        world.step_index += 1
        completed_steps += 1
        states = tuple(
            p.getJointState(body, idx, physicsClientId=cid)
            for idx in plan.joint_indices
        )
        current = tuple(float(state[0]) for state in states)
        readings = [
            float(value) for state in states
            for value in (state[0], state[1], state[3])
        ]
        if not all(math.isfinite(value) for value in readings):
            _request_simulation_brake(world, plan, robot)
            raise ArithmeticError(
                "Bullet produced nonfinite joint feedback; simulation motor targets halted"
            )
        for name, state in zip(plan.names, states):
            measured_speed = abs(float(state[1]))
            measured_effort = abs(float(state[3]))
            max_measured_speed[name] = max(
                max_measured_speed[name], measured_speed
            )
            max_measured_effort[name] = max(
                max_measured_effort[name], measured_effort
            )
            limits = actual[name]
            if measured_speed > limits["max_velocity"]*1.05 + .001:
                speed_violations.append({
                    "step":i, "name":name,
                    "measured_speed":measured_speed,
                    "declared_limit":limits["max_velocity"],
                })
            if measured_effort > limits["max_effort"]*1.05 + .001:
                effort_violations.append({
                    "step":i, "name":name,
                    "measured_effort":measured_effort,
                    "declared_limit":limits["max_effort"],
                })
        error = max(abs(ref-got) for ref, got in zip(targets,current))
        errors.append(error)
        worst = max(worst,error)
        contacts = len(p.getContactPoints(
            bodyA=body,physicsClientId=cid,
        ))
        high_contacts = max(high_contacts, contacts)
        for name,q in zip(plan.names,current):
            j = actual[name]
            if j["lower"] <= j["upper"] and (
                q < j["lower"]-.01 or q > j["upper"]+.01
            ):
                violations.append({
                    "step":i,"name":name,"position":q,
                })
        if runtime_clearance_m is not None:
            # This queries ACTUAL Bullet state AFTER a step and sees moving
            # obstacles that could differ from the preflight's static scene.
            # It cannot prevent first contact, and self-collision is excluded.
            for k in range(p.getNumBodies(physicsClientId=cid)):
                other = p.getBodyUniqueId(k,physicsClientId=cid)
                if other == body:
                    continue
                points = p.getClosestPoints(
                    bodyA=body, bodyB=other,
                    distance=max(float(runtime_clearance_m),1e-6),
                    physicsClientId=cid,
                )
                for cp in points:
                    link = int(cp[3])
                    separation = float(cp[8])
                    if other == world.ground and link == -1:
                        continue  # Fixed root on the support plane
                    if not math.isfinite(separation):
                        abort_reason = "NONFINITE_CONTACT_DISTANCE"
                        break
                    if separation <= runtime_clearance_m:
                        monitored_contacts += 1
                        abort_reason = (
                            "POST_STEP_OBSTACLE_CLEARANCE_VIOLATION"
                            f" body={other} link={link} distance={separation:.9g}"
                        )
                        break
                if abort_reason:
                    break
        if speed_violations:
            abort_reason = abort_reason or "SOURCE_JOINT_VELOCITY_VIOLATION"
        if effort_violations:
            abort_reason = abort_reason or "SOURCE_MOTOR_EFFORT_VIOLATION"
        if violations:
            abort_reason = abort_reason or "SOURCE_JOINT_TRAVEL_VIOLATION"
        if max_tracking_error is not None and error > max_tracking_error:
            abort_reason = abort_reason or "TRACKING_ERROR_THRESHOLD_VIOLATION"
        if i % record_every == 0 or i == len(plan.samples)-1 or abort_reason:
            records.append({
                "step":i,"time_s":i*PHYSICS_DT,
                "measured_joint_states":dict(zip(plan.names,current)),
                "reference_joint_states":dict(zip(plan.names,targets)),
                "max_tracking_error":error,
            })
        if abort_reason:
            post_step_neutralization_requested = _request_simulation_brake(
                world,plan,robot,
            )
            break  # Do not command any later samples
    ordered = sorted(errors)
    p95 = ordered[math.ceil(.95*len(ordered))-1] if ordered else 0.
    completed = abort_reason is None and completed_steps == len(plan.samples)-1
    return {
        "asset_id":plan.asset_id,
        "source_urdf_sha256":robot.get("source_urdf_sha256"),
        "joint_units_from_loaded_urdf":{
            name:("m" if actual[name]["type"]==p.JOINT_PRISMATIC else "rad")
            for name in plan.names
        },
        "execution_preflight":verification,
        "simulated_steps":completed_steps,
        "planned_steps":len(plan.samples)-1,
        "duration_s":plan.duration_s,
        "actual_simulated_duration_s":completed_steps*PHYSICS_DT,
        "runtime_monitor_passed":completed,
        "trajectory_completed":completed,
        "runtime_abort_reason":abort_reason,
        "post_step_neutralization_requested":post_step_neutralization_requested,
        "post_step_clearance_threshold_m":runtime_clearance_m,
        "post_step_obstacle_contact_events":monitored_contacts,
        "max_tracking_error_threshold":max_tracking_error,
        "monitor_type":"POST_STEP_SOFTWARE_MONITOR_NOT_HARDWARE_SAFETY",
        "max_joint_tracking_error":worst,
        "p95_joint_tracking_error":p95,
        "final_actual":dict(zip(plan.names,current)),
        "final_requested":dict(zip(plan.names,plan.goal)),
        "max_robot_contact_points_seen":high_contacts,
        "joint_limit_violations":violations,
        "max_measured_joint_speed":max_measured_speed,
        "max_solver_reported_motor_effort":max_measured_effort,
        "velocity_limit_violations":speed_violations,
        "motor_effort_limit_violations":effort_violations,
        "actual_load_profile_independently_verified":False,
        "sampled_reports":records,
        "source_velocity_limits_used":True,
        "collision_free_certified":False,
        "continuous_collision_detection_qualified":False,
        "self_collision_qualified":False,
        "torque_acceleration_qualified":False,
        "physical_hardware_qualified":False,
        "note":(
            "Post-step software monitor halts further simulated target commands "
            "and requests a Bullet brake on faults. It cannot prevent the first "
            "violating step or act as a physical protective stop."
        ),
    }
