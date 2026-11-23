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


def execute_imported_trajectory(world, plan: ImportedTrajectory,
                                *, record_every: int = 24) -> dict:
    if type(record_every) is not int or not 1<=record_every<=500:
        raise ValueError("Recorded sampling interval must be 1..500")
    robot=world.imported_robots.get(plan.asset_id)
    if not robot:
        raise ValueError("Planned URDF robot is no longer available")
    if not math.isclose(plan.timestep_s,PHYSICS_DT,abs_tol=1e-12):
        raise ValueError("Plan and Bullet timestep differ")
    actual={j["name"]:j for j in robot["joints"] if j["lower"] is not None}
    if tuple(actual)!=plan.names:
        raise ValueError("Robot articulation changed since planning")
    if any(actual[name]["joint_index"]!=index
           for name,index in zip(plan.names,plan.joint_indices)):
        raise ValueError("Motor joint indices changed since planning")
    now=tuple(float(world.p.getJointState(
        robot["body_id"],idx,physicsClientId=world.client
    )[0]) for idx in plan.joint_indices)
    if max(abs(x-y) for x,y in zip(now,plan.start))>1e-3:
        raise ValueError("Robot measured start state has changed; replan before execution")
    worst=0.
    errors=[]
    records=[]
    high_contacts=0
    violations=[]
    max_measured_speed={name:0. for name in plan.names}
    max_measured_effort={name:0. for name in plan.names}
    speed_violations=[]
    effort_violations=[]
    for i,targets in enumerate(plan.samples[1:],1):
        for name,target in zip(plan.names,targets):
            world.command_imported_joint(plan.asset_id,name,target)
        world.p.stepSimulation(physicsClientId=world.client)
        world.step_index+=1
        states=tuple(world.p.getJointState(
            robot["body_id"],idx,physicsClientId=world.client
        ) for idx in plan.joint_indices)
        current=tuple(float(state[0]) for state in states)
        for name,state in zip(plan.names,states):
            measured_speed=abs(float(state[1]))
            measured_effort=abs(float(state[3]))
            if not math.isfinite(measured_speed) or not math.isfinite(measured_effort):
                raise ArithmeticError("Bullet motor feedback contains nonfinite velocity/effort")
            max_measured_speed[name]=max(max_measured_speed[name],measured_speed)
            max_measured_effort[name]=max(max_measured_effort[name],measured_effort)
            limits=actual[name]
            if measured_speed>limits["max_velocity"]*1.05+.001:
                speed_violations.append({
                    "step":i,"name":name,"measured_speed":measured_speed,
                    "declared_limit":limits["max_velocity"],
                })
            if measured_effort>limits["max_effort"]*1.05+.001:
                effort_violations.append({
                    "step":i,"name":name,"measured_effort":measured_effort,
                    "declared_limit":limits["max_effort"],
                })
        if any(not math.isfinite(x) for x in current):
            raise ArithmeticError("Bullet motor yielded nonfinite state")
        error=max(abs(ref-got) for ref,got in zip(targets,current))
        errors.append(error)
        worst=max(worst,error)
        contacts=len(world.p.getContactPoints(
            bodyA=robot["body_id"],physicsClientId=world.client,
        ))
        high_contacts=max(high_contacts,contacts)
        for name,q in zip(plan.names,current):
            j=actual[name]
            if j["lower"]<=j["upper"] and (
                q<j["lower"]-.01 or q>j["upper"]+.01
            ):
                violations.append({"step":i,"name":name,"position":q})
        if i % record_every==0 or i==len(plan.samples)-1:
            records.append({
                "step":i,"time_s":i*PHYSICS_DT,
                "measured_joint_states":dict(zip(plan.names,current)),
                "reference_joint_states":dict(zip(plan.names,targets)),
                "max_tracking_error":error,
            })
    ordered=sorted(errors)
    p95=ordered[math.ceil(.95*len(ordered))-1] if ordered else 0.
    return {
        "asset_id":plan.asset_id,
        "simulated_steps":len(plan.samples)-1,
        "duration_s":plan.duration_s,
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
        "torque_acceleration_qualified":False,
        "physical_hardware_qualified":False,
        "note":"Real Bullet measured motor tracking, not manufacturer motion qualification.",
    }
