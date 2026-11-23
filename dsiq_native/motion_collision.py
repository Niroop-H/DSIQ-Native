"""Fail-closed DISCRETE collision screening of imported robot joint paths.

This checks real Bullet collision geometry at each sampled joint state,
without advancing simulation time. It snapshots/restores the physics
world so editing/planning does not teleport or corrupt the live robot.

IMPORTANT: even adaptively subdivided joint sampling can miss swept
or tunneling collisions between samples. Adjacent self-collisions and
continuous-time safety remain UNQUALIFIED. No real hardware command is issued from this module.
"""
from __future__ import annotations

import math

from .imported_trajectory import ImportedTrajectory

MAX_REPORTED_OBSTACLES=24


def screen_imported_path(world, plan: ImportedTrajectory,
                         *, clearance_m: float = .003,
                         max_joint_step: float = .04,
                         max_total_samples: int = 50000) -> dict:
    if type(clearance_m) not in (float,int) or not math.isfinite(clearance_m):
        raise ValueError("Clearance must be finite numeric")
    if not 0.<=clearance_m<=.05:
        raise ValueError("Discrete clearance margin must be between 0 and 0.05 meters")
    if type(max_joint_step) not in (int,float) or not math.isfinite(max_joint_step) or not .001<=max_joint_step<=.25:
        raise ValueError("Adaptive max_joint_step must be finite in [.001,.25] joint units")
    if type(max_total_samples) is not int or not 100<=max_total_samples<=100000:
        raise ValueError("Adaptive screening budget must be 100..100000 samples")
    robot=world.imported_robots.get(plan.asset_id)
    if not robot:
        raise ValueError("URDF robot not present; cannot inspect nonexistent geometry")
    if tuple(j["name"] for j in robot["joints"] if j["lower"] is not None)!=plan.names:
        raise ValueError("Plan mechanism identity differs from currently imported robot")
    if len(plan.samples)<2 or len(plan.samples)>7200:
        raise ValueError("Joint trajectory exceeds bounded collision sampling budget")
    # Interpolate every segment in joint coordinates rather than trusting
    # only externally supplied coarse samples. No single global angular
    # threshold guarantees Cartesian link clearance for all geometries.
    substeps=[]
    expanded=1
    for start_joint,end_joint in zip(plan.samples,plan.samples[1:]):
        count=max(1,math.ceil(max(
            abs(float(b)-float(a)) for a,b in zip(start_joint,end_joint)
        ) / max_joint_step))
        if expanded+count>max_total_samples:
            raise ValueError("Adaptive collision sample budget exhausted: refuse unsafe plan")
        expanded+=count
        substeps.append(count)
    p=world.p
    cid=world.client
    body=robot["body_id"]
    obstacles=[
        p.getBodyUniqueId(i,physicsClientId=cid)
        for i in range(p.getNumBodies(physicsClientId=cid))
    ]
    obstacles=[item for item in obstacles if item!=body]
    state=p.saveState(physicsClientId=cid)
    if state<0:
        raise RuntimeError("Bullet cannot snapshot collision-screening world")
    tested=0
    first=None
    close_contacts=[]
    try:
        # Explicit time samples include the initial posture and all
        # interpolated intermediate postures. We check the actual contact
        # geometry at each, without simulating motors.
        interpolation=[(0.,plan.samples[0])]
        for segment,(a,b,count) in enumerate(
            zip(plan.samples,plan.samples[1:],substeps),1
        ):
            for local in range(1,count+1):
                alpha=local/count
                q=tuple(x+(y-x)*alpha for x,y in zip(a,b))
                interpolation.append((segment-1+alpha,q))
        for fractional_index,angles in interpolation:
            for joint,value in zip(plan.joint_indices,angles):
                p.resetJointState(
                    body,joint,targetValue=float(value),targetVelocity=0.,
                    physicsClientId=cid,
                )
            p.performCollisionDetection(physicsClientId=cid)
            tested+=1
            for other in obstacles:
                contacts=p.getClosestPoints(
                    bodyA=body,bodyB=other,distance=max(clearance_m,1e-6),
                    physicsClientId=cid,
                )
                for cp in contacts:
                    # Base frames are allowed to sit on the facility
                    # support ground; actual movable links are NOT exempt.
                    link_a=int(cp[3])
                    if other==world.ground and link_a==-1:
                        continue
                    separation=float(cp[8])
                    if not math.isfinite(separation) or separation>clearance_m:
                        continue
                    obstacle={
                        "sample_index":fractional_index,
                        "time_s":fractional_index*plan.timestep_s,
                        "other_body_id":int(other),
                        "joint_link_index":link_a,
                        "contact_signed_separation_m":separation,
                    }
                    if first is None:
                        first=obstacle
                    if len(close_contacts)<MAX_REPORTED_OBSTACLES:
                        close_contacts.append(obstacle)
            if first is not None:
                # Fail fast; never claim later untested samples are clear.
                break
    finally:
        try:
            p.restoreState(stateId=state,physicsClientId=cid)
        finally:
            p.removeState(state,physicsClientId=cid)
    return {
        "asset_id":plan.asset_id,
        "collision_screen_passed":first is None,
        "evaluated_samples":tested,
        "total_samples":len(plan.samples),
        "adaptive_sample_count":expanded,
        "max_joint_step":max_joint_step,
        "subdivision_applied":expanded>len(plan.samples),
        "clearance_m":float(clearance_m),
        "first_blocking_collision":first,
        "collision_examples":close_contacts,
        "world_snapshot_restored":True,
        "motion_executed":False,
        "check_mode":"BOUNDED_ADAPTIVE_JOINT_INTERPOLATION_BULLET_COLLISION_GEOMETRY",
        "inter_sample_collision_guaranteed":False,
        "full_swept_collision_detection":False,
        "self_collision_qualified":False,
        "industrial_safety_certified":False,
    }
