"""Seeded *actual Bullet* parameter randomization for CPU robot learning.

Varies per-environment gravity and physical contact friction on a verified
source gantry. Observes measured joint positions, velocities and solver steps.

This is a small physics-diversity research baseline. It does not calibrate
probability distributions against real-world equipment and cannot establish
sim-to-real performance, safe hardware control or GPU Isaac Lab parity.
"""
from __future__ import annotations

import hashlib
import json
import math
import random
import time

from .vector_learning import BulletVectorGantry


def run_domain_randomization(*, envs: int=3, iterations: int=24,
                             seed: int=1007) -> dict:
    if type(envs) is not int or not 1<=envs<=8:
        raise ValueError("Domain randomization supports 1..8 independent Bullet worlds")
    if type(iterations) is not int or not 2<=iterations<=300:
        raise ValueError("Randomization iterations must be 2..300")
    if type(seed) is not int or not 0<=seed<=2**32-1:
        raise ValueError("Domain randomization seed out of unsigned 32-bit range")
    rng=random.Random(seed)
    start=time.perf_counter()
    environment_data=[]
    trajectory=[]
    with BulletVectorGantry(num_envs=envs,seed=seed) as vector:
        initial=vector.reset()
        for index,(world,robot) in enumerate(zip(vector._worlds,vector._data)):
            gravity=rng.uniform(0,9.81)
            lateral_friction=rng.uniform(.2,1.2)
            world.p.setGravity(0,0,-gravity,physicsClientId=world.client)
            body=int(robot["body_id"])
            link=int(robot["index"])
            world.p.changeDynamics(
                body,link,lateralFriction=lateral_friction,
                physicsClientId=world.client,
            )
            measured_friction=float(world.p.getDynamicsInfo(
                body,link,physicsClientId=world.client
            )[1])
            if abs(measured_friction-lateral_friction)>1e-6:
                raise ArithmeticError("Bullet did not apply documented friction randomization")
            environment_data.append({
                "env":index,
                "simulator_gravity_m_s2":gravity,
                "actual_solver_lateral_friction":measured_friction,
                "robot_link_index":link,
                "parameter_distribution":"uniform_reference_range_not_oem_calibrated",
            })
        for tick in range(iterations):
            actions=[]
            for item in initial:
                difference=item["goal_m"]-item["joint_position_m"]
                actions.append(1 if difference>.02 else -1 if difference<-.02 else 0)
            initial=vector.step(actions,frame_skip=8)
            if tick==0 or tick==iterations-1:
                trajectory.append({
                    "iteration":tick+1,
                    "actual_bullet_joint_feedback":initial,
                })
        simulated=vector._steps
    duration=time.perf_counter()-start
    canonical=json.dumps(environment_data,sort_keys=True,
                         separators=(",",":"),allow_nan=False)
    return {
        "schema":"dsiq-cpu-bullet-parameter-randomization-v1",
        "actual_bullet_stepped":True,
        "seed":seed,
        "isolated_clients":envs,
        "physics_steps":simulated,
        "measured_wall_seconds":duration,
        "measured_steps_per_second":simulated/duration,
        "environment_parameters":environment_data,
        "parameter_draws_sha256":hashlib.sha256(canonical.encode()).hexdigest(),
        "simulation_trajectories":trajectory,
        "calibrated_physical_parameter_distributions":False,
        "real_world_transfer_tested":False,
        "safe_robot_policy_demonstrated":False,
        "gpu_parallel_training":False,
    }
