"""Bounded parallel Bullet DIRECT environments for genuine robot learning.

Multiple isolated physics clients execute in one CPU process, each with a
verified source URDF. This is not Isaac Lab GPU vectorization, PPO/SAC,
or evidence of sim-to-real policy safety. Never controls hardware.
"""
from __future__ import annotations

import math
from pathlib import Path
import random
from tempfile import TemporaryDirectory
import time

from .bullet_runtime import BulletPlayground, BulletSettings
from .machine_designer import MachineDesign, save_project


class BulletVectorGantry:
    def __init__(self, *, num_envs: int=4, seed: int=1007):
        if type(num_envs) is not int or not 1<=num_envs<=8:
            raise ValueError("CPU Bullet vector env count must be 1..8")
        if type(seed) is not int or not 0<=seed<=2**32-1:
            raise ValueError("Training seed must be unsigned 32-bit integer")
        self.num_envs=num_envs
        self.seed=seed
        self._rng=[random.Random(seed+index) for index in range(num_envs)]
        self._temp=TemporaryDirectory()
        self._worlds=[]
        self._data=[]
        try:
            manifest=save_project(
                MachineDesign(asset_id="reference_vector_gantry"),
                Path(self._temp.name)/"gantry",
            )["manifest"]
            for i in range(num_envs):
                world=BulletPlayground(BulletSettings(
                    payload_count=0,gravity_m_s2=0.
                ))
                self._worlds.append(world)
                imported=world.import_urdf_robot(
                    manifest,base_position_m=(-1.,1.,0.)
                )
                index=next(j["joint_index"] for j in imported["joints"]
                           if j["name"]=="actuator_x")
                self._data.append({
                    "asset_id":imported["asset_id"],
                    "body_id":imported["body_id"],
                    "index":index,
                    "goal_m":.30,
                })
        except Exception:
            self.close()
            raise
        self._steps=0

    def reset(self):
        output=[]
        for i,(world,entry) in enumerate(zip(self._worlds,self._data)):
            r=self._rng[i]
            start=r.randrange(13)*.05
            goal=r.randrange(13)*.05
            while abs(start-goal)<.1:
                goal=r.randrange(13)*.05
            entry["goal_m"]=goal
            world.p.resetJointState(
                entry["body_id"],entry["index"],start,
                targetVelocity=0.,physicsClientId=world.client,
            )
            world.command_imported_joint(entry["asset_id"],"actuator_x",start)
            output.append({
                "env":i,"joint_position_m":start,
                "goal_m":goal,"error_m":abs(start-goal),
            })
        return output

    def step(self, actions: list[int], *, frame_skip: int=8):
        if (not isinstance(actions,list) or len(actions)!=self.num_envs
            or any(type(a) is not int or a not in (-1,0,1) for a in actions)):
            raise ValueError("Actions must be one ternary motor command per isolated world")
        if type(frame_skip) is not int or not 1<=frame_skip<=24:
            raise ValueError("Bullet frame_skip must be 1..24")
        result=[]
        for i,(world,entry,action) in enumerate(
                zip(self._worlds,self._data,actions)):
            p=world.p
            q=float(p.getJointState(entry["body_id"],entry["index"],
                                    physicsClientId=world.client)[0])
            target=max(0.,min(.6,q+action*.05))
            world.command_imported_joint(
                entry["asset_id"],"actuator_x",target
            )
            world.step(frame_skip)
            after=float(p.getJointState(
                entry["body_id"],entry["index"],
                physicsClientId=world.client,
            )[0])
            speed=float(p.getJointState(
                entry["body_id"],entry["index"],
                physicsClientId=world.client,
            )[1])
            if not math.isfinite(after) or not math.isfinite(speed):
                raise ArithmeticError("Nonfinite simulator observation")
            error=abs(entry["goal_m"]-after)
            result.append({
                "env":i,"joint_position_m":after,
                "goal_m":entry["goal_m"],
                "error_m":error,
                "velocity_m_s":speed,
                "reward":-.015-error,
                "done":error<.025,
                "simulation_step_index":world.step_index,
            })
        self._steps+=frame_skip*self.num_envs
        return result

    def close(self):
        for world in getattr(self,"_worlds",[]):
            world.close()
        self._worlds=[]
        if getattr(self,"_temp",None) is not None:
            self._temp.cleanup()
            self._temp=None

    def __enter__(self):
        return self

    def __exit__(self,typ,value,tb):
        self.close()


def benchmark_vector_learning(*,num_envs=4,steps=30,seed=1007):
    if type(steps) is not int or not 1<=steps<=1000:
        raise ValueError("Training benchmark steps must be 1..1000")
    start=time.perf_counter()
    with BulletVectorGantry(num_envs=num_envs,seed=seed) as env:
        initial=env.reset()
        records=[]
        for step in range(steps):
            actions=[]
            for item in initial:
                distance=item["goal_m"]-item["joint_position_m"]
                actions.append(1 if distance>.02 else -1 if distance<-.02 else 0)
            initial=env.step(actions)
            if step%max(1,steps//8)==0 or step==steps-1:
                records.append({"step":step+1,"states":initial})
        simulated=env._steps
    elapsed=time.perf_counter()-start
    return {
        "schema":"dsiq-bullet-cpu-vector-learning-v1",
        "num_isolated_bullet_clients":num_envs,
        "iterations":steps,"fixed_physics_steps":simulated,
        "wall_seconds_including_client_creation":elapsed,
        "measured_total_physics_steps_per_second":simulated/elapsed,
        "traces":records,
        "seed":seed,
        "cpu_multiclient_real_bullet":True,
        "gpu_parallel_training":False,
        "real_hardware_policy_qualified":False,
        "reference_geometry_not_oem":True,
    }
