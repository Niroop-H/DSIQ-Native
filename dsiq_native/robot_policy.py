"""Small, genuine tabular Q-learning policy on actual Bullet gantry motor.

This is a strictly bounded reference *learning* experiment, not Isaac Lab
parity, an industrial robot policy, or a machine-safe autonomous controller.
Actual actions command a user-authored URDF prismatic joint and step the
C++ Bullet solver. Only measured joint position determines reward/state.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import random
from tempfile import TemporaryDirectory

from .bullet_runtime import BulletPlayground,BulletSettings
from .machine_designer import MachineDesign,save_project

BIN_COUNT=13
DELTA=.05
ACTIONS=(-1,0,1)


def train_gantry_q_policy(output_file: str|Path, *,
                          episodes: int=96, steps_per_episode: int=24,
                          seed: int=1007) -> dict:
    if type(episodes) is not int or not 12<=episodes<=512:
        raise ValueError("Training episodes must be integer 12..512")
    if type(steps_per_episode) is not int or not 8<=steps_per_episode<=40:
        raise ValueError("Training horizon must be integer 8..40")
    if type(seed) is not int or not 0<=seed<=2**32-1:
        raise ValueError("Training seed must be an unsigned 32-bit integer")
    target=Path(output_file).resolve()
    if target.exists():
        raise FileExistsError("Policy file exists; refusing overwrite")
    rng=random.Random(seed)
    q=[[[0. for _ in ACTIONS] for goal in range(BIN_COUNT)]
       for current in range(BIN_COUNT)]
    update_count=0
    sim_steps=0
    training_errors=[]
    trained_success=0
    with TemporaryDirectory() as tmp:
        machine=save_project(MachineDesign(asset_id="learned_reference_gantry"),
                             Path(tmp)/"gantry")
        with BulletPlayground(BulletSettings(
            payload_count=0,gravity_m_s2=0.
        )) as world:
            imported=world.import_urdf_robot(
                machine["manifest"],base_position_m=(-1.,1.3,0.)
            )
            joint=next(j for j in imported["joints"]
                       if j["name"]=="actuator_x")
            idx=joint["joint_index"]
            def reset(state_bin):
                world.p.resetJointState(
                    imported["body_id"],idx,state_bin*DELTA,
                    targetVelocity=0.,physicsClientId=world.client,
                )
                world.command_imported_joint(
                    imported["asset_id"],"actuator_x",state_bin*DELTA,
                )
            def measured():
                pos=float(world.p.getJointState(
                    imported["body_id"],idx,physicsClientId=world.client
                )[0])
                if not math.isfinite(pos):
                    raise ArithmeticError("Nonfinite Bullet joint in training")
                return pos
            def discretize(pos):
                return min(BIN_COUNT-1,max(0,int(round(pos/DELTA))))
            for epoch in range(episodes):
                start=rng.randrange(BIN_COUNT)
                goal=rng.randrange(BIN_COUNT)
                while abs(goal-start)<2:
                    goal=rng.randrange(BIN_COUNT)
                reset(start)
                goal_m=goal*DELTA
                epsilon=max(.05,.60*(1.-epoch/episodes))
                done=False
                for _ in range(steps_per_episode):
                    actual=measured()
                    current=discretize(actual)
                    if rng.random()<epsilon:
                        action_index=rng.randrange(3)
                    else:
                        action_index=max(range(3),
                            key=lambda i:(q[current][goal][i],-abs(ACTIONS[i])))
                    action=ACTIONS[action_index]
                    command=max(0.,min(.6,(current+action)*DELTA))
                    world.command_imported_joint(
                        imported["asset_id"],"actuator_x",command
                    )
                    world.step(8)
                    sim_steps+=8
                    after=measured()
                    next_state=discretize(after)
                    reward=8.*(abs(goal_m-actual)-abs(goal_m-after))-.015
                    done=abs(goal_m-after)<=.035
                    if done:
                        reward+=2.
                    old=q[current][goal][action_index]
                    future=0. if done else max(q[next_state][goal])
                    q[current][goal][action_index]=old+.35*(
                        reward+.92*future-old
                    )
                    update_count+=1
                    if done:
                        break
                training_errors.append(abs(goal_m-measured()))
                trained_success+=int(done)

            # Distinct held-out pseudo-random episodes. No promise of
            # physical transfer or superior policy vs other planners.
            eval_rng=random.Random(seed^0x5A17)
            evaluation=[]
            for _ in range(12):
                start=eval_rng.randrange(BIN_COUNT)
                goal=eval_rng.randrange(BIN_COUNT)
                while abs(goal-start)<2:
                    goal=eval_rng.randrange(BIN_COUNT)
                reset(start)
                target_m=goal*DELTA
                for _ in range(steps_per_episode):
                    state=discretize(measured())
                    best=max(range(3),key=lambda i: q[state][goal][i])
                    command=max(0.,min(.6,(state+ACTIONS[best])*DELTA))
                    world.command_imported_joint(
                        imported["asset_id"],"actuator_x",command
                    )
                    world.step(8)
                    sim_steps+=8
                    if abs(target_m-measured())<=.035:
                        break
                actual=measured()
                evaluation.append({
                    "initial_m":start*DELTA,
                    "goal_m":target_m,
                    "final_measured_m":actual,
                    "absolute_error_m":abs(target_m-actual),
                    "success":abs(target_m-actual)<=.035,
                })
    canonical=json.dumps(q,separators=(",",":"),allow_nan=False)
    data={
        "schema":"dsiq-tabular-bullet-gantry-rl-v1",
        "algorithm":"tabular_Q_learning_epsilon_greedy",
        "source_mechanism":"user_authored_parametric_reference_gantry",
        "motor":"actuator_x",
        "action_steps_m":list(ACTIONS),
        "action_step_scale_m":DELTA,
        "discrete_states":BIN_COUNT,
        "seed":seed,
        "train_episodes":episodes,
        "training_updates":update_count,
        "physics_steps":sim_steps,
        "mean_train_absolute_error_m":sum(training_errors)/len(training_errors),
        "training_success_fraction":trained_success/episodes,
        "heldout_evaluation":evaluation,
        "heldout_success_fraction":sum(x["success"] for x in evaluation)/len(evaluation),
        "heldout_mean_abs_error_m":sum(x["absolute_error_m"] for x in evaluation)/len(evaluation),
        "q_values_sha256":hashlib.sha256(canonical.encode()).hexdigest(),
        "q_values":q,
        "hardware_control":False,
        "robot_policy_certified":False,
        "sim_to_real_qualified":False,
        "policy_competitive_with_isaac_lab":False,
        "purpose":"tiny reproducible REAL Bullet motor RL baseline, not industrial training",
    }
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(data,indent=2,allow_nan=False)+"\n",encoding="utf8")
    return {
        "file":str(target),
        "training_updates":update_count,
        "physics_steps":sim_steps,
        "train_success_fraction":data["training_success_fraction"],
        "heldout_success_fraction":data["heldout_success_fraction"],
        "heldout_mean_abs_error_m":data["heldout_mean_abs_error_m"],
        "q_values_sha256":data["q_values_sha256"],
        "sim_to_real_qualified":False,
        "hardware_control":False,
    }
