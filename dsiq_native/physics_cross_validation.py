"""Independent solver-to-solver and analytic unconstrained gravity oracle.

Both worlds genuinely execute their respective native C++ physics engines
with the same measured SI values and fixed time step. This test deliberately
has no motor/control law or contacts, isolating integrator behavior.

A pass applies ONLY to this free-body analytic fixture, not general solver
equivalence, robotic drives, contacts, or industrial functional safety.
"""
from __future__ import annotations

import math

from .bullet_runtime import PHYSICS_DT


def compare_freefall_between_solvers(*, steps: int = 72,
                                     residual_tolerance_m: float = .001) -> dict:
    if type(steps) is not int or not 24 <= steps <= 120:
        raise ValueError("Free-fall comparison steps must be 24..120")
    if (isinstance(residual_tolerance_m,bool) or
        not isinstance(residual_tolerance_m,(int,float)) or
        not math.isfinite(residual_tolerance_m) or
        not 0 < residual_tolerance_m <= .02):
        raise ValueError("Residual tolerance must be in (0,0.02] m")
    import pybullet as bullet
    import mujoco

    g=9.81
    initial_z=3.
    side=.1
    mass=2.
    dt=PHYSICS_DT
    cid=bullet.connect(bullet.DIRECT)
    if cid<0:
        raise RuntimeError("Real Bullet reference solver did not connect")
    try:
        bullet.setGravity(0,0,-g,physicsClientId=cid)
        bullet.setTimeStep(dt,physicsClientId=cid)
        bullet.setRealTimeSimulation(0,physicsClientId=cid)
        shape=bullet.createCollisionShape(
            bullet.GEOM_BOX,halfExtents=[side/2]*3,physicsClientId=cid,
        )
        body=bullet.createMultiBody(
            baseMass=mass,baseCollisionShapeIndex=shape,
            basePosition=[0.,0.,initial_z],physicsClientId=cid,
        )
        bullet.changeDynamics(
            body,-1,linearDamping=0,angularDamping=0,
            physicsClientId=cid,
        )
        for _ in range(steps):
            bullet.stepSimulation(physicsClientId=cid)
        bz=float(bullet.getBasePositionAndOrientation(
            body,physicsClientId=cid,
        )[0][2])
        bv=float(bullet.getBaseVelocity(body,physicsClientId=cid)[0][2])
    finally:
        bullet.disconnect(physicsClientId=cid)

    # True independent native MuJoCo freejoint, with equal mass, shape,
    # gravity, initial conditions and Euler fixed step.
    spec=f"""<mujoco model="dsiq_unconstrained_gravity_oracle">
    <option timestep="{dt:.17g}" gravity="0 0 -9.81" integrator="Euler"/>
    <worldbody>
      <body name="free_box" pos="0 0 3">
        <freejoint name="free6dof"/>
        <geom name="symmetric_mass" type="box" size="0.05 0.05 0.05"
              mass="2" contype="1" conaffinity="1"/>
      </body>
    </worldbody>
    </mujoco>"""
    model=mujoco.MjModel.from_xml_string(spec)
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    initial_q=float(data.qpos[2])
    if not math.isclose(initial_q,initial_z,abs_tol=1e-12):
        raise AssertionError("MuJoCo freefall initial position mismatch")
    for _ in range(steps):
        mujoco.mj_step(model,data)
    mz=float(data.qpos[2])
    mv=float(data.qvel[2])

    total_time=steps*dt
    az=initial_z-.5*g*total_time**2
    av=-g*total_time
    result={
        "schema":"dsiq-cross-solver-freefall-analytic-v1",
        "solver_pair":["bullet_cpu_direct_v1","mujoco_cpu_direct_v1"],
        "fixture":"self_authored_unconstrained_free_body",
        "gravity_m_s2":g,"body_mass_kg":mass,"cube_side_m":side,
        "initial_z_m":initial_z,
        "fixed_timestep_s":dt,"steps":steps,"elapsed_s":total_time,
        "analytical_z_m":az,"analytical_vz_m_s":av,
        "bullet_z_m":bz,"bullet_vz_m_s":bv,
        "mujoco_z_m":mz,"mujoco_vz_m_s":mv,
        "bullet_analytic_position_residual_m":abs(bz-az),
        "mujoco_analytic_position_residual_m":abs(mz-az),
        "bullet_mujoco_position_residual_m":abs(bz-mz),
        "bullet_mujoco_velocity_residual_m_s":abs(bv-mv),
        "position_inter_solver_tolerance_m":residual_tolerance_m,
        "analytical_position_tolerance_m":.02,
        "velocity_inter_solver_tolerance_m_s":.01,
        "neither_backend_fallback":True,
        "not_a_contact_or_motor_comparison":True,
        "oem_or_robot_safety_qualified":False,
    }
    vals=(bz,bv,mz,mv,az,av)
    result["passed"]=(
        all(math.isfinite(x) for x in vals) and
        result["bullet_mujoco_position_residual_m"] <= residual_tolerance_m and
        result["bullet_mujoco_velocity_residual_m_s"] <= .01 and
        result["bullet_analytic_position_residual_m"] <= .02 and
        result["mujoco_analytic_position_residual_m"] <= .02
    )
    return result
