# DSIQ Native — Reference Six-Axis Trajectory Planner

A **real, tested quintic joint-space planner** for the original Reference6R,
plus optional motor-level replay in Bullet. This is engineering tooling, not
manufacturer-qualified industrial robot control or collision avoidance.

## Generate a mathematically bounded path

~~~powershell
python -m dsiq_native robot-plan --start 0 0 0 0 0 0 --goal 10 -20 25 5 -8 4

# Request an explicit duration; the planner rejects timing that violates
# the declared reference velocity or acceleration bounds.
python -m dsiq_native robot-plan --start 0 0 0 0 0 0 --goal 10 -20 25 5 -8 4 --duration 4
~~~

The solver computes per-joint piecewise smooth (quintic, C2) positions,
velocities and accelerations. Start and finish have zero commanded velocity
and acceleration, and all joint positions remain inside specified limits.

For normalized time u=t/T, s(u)=10u³−15u⁴+6u⁵. The joint target is
q(t)=q_start+(q_goal−q_start)s(u). The exact continuous-time peak reference
speed multiplier is 1.875/T and acceleration multiplier is
(10/sqrt(3))/T². The program derives the smallest allowed duration from
user/reference joint velocity/acceleration limits before quantizing to the
fixed 1/240-second simulation timestep.

Default reference limits are independent DSIQ design constants, not
manufacturer motor or drive ratings. The planner refuses nonfinite
coordinates, limit violations, unattainable requested durations and
oversized output arrays.

## Execute an actual Bullet motor program

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native bullet-trajectory --goal 8 -14 18 4 -3 2
~~~

This replays each planned joint target on an actual C++ Bullet multibody
motor controller, advances precisely one 1/240-second physics step per
sample, reads measured joint angles, and calculates maximum and p95
joint tracking errors.

**Measured tracking error matters:** merely calculating a smooth target
does not mean the robot faithfully follows it. The output explicitly
records differences. Contact counts in the scene are also reported, but
they do not establish an obstacle-free path.

## Explicitly not supported

- End-effector six-dimensional pose planning, collision-free or
  self-collision-safe routing, continuous-collision detection, singularity
  avoidance, measured servo motor torque feasibility, backlash, thermal
  motor losses or emergency safety interlocks.
- Controller matching to a real OEM manipulator, or learned/RL policies.
- Trajectory equivalence with NVIDIA Isaac Sim/PhysX without independent
  matched reference conditions.

The planner has unit tests of endpoints, derivatives, speed/acceleration
bounds, fixed timestep, limit rejection, and actual Bullet replay, and
identifies paths as **reference-only** rather than certified motion plans.
