# DSIQ Native — Extended physics evidence

This milestone provides additional independent, headless acceptance experiments
for the CPU Bullet engine. These experiments are **not** OEM machine
validation, PhysX parity, or safety certification.

## Execute

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native physics-verify
python -m dsiq_native physics-extended
~~~

The base suite checks free-fall kinematics, static resting contact,
torque/inertia, isolated-client repeatability and gravity timestep
convergence at 120/240/480 Hz.

The **new** extended suite measures:

1. **Contact timestep stability.** The same falling rigid box settles
   against a real floor collider at 120, 240 and 480 Hz. It records
   resting height error, maximum signed penetration, vertical speed,
   contact count, force reported by Bullet, and spread across timesteps.
   Passing requires bounded values at all three rates; unlike smooth
   free fall, collision dynamics are nonsmooth, so the suite does not
   incorrectly demand monotonic convergence.
2. **Friction response.** Two independent trials use equal initial
   horizontal speed and different lateral-friction parameters. The
   resulting **measured** travel and speed are compared. This tests
   directional sensitivity, not correspondence with a measured
   material friction curve.
3. **Imported motor constraints.** The existing editable XYZ gantry is
   generated as a real URDF, imported into Bullet and driven with its
   declared actuator effort and velocity limits. Out-of-range travel
   requests must fail; a valid request must produce bounded actual
   joint motion. This is not a servo-drive accuracy certificate.

All results are structured objects with reference expectations,
observed measurements, explicit limits and a per-case pass/fail result.

## Next qualification gates

The more advanced tests still missing include controlled coefficient-of-
restitution measurements, variable-mass/contact studies, self-collision,
joint torque/position saturation tests under load, swept high-speed CCD,
multi-body constraint stress, stable stacking, recorded p95/p99 step
latencies and independent instrumented hardware datasets.

Some of those require different numerical/contact settings, so failures
must be measured and understood rather than hidden by wider thresholds.
Substantial real-world claims require experimental validation and
uncertainty statements.

The current solver is Bullet CPU, not NVIDIA PhysX. No real physical
robot or industrial machine can be commanded from this module.
