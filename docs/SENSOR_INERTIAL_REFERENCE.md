# DSIQ Native — Real Bullet IMU-style inertial reference capture

**Status:** Implemented CPU physics-derived reference sensor, **not** a calibrated or verified physical IMU.

The `dsiq_native.inertial_sensor.IdealBulletInertialProbe` captures actual Bullet center-of-mass link kinematics in an isolated PyBullet world. Bullet reports the world-frame linear and angular velocities using `getLinkState(..., computeLinkVelocity=True)`. The probe measures velocity at two *different* solver step indices and computes

\[
a_{world}=\frac{v_{world}(t_2)-v_{world}(t_1)}{(step_2-step_1)\,\Delta t}
\]

using the engine's fixed time step `Δt = 1/240 s`. Idealized accelerometer specific force is

\[
f_{COM}=R_{world\to COM}(a_{world}-g_{world})
\]

where `g_world = [0,0,-9.81] m/s²` for the default world. Angular velocity is reported in world coordinates and rotated into center-of-mass axes. Units are metres, seconds, metres/second² and radians/second.

The output includes source metadata, actual physics step index and timestamp, elapsed sample time, COM orientation, directly sampled angular velocities, estimated acceleration, idealized specific force and explicit no-calibration/no-noise labels. The first reading or a repeated reading **at the same simulation step** has `acceleration_is_valid=false` and null acceleration; it does not invent a zero.

## Reproduce

```powershell
python -m pip install -e ".[physics]"
python -m dsiq_native imu-sample --steps 24 --link 5
python -m unittest discover -s tests -p test_inertial_sensor.py -v
```

The CPU CI gate invokes this command and tests on both Windows and Linux. It does **not** rely on a camera image, RTX hardware, Isaac Lab or any real physical sensor.

## Limitations

- Acceleration is a finite-difference estimate based on the *center-of-mass* velocity reported by Bullet. With step intervals larger than one, high-frequency acceleration is averaged; it is not continuous-time ground truth.
- The probe represents link COM axes, **not** an actual mounting orientation or factory IMU extrinsics. It has no measurement noise, bias, Allan deviation, saturation, thermal response, time offset or calibration uncertainty.
- The code cannot be used for safety-rated robot control or a real inertial navigation system without independent physical commissioning.
- The reference six-link model is an original DSIQ engineering fixture, not a manufacturer-calibrated robot.
- Validation is limited to measured software behavior; sensor accuracy must be compared with physical measurements before any industrial fidelity claim.

Reference semantics: [PyBullet Quickstart Guide, `getLinkState` return fields](https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstartguide.pdf).
