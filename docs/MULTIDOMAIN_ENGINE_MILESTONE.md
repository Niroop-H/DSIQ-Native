# DSIQ Native — Motion, OpenUSD, ROS 2, AI and release engineering milestone

## What actually changed

**Physics and robot motion**
- Existing real Bullet imported-robot trajectory screening checks intermediate joint configurations when path segments are coarse. Max interpolation step is configurable, with hard sample budget and fail-closed behaviour. This detects thin obstacles endpoint-only checking can miss. This is NOT mathematically continuous collision detection: untested configurations between samples remain possible.
- The motor executor measures actual per-joint maximum solver speed and reported effort at every step, and deviations from URDF limits. These diagnostics are not calibrated hardware torque sensors.

**OpenUSD and machine CAD**
- The usd-urdf-visual-record command imports original verified URDF box/cylinder/sphere geometry, visual offsets and color. Each link is animated from real Bullet world-space poses and solver timecodes.
- Unsupported mesh formats (STL/OBJ/DAE) fail closed, never silently replaced with generic geometry. A separately qualified mesh converter is required for full industrial CAD fidelity.
- Metric Z-up world-space link transforms under an identity model root avoid accidentally double-composing transforms.

**ROS 2**
- A ROS Jazzy CI container subscribes to four real DDS topics, verifying published JointState, LaserScan, Clock and TFMessage messages from previously captured actual Bullet data. This tests middleware delivery of captured data, not full industrial controller integration.

**Physical AI**
- robot-policy-train executes genuine tabular Q-learning on the real Bullet imported gantry: actions command a motor, Q-values update from measured joint positions, and held-out goal trials are recorded with success/error metrics.
- This is one-axis reference research learning only, not Isaac Lab-scale reinforcement learning or sim-to-real qualification.

**Commercial security**
- release-attest and release-verify build and validate a SHA-256 byte-for-byte manifest of the Windows portable ZIP, all its members and safe archive paths.
- This is NOT digital publisher signing, complete SBOM, Windows/MS Store certification or anti-malware verification.

## Reproducible commands

    python -m pip install -e ".[robotics]"
    python -m dsiq_native robot-urdf-motion machine/machine.manifest.json --target actuator_x=0.25 --max-joint-step 0.01
    python -m dsiq_native usd-urdf-visual-record machine/machine.manifest.json source_motion.usda --frames 8 --joint actuator_x --target 0.25
    python -m dsiq_native robot-policy-train reference_policy.json --episodes 96 --horizon 24 --seed 1007
    python -m dsiq_native release-attest DSIQ-Native-CPU-Preview-Windows.zip release.attestation.json
    python -m dsiq_native release-verify DSIQ-Native-CPU-Preview-Windows.zip release.attestation.json

## Remaining industrial blockers

- Continuous swept collision and complete self-collision models; measured actuator torque, acceleration and load qualification.
- Licensed OEM mesh CAD conversion with faithful geometry, material and assembly mates; scalable USD visual assembly playback.
- GPU native RTX rendering, textures, lighting, mesh streaming and real RTX 3050 FPS/VRAM measurements.
- End-to-end live ROS publisher lifecycle and physical robot/controller interoperability validation.
- Calibrated optical/ranging sensors, scalable AI training and real-world policy transfer.
- Signed installers, full licensing/security audits, accurate SBOM, production update rollout and customer pilot evidence.

Green CI verifies only the explicitly defined software contracts; it does not prove all physical phenomena, exact OEM fidelity or NVIDIA parity.
