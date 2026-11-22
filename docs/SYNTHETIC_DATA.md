# DSIQ Native — Physics-backed synthetic perception data

This is a deliberately bounded CPU dataset capture workflow toward
the sensor and synthetic-data capabilities of NVIDIA Isaac Sim.
It does not reproduce the photorealism of Replicator/RTX sensors.

## Use

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native dataset-generate training-data --frames 8 --seed 1007 --width 160 --height 120 --boxes 3 --steps 72
python -m dsiq_native dataset-verify training-data
~~~

Generated files:
- One actual Bullet TinyRenderer RGBA PNG per frame;
- A matching NumPy NPZ file containing metric camera depth and
  actual Bullet body/link integer segmentation IDs;
- A JSON manifest with seeded scenario input parameters, real
  observed Bullet body positions and motor states, frame timestamps,
  camera poses, rendering metadata and SHA-256 checksums.

Each sample varies physical properties (body masses, friction,
restitution, size, object positions), robot motor targets and camera
extrinsics. Values come from a seeded local PRNG; the seed is recorded.

## Resource bounds

Maximum of 64 frames, 12 payloads per frame and 76,800 image pixels.
One Bullet world, RGB frame and depth buffer are used at a time.
No GPU RTX/Kit allocation is required. Total output disk space grows
with frame count, and a larger project still requires profiling.

## Accuracy and licensing limits

- The robot is an original reference assembly, not an OEM-certified
  CAD model, and physical values are illustrative within declared bounds.
- Depth is computed from a software renderer. Camera noise, exposure,
  rolling shutter, radiometry and LiDAR physics are not calibrated.
- Segmentation uses actual Bullet body/link IDs; it does not claim
  semantic ground truth for arbitrary real-world industrial objects.
- Cross-platform pixel-for-pixel determinism is not guaranteed. Only
  the seeded scenario input parameters are intended to match.
- No learned robotics policy or model training is included.
- Existing output directories with data are never overwritten.
- Each saved frame is independently verified by SHA-256.

These restrictions distinguish useful *simulated perception data*
from a validated sensor dataset suitable for real industrial deployment.
