# DSIQ Native — Bullet Robotics Sensors (CPU)

## Operational capabilities

This is a **geometry-backed sensor pipeline** using the C++ Bullet
simulation and its software renderer. The outputs are synthesized from the
actual shapes and positions in the local physics scene; they are **not**
calibrated measurements of real industrial sensors, nor NVIDIA RTX sensors.

Current APIs:
- **Idealized planar 360° LiDAR**: 4–360 rays, range 0.1–50 m, Bullet
  rayTestBatch body IDs, distances and surface normals; configurable pose.
- **RGB-D camera**: Bullet TinyRenderer CPU color pixels, segmentation IDs,
  and depth reconstructed in meters from the OpenGL depth buffer; constrained
  resolution 32×32 to 640×480, maximum 200,000 pixels per frame.
- **Robot link velocity**: Bullet-reported world-frame linear/angular velocities
  for the six-axis reference robot, with simulation timestamps.

## Run without Kit/RTX

```powershell
python -m pip install -e ".[physics]"
python -m dsiq_native bullet-run --steps 240 --lidar-rays 36
python -m dsiq_native bullet-run --steps 120 --boxes 5 --camera-png camera.png --depth-npz camera-depth.npz --json-out sensor-report.json
```

PNG is RGBA generated from Bullet's actual TinyRenderer pixels. NPZ stores
depth_m in meters along camera optical axis and segmentation body/link IDs.
Color/depth share the same camera intrinsics/extrinsics.

## Engineering boundaries

- No calibrated sensor noise, distortion, latency, exposure, LiDAR divergence,
  reflectivity, return intensity, rain/fog/dust effects, time synchronization
  with hardware, or radiometric accuracy.
- Geometry is a declared parametric robot and sandbox objects, not OEM CAD.
- TinyRenderer is **CPU rasterization**, not physically based RTX sensing.
- Image/ray counts are bounded to protect low-memory users.
- Bullet and NVIDIA Kit are separate rendering/physics backends without
  validated dynamic parity.
- No real robot or industrial machine control is enabled.

## Acceptance

Automated tests exercise a ray hitting a known obstacle, measured distances,
finite depth, segmentation size, real link velocity, PNG file format, NPZ
array dimensions, input validation and actual CLI-generated sensor files.

Next steps include validated sensors, collision-aware robot planning, real
URDF/MJCF ingestion, ROS 2 adapters, and GPU-aware sensor scheduling.
