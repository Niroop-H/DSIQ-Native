# DSIQ Native — Read-only ROS 2 simulator bridge

This implements the first **actual ROS 2 message publication boundary**
for DSIQ's CPU Bullet reference robot and geometric LiDAR. It helps with
ROS-facing robotics workflows while retaining a clear safety boundary.

## No middleware required: inspect real simulation messages

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native ros2-preview --steps-per-frame 12 --lidar-rays 36
~~~

The result is based on **actual Bullet physics** and exposes field values
compatible with ROS 2 standard messages:
- sensor_msgs/msg/JointState: the six reference joint names, positions,
  velocities and solver-reported applied motor torques;
- sensor_msgs/msg/LaserScan: 2D angular beam layout, real geometric ray
  intersections, range convention and simulated scan period;
- rosgraph_msgs/msg/Clock: the actual simulation tick converted to
  seconds/nanoseconds.
- tf2_msgs/msg/TFMessage: actual Bullet link frame transforms expressed
  relative to the current reference base (plus a world-frame LiDAR fixture).
  The preview contains 8 distinct child frames and synchronized stamps.

JSON cannot contain nonfinite floating values, so the preview records
no-return LiDAR readings as null. The live ROS publisher converts those
to +inf values for LaserScan.

## Publish through a real ROS 2 installation

Install and source a compatible supported ROS 2 distribution separately.
ROS 2 Python message packages and rclpy are **not** bundled into pip
extras and are not secretly replaced by a mock transport.

~~~bash
# In an environment where ROS 2 and rclpy are actually available:
python -m dsiq_native ros2-stream --frames 200 --steps-per-frame 12 --lidar-rays 36

# Inspect with the actual ROS 2 CLI in another sourced terminal:
ros2 topic echo /dsiq/joint_states
ros2 topic echo /dsiq/scan
ros2 topic echo /dsiq/clock
ros2 topic echo /dsiq/tf
~~~

Topics:
- /dsiq/joint_states
- /dsiq/scan
- /dsiq/clock
- /dsiq/tf (reference-base, 6 robot links, LiDAR fixture)

The node publishes at a *best-effort wall-time pace* matching simulated
steps, using an independent Bullet client in the current process. This
is **not a hard real-time scheduler**. Sample count and step count are
bounded: max 1,200 frames, 24 solver steps/frame and 360 LiDAR rays.

## Important interoperability and safety boundaries

- There is **no subscriber**, trajectory execution service, action
  server, controller manager, ros2_control interface or real machine
  command bridge.
- A real measured reference TF tree is now constructed in the preview
  and optionally published as /dsiq/tf. Transform positions and quaternions
  come from Bullet world-link frames converted to parent-relative frames,
  not guessed frame offsets. Complete RViz interoperability and source
  URDF frame matching still require a ROS 2 integration test. No OEM
  robot calibration is implied.
- The scan origin and orientation are a DSIQ reference fixture. A 2D
  geometric raycaster is not a calibrated LiDAR or an RTX sensor.
- Simulation time in /dsiq/clock is not a global ROS /clock topic; this
  avoids inadvertently altering another application's simulation time.
- The ROS Python runtime must be present to start actual live publishing.
  If absent, the command fails rather than claiming success.
- Windows/Linux CI validates the serialized ROS message field contract
  with actual Bullet measurements. **CI does not currently install or
  exercise ROS 2 DDS middleware**, so live topics, discovery, QoS
  and downstream consumer interoperability are unverified until
  tested with a sourced ROS 2 environment.
- Industrial machine and robot control are always disabled. Before any
  eventual real-hardware work, add authentication, safety interlocks,
  authorization, rate limits, emergency stops, commissioning evidence
  and a clear operator-controlled separation from simulation.

The next robotics integration milestone is qualified TF/URDF matching,
DDS middleware live publishing and a test matrix across ROS 2 versions, followed by
explicit opt-in software-only robot planning interfaces. No ROS 2
feature is represented as an industry-qualified hardware controller.
