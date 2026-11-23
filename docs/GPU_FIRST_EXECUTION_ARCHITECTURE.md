# DSIQ Native — GPU-first execution program

**Updated decision: 23 November 2026.** DSIQ Native prioritizes the graphics processor for scalable simulation, robotics, sensor generation, rendering and Physical AI. The CPU still handles OS/UI events, source-file validation, orchestration, safety checks and independent numeric reference validation. **There is no silent CPU fallback** when an operator explicitly selects CUDA.

The long-term target remains *full NVIDIA Omniverse-class capability coverage*, not a restricted-feature prototype. This decision changes execution priority; it does not claim that full robot physics, Vulkan, Hydra, ray tracing or neural training already execute on CUDA.

## What has been implemented

- **Real NVIDIA Warp kernels:** optional native CUDA-selectable compute for batched, source-bounded, independent gantry actuator states.
- **One-axis compute path:** independently bounded X actuator; up to 262,144 elements, subject to an 80-million-update limit and explicit memory budget.
- **Three-axis compute path:** independent X/Y/Z actuators with actual source-URDF limits and downstream effective moving masses. Up to 65,536 triple-axis environments (196,608 axis elements), subject to the same update/device-memory ceilings.
- **Source checks:** the fixture has an original parametric URDF, a validated SHA-256 and rights metadata. Bounds, speed, maximum motor force and moving-subtree link masses are extracted from the source, rather than invented for a GPU benchmark.
- **Device-resident state:** positions, velocities and targets remain on the selected device while Warp kernels advance; only final states return to the host for numerical checks.
- **Fail-closed GPU selection:** the default benchmark device is CUDA device zero. If the GPU or driver is unavailable, the benchmark raises an error instead of running the CPU version and claiming GPU acceleration.
- **Real compiled CPU kernel correctness CI:** standard GitHub Actions currently runs the exact Warp kernels explicitly on the CPU and compares to an independent scalar integrator, checking source limits, seed repeatability and nonfinite results. **This does not prove CUDA execution or GPU speed.**

**Model caveat:** the three X/Y/Z axes are independently stepped single-DOF effective-mass servos, not a coupled articulated-body/contact solver. Vertical gravity load, actual friction, tool coupling, robot self-collision, mass-matrix off-diagonal entries and OEM servos are outside the current model. This is a GPU compute foundation, not PhysX/Newton/Isaac Lab functionality.

## CUDA workstation setup (Windows x64, Python 3.11)

In the DSIQ-Native repository:

    python -m pip install -e ".[gpu-compute]"
    nvidia-smi
    python -m dsiq_native gpu-compute-check --require-cuda
    python -m dsiq_native gpu-vector-benchmark --device cuda:0 --envs 4096 --steps 240
    python -m dsiq_native gpu-xyz-benchmark --device cuda:0 --envs 16384 --steps 240

The CLI commands produce JSON with actual execution-device identity, environment/step counts, synchronized wall-clock kernel time, independently checked numerical residuals, source travel/speed limits and explicit no-safety-certification flags.

For one immutable, consolidated real NVIDIA GPU hardware evidence file:

    python scripts/gpu_cuda_hardware_acceptance.py --device cuda:0 --one-axis-envs 4096 --xyz-envs 16384 --steps 240 --output build/gpu-cuda-hardware.json

If no suitable GPU exists or the kernel/model checks fail, this script exits without claiming hardware success. It does not overwrite evidence already saved at that path.

**Important driver compatibility:** official NVIDIA Warp 1.20 default PyPI wheels use CUDA Toolkit 13.4 and require an R580-series or newer NVIDIA driver for GPU acceleration. NVIDIA's official Windows x64 CUDA12 compatibility build requires an R525-series or newer driver. For an older compatible driver, use the vendor-provided wheel instead of claiming that the latest default wheel works:

    python -m pip install --force-reinstall "https://github.com/NVIDIA/warp/releases/download/v1.20.0/warp_lang-1.20.0+cu12-py3-none-win_amd64.whl"
    python -m dsiq_native gpu-compute-check --require-cuda

**Sources:** NVIDIA official Warp installation guide (https://nvidia.github.io/warp/latest/user_guide/installation.html), Warp code and Apache-2.0 licensing (https://github.com/NVIDIA/warp), and Warp runtime/device documentation (https://nvidia.github.io/warp/stable/user_guide/basics.html). Check exact wheel hashes, CUDA support and package distribution rights before commercial packaging.

## GPU memory and scale strategy

The GPU runtime has a conservatively budgeted number of float32 arrays and bounded total element count (default working-buffer budget 64 MiB; allowed 8–256 MiB). **Before CUDA allocation, DSIQ now queries the real selected Warp GPU's free and total device memory** and requires room for the estimated arrays **plus 256 MiB of unallocated headroom**. Unknown or inconsistent CUDA memory telemetry blocks the launch. The report preserves the free/total snapshot and its limitations.

This preflight **does not** prevent all device out-of-memory failures: the estimate excludes the native Warp memory pool, compiled module/JIT cache, device runtime, graphics buffers, shader/texture memory and concurrent allocations by other applications. Never interpret a passing software budget or preflight as guaranteed 4 GiB GPU safety.

Memory and performance tiers to benchmark on actual hardware: 4 GiB RTX3050/16 GiB system RAM as minimum, then 8/12/24 GiB and eventually multi-GPU. Measure driver and device identity, peak process memory with a supported profiler, p50/p95/p99 latency, sustained throughput, device memory errors and recovery, shader/compute contention, and actual workload accuracy on both NVIDIA and non-NVIDIA rendering targets where possible.

**No NVIDIA RTX3050 hardware qualification has been performed by hosted GitHub Actions.** A standard hosted Ubuntu runner without a CUDA GPU cannot establish actual GPU acceleration. The executable CUDA path and fail-closed gate are present; hardware acceptance requires running the command on an appropriate NVIDIA device.

## GPU-first parallel feature workstreams

| Workstream | Target acceleration | Current status |
| --- | --- | --- |
| Vector robotics | NVIDIA Warp native CUDA kernels | 1-axis + 3-axis independent actuator model, real kernel CPU conformance |
| Articulated physics | PhysX/Newton GPU solvers, collision/contact processing | Not yet implemented; CPU Bullet/MuJoCo remain independent reference |
| Scene rendering | Vulkan + USD Hydra, raster/PBR then RTX where available | Panda3D OpenGL, geometry reuse; Vulkan/Hydra unimplemented |
| Sensor generation | CUDA ray/texture processing, GPU AOVs | Bullet reference camera/LiDAR/IMU only |
| Physical AI | GPU-resident state, RL rollout and train loops | Warp state kernels available, GPU policy training not implemented |
| Asset streaming | USD payload streaming, device buffer paging, async transfer | Source-validator and static OpenGL buffer reuse; no GPU streaming yet |
| Scale/distribution | Device-aware GPU dispatch, multi-GPU and large VRAM tiers | Single explicitly chosen Warp device, conservative memory budget |

## Evidence and stop rules

1. A CUDA benchmark passes only if actual native kernels run on CUDA and the independent reference/bounds checks pass.
2. The CPU Warp test may be used to verify kernel equations only; never substitute its reported updates/second for a GPU hardware figure.
3. The current reduced-order X/Y/Z model cannot be used to claim industrial robot dynamics, full PhysX equivalence, machine safety or OEM calibration.
4. Performance comparisons must use identical physical equations, step sizes, contact dynamics, precision, memory accounting and hardware.
5. Preserve CPU Bullet/MuJoCo as *reference and compatibility solvers* while building GPU physics; do not delete a working fallback from users unless a genuinely equivalent replacement is verified.
6. Separate renderer GPU usage from CUDA compute. Mesa software OpenGL does not establish that simulation executed on a GPU.

**Next milestones:** true GPU forward kinematics / Jacobians / articulated contacts, verified PhysX/Newton GPU pipeline, CUDA sensor synthesis, CUDA/graphics interop, GPU vectorized physical-AI learning and independently measured native RTX3050 performance. All remain scoped engineering goals, not complete feature parity.
