"""GPU-first reduced-order XYZ gantry X-actuator batch with NVIDIA Warp.

This is ACTUAL compiled Warp compute on device='cuda:0' if compatible GPU
hardware exists. 'cpu' is a deliberate kernel-conformance reference choice,
never a silent CUDA fallback. Source limits and moving mass come from a
validated, rights-labelled, self-authored URDF.

The kernel is a one-axis independent point-mass / force-limited actuator
model with semi-implicit Euler integration. It is NOT Bullet/MuJoCo/PhysX,
not complete URDF multibody dynamics, not contact-rich robot simulation,
and not an OEM-calibrated physical machine.

Only final state / small samples are moved to host. GPU work arrays remain
resident across the batch; no per-step CPU readbacks are performed.
"""
from __future__ import annotations

import math
import random
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory


def validate_batch_options(*, envs: int, steps: int, seed: int,
                           memory_limit_mib: int) -> None:
    if type(envs) is not int or not 1 <= envs <= 262144:
        raise ValueError("Warp vector environments must be 1..262144")
    if type(steps) is not int or not 1 <= steps <= 2400:
        raise ValueError("Warp vector steps must be 1..2400")
    if type(seed) is not int or not 0 <= seed <= 2**32-1:
        raise ValueError("Warp seed must be an unsigned 32-bit integer")
    if type(memory_limit_mib) is not int or not 8 <= memory_limit_mib <= 256:
        raise ValueError("Warp device-buffer budget must be 8..256 MiB")
    if envs * steps > 80_000_000:
        raise ValueError("Total bounded Warp physics updates exceed 80 million")
    # Initial, target, current and next q/v = six float32 arrays.
    estimated=envs * 6 * 4
    if estimated > memory_limit_mib * 1024 * 1024:
        raise ValueError("Warp batch exceeds explicit device-array budget")


def load_verified_reference_axis():
    """Return actual X-axis URDF bounds, moving mass and source SHA256.

    The fixture is explicitly a parametric test asset, not manufacturer
    hardware. Assembled downstream moving-link masses are approximate
    single-axis effective mass, not full Cartesian mass matrix.
    """
    from .machine_designer import MachineDesign, save_project
    from .urdf_bridge import validate_robot_manifest

    with TemporaryDirectory(prefix="dsiq-warp-urdf-") as root:
        files=save_project(MachineDesign(asset_id="warp_vector_reference"),
                           Path(root)/"source")
        manifest, inspection=validate_robot_manifest(files["manifest"])
        xml=ET.parse(inspection.path).getroot()
        joint=next((j for j in xml.findall("joint")
                    if j.attrib.get("name")=="actuator_x"),None)
        if joint is None or joint.attrib.get("type")!="prismatic":
            raise ValueError("Source reference X-axis prismatic joint missing")
        limits=joint.find("limit")
        if limits is None:
            raise ValueError("Source reference X-axis limits missing")
        lower=float(limits.attrib["lower"])
        upper=float(limits.attrib["upper"])
        max_speed=float(limits.attrib["velocity"])
        max_effort=float(limits.attrib["effort"])
        # Source forward slider plus all mechanically attached children.
        masses=[]
        for name in ("x_axis_carriage", "y_axis_carriage",
                     "z_axis_lift", "inspection_tool_flange"):
            part=next((e for e in xml.findall("link")
                       if e.attrib.get("name")==name),None)
            if part is None:
                raise ValueError("Reference URDF moving link missing: "+name)
            inertial=part.find("inertial")
            mass_node=inertial.find("mass") if inertial is not None else None
            if mass_node is None:
                raise ValueError("Reference URDF moving link lacks mass")
            masses.append(float(mass_node.attrib["value"]))
        mass=sum(masses)
        if not all(math.isfinite(x) for x in (lower,upper,max_speed,max_effort,mass)):
            raise ValueError("Nonfinite source-authored reference dynamics")
        if not (0<=lower<upper<=2.5 and max_speed>0 and max_effort>0 and mass>0):
            raise ValueError("Invalid source reference actuator bounds/physical parameters")
        return {
            "axis_name":"actuator_x",
            "lower_m":lower,"upper_m":upper,
            "max_speed_m_s":max_speed,
            "max_effort_n":max_effort,
            "effective_moving_mass_kg":mass,
            "source_urdf_sha256":manifest["geometry"]["sha256"],
            "source_asset_class":manifest["asset_class"],
            "effective_mass_not_exact_joint_inertia":True,
            "full_multibody_dynamics":False,
        }


# Optional native kernel import. Unrelated core/CLI entrypoints are importable
# without requiring heavy CUDA-specific packages.
try:
    import warp as wp
except ImportError:
    wp=None

if wp is not None:
    @wp.kernel
    def _step_reduced_order_slide(
        q: wp.array(dtype=wp.float32),
        velocity: wp.array(dtype=wp.float32),
        targets: wp.array(dtype=wp.float32),
        next_q: wp.array(dtype=wp.float32),
        next_velocity: wp.array(dtype=wp.float32),
        lower: wp.float32, upper: wp.float32,
        vmax: wp.float32, effort_limit: wp.float32,
        moving_mass: wp.float32, dt: wp.float32
    ):
        i=wp.tid()
        position=q[i]
        speed=velocity[i]
        target=targets[i]

        # Source-speed-limited cascaded reference servo. Distinct from real
        # Bullet/MuJoCo, not calibrated for external equipment.
        desired_speed=wp.clamp(wp.float32(12.0)*(target-position),-vmax,vmax)
        drive_force=wp.clamp(
            wp.float32(450.0)*(desired_speed-speed),
            -effort_limit,effort_limit,
        )
        speed_next=wp.clamp(speed+dt*drive_force/moving_mass,-vmax,vmax)
        position_next=position+dt*speed_next
        if position_next<lower:
            position_next=lower
            if speed_next<wp.float32(0.0):
                speed_next=wp.float32(0.0)
        if position_next>upper:
            position_next=upper
            if speed_next>wp.float32(0.0):
                speed_next=wp.float32(0.0)
        next_q[i]=position_next
        next_velocity[i]=speed_next


def _reference_step(q, velocity, target, source, dt):
    """Independent scalar oracle, deliberately without Warp functions."""
    desired=max(-source["max_speed_m_s"],
                min(source["max_speed_m_s"],12.0*(target-q)))
    force=max(-source["max_effort_n"],
              min(source["max_effort_n"],450.0*(desired-velocity)))
    velocity=max(-source["max_speed_m_s"],
                 min(source["max_speed_m_s"],
                     velocity+dt*force/source["effective_moving_mass_kg"]))
    q+=dt*velocity
    if q<source["lower_m"]:
        q=source["lower_m"]
        velocity=max(0.0,velocity)
    elif q>source["upper_m"]:
        q=source["upper_m"]
        velocity=min(0.0,velocity)
    return q, velocity


def _initial_conditions(n, source, seed):
    rng=random.Random(seed)
    lower,upper=source["lower_m"],source["upper_m"]
    return (
        [lower+rng.random()*(upper-lower) for _ in range(n)],
        [lower+rng.random()*(upper-lower) for _ in range(n)],
    )


def benchmark_warp_reduced_order(*, envs: int=4096, steps: int=240,
                                 seed: int=1007, device: str="cuda:0",
                                 memory_limit_mib: int=64,
                                 oracle_samples: int=8) -> dict:
    """Execute real Warp kernels, validating outputs vs an independent oracle.

    When 'cuda:0' is requested the function refuses unavailable GPU/driver.
    CPU is a user-selected semantic test target; it is not reported as GPU.
    Compilation/allocation and host copy timings are clearly separated from
    measured synchronized kernel execution wall-clock timings.
    """
    validate_batch_options(envs=envs,steps=steps,seed=seed,
                           memory_limit_mib=memory_limit_mib)
    if type(oracle_samples) is not int or not 1<=oracle_samples<=32:
        raise ValueError("Independent scalar oracle samples must be 1..32")
    if not isinstance(device,str) or not (
        device=="cpu" or (device.startswith("cuda:") and
                           device[5:].isdigit() and len(device)<=8)):
        raise ValueError("Warp device must be explicitly 'cpu' or 'cuda:<ordinal>'")
    if wp is None:
        raise RuntimeError('NVIDIA Warp is required: pip install -e ".[gpu-compute]"')
    wp.init()
    if device.startswith("cuda:") and not wp.is_device_available(device):
        raise RuntimeError(
            "Requested real CUDA GPU unavailable or unsupported; "
            "no CPU fallback. Check NVIDIA driver and warp-lang CUDA wheel."
        )
    selected=wp.get_device(device)
    cuda=bool(selected.is_cuda)
    if device.startswith("cuda:") and not cuda:
        raise RuntimeError("Refusing false GPU execution: selected device is not CUDA")
    if device=="cpu" and cuda:
        raise RuntimeError("Requested Warp CPU oracle path is unexpectedly CUDA")

    source=load_verified_reference_axis()
    from .bullet_runtime import PHYSICS_DT
    import numpy as np

    init_positions,init_targets=_initial_conditions(envs,source,seed)
    q_np=np.array(init_positions,dtype=np.float32)
    target_np=np.array(init_targets,dtype=np.float32)
    v_np=np.zeros(envs,dtype=np.float32)
    device_start=time.perf_counter()
    q=wp.array(q_np,dtype=wp.float32,device=device)
    v=wp.array(v_np,dtype=wp.float32,device=device)
    target=wp.array(target_np,dtype=wp.float32,device=device)
    nq=wp.empty(envs,dtype=wp.float32,device=device)
    nv=wp.empty(envs,dtype=wp.float32,device=device)
    # Compile on separate warmup buffers; the source state is not advanced.
    warmq=wp.empty(envs,dtype=wp.float32,device=device)
    warmv=wp.empty(envs,dtype=wp.float32,device=device)
    inputs=[q,v,target,nq,nv,
            np.float32(source["lower_m"]),np.float32(source["upper_m"]),
            np.float32(source["max_speed_m_s"]),
            np.float32(source["max_effort_n"]),
            np.float32(source["effective_moving_mass_kg"]),
            np.float32(PHYSICS_DT)]
    wp.launch(_step_reduced_order_slide,dim=envs,
              inputs=[q,v,target,warmq,warmv,*inputs[5:]],device=device)
    wp.synchronize_device(device)
    allocation_compile_wall_s=time.perf_counter()-device_start
    kernel_start=time.perf_counter()
    for step in range(steps):
        wp.launch(_step_reduced_order_slide,dim=envs,
                  inputs=[q,v,target,nq,nv,*inputs[5:]],device=device)
        q,nq=nq,q
        v,nv=nv,v
    wp.synchronize_device(device)
    kernel_wall_seconds=time.perf_counter()-kernel_start
    transfer_start=time.perf_counter()
    final_q=q.numpy()
    final_v=v.numpy()
    transfer_wall_seconds=time.perf_counter()-transfer_start
    if not np.isfinite(final_q).all() or not np.isfinite(final_v).all():
        raise ArithmeticError("Warp compute returned nonfinite state")
    lo,hi=source["lower_m"],source["upper_m"]
    vmax=source["max_speed_m_s"]
    if (np.min(final_q)<lo-1e-4 or np.max(final_q)>hi+1e-4 or
        np.max(np.abs(final_v))>vmax+1e-4):
        raise ArithmeticError("Warp state violated source authored travel or speed")

    samples=[]
    worst_position_error=0.0
    worst_velocity_error=0.0
    for i in range(min(oracle_samples,envs)):
        position=float(q_np[i])
        speed=0.0
        target_value=float(target_np[i])
        for _ in range(steps):
            position,speed=_reference_step(
                position,speed,target_value,source,PHYSICS_DT
            )
        pos_delta=abs(position-float(final_q[i]))
        vel_delta=abs(speed-float(final_v[i]))
        worst_position_error=max(worst_position_error,pos_delta)
        worst_velocity_error=max(worst_velocity_error,vel_delta)
        samples.append({
            "environment_index":i,
            "initial_position_m":float(q_np[i]),
            "target_position_m":target_value,
            "final_solver_position_m":float(final_q[i]),
            "final_solver_velocity_m_s":float(final_v[i]),
            "scalar_reference_position_m":position,
            "position_residual_m":pos_delta,
            "velocity_residual_m_s":vel_delta,
        })
    position_pass=worst_position_error<=5e-4
    speed_pass=worst_velocity_error<=5e-4
    return {
        "schema":"dsiq-warp-reduced-order-gpu-batch-v1",
        "backend_id":"warp_reduced_order_slide_v1",
        "execution_device":str(selected),
        "execution_device_is_cuda":cuda,
        "actual_gpu_kernel_executed":cuda,
        "cpu_mode_is_explicit_semantic_oracle":not cuda,
        "uses_real_warp_compiled_kernel":True,
        "hardware_qualified_on_requested_gpu":False,
        "model":"one_degree_of_freedom_effective_mass_reference_actuator",
        "verified_reference_source":source,
        "envs":envs,"steps_per_env":steps,
        "total_actuator_updates":envs*steps,
        "fixed_timestep_s":PHYSICS_DT,
        "seed":seed,
        "estimated_min_device_array_bytes":envs*7*4,
        "explicit_device_array_budget_mib":memory_limit_mib,
        "allocation_and_first_compile_wall_seconds":allocation_compile_wall_s,
        "synchronized_kernel_wall_seconds":kernel_wall_seconds,
        "final_state_transfer_wall_seconds":transfer_wall_seconds,
        "synchronized_update_throughput_per_second":
            (envs*steps)/kernel_wall_seconds if kernel_wall_seconds>0 else None,
        "position_residual_max_m":worst_position_error,
        "velocity_residual_max_m_s":worst_velocity_error,
        "independent_scalar_samples":samples,
        "independent_oracle_passed":position_pass and speed_pass,
        "source_bounds_checked_for_all_environments":True,
        "no_hidden_cpu_fallback":True,
        "physical_equipment_controlled":False,
        "has_full_multibody_or_contacts":False,
        "scientific_limitations":[
            "Single-axis effective-mass approximation; no 3D rigid or articulated contacts",
            "Reference servo coefficients and parameters are not OEM calibrated",
            "Measured wall time includes Python kernel launch/sync overhead, not pure device time",
            "Hardware CUDA qualification requires this command to run on the actual target GPU",
        ],
    }


def inspect_warp_compute() -> dict:
    """Enumerate genuine GPU compute capability without allocating a scene."""
    if wp is None:
        return {"warp_installed":False,"cuda_available":False,
                "gpu_compute_qualified":False,
                "install":'pip install -e ".[gpu-compute]"'}
    wp.init()
    devices=[]
    for x in wp.get_cuda_devices():
        devices.append({"device":str(x),"name":str(x.name),
                        "is_cuda":bool(x.is_cuda)})
    return {"warp_installed":True,"warp_version":getattr(wp,"__version__",None),
            "cuda_available":bool(wp.is_cuda_available()),
            "cuda_devices":devices,
            "full_multibody_gpu_physics_implemented":False,
            "gpu_compute_qualified":False,
            "note":"GPU availability is not a kernel numerical/performance acceptance test"}
