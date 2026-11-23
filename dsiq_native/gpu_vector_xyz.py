"""GPU-first batched XYZ actuator reference using real NVIDIA Warp kernels.

One GPU thread per independent axis of each source-bound reference gantry.
Actual CUDA when device='cuda:0' and a valid GPU exists; no CPU fallback.
'cpu' is ONLY an explicitly selected compiled-kernel correctness target.

This is three independent 1D effective-mass slides, not 3D articulated-body
physics: no off-diagonal mass matrix, constraint coupling, friction/contact,
tool inertia reflection, accurate vertical gravity load, OEM servo, or safety.
Do not use its positions to command real machinery.
"""
from __future__ import annotations

import math
import random
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory

from .gpu_vector_gantry import _reference_step, wp


def source_verified_xyz_parameters() -> dict:
    """Parse exact mechanical limits and downstream subtree masses in URDF."""
    from .machine_designer import MachineDesign, save_project
    from .urdf_bridge import validate_robot_manifest
    with TemporaryDirectory(prefix="dsiq-warp-xyz-") as root:
        files=save_project(MachineDesign(asset_id="warp_xyz_reference"),
                           Path(root)/"asset")
        manifest, inspected=validate_robot_manifest(files["manifest"])
        xml=ET.parse(inspected.path).getroot()
        links={x.get("name"):x for x in xml.findall("link")}
        joints={x.get("name"):x for x in xml.findall("joint")}
        children={}
        for joint in joints.values():
            parent=joint.find("parent")
            child=joint.find("child")
            if parent is None or child is None:
                raise ValueError("Source URDF joint parent/child absent")
            children.setdefault(parent.get("link"),[]).append(child.get("link"))

        def subtree_mass(root_name):
            seen=set()
            stack=[root_name]
            total=0.0
            while stack:
                link_name=stack.pop()
                if link_name in seen or link_name not in links:
                    raise ValueError("Source URDF subtree invalid or cyclic")
                seen.add(link_name)
                part=links[link_name]
                inertia=part.find("inertial")
                mass=inertia.find("mass") if inertia is not None else None
                if mass is None:
                    raise ValueError("Moving source link lacks inertial mass")
                m=float(mass.get("value","nan"))
                if not math.isfinite(m) or m<=0:
                    raise ValueError("Moving source mass must be positive finite")
                total+=m
                stack.extend(children.get(link_name,()))
            return total

        axes=[]
        for name in ("actuator_x","actuator_y","actuator_z"):
            j=joints.get(name)
            if j is None or j.get("type")!="prismatic":
                raise ValueError("Source gantry prismatic axis missing: "+name)
            child=j.find("child")
            limit=j.find("limit")
            if child is None or limit is None:
                raise ValueError("Source axis has no child or motor limits")
            lower=float(limit.get("lower","nan"))
            upper=float(limit.get("upper","nan"))
            speed=float(limit.get("velocity","nan"))
            effort=float(limit.get("effort","nan"))
            mass=subtree_mass(child.get("link"))
            if (not all(math.isfinite(x)
                        for x in (lower,upper,speed,effort,mass)) or
                not 0<=lower<upper<=2.5 or speed<=0 or effort<=0):
                raise ValueError("Source actuator has invalid units or bounds")
            axes.append({
                "name":name,
                "lower_m":lower,"upper_m":upper,
                "max_speed_m_s":speed,"max_effort_n":effort,
                "effective_moving_mass_kg":mass,
            })
        return {
            "source_urdf_sha256":manifest["geometry"]["sha256"],
            "source_asset_class":manifest["asset_class"],
            "axes":axes,
            "moving_subtree_mass_not_multibody_mass_matrix":True,
        }


if wp is not None:
    @wp.kernel
    def _xyz_axis_step(
        q: wp.array(dtype=wp.float32),
        v: wp.array(dtype=wp.float32),
        targets: wp.array(dtype=wp.float32),
        new_q: wp.array(dtype=wp.float32),
        new_v: wp.array(dtype=wp.float32),
        lower: wp.array(dtype=wp.float32),
        upper: wp.array(dtype=wp.float32),
        speed_limit: wp.array(dtype=wp.float32),
        effort_limit: wp.array(dtype=wp.float32),
        mass: wp.array(dtype=wp.float32),
        dt: wp.float32,
    ):
        i=wp.tid()
        axis=i%3
        position=q[i]
        velocity=v[i]
        speed_max=speed_limit[axis]
        effort_max=effort_limit[axis]
        accel_force=wp.clamp(
            wp.float32(450.0)*
            (wp.clamp(wp.float32(12.0)*(targets[i]-position),
                      -speed_max,speed_max)-velocity),
            -effort_max,effort_max,
        )
        vel_new=wp.clamp(
            velocity+dt*accel_force/mass[axis],-speed_max,speed_max,
        )
        pos_new=position+dt*vel_new
        if pos_new<lower[axis]:
            pos_new=lower[axis]
            if vel_new<wp.float32(0.0):
                vel_new=wp.float32(0.0)
        if pos_new>upper[axis]:
            pos_new=upper[axis]
            if vel_new>wp.float32(0.0):
                vel_new=wp.float32(0.0)
        new_q[i]=pos_new
        new_v[i]=vel_new


def benchmark_warp_xyz(*,envs: int=16384,steps: int=240,
                       seed: int=1007,device: str="cuda:0",
                       memory_limit_mib: int=64,
                       oracle_samples: int=8) -> dict:
    """Run native 3-axis kernel, bound device memory, verify scalar dynamics."""
    from .gpu_vector_gantry import validate_batch_options
    # This batch executes three independent kernels' worth of axis threads
    # per environment, and must obey the same absolute update admission cap.
    if type(envs) is not int or not 1<=envs<=65536:
        raise ValueError("Warp XYZ environment count must be 1..65536")
    validate_batch_options(envs=envs,steps=steps,seed=seed,
                           memory_limit_mib=memory_limit_mib)
    if 3*envs*steps>80_000_000:
        raise ValueError("Warp XYZ total axis updates exceed 80 million")
    if envs*3*7*4+5*3*4>memory_limit_mib*1024*1024:
        raise ValueError("Warp XYZ working arrays exceed memory budget")
    if type(oracle_samples) is not int or not 1<=oracle_samples<=32:
        raise ValueError("Warp XYZ scalar verification samples must be 1..32")
    if not isinstance(device,str) or not (
        device=="cpu" or (device.startswith("cuda:") and
                          device[5:].isdigit() and len(device)<=8)):
        raise ValueError("XYZ device must be explicit cpu or cuda:<ordinal>")
    if wp is None:
        raise RuntimeError('NVIDIA Warp required: pip install -e ".[gpu-compute]"')
    wp.config.log_level=wp.LOG_WARNING
    wp.init()
    if device.startswith("cuda:") and not wp.is_device_available(device):
        raise RuntimeError("Requested CUDA GPU unavailable; refusing CPU fallback")
    selected=wp.get_device(device)
    cuda=bool(selected.is_cuda)
    if cuda != device.startswith("cuda:"):
        raise RuntimeError("Warp device type does not match requested execution")

    params=source_verified_xyz_parameters()
    from .bullet_runtime import PHYSICS_DT
    import numpy as np
    rng=random.Random(seed)
    q0=[]
    goals=[]
    for i in range(envs):
        for axis in params["axes"]:
            lo,hi=axis["lower_m"],axis["upper_m"]
            q0.append(lo+(hi-lo)*rng.random())
            goals.append(lo+(hi-lo)*rng.random())
    q_initial=np.asarray(q0,dtype=np.float32)
    target_initial=np.asarray(goals,dtype=np.float32)
    zeros=np.zeros(envs*3,dtype=np.float32)
    t0=time.perf_counter()
    q=wp.array(q_initial,dtype=wp.float32,device=device)
    v=wp.array(zeros,dtype=wp.float32,device=device)
    targets=wp.array(target_initial,dtype=wp.float32,device=device)
    nq=wp.empty(envs*3,dtype=wp.float32,device=device)
    nv=wp.empty(envs*3,dtype=wp.float32,device=device)
    wq=wp.empty(envs*3,dtype=wp.float32,device=device)
    wv=wp.empty(envs*3,dtype=wp.float32,device=device)
    def param(field):
        return wp.array(
            np.asarray([a[field] for a in params["axes"]],dtype=np.float32),
            dtype=wp.float32,device=device,
        )
    lower=param("lower_m")
    upper=param("upper_m")
    speed_max=param("max_speed_m_s")
    effort_max=param("max_effort_n")
    moving_mass=param("effective_moving_mass_kg")
    settings=[lower,upper,speed_max,effort_max,moving_mass,
              np.float32(PHYSICS_DT)]
    wp.launch(_xyz_axis_step,dim=envs*3,
              inputs=[q,v,targets,wq,wv,*settings],device=device)
    wp.synchronize_device(device)
    setup_s=time.perf_counter()-t0

    start=time.perf_counter()
    for _ in range(steps):
        wp.launch(_xyz_axis_step,dim=envs*3,
                  inputs=[q,v,targets,nq,nv,*settings],device=device)
        q,nq=nq,q
        v,nv=nv,v
    wp.synchronize_device(device)
    execution_s=time.perf_counter()-start
    copy_start=time.perf_counter()
    final_q=q.numpy()
    final_v=v.numpy()
    copy_s=time.perf_counter()-copy_start
    if not np.isfinite(final_q).all() or not np.isfinite(final_v).all():
        raise ArithmeticError("Warp XYZ state contains nonfinite positions/speeds")
    for axis_id,axis in enumerate(params["axes"]):
        values=final_q[axis_id::3]
        speeds=final_v[axis_id::3]
        if (np.min(values)<axis["lower_m"]-1e-5 or
            np.max(values)>axis["upper_m"]+1e-5 or
            np.max(np.abs(speeds))>axis["max_speed_m_s"]+1e-5):
            raise ArithmeticError("Warp XYZ violated source-authored motor bounds")
    samples=[]
    max_pos=0.0;max_vel=0.0
    for e in range(min(oracle_samples,envs)):
        record={"environment":e,"axes":[]}
        for axis_id,axis in enumerate(params["axes"]):
            idx=e*3+axis_id
            position=float(q_initial[idx])
            velocity=0.0
            target=float(target_initial[idx])
            for _ in range(steps):
                position,velocity=_reference_step(
                    position,velocity,target,axis,PHYSICS_DT)
            diffq=abs(position-float(final_q[idx]))
            diffv=abs(velocity-float(final_v[idx]))
            max_pos=max(max_pos,diffq)
            max_vel=max(max_vel,diffv)
            record["axes"].append({
                "name":axis["name"],"target_m":target,
                "solver_position_m":float(final_q[idx]),
                "solver_velocity_m_s":float(final_v[idx]),
                "reference_position_m":position,
                "position_abs_error_m":diffq,
                "velocity_abs_error_m_s":diffv,
            })
        samples.append(record)
    return {
        "schema":"dsiq-warp-three-axis-reduced-order-batch-v1",
        "backend_id":"warp_xyz_independent_actuators_v1",
        "execution_device":str(selected),
        "execution_is_cuda":cuda,
        "actual_cuda_kernel_executed":cuda,
        "explicit_cpu_reference_mode":not cuda,
        "warp_compiled_kernel_executed":True,
        "envs":envs,"axes_per_env":3,"steps_per_env":steps,
        "total_axis_updates":envs*3*steps,
        "seed":seed,
        "source_verified_urdf_sha256":params["source_urdf_sha256"],
        "source_axis_limits":params["axes"],
        "estimated_device_arrays_bytes":envs*3*7*4+5*3*4,
        "memory_budget_mib":memory_limit_mib,
        "warmup_compile_allocation_wall_s":setup_s,
        "synchronized_kernel_wall_s":execution_s,
        "final_host_transfer_wall_s":copy_s,
        "synchronized_axis_updates_per_s":(
            envs*3*steps/execution_s if execution_s>0 else None),
        "independent_reference_max_position_error_m":max_pos,
        "independent_reference_max_velocity_error_m_s":max_vel,
        "independent_oracle_passed":max_pos<=5e-4 and max_vel<=5e-4,
        "source_joint_travel_and_speed_checked_all_envs":True,
        "independent_oracle_samples":samples,
        "not_a_general_multibody_physics_engine":True,
        "not_qualified_for_hardware_robot_control":True,
        "no_silent_device_fallback":True,
        "limitations":[
            "Three uncoupled effective-mass actuators; excludes multibody mass matrix and contacts",
            "No vertical gravity load, OEM controller, tool dynamics or motor calibration",
            "Source URDF verified but reference geometry not independently OEM qualified",
            "Real CUDA hardware acceptance required separately; CPU run does not verify GPU",
        ],
    }
