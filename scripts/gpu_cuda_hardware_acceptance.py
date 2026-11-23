"""DSIQ Native: strictly hardware-bound NVIDIA CUDA Warp acceptance.

Windows RTX3050 / CUDA workstation operator command:
    python scripts/gpu_cuda_hardware_acceptance.py --output build/gpu-cuda-hardware.json

All CUDA kernels must actually execute on the explicitly requested
NVIDIA device. Exits nonzero before writing if GPU/driver/kernel/oracle
checks fail. No GUI Mesa or CPU Warp run can create a passing record.
"""
from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path
import subprocess
import sys
from datetime import datetime,timezone


def _driver_identification():
    try:
        result=subprocess.run(
            ["nvidia-smi",
             "--query-gpu=index,name,driver_version,memory.total,memory.free",
             "--format=csv,noheader,nounits"],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,
            text=True,timeout=10,check=False,
        )
    except (OSError,subprocess.TimeoutExpired):
        return {"nvidia_smi_available":False,"note":"CUDA kernel proof is required independently"}
    return {
        "nvidia_smi_available":result.returncode==0,
        "gpu_rows":result.stdout.strip().splitlines()
                   if result.returncode==0 else [],
        "nvidia_smi_exit_code":result.returncode,
        "note":"Memory snapshots are system-global; not isolated DSIQ GPU allocation",
    }


def hardware_acceptance(*,device: str="cuda:0",
                        one_axis_envs: int=4096,
                        xyz_envs: int=16384,
                        steps: int=240,
                        seed: int=1007) -> dict:
    from dsiq_native.gpu_vector_gantry import (
        benchmark_warp_reduced_order,inspect_warp_compute
    )
    from dsiq_native.gpu_vector_xyz import benchmark_warp_xyz

    if not isinstance(device,str) or not device.startswith("cuda:"):
        raise ValueError("CUDA hardware acceptance requires an explicit cuda:<ordinal> device")
    probe=inspect_warp_compute()
    if not probe.get("cuda_available",False):
        raise RuntimeError("No usable NVIDIA CUDA device; hardware acceptance is blocked")
    single=benchmark_warp_reduced_order(
        device=device,envs=one_axis_envs,steps=steps,seed=seed,
        oracle_samples=16,
    )
    xyz=benchmark_warp_xyz(
        device=device,envs=xyz_envs,steps=steps,seed=seed,
        oracle_samples=16,
    )
    for label,r in (("one_axis",single),("xyz",xyz)):
        if (not r["independent_oracle_passed"] or
            not r["no_hidden_cpu_fallback"] if label=="one_axis"
            else False):
            # Direct assertions below are kept separate for clear feedback.
            raise AssertionError("Unexpected Warp benchmark status")
        if r["execution_device"]!=device:
            raise AssertionError("Warp compute did not run on requested device")
    if not single["actual_gpu_kernel_executed"] or not single["execution_device_is_cuda"]:
        raise AssertionError("One-axis kernel did not execute on real NVIDIA CUDA")
    if not xyz["actual_cuda_kernel_executed"] or not xyz["execution_is_cuda"]:
        raise AssertionError("XYZ kernel did not execute on real NVIDIA CUDA")
    if not single["independent_oracle_passed"] or not xyz["independent_oracle_passed"]:
        raise AssertionError("A real GPU kernel failed source-bounded scalar conformance")
    if not xyz["no_silent_device_fallback"] or not single["no_hidden_cpu_fallback"]:
        raise AssertionError("CUDA device policy was violated")
    if not xyz["source_joint_travel_and_speed_checked_all_envs"]:
        raise AssertionError("One of XYZ source travel/speed checks failed")

    return {
        "schema":"dsiq-real-nvidia-cuda-hardware-acceptance-v1",
        "collected_utc":datetime.now(timezone.utc).isoformat(),
        "python":sys.version.split()[0],
        "os":platform.platform(),
        "device_requested":device,
        "hardware_probe":probe,
        "nvidia_driver_metadata":_driver_identification(),
        "one_axis":single,
        "xyz_three_axis":xyz,
        "actual_cuda_device_kernel_execution_validated":True,
        "numerical_model_reference_acceptance_passed":True,
        "full_multibody_gpu_robot_physics_qualified":False,
        "real_oem_machine_calibration_qualified":False,
        "industrial_robot_safety_certified":False,
        "renderer_gpu_vulkan_or_hydra_qualified":False,
        "performance_scope":"synchronized Warp batch wall-time excluding JIT compile and host copy",
    }


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",required=True,
                        help="New JSON evidence file; refuses to overwrite")
    parser.add_argument("--device",default="cuda:0")
    parser.add_argument("--one-axis-envs",type=int,default=4096)
    parser.add_argument("--xyz-envs",type=int,default=16384)
    parser.add_argument("--steps",type=int,default=240)
    parser.add_argument("--seed",type=int,default=1007)
    args=parser.parse_args(argv)
    report=hardware_acceptance(
        device=args.device,
        one_axis_envs=args.one_axis_envs,
        xyz_envs=args.xyz_envs,steps=args.steps,seed=args.seed,
    )
    output=Path(args.output).resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("x",encoding="utf-8") as stream:
        json.dump(report,stream,indent=2,sort_keys=True)
        stream.write("\n")
    print(json.dumps({
        "hardware_cuda_accepted":True,
        "evidence_path":str(output),
        "cuda_device":args.device,
        "one_axis_updates":report["one_axis"]["total_actuator_updates"],
        "xyz_axis_updates":report["xyz_three_axis"]["total_axis_updates"],
        "industrial_safety_qualified":False,
    },indent=2))


if __name__=="__main__":
    main()
