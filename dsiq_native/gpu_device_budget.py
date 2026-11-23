"""Conservative prelaunch CUDA device admission, not an OOM guarantee.

This reads the NVIDIA Warp Device's actual free/total memory on the selected
CUDA context. The array footprint check deliberately leaves uncommitted
headroom for the display, JIT modules, drivers and other applications.

Reported free/total memory is device-global point-in-time information, not
peak process VRAM. Other applications may change allocations immediately.
"""
from __future__ import annotations


def check_cuda_memory_admission(device, *, estimated_arrays_bytes: int,
                                reserve_mib: int=256) -> dict:
    if type(estimated_arrays_bytes) is not int or estimated_arrays_bytes<0:
        raise ValueError("Estimated CUDA allocation bytes must be nonnegative int")
    if type(reserve_mib) is not int or not 128<=reserve_mib<=1024:
        raise ValueError("CUDA reserve MiB must be 128..1024")
    if not getattr(device,"is_cuda",False):
        return {
            "cuda_admission_checked":False,
            "reserved_device_headroom_mib":reserve_mib,
            "note":"CPU Warp reference has no CUDA memory allocation",
        }
    try:
        total=int(device.total_memory)
        free=int(device.free_memory)
    except (TypeError,ValueError,AttributeError,RuntimeError) as exc:
        raise RuntimeError("CUDA device memory information unavailable; refusing unsafe allocation") from exc
    if not 0<free<=total:
        raise RuntimeError("CUDA device did not report valid free/total memory")
    required=estimated_arrays_bytes+reserve_mib*1024*1024
    if free<required:
        raise MemoryError(
            f"CUDA batch requires {estimated_arrays_bytes} device array bytes "
            f"and {reserve_mib} MiB headroom; only {free} bytes reported free"
        )
    return {
        "cuda_admission_checked":True,
        "cuda_device_total_memory_bytes":total,
        "cuda_device_free_before_allocation_bytes":free,
        "estimated_min_array_bytes":estimated_arrays_bytes,
        "reserved_device_headroom_mib":reserve_mib,
        "remaining_reported_headroom_bytes":free-required,
        "device_global_snapshot_not_process_peak":True,
        "out_of_memory_still_possible":True,
    }
