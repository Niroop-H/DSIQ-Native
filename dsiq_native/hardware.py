"""Device introspection and advisory budgets. Not a driver-level memory allocator.

We intentionally report the actual observed RTX device. A GeForce RTX 3050
can have 4, 6, or 8 GiB VRAM depending on SKU. 16 GiB system RAM is not VRAM.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import csv
import ctypes
from io import StringIO
import os
import re
import subprocess
from typing import Literal

MIB = 1024 * 1024
RTX_3050_LOWEST_VRAM_MIB = 4096
SYSTEM_RAM_TARGET_MIB = 16 * 1024


@dataclass(frozen=True)
class GPU:
    name: str
    vram_total_mib: int
    vram_free_mib: int
    driver: str

    @property
    def is_rtx(self) -> bool:
        return "RTX" in self.name.upper()


@dataclass(frozen=True)
class DeviceBudget:
    mode: Literal["headless_only", "experimental_lightweight"]
    system_ram_mib: int | None
    gpu: GPU | None
    managed_asset_budget_mib: int
    reserved_gpu_mib: int
    headroom_after_budget_mib: int
    warnings: tuple[str, ...]

    def as_dict(self) -> dict:
        d = asdict(self)
        d["warnings"] = list(self.warnings)
        return d


def _mib(value: str) -> int:
    m = re.search(r"(\d+)", value)
    if not m:
        raise ValueError(f"Cannot parse GPU memory from {value!r}")
    return int(m.group(1))


def parse_nvidia_smi_csv(text: str) -> list[GPU]:
    """Parse explicit nvidia-smi query fields; ignore malformed response rows."""
    result: list[GPU] = []
    for row in csv.reader(StringIO(text)):
        if len(row) != 4:
            continue
        try:
            name, total, free, driver = (v.strip() for v in row)
            t, f = _mib(total), _mib(free)
            if t < 1 or f > t or f < 0:
                continue
            result.append(GPU(name, t, f, driver))
        except ValueError:
            continue
    return result


def detect_nvidia_gpus() -> list[GPU]:
    args = [
        "nvidia-smi",
        "--query-gpu=name,memory.total,memory.free,driver_version",
        "--format=csv,noheader,nounits",
    ]
    try:
        proc = subprocess.run(
            args, capture_output=True, text=True,
            timeout=5, check=False,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return []
    return parse_nvidia_smi_csv(proc.stdout) if proc.returncode == 0 else []


def detect_system_ram_mib() -> int | None:
    """Physical RAM only (not paging file or unified GPU memory)."""
    if os.name == "nt":
        class MEMORYSTATUSEX(ctypes.Structure):
            _fields_ = [
                ("dwLength", ctypes.c_ulong),
                ("dwMemoryLoad", ctypes.c_ulong),
                ("ullTotalPhys", ctypes.c_ulonglong),
                ("ullAvailPhys", ctypes.c_ulonglong),
                ("ullTotalPageFile", ctypes.c_ulonglong),
                ("ullAvailPageFile", ctypes.c_ulonglong),
                ("ullTotalVirtual", ctypes.c_ulonglong),
                ("ullAvailVirtual", ctypes.c_ulonglong),
                ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
            ]
        mem = MEMORYSTATUSEX()
        mem.dwLength = ctypes.sizeof(mem)
        try:
            if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(mem)):
                return int(mem.ullTotalPhys // MIB)
        except (OSError, AttributeError):
            pass
        return None

    try:
        return int(os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE") // MIB)
    except (AttributeError, OSError, ValueError, TypeError):
        return None


def calculate_budget(gpu: GPU | None, system_ram_mib: int | None) -> DeviceBudget:
    """A conservative admission-control *estimate*, not an allocation guarantee.

    Budget represents owned/cached asset data only; no claim is made that NVIDIA
    Kit/RTX, operating system, driver, buffers, or physics fit inside this number.
    """
    warnings: list[str] = []
    if system_ram_mib is None:
        warnings.append("Physical system RAM could not be detected; native render readiness unverified.")
    elif system_ram_mib < SYSTEM_RAM_TARGET_MIB:
        warnings.append("Physical RAM is below DSIQ's 16 GiB design target.")
    if gpu is None:
        warnings.append("No NVIDIA GPU detected. USD authoring and validation remain available.")
        return DeviceBudget("headless_only", system_ram_mib, None, 0, 0, 0, tuple(warnings))
    if not gpu.is_rtx:
        warnings.append("Detected NVIDIA GPU is not identified as RTX; RTX rendering is not enabled.")
        return DeviceBudget("headless_only", system_ram_mib, gpu, 0, gpu.vram_total_mib, 0, tuple(warnings))

    reserved = max(1024, gpu.vram_total_mib // 3)
    # 50% of total VRAM maximum for assets, *also* respect currently free VRAM.
    owned_budget = max(0, min(gpu.vram_total_mib // 2, gpu.vram_free_mib - reserved))
    remaining = max(0, gpu.vram_free_mib - owned_budget)
    can_experiment = (gpu.vram_total_mib >= RTX_3050_LOWEST_VRAM_MIB
                      and owned_budget >= 512
                      and (system_ram_mib is None or system_ram_mib >= 12 * 1024))
    if "3050" in gpu.name:
        warnings.append(
            "RTX 3050 is below NVIDIA's published Kit minimum (RTX 3070); "
            "Kit GUI/RTX operation is experimental even when a budget is available."
        )
    if gpu.vram_total_mib < 16 * 1024:
        warnings.append("GPU VRAM is below Isaac Sim 6.0's published 16 GiB minimum.")
    warnings.append(
        "Managed-asset budget is advisory; Kit/RTX/driver allocations must be measured "
        "in a running scene. Do not assume guaranteed VRAM, FPS, or stability."
    )
    return DeviceBudget(
        "experimental_lightweight" if can_experiment else "headless_only",
        system_ram_mib, gpu, owned_budget, reserved, remaining, tuple(warnings),
    )


def inspect_device() -> DeviceBudget:
    ram = detect_system_ram_mib()
    gpus = detect_nvidia_gpus()
    # Choose the RTX device with the highest available memory, if present.
    gpu = max(gpus, key=lambda g: (g.is_rtx, g.vram_free_mib), default=None)
    return calculate_budget(gpu, ram)
