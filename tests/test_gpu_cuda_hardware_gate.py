"""Real CUDA acceptance must fail closed, even with no GPU in CI."""
from __future__ import annotations
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from scripts.gpu_cuda_hardware_acceptance import hardware_acceptance, main


class CUDAHardwareEvidenceIntegrity(unittest.TestCase):
    def test_cpu_cannot_pass_cuda_hardware_gate(self):
        with self.assertRaisesRegex(ValueError,"explicit cuda"):
            hardware_acceptance(device="cpu")

    def test_missing_cuda_device_refuses_all_hardware_claims(self):
        with patch(
            "dsiq_native.gpu_vector_gantry.inspect_warp_compute",
            return_value={"cuda_available":False,"cuda_devices":[]},
        ):
            with self.assertRaisesRegex(RuntimeError,"No usable NVIDIA CUDA"):
                hardware_acceptance(device="cuda:0",
                                    one_axis_envs=4,xyz_envs=4,steps=2)

    def test_failure_never_creates_passing_evidence_file(self):
        with TemporaryDirectory() as root:
            destination=Path(root)/"unearned-gpu-certificate.json"
            with patch(
                "dsiq_native.gpu_vector_gantry.inspect_warp_compute",
                return_value={"cuda_available":False},
            ):
                with self.assertRaises(RuntimeError):
                    main(["--output",str(destination),
                          "--one-axis-envs","4","--xyz-envs","4",
                          "--steps","2"])
            self.assertFalse(destination.exists())


if __name__=="__main__":
    unittest.main()
