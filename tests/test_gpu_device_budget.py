"""CUDA memory preflight is deterministic even without NVIDIA device."""
from __future__ import annotations
from types import SimpleNamespace
import unittest

from dsiq_native.gpu_device_budget import check_cuda_memory_admission


class CUDAFreeMemoryGuardTests(unittest.TestCase):
    def test_cuda_admission_passes_with_reported_memory_and_headroom(self):
        gib=1024*1024*1024
        obj=SimpleNamespace(is_cuda=True,total_memory=4*gib,free_memory=3*gib)
        r=check_cuda_memory_admission(
            obj,estimated_arrays_bytes=8*1024*1024
        )
        self.assertTrue(r["cuda_admission_checked"])
        self.assertEqual(r["cuda_device_total_memory_bytes"],4*gib)
        self.assertEqual(r["reserved_device_headroom_mib"],256)
        self.assertTrue(r["out_of_memory_still_possible"])

    def test_gpu_allocation_blocks_low_free_memory(self):
        gib=1024*1024*1024
        obj=SimpleNamespace(is_cuda=True,total_memory=4*gib,
                            free_memory=128*1024*1024)
        with self.assertRaises(MemoryError):
            check_cuda_memory_admission(
                obj,estimated_arrays_bytes=64*1024*1024,
            )

    def test_untrustworthy_cuda_memory_fails_closed(self):
        gib=1024*1024*1024
        for values in (
            (0,0),(4*gib,0),(4*gib,-1),
            (4*gib,5*gib),
        ):
            with self.subTest(values=values):
                obj=SimpleNamespace(is_cuda=True,
                                    total_memory=values[0],
                                    free_memory=values[1])
                with self.assertRaises(RuntimeError):
                    check_cuda_memory_admission(obj,estimated_arrays_bytes=1)
        with self.assertRaises(RuntimeError):
            check_cuda_memory_admission(
                SimpleNamespace(is_cuda=True),estimated_arrays_bytes=1
            )

    def test_cpu_reference_requires_no_gpu_memory(self):
        obj=SimpleNamespace(is_cuda=False)
        report=check_cuda_memory_admission(
            obj,estimated_arrays_bytes=1024
        )
        self.assertFalse(report["cuda_admission_checked"])

    def test_malformed_budget_rejected(self):
        obj=SimpleNamespace(is_cuda=False)
        for bytes_val in (-1,True,1.5):
            with self.subTest(value=bytes_val):
                with self.assertRaises(ValueError):
                    check_cuda_memory_admission(
                        obj,estimated_arrays_bytes=bytes_val
                    )
        for reserve in (-1,0,1025,True):
            with self.subTest(reserve=reserve):
                with self.assertRaises(ValueError):
                    check_cuda_memory_admission(
                        obj,estimated_arrays_bytes=0,reserve_mib=reserve,
                    )


if __name__=="__main__":
    unittest.main()
