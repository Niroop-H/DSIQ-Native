import unittest

from dsiq_native.hardware import GPU, calculate_budget, parse_nvidia_smi_csv


class HardwareTests(unittest.TestCase):
    def test_parse_actual_nvidia_smi_csv(self):
        row = '"NVIDIA GeForce RTX 3050 Laptop GPU", 4096, 3072, 595.97\n'
        gpus = parse_nvidia_smi_csv(row)
        self.assertEqual(len(gpus), 1)
        self.assertEqual(gpus[0].vram_total_mib, 4096)
        self.assertEqual(gpus[0].vram_free_mib, 3072)
        self.assertTrue(gpus[0].is_rtx)

    def test_parse_rejects_bad_memory(self):
        self.assertEqual(parse_nvidia_smi_csv("RTX 3050, 4096, 5000, 1\n"), [])
        self.assertEqual(parse_nvidia_smi_csv("broken row\n"), [])

    def test_no_nvidia_gpu_remains_headless(self):
        budget = calculate_budget(None, 16 * 1024)
        self.assertEqual(budget.mode, "headless_only")
        self.assertEqual(budget.managed_asset_budget_mib, 0)

    def test_4gb_3050_does_not_claim_isaac_compatibility(self):
        gpu = GPU("NVIDIA GeForce RTX 3050 Laptop GPU", 4096, 4096, "595.97")
        budget = calculate_budget(gpu, 16 * 1024)
        self.assertEqual(budget.mode, "experimental_lightweight")
        self.assertLessEqual(budget.managed_asset_budget_mib, 2048)
        self.assertGreaterEqual(budget.headroom_after_budget_mib, 1024)
        self.assertTrue(any("Kit minimum" in warning for warning in budget.warnings))
        self.assertTrue(any("Isaac Sim" in warning for warning in budget.warnings))

    def test_low_free_gpu_memory_rejects_rtx_experiment(self):
        gpu = GPU("NVIDIA GeForce RTX 3050", 4096, 700, "595.97")
        self.assertEqual(calculate_budget(gpu, 16 * 1024).mode, "headless_only")

    def test_low_system_ram_warns(self):
        gpu = GPU("NVIDIA GeForce RTX 3050", 4096, 4096, "595.97")
        result = calculate_budget(gpu, 8 * 1024)
        self.assertEqual(result.mode, "headless_only")
        self.assertTrue(any("16 GiB" in w for w in result.warnings))


if __name__ == "__main__":
    unittest.main()
