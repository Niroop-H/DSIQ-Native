"""SPDX inventory may enumerate actual local distributions, not invent rights."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.release_sbom import build_spdx_inventory


class SPDXInstalledComponentsTests(unittest.TestCase):
    def test_real_installed_dsiq_package_inventory(self):
        with TemporaryDirectory() as tmp:
            file=Path(tmp)/"spdx.json"
            report=build_spdx_inventory(file)
            self.assertEqual(report["schema"],"SPDX-2.3")
            self.assertTrue(file.is_file())
            payload=json.loads(file.read_text())
            self.assertEqual(payload["spdxVersion"],"SPDX-2.3")
            self.assertTrue(any(x["name"].lower()=="dsiq-native"
                                for x in payload["packages"]))
            self.assertTrue(all(x["licenseConcluded"]=="NOASSERTION"
                                for x in payload["packages"]))
            self.assertFalse(report["binary_dependencies_exhaustively_inventoried"])
            with self.assertRaises(FileExistsError):
                build_spdx_inventory(file)

if __name__=="__main__":
    unittest.main()
