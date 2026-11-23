"""GPU source CAD pipeline tests with authored OBJ mesh, SHA lock, exact geometry."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

from dsiq_native.cad_geometry import extract_verified_urdf_geometry,lock_mesh_hashes
from dsiq_native.machine_designer import MachineDesign,save_project


def make_locked_fixture(path: Path):
    project=save_project(
        MachineDesign(asset_id="source_locked_gpu_mesh"),path/"machine"
    )
    obj=path/"machine"/"source_panel.obj"
    obj.write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\n",
        encoding="utf8",
    )
    urdf=Path(project["urdf"])
    root=ET.parse(urdf).getroot()
    visual=root.find("link").find("visual")
    geometry=visual.find("geometry")
    geometry.clear()
    ET.SubElement(geometry,"mesh",{
        "filename":"source_panel.obj",
        "scale":"0.5 0.5 0.5",
    })
    ET.indent(root,space="  ")
    urdf.write_bytes(ET.tostring(root,encoding="utf8"))
    manifest=Path(project["manifest"])
    doc=json.loads(manifest.read_text())
    doc["geometry"]["sha256"]=hashlib.sha256(urdf.read_bytes()).hexdigest()
    doc["geometry"]["mesh_sha256"]={
        "source_panel.obj":hashlib.sha256(obj.read_bytes()).hexdigest()
    }
    manifest.write_text(json.dumps(doc,indent=2)+"\n")
    return manifest,obj,project


class CADGPUContracts(unittest.TestCase):
    def test_generated_source_cad_not_fabricated_dimensions(self):
        with TemporaryDirectory() as tmp:
            project=save_project(MachineDesign(asset_id="gpu_geometric_gantry"),
                                 Path(tmp)/"gantry")
            result=extract_verified_urdf_geometry(project["manifest"])
            self.assertEqual(result["asset_id"],"gpu_geometric_gantry")
            self.assertEqual(result["part_count"],17)
            self.assertGreater(result["triangle_count"],200)
            self.assertTrue(result["source_mesh_hashes_validated"])
            self.assertTrue(all(part.source in ("box","cylinder","sphere")
                                for part in result["visual_parts"]))
            self.assertFalse(result["independent_oem_calibration"])
            with self.assertRaisesRegex(ValueError,"budget"):
                extract_verified_urdf_geometry(project["manifest"],max_triangles=12)

    def test_provenance_lock_hint_does_not_modify_manifest(self):
        with TemporaryDirectory() as tmp:
            manifest,source,_=make_locked_fixture(Path(tmp))
            previous=manifest.read_bytes()
            mapping=lock_mesh_hashes(manifest)
            self.assertEqual(mapping["count"],1)
            self.assertEqual(mapping["mesh_sha256"]["source_panel.obj"],
                             hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertEqual(manifest.read_bytes(),previous)


@unittest.skipUnless(importlib.util.find_spec("trimesh"),
                     "Optional licensed CAD mesh parser required")
class CADMeshGeometryTests(unittest.TestCase):
    def test_preserves_real_authored_obj_vertices_faces_and_scale(self):
        with TemporaryDirectory() as tmp:
            manifest,source,_=make_locked_fixture(Path(tmp))
            result=extract_verified_urdf_geometry(manifest)
            part=next(x for x in result["visual_parts"] if x.source=="source_panel.obj")
            self.assertEqual(part.triangle_count,1)
            self.assertEqual(len(part.vertices),3)
            # A URDF visual origin is preserved, so verify its declared
            # 0.5m mesh scale by relative vertex extents, not absolute XYZ.
            self.assertAlmostEqual(
                max(v[0] for v in part.vertices)-min(v[0] for v in part.vertices),
                .5,places=6,
            )
            self.assertAlmostEqual(
                max(v[1] for v in part.vertices)-min(v[1] for v in part.vertices),
                .5,places=6,
            )
            self.assertEqual(part.source_sha256,hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertEqual(result["source_urdf_sha256"],
                hashlib.sha256(Path(manifest.parent/"machine.urdf").read_bytes()).hexdigest())
            with source.open("a",encoding="utf8") as f:
                f.write("\n# modified source content\n")
            with self.assertRaisesRegex(ValueError,"SHA256 mismatch"):
                extract_verified_urdf_geometry(manifest)


@unittest.skipUnless(importlib.util.find_spec("trimesh") and
                     importlib.util.find_spec("pxr") and
                     importlib.util.find_spec("pybullet"),
                     "Requires CAD parser, real Bullet and Pixar USD")
class VerifiedCADOpenUSDTests(unittest.TestCase):
    def test_actual_source_obj_roundtrip_with_link_motion(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.usd_cad_motion import record_source_cad_usd
        from pxr import Usd,UsdGeom
        with TemporaryDirectory() as tmp:
            manifest,_,_=make_locked_fixture(Path(tmp))
            out=Path(tmp)/"source-cad-motion.usda"
            with BulletPlayground(BulletSettings(payload_count=0,
                                                  gravity_m_s2=0.)) as world:
                model=world.import_urdf_robot(str(manifest),base_position_m=(-1.,1.3,0.))
                world.command_imported_joint(model["asset_id"],"actuator_x",.25)
                result=record_source_cad_usd(world,manifest,out,frames=4,steps_per_frame=12)
                self.assertEqual(result["source_triangles"],extract_verified_urdf_geometry(manifest)["triangle_count"])
                self.assertTrue(result["source_mesh_hashes_verified"])
                self.assertFalse(result["geometry_substituted"])
                stage=Usd.Stage.Open(str(out))
                source_meshes=[p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
                self.assertEqual(len(source_meshes),result["visual_meshes"])
                obj_mesh=next(p for p in source_meshes
                              if p.GetCustomDataByKey("dsiq_source_type")=="source_panel.obj")
                self.assertEqual(len(obj_mesh.GetAttribute("points").Get()),3)
                self.assertEqual(list(obj_mesh.GetAttribute("faceVertexCounts").Get()),[3])
                world_link=next(p for p in stage.Traverse()
                                if p.GetCustomDataByKey("dsiq_original_urdf_link")=="x_axis_carriage")
                self.assertEqual(world_link.GetAttribute("xformOp:translate").GetTimeSamples(),
                                 [12.,24.,36.,48.])
                self.assertFalse(stage.GetPrimAtPath("/World/SourceCAD").GetCustomDataByKey(
                    "dsiq_physical_oem_calibrated"
                ))


if __name__=="__main__":
    unittest.main()
