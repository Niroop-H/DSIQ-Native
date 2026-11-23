"""Provenance-bound machine mass, joint, and geometry inventory for ANY URDF.

No generated OEM specifications, no fake component dimensions: the report
uses only the rights-verified URDF and its actual declared material,
geometry, inertial and motor data. Numeric source declarations have NOT
been externally measured or calibrated by this inventory.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from .urdf_bridge import validate_robot_manifest


def machine_inventory(manifest_file: str | Path) -> dict:
    manifest, validated=validate_robot_manifest(manifest_file)
    path=(Path(manifest_file).resolve().parent /
          manifest["geometry"]["urdf_file"]).resolve()
    root=ET.parse(path).getroot()
    links=[]
    mass_total=0.
    visuals=0
    colliders=0
    primitive_count=0
    mesh_count=0
    for part in root.findall("link"):
        name=part.attrib["name"]
        inertial=part.find("inertial")
        mass=(float(inertial.find("mass").attrib["value"])
              if inertial is not None else None)
        tensor=(inertial.find("inertia").attrib
                if inertial is not None else None)
        if mass is not None:
            mass_total+=mass
        material=[]
        for visual in part.findall("visual"):
            for m in visual.findall("material"):
                material.append(m.attrib.get("name","unlabelled"))
        entries={
            "link_name":name,
            "declared_mass_kg":mass,
            "declared_inertia":{
                key:float(tensor[key])
                for key in ("ixx","iyy","izz","ixy","ixz","iyz")
            } if tensor is not None else None,
            "visual_count":len(part.findall("visual")),
            "collision_geometry_count":len(part.findall("collision")),
            "material_names":material,
        }
        visuals+=entries["visual_count"]
        colliders+=entries["collision_geometry_count"]
        for mode in ("visual","collision"):
            for shape in part.findall(mode):
                geometry=shape.find("geometry")
                if geometry is None or len(geometry)!=1:
                    continue  # validated importer will reject an invalid source
                if geometry[0].tag=="mesh":
                    mesh_count+=1
                else:
                    primitive_count+=1
        links.append(entries)
    joints=[]
    for part in root.findall("joint"):
        kind=part.attrib["type"]
        limit=part.find("limit")
        detail={
            "joint_name":part.attrib["name"],
            "joint_type":kind,
            "parent":part.find("parent").attrib["link"],
            "child":part.find("child").attrib["link"],
            "declared_effort":float(limit.attrib["effort"])
                 if limit is not None else None,
            "declared_max_speed":float(limit.attrib["velocity"])
                 if limit is not None else None,
            "declared_lower":float(limit.attrib["lower"])
                 if limit is not None and "lower" in limit.attrib else None,
            "declared_upper":float(limit.attrib["upper"])
                 if limit is not None and "upper" in limit.attrib else None,
        }
        joints.append(detail)
    return {
        "schema":"dsiq-rights-verified-machine-inventory-v1",
        "asset_id":manifest["asset_id"],
        "asset_class":manifest["asset_class"],
        "model":manifest["model"],
        "revision":manifest["revision"],
        "source_sha256":manifest["geometry"]["sha256"],
        "link_count":len(links),
        "joint_count":len(joints),
        "actuated_joint_count":sum(j["joint_type"]!="fixed" for j in joints),
        "declared_total_mass_kg":mass_total if all(
            x["declared_mass_kg"] is not None for x in links
        ) else None,
        "sum_is_not_measured_mass":True,
        "visual_subcomponents":visuals,
        "collision_subcomponents":colliders,
        "mesh_references":mesh_count,
        "primitive_geometries":primitive_count,
        "links":links,"joints":joints,
        "rights_and_source_integrity_verified":True,
        "real_oem_dimensions_qualified":False,
        "real_machine_dynamics_qualified":False,
    }
