"""Design, export and simulate a mechanically meaningful XYZ gantry reference.

Unlike a stack of decorative random boxes, this machine has named structural
frames, three actual prismatic actuators, mechanical travel limits, inertial
properties, a tool flange, and colliding bodies. Geometry is deliberately
PARAMETRIC ENGINEERING REFERENCE, not OEM CAD or calibrated fabrication data.

Saved projects are editable JSON; generated URDF can be loaded directly by
the existing verified URDF/real Bullet import pipeline.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

_ID = re.compile(r"^[a-z][a-z0-9_]{2,46}$")
SCHEMA = "dsiq-machine-v1"
PROVENANCE = "USER_AUTHORED_PARAMETRIC_REFERENCE_NOT_OEM"


@dataclass(frozen=True)
class MachineDesign:
    asset_id: str = "dsiq_xyz_inspection_gantry"
    x_travel_m: float = 1.20
    y_travel_m: float = .80
    z_travel_m: float = .45
    clearance_height_m: float = 1.30
    frame_mass_kg: float = 20.
    x_carriage_mass_kg: float = 6.
    y_carriage_mass_kg: float = 4.
    z_carriage_mass_kg: float = 2.
    tool_mass_kg: float = .8
    x_motor_effort_n: float = 250.
    y_motor_effort_n: float = 160.
    z_motor_effort_n: float = 220.

    def validate(self):
        if not isinstance(self.asset_id, str) or not _ID.fullmatch(self.asset_id):
            raise ValueError("asset_id must be lowercase snake_case, 3–47 characters")
        ranges = {
            "x_travel_m":(.25,2.5), "y_travel_m":(.25,2.5),
            "z_travel_m":(.1,1.0), "clearance_height_m":(.8,2.5),
            "frame_mass_kg":(2.,250.), "x_carriage_mass_kg":(.2,100.),
            "y_carriage_mass_kg":(.2,100.), "z_carriage_mass_kg":(.2,100.),
            "tool_mass_kg":(.1,25.), "x_motor_effort_n":(30.,2000.),
            "y_motor_effort_n":(30.,2000.), "z_motor_effort_n":(30.,2000.),
        }
        for key,(lo,hi) in ranges.items():
            value = getattr(self,key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError(f"{key} must be finite SI numeric")
            if not lo <= value <= hi:
                raise ValueError(f"{key} outside engineering reference envelope [{lo},{hi}]")
        if self.clearance_height_m < self.z_travel_m + .30:
            raise ValueError("Gantry clearance is less than vertical travel plus tool safety margin")

    def to_dict(self):
        return {"schema":SCHEMA,"fidelity":"parametric_reference",
                "mechanism":"xyz_gantry","parameters":asdict(self),
                "limitations":"Not OEM CAD; mass and stiffness are reference estimates"}

    @classmethod
    def from_dict(cls, source):
        if not isinstance(source,dict) or source.get("schema")!=SCHEMA or source.get("mechanism")!="xyz_gantry":
            raise ValueError("Expected DSIQ v1 XYZ gantry machine JSON")
        params=source.get("parameters")
        if not isinstance(params,dict):
            raise ValueError("parameters must be an object")
        allowed={f.name for f in fields(cls)}
        if set(params)-allowed:
            raise ValueError("Unknown gantry parameter(s): " + ", ".join(sorted(set(params)-allowed)))
        spec=cls(**params)
        spec.validate()
        return spec


def load_design(path: str | Path) -> MachineDesign:
    file=Path(path).resolve()
    if file.stat().st_size>64*1024:
        raise ValueError("Machine specification exceeds 64 KiB")
    return MachineDesign.from_dict(json.loads(file.read_text(encoding="utf-8")))


def _vector(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def _origin(parent, xyz=(0.,0.,0.), rpy=(0.,0.,0.)):
    return ET.SubElement(parent,"origin",{"xyz":_vector(xyz),"rpy":_vector(rpy)})


def _geometry(parent, kind, dimensions):
    geometry=ET.SubElement(parent,"geometry")
    if kind=="box":
        ET.SubElement(geometry,"box",{"size":_vector(dimensions)})
    elif kind=="cylinder":
        ET.SubElement(geometry,"cylinder",{
            "radius":f"{dimensions[0]:.9g}","length":f"{dimensions[1]:.9g}"})
    else:
        raise ValueError("Unsupported mechanical component geometry")


def _segment(link, name, kind, dimensions, at, rgb):
    """Same geometry for visual and collision on parametric reference components."""
    for section in ("visual","collision"):
        node=ET.SubElement(link,section,{"name":name+"_"+section})
        _origin(node,at)
        _geometry(node,kind,dimensions)
        if section=="visual":
            material=ET.SubElement(node,"material",{"name":name+"_paint"})
            ET.SubElement(material,"color",{"rgba":_vector((*rgb,1.0))})


def _inertial(link, mass, dims, center=(0.,0.,0.)):
    """Positive principal reference inertia approximation from bounding cuboid."""
    x,y,z=dims
    ixx=mass*(y*y+z*z)/12
    iyy=mass*(x*x+z*z)/12
    izz=mass*(x*x+y*y)/12
    node=ET.SubElement(link,"inertial")
    _origin(node,center)
    ET.SubElement(node,"mass",{"value":f"{mass:.9g}"})
    ET.SubElement(node,"inertia",{
        "ixx":f"{ixx:.9g}","ixy":"0","ixz":"0",
        "iyy":f"{iyy:.9g}","iyz":"0","izz":f"{izz:.9g}"})


def _joint(root,name,kind,parent,child,origin,axis,lower=None,upper=None,effort=100.,velocity=.8):
    node=ET.SubElement(root,"joint",{"name":name,"type":kind})
    ET.SubElement(node,"parent",{"link":parent})
    ET.SubElement(node,"child",{"link":child})
    _origin(node,origin)
    ET.SubElement(node,"axis",{"xyz":_vector(axis)})
    if kind!="fixed":
        ET.SubElement(node,"limit",{
            "lower":f"{lower:.9g}", "upper":f"{upper:.9g}",
            "effort":f"{effort:.9g}", "velocity":f"{velocity:.9g}"})


def build_urdf(spec: MachineDesign) -> bytes:
    """Generate a complete named machine with 3 physical linear slides."""
    spec.validate()
    a,b,z=spec.x_travel_m,spec.y_travel_m,spec.z_travel_m
    h=spec.clearance_height_m
    root=ET.Element("robot",{"name":spec.asset_id})
    base=ET.SubElement(root,"link",{"name":"fixed_structural_frame"})
    _inertial(base,spec.frame_mass_kg,(a+.55,b+.55,h),center=(0.,0.,h/2))
    # Four uprights, real longitudinal gantry rails and two cross braces.
    # These are legitimate parametric structural elements, not claimed OEM CAD.
    for ix in (-1,1):
        for iy in (-1,1):
            _segment(base,f"upright_{ix}_{iy}","box",(.085,.085,h),
                     (ix*(a/2+.20),iy*(b/2+.20),h/2),
                     (.30,.37,.44))
    for iy in (-1,1):
        _segment(base,f"x_axis_rail_{iy}","box",(a+.50,.095,.095),
                 (0.,iy*(b/2+.20),h),
                 (.64,.70,.75))
    for ix in (-1,1):
        _segment(base,f"transverse_stiffener_{ix}","box",(.09,b+.50,.08),
                 (ix*(a/2+.20),0.,h),
                 (.34,.40,.46))
    cross=ET.SubElement(root,"link",{"name":"x_axis_carriage"})
    _inertial(cross,spec.x_carriage_mass_kg,(.22,b+.16,.18),
              center=(0.,0.,0.))
    _segment(cross,"x_slide_crossbeam","box",(.22,b+.16,.13),
             (0.,0.,0.),(.15,.40,.72))
    for iy in (-1,1):
        _segment(cross,f"x_linear_bearing_{iy}","box",(.26,.13,.18),
                 (0.,iy*b/2,0.),(.22,.25,.29))
    _joint(root,"actuator_x","prismatic","fixed_structural_frame",
           "x_axis_carriage",(-a/2,0.,h),(1.,0.,0.),
           0.,a,spec.x_motor_effort_n,.8)
    y_car=ET.SubElement(root,"link",{"name":"y_axis_carriage"})
    _inertial(y_car,spec.y_carriage_mass_kg,(.24,.22,.20))
    _segment(y_car,"y_slide_housing","box",(.24,.22,.20),(0.,0.,0.),
             (.22,.43,.69))
    _segment(y_car,"y_drive_cover","box",(.16,.14,.09),(0.,0.,.135),
             (.75,.76,.77))
    _joint(root,"actuator_y","prismatic","x_axis_carriage",
           "y_axis_carriage",(0.,-b/2,0.),(0.,1.,0.),
           0.,b,spec.y_motor_effort_n,.65)
    lift=ET.SubElement(root,"link",{"name":"z_axis_lift"})
    _inertial(lift,spec.z_carriage_mass_kg,(.13,.13,z+.22),
              center=(0.,0.,-.12))
    _segment(lift,"z_guided_slide","box",(.13,.13,z+.22),
             (0.,0.,-(z+.22)/2),(.24,.50,.65))
    _segment(lift,"z_bearing_block","box",(.22,.22,.13),
             (0.,0.,-.02),(.25,.29,.32))
    _joint(root,"actuator_z","prismatic","y_axis_carriage",
           "z_axis_lift",(0.,0.,0.),(0.,0.,-1.),
           0.,z,spec.z_motor_effort_n,.4)
    tool=ET.SubElement(root,"link",{"name":"inspection_tool_flange"})
    _inertial(tool,spec.tool_mass_kg,(.14,.14,.13),center=(0.,0.,-.045))
    _segment(tool,"tool_mounting_plate","cylinder",(.10,.045),
             (0.,0.,0.),(.68,.69,.73))
    _segment(tool,"sensor_mount_shroud","cylinder",(.055,.12),
             (0.,0.,-.075),(.14,.18,.23))
    _joint(root,"flange_attachment","fixed","z_axis_lift",
           "inspection_tool_flange",(0.,0.,-z/2-.17),(0.,0.,1.))
    ET.indent(root,space="  ")
    return b'<?xml version="1.0" encoding="utf-8"?>\n'+ET.tostring(root,encoding="utf-8")+b"\n"


def save_project(spec: MachineDesign, outdir: str | Path) -> dict:
    """Create an entirely new editable machine project; never clobber work."""
    spec.validate()
    folder=Path(outdir).resolve()
    if folder.exists() and any(folder.iterdir()):
        raise FileExistsError(f"Machine output folder is not empty: {folder}")
    folder.mkdir(parents=True,exist_ok=True)
    design_file=folder/"machine.design.json"
    urdf_file=folder/"machine.urdf"
    rights_file=folder/"author-rights.txt"
    manifest_file=folder/"machine.manifest.json"
    design_file.write_text(json.dumps(spec.to_dict(),indent=2)+"\n",encoding="utf-8")
    content=build_urdf(spec)
    urdf_file.write_bytes(content)
    rights_file.write_text(
        "This is an original parametric REFERENCE machine generated from "
        "user-provided parameters, not real OEM CAD. User is responsible "
        "for reviewing output and third-party rights.\n",encoding="utf-8",
    )
    manifest={
        "schema":"dsiq-robot-urdf-v1",
        "asset_id":spec.asset_id,"asset_class":"test_fixture",
        "manufacturer":"DSIQ user-authored reference design",
        "model":spec.asset_id,"revision":"v1",
        "source_uri":"dsiq://user-parametric/"+spec.asset_id,
        "license":{"usage_rights":"owned","evidence_file":rights_file.name},
        "geometry":{
            "urdf_file":urdf_file.name,
            "sha256":hashlib.sha256(content).hexdigest(),
        },
    }
    manifest_file.write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    return {
        "design_json":str(design_file),
        "urdf":str(urdf_file),
        "manifest":str(manifest_file),
        "asset_class":"test_fixture",
        "machine_type":"mechanically_actuated_xyz_gantry",
        "joints":["actuator_x","actuator_y","actuator_z","flange_attachment"],
        "fidelity":"parametric_reference_not_oem",
    }
