"""Semantically authored reference engineering environments for Bullet.

This is NOT a library of manufacturer equipment or a photorealistic renderer.
Architectural/safety/handling components are placed with explicit functions,
materials and collision flags. Equipment interiors and OEM geometry belong
in the verified CAD asset pipeline, never anonymous placeholder blocks.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import math

PRESETS=("open","robotics_cell","cleanroom_bay","logistics_bay")
MAX_PARTS=200

@dataclass(frozen=True)
class EnvironmentPart:
    name: str
    role: str
    shape: str
    center_m: tuple[float,float,float]
    dimensions_m: tuple[float,float,float]
    color: tuple[float,float,float,float]
    solid: bool
    body_id: int

    def as_dict(self):
        return asdict(self)


class ReferenceEnvironment:
    def __init__(self, world):
        self.world=world
        self.preset="open"
        self.parts: list[EnvironmentPart]=[]

    def clear(self):
        for part in reversed(self.parts):
            self.world.p.removeBody(part.body_id,physicsClientId=self.world.client)
        self.parts.clear()
        self.preset="open"

    def _part(self,name,role,center,dimensions,color,*,solid=True,shape="box"):
        if len(self.parts)>=MAX_PARTS:
            raise ValueError("Reference environment body budget exhausted")
        xyz=tuple(float(x) for x in center)
        dims=tuple(float(v) for v in dimensions)
        if not all(math.isfinite(x) for x in (*xyz,*dims)):
            raise ValueError("Environment geometry must be finite")
        if any(x<=0 for x in dims) or any(abs(x)>20 for x in xyz):
            raise ValueError("Environment component dimensions/position out of bounds")
        if len(color)!=4 or any(not 0<=x<=1 or not math.isfinite(x) for x in color):
            raise ValueError("Invalid environment material RGBA")
        p=self.world.p
        cid=self.world.client
        if shape=="box":
            collision=p.createCollisionShape(p.GEOM_BOX,halfExtents=[v/2 for v in dims],
                physicsClientId=cid) if solid else -1
            visual=p.createVisualShape(p.GEOM_BOX,halfExtents=[v/2 for v in dims],
                rgbaColor=color,physicsClientId=cid)
        elif shape=="cylinder":
            collision=p.createCollisionShape(p.GEOM_CYLINDER,radius=dims[0]/2,
                height=dims[2],physicsClientId=cid) if solid else -1
            visual=p.createVisualShape(p.GEOM_CYLINDER,radius=dims[0]/2,
                length=dims[2],rgbaColor=color,physicsClientId=cid)
        else:
            raise ValueError("Unsupported scene construction primitive")
        orientation=(p.getQuaternionFromEuler([math.pi/2,0.,0.])
                     if shape=="cylinder" else [0.,0.,0.,1.])
        body=p.createMultiBody(
            baseMass=0., baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,basePosition=xyz,
            baseOrientation=orientation,physicsClientId=cid,
        )
        if body<0:
            raise RuntimeError("Failed to author environment part "+name)
        self.parts.append(EnvironmentPart(name,role,shape,xyz,dims,color,solid,body))
        return body

    def _floor_zones(self,mode):
        # Floor paints are visual-only and sit 2mm above actual ground.
        line=(.86,.67,.19,1.)
        for x in (-1.95,1.95):
            self._part(f"{mode}_aisle_mark_x_{x}","pedestrian/robot safety lane",
                (x,0.,.003),(.035,4.65,.005),line,solid=False)
        for y in (-2.,2.):
            self._part(f"{mode}_aisle_mark_y_{y}","workcell perimeter marking",
                (0.,y,.003),(3.98,.035,.005),line,solid=False)
        for index in range(10):
            xx=-1.5+index*.34
            self._part(f"{mode}_hazard_tick_{index:02d}","safety exclusion zone marker",
                (xx,-2.25,.004),(.12,.045,.006),(.95,.78,.22,1.),solid=False)

    def _roof(self,mode):
        # High, sparse ceiling architecture. No collisions with robot operation.
        aluminum=(.55,.64,.70,1.)
        for x in (-2.7,2.7):
            for y in (-2.7,2.7):
                self._part(f"{mode}_structure_{x}_{y}","load-bearing facility post",
                           (x,y,1.65),(.09,.09,3.3),aluminum)
        for y in (-2.7,2.7):
            self._part(f"{mode}_truss_y_{y}","roof carrier beam",
                       (0.,y,3.20),(5.5,.12,.14),aluminum)
        for x in (-2.7,2.7):
            self._part(f"{mode}_truss_x_{x}","roof structural tie",
                       (x,0.,3.20),(.12,5.5,.14),aluminum)

    def _lights(self,mode,*,color=(.89,.96,1.,1.)):
        # Visual luminous-color reference housings. Bullet TinyRenderer
        # does NOT provide physical lighting/reflectance.
        for i in range(4):
            x=-1.8+(i%2)*3.6
            y=-1.8+(i//2)*3.6
            self._part(f"{mode}_luminaire_frame_{i}","ceiling light housing",
                       (x,y,3.16),(.75,.22,.05),(.28,.33,.37,1.),solid=False)
            self._part(f"{mode}_luminaire_diffuser_{i}","ceiling light diffuser (visual only)",
                       (x,y,3.12),(.67,.16,.018),color,solid=False)

    def _robotics_cell(self):
        self._floor_zones("robotics")
        self._roof("robotics")
        self._lights("robotics")
        steel=(.35,.46,.51,1.)
        panel=(.39,.55,.64,.28)
        warning=(.92,.69,.10,1.)
        # Safety enclosure: posts, structural rails and see-through
        # reference protective glazing; front section has a real opening.
        for side in (-1,1):
            for i in range(8):
                p=-2.45+i*.7
                self._part(f"side_{side}_guard_post_{i}","robotics guard upright",
                    (side*2.48,p,.87),(.055,.055,1.74),steel)
                if i<7:
                    self._part(f"side_{side}_rail_{i}","robotics safety guard top rail",
                        (side*2.48,p+.35,1.67),(.045,.7,.045),steel)
                    self._part(f"side_{side}_guard_panel_{i}","transparent safety panel reference",
                        (side*2.48,p+.35,.93),(.018,.66,1.33),panel,solid=False)
        for y in (-2.48,2.48):
            for i in range(7):
                x=-2.4+i*.8
                if y<0 and i in (3,4):
                    continue # accessible gate opening
                self._part(f"rear_front_{y}_post_{i}","safety perimeter guard post",
                    (x,y,.86),(.055,.055,1.72),steel)
                if i<6 and not (y<0 and i in (2,3,4)):
                    self._part(f"rear_front_{y}_rail_{i}","safety guard rail",
                        (x+.4,y,1.62),(.8,.05,.045),steel)
        # Actual reference inspection station: tabletop, stanchions and
        # instrument mounting surface, distinct from unclaimed OEM tooling.
        self._part("inspection_worktable","workholding table top",
            (1.45,1.4,.83),(.75,.65,.065),(.22,.27,.33,1.))
        for ix in (-1,1):
            for iy in (-1,1):
                self._part(f"table_leg_{ix}_{iy}","worktable support leg",
                    (1.45+ix*.28,1.4+iy*.23,.40),(.06,.06,.8),steel)
        self._part("workcell_cable_bridge","overhead routed cable tray",
            (-.75,1.65,2.52),(2.4,.30,.10),(.29,.34,.40,1.),solid=False)
        for i in range(6):
            self._part(f"bridge_partition_{i}","power/data cable tray divider",
                (-1.75+i*.4,1.65,2.60),(.045,.3,.12),(.49,.52,.55,1.),solid=False)
        for k in range(6):
            x=-1.75+k*.12
            self._part(f"tool_rack_position_{k}","tool carriage docking guide",
                       (x,1.9,.55),(.05,.24,.25),warning)

    def _cleanroom(self):
        self._floor_zones("cleanroom")
        self._roof("cleanroom")
        self._lights("cleanroom",color=(.91,1.,.95,1.))
        wall=(.78,.83,.86,.85)
        # Raised tile references and cleanroom service chases.
        for i in range(7):
            x=-2.25+i*.75
            for j in range(7):
                y=-2.25+j*.75
                self._part(f"cleanroom_floor_tile_{i}_{j}","raised floor inspection tile seam",
                           (x,y,.005),(.72,.72,.007),(.64,.74,.79,1.),solid=False)
        for i in range(7):
            x=-2.25+i*.75
            self._part(f"ceiling_hepa_module_{i}","HEPA supply cassette housing (visual)",
                (x,2.2,3.04),(.68,.68,.15),wall,solid=False)
            self._part(f"hepa_filter_face_{i}","reference HEPA filter mesh",
                (x,2.2,2.95),(.61,.61,.012),(.82,.92,.89,1.),solid=False)
        for y in (-2.63,2.63):
            for i in range(6):
                x=-2.25+i*.83
                self._part(f"cleanroom_window_{y}_{i}","cleanroom partition glazing",
                    (x,y,1.65),(.75,.025,2.4),(.68,.81,.88,.18),solid=False)
        for x in (-2.55,2.55):
            self._part(f"service_chase_{x}","ventilation or utility chase",
                (x,.3,2.6),(.3,4.1,.24),(.45,.59,.65,1.),solid=False)
        for i in range(4):
            self._part(f"floor_air_return_{i}","low-level reference air return grille",
                (-2.45,-1.8+i*1.1,.3),(.08,.5,.4),(.32,.47,.53,1.),solid=False)
        self._part("equipment_safety_footprint","reserved machine installation footprint",
            (1.0,-1.0,.015),(1.45,1.2,.01),(.76,.61,.18,.70),solid=False)

    def _logistics(self):
        self._floor_zones("logistics")
        self._roof("logistics")
        self._lights("logistics",color=(1.,.9,.76,1.))
        rack=(.25,.39,.52,1.)
        shelving=(.59,.62,.65,1.)
        # Two large three-level pallet racks; uprights and decks collide.
        for rack_no,x in enumerate((-2.25,2.25)):
            for end,y in enumerate((-1.5,0.,1.5)):
                self._part(f"rack{rack_no}_stanchion_{end}","vertical pallet rack frame",
                    (x,y,1.1),(.12,.12,2.2),rack)
            for level,z in enumerate((.48,1.12,1.75)):
                self._part(f"rack{rack_no}_load_deck_{level}","industrial storage beam/deck",
                    (x,0.,z),(.65,3.08,.10),shelving)
                for slot,y in enumerate((-.8,.8)):
                    self._part(f"rack{rack_no}_pallet_{level}_{slot}",
                        "training reference pallet support", (x,y,z+.18),
                        (.51,.69,.18),(.65,.43,.25,1.))
        # Physically meaningful free-rolling roller-deck architecture.
        # Rollers are STATIC in this preview: no conveyor motor is implied.
        self._part("roller_support_left","roller conveyor rail",
            (.0,-1.65,.64),(2.8,.12,.16),rack)
        self._part("roller_support_right","roller conveyor rail",
            (.0,-.95,.64),(2.8,.12,.16),rack)
        for i in range(13):
            x=-1.30+i*.21
            self._part(f"unpowered_roller_{i}","static roller conveyor (unpowered)",
                (x,-1.3,.68),(.08,.08,.68),(.62,.65,.68,1.),
                shape="cylinder")
        self._part("dispatch_lane","traffic separation indicator",
            (0.,1.85,.009),(3.6,.10,.012),(.93,.67,.22,1.),solid=False)

    def build(self,preset: str):
        if preset not in PRESETS:
            raise ValueError(f"Unknown background {preset!r}; choose {PRESETS}")
        self.clear()
        try:
            if preset=="robotics_cell":
                self._robotics_cell()
            elif preset=="cleanroom_bay":
                self._cleanroom()
            elif preset=="logistics_bay":
                self._logistics()
            self.preset=preset
        except Exception:
            self.clear()
            raise
        return self.report()

    def report(self):
        return {
            "preset":self.preset,
            "part_count":len(self.parts),
            "collision_parts":sum(p.solid for p in self.parts),
            "semantic_part_names":[p.name for p in self.parts],
            "semantic_roles":sorted({p.role for p in self.parts}),
            "fidelity":"architectural_reference_not_verified_oem_equipment",
            "renderer_note":"TinyRenderer uses software colors without PBR, RTX or physical lighting",
        }
