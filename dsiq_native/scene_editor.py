"""Native Bullet scene hierarchy, authoring transforms and reversible edits.

DSIQ scene-editor changes intentionally teleport an assembly in the
simulation editor (not a physical robot motion command). This is a real
live 3D model operation, not a screenshot manipulation. Dynamic trajectories
must be performed through the motor controller APIs instead.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Sequence

MAX_HISTORY = 128


def _xyz(value: Sequence[float]) -> tuple[float,float,float]:
    if len(value) != 3:
        raise ValueError("A scene edit requires exactly three coordinates")
    result = tuple(float(v) for v in value)
    if any(not math.isfinite(x) or abs(x) > 20. for x in result):
        raise ValueError("Scene positions must be finite meters within +/-20m")
    return result


@dataclass(frozen=True)
class TransformChange:
    asset_id: str
    previous_m: tuple[float,float,float]
    updated_m: tuple[float,float,float]


class SceneEditor:
    """Live Bullet hierarchy and undo/redo with bounded history."""

    def __init__(self, world):
        self.world = world
        self._undo: list[TransformChange] = []
        self._redo: list[TransformChange] = []

    def entities(self) -> list[dict]:
        p = self.world.p
        cid = self.world.client
        items = [{
            "asset_id":"reference_6r","group":"Reference robot",
            "body_id":self.world.robot,
            "editable":False,
            "reason":"Reference motor base is locked",
        }, {
            "asset_id":"ground","group":"Environment",
            "body_id":self.world.ground,
            "editable":False,
            "reason":"Simulation ground is locked",
        }]
        for asset_id, imported in sorted(self.world.imported_robots.items()):
            items.append({
                "asset_id":asset_id,
                "group":"Imported machines",
                "body_id":imported["body_id"],
                "editable":True,
                "asset_class":imported["asset_class"],
                "joint_count":imported["joint_count"],
            })
        for index,body_id in enumerate(self.world.payloads):
            items.append({
                "asset_id":f"payload_{index:03d}",
                "group":"Sandbox payloads",
                "body_id":body_id,
                "editable":True,
                "asset_class":"test_fixture",
            })
        for item in items:
            xyz,quat=p.getBasePositionAndOrientation(
                item["body_id"],physicsClientId=cid,
            )
            item["position_m"]=[float(v) for v in xyz]
            item["orientation_xyzw"]=[float(v) for v in quat]
        return items

    def _find(self, asset_id):
        if not isinstance(asset_id,str):
            raise ValueError("Asset selection must be a string")
        entry=next((e for e in self.entities() if e["asset_id"]==asset_id),None)
        if entry is None:
            raise ValueError("Selected asset does not exist in scene")
        if not entry["editable"]:
            raise ValueError(entry.get("reason","Scene entity is locked"))
        return entry

    def _apply_position(self, asset_id, xyz):
        entry=self._find(asset_id)
        p=self.world.p
        cid=self.world.client
        _,quat=p.getBasePositionAndOrientation(
            entry["body_id"],physicsClientId=cid,
        )
        p.resetBasePositionAndOrientation(
            entry["body_id"],list(xyz),quat,physicsClientId=cid,
        )
        p.resetBaseVelocity(
            entry["body_id"],linearVelocity=[0.,0.,0.],
            angularVelocity=[0.,0.,0.],physicsClientId=cid,
        )
        pose=p.getBasePositionAndOrientation(
            entry["body_id"],physicsClientId=cid,
        )[0]
        if math.dist(pose,xyz)>1e-6:
            raise RuntimeError("Bullet did not apply the authored transform")
        return tuple(float(v) for v in pose)

    def move(self, asset_id: str, position_m: Sequence[float]) -> dict:
        target=_xyz(position_m)
        entry=self._find(asset_id)
        previous=tuple(entry["position_m"])
        if math.dist(previous,target)<1e-9:
            return {"asset_id":asset_id,"position_m":list(target),
                    "changed":False,"operation":"AUTHORING_TELEPORT_NOT_PHYSICAL_MOTION"}
        actual=self._apply_position(asset_id,target)
        self._undo.append(TransformChange(asset_id,previous,actual))
        del self._undo[:-MAX_HISTORY]
        self._redo.clear()
        return {
            "asset_id":asset_id,"position_m":list(actual),
            "changed":True,"undo_depth":len(self._undo),
            "operation":"AUTHORING_TELEPORT_NOT_PHYSICAL_MOTION",
        }

    def undo(self) -> dict:
        if not self._undo:
            raise ValueError("Scene editor has no transform to undo")
        edit=self._undo.pop()
        self._apply_position(edit.asset_id,edit.previous_m)
        self._redo.append(edit)
        return {"operation":"undo",**asdict(edit),
                "position_m":list(edit.previous_m)}

    def redo(self) -> dict:
        if not self._redo:
            raise ValueError("Scene editor has no transform to redo")
        edit=self._redo.pop()
        self._apply_position(edit.asset_id,edit.updated_m)
        self._undo.append(edit)
        return {"operation":"redo",**asdict(edit),
                "position_m":list(edit.updated_m)}

    def pick_viewport(
        self, eye_m: Sequence[float], look_at_m: Sequence[float],
        pixel_xy: Sequence[float], image_wh: Sequence[int],
        *, fov_degrees: float = 60.,
        max_distance_m: float = 60.,
    ) -> dict:
        """Pick actual Bullet geometry with a perspective-correct camera ray.

        Uses the same vertical FOV and Z-up convention as the current
        Bullet TinyRenderer camera. Does not claim pixel-perfect agreement
        with every renderer, lens model or viewport display scaling.
        """
        eye=_xyz(eye_m)
        look=_xyz(look_at_m)
        if len(pixel_xy)!=2 or len(image_wh)!=2:
            raise ValueError("Camera picking requires pixel XY and image width/height")
        if any(type(v) not in (float,int) or not math.isfinite(v)
               for v in pixel_xy):
            raise ValueError("Pixel positions must be finite")
        if any(type(v) is not int or v<2 or v>4096 for v in image_wh):
            raise ValueError("Image dimensions must be supported integers")
        width,height=image_wh
        px,py=[float(v) for v in pixel_xy]
        if not 0<=px<width or not 0<=py<height:
            raise ValueError("Pixel is outside the rendered image")
        if not math.isfinite(fov_degrees) or not 15<=fov_degrees<=120:
            raise ValueError("Perspective vertical FOV must be 15..120 degrees")
        if not math.isfinite(max_distance_m) or not .1<=max_distance_m<=100:
            raise ValueError("Ray distance must be within [.1,100] meters")

        def normalize(v):
            norm=math.sqrt(sum(x*x for x in v))
            if norm<1e-9:
                raise ValueError("Invalid camera direction/up vector")
            return tuple(x/norm for x in v)
        def cross(a,b):
            return (a[1]*b[2]-a[2]*b[1],
                    a[2]*b[0]-a[0]*b[2],
                    a[0]*b[1]-a[1]*b[0])
        forward=normalize(tuple(look[i]-eye[i] for i in range(3)))
        up=(0.,0.,1.)
        if abs(sum(forward[i]*up[i] for i in range(3)))>.99:
            up=(0.,1.,0.)
        right=normalize(cross(forward,up))
        camera_up=normalize(cross(right,forward))
        # Camera projection uses pixel-center coordinates.
        u=2*((px+.5)/width)-1.
        v=1.-2*((py+.5)/height)
        t=math.tan(math.radians(fov_degrees)/2)
        aspect=width/height
        direction=normalize(tuple(
            forward[i]+right[i]*(u*t*aspect)+camera_up[i]*(v*t)
            for i in range(3)
        ))
        end=tuple(eye[i]+direction[i]*max_distance_m for i in range(3))
        hit=self.world.p.rayTest(
            eye,end,physicsClientId=self.world.client,
        )[0]
        body_id=int(hit[0])
        selected=next((item for item in self.entities()
                       if item["body_id"]==body_id),None)
        return {
            "hit":body_id>=0,
            "body_id":body_id,
            "asset_id":selected["asset_id"] if selected else None,
            "editable":bool(selected and selected["editable"]),
            "hit_position_m":list(hit[3]) if body_id>=0 else None,
            "hit_fraction":float(hit[2]),
            "selection_type":"GEOMETRY_RAYCAST_REAL_BULLET",
        }

    def status(self):
        return {"entities":self.entities(),"undo_depth":len(self._undo),
                "redo_depth":len(self._redo),
                "renderer":"bullet_cpu",
                "edit_mode":"kinematic_authoring_teleport",
                "not_equivalent_to_physical_robot_motion":True}
