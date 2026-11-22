"""Six-degree-of-freedom free navigation, independent of RTX/Kit.

The scene is Z-up meters. Movement is in the camera's local horizontal
heading (W/S), strafe (A/D), vertical (Q/E); yaw/pitch mouse-look changes
orientation. Hardware sampling and Bullet rendering are kept separate.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence

Vec3 = tuple[float,float,float]
VALID_KEYS={"w","a","s","d","q","e"}

def _finite_xyz(v):
    if len(v)!=3:
        raise ValueError("Expected three XYZ coordinates")
    value=tuple(float(x) for x in v)
    if any(not math.isfinite(x) or abs(x)>1000 for x in value):
        raise ValueError("Camera XYZ coordinates must be finite within 1000m")
    return value


@dataclass
class FreeCamera:
    position_m: Vec3 = (2.9,-3.0,1.8)
    yaw_rad: float = 2.34
    pitch_rad: float = -.27
    speed_m_s: float = 2.2

    def validate(self):
        self.position_m = _finite_xyz(self.position_m)
        if any(not math.isfinite(x) for x in (self.yaw_rad,self.pitch_rad,self.speed_m_s)):
            raise ValueError("Camera orientation and speed must be finite")
        if abs(self.pitch_rad)>1.49 or not .1<=self.speed_m_s<=12.:
            raise ValueError("Camera pitch or speed outside supported limits")

    @classmethod
    def look_at(cls, eye: Sequence[float], target: Sequence[float],speed=2.2):
        p=_finite_xyz(eye); t=_finite_xyz(target)
        dx,dy,dz=(t[i]-p[i] for i in range(3))
        norm=math.sqrt(dx*dx+dy*dy+dz*dz)
        if norm<.01:
            raise ValueError("Camera eye and target must be distinct")
        yaw=math.atan2(dy,dx)
        pitch=math.atan2(dz,math.hypot(dx,dy))
        camera=cls(p,yaw,max(-1.49,min(1.49,pitch)),speed)
        camera.validate()
        return camera

    def forward(self) -> Vec3:
        self.validate()
        c=math.cos(self.pitch_rad)
        return (c*math.cos(self.yaw_rad),
                c*math.sin(self.yaw_rad),
                math.sin(self.pitch_rad))

    def direction_horizontal(self) -> Vec3:
        return (math.cos(self.yaw_rad),math.sin(self.yaw_rad),0.)

    def right(self) -> Vec3:
        return (-math.sin(self.yaw_rad),math.cos(self.yaw_rad),0.)

    def eye_target(self, *, sight_distance_m=2.) -> tuple[Vec3,Vec3]:
        if not math.isfinite(sight_distance_m) or not .1<=sight_distance_m<=100:
            raise ValueError("Sight distance must be within [.1,100]m")
        direction=self.forward()
        return self.position_m,tuple(
            self.position_m[i]+direction[i]*sight_distance_m for i in range(3)
        )  # type: ignore[return-value]

    def mouse_look(self,delta_x_px,delta_y_px,sensitivity=.004):
        if not all(math.isfinite(v) for v in (delta_x_px,delta_y_px,sensitivity)):
            raise ValueError("Mouse look contains invalid value")
        if not 0<sensitivity<=.05:
            raise ValueError("Invalid mouse sensitivity")
        self.yaw_rad=(self.yaw_rad+delta_x_px*sensitivity)%(2*math.pi)
        self.pitch_rad=max(-1.45,min(1.45,self.pitch_rad-delta_y_px*sensitivity))

    def move(self,key: str,seconds: float=.10,*,sprint=False):
        self.validate()
        if key not in VALID_KEYS:
            raise ValueError(f"Invalid free camera movement: {key!r}")
        if not math.isfinite(seconds) or not 0<seconds<=1.:
            raise ValueError("Camera movement interval must be in (0,1]s")
        velocity=self.speed_m_s*(3 if sprint else 1)*seconds
        f=self.direction_horizontal()
        r=self.right()
        mapping={
            "w":f,"s":tuple(-x for x in f),
            "a":tuple(-x for x in r),"d":r,
            "q":(0.,0.,-1.),"e":(0.,0.,1.),
        }
        v=mapping[key]
        new=tuple(self.position_m[i]+velocity*v[i] for i in range(3))
        self.position_m=_finite_xyz(new)  # type: ignore[assignment]
        return self.position_m
