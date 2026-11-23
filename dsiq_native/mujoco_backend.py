"""True MuJoCo CPU backend for the DSIQ physics world protocol.

MuJoCo directly parses a source-validated local URDF. No Bullet solver or
synthetic state values are used in the simulation. URI, licence, joint and
source-byte checks are performed by the pre-existing URDF manifest gate.

Scope: one fixed-base licensed URDF per isolated world. An effort-limited
PD reference controller is applied to qfrc_applied in simulation space.
It is not an industrial drive, OEM calibration or safety-rated mechanism.
"""
from __future__ import annotations

from dataclasses import asdict
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .bullet_runtime import PHYSICS_DT
from .physics_backend import BackendCapabilities
from .urdf_bridge import validate_robot_manifest


class MuJoCoPhysicsBackend:
    def __init__(self):
        try:
            import mujoco as mj
        except ImportError as exc:
            raise RuntimeError(
                'Real MuJoCo Python bindings missing. '
                'Install optional "[mujoco]" dependency.'
            ) from exc
        self.mj=mj
        self._closed=False
        self.model=None
        self.data=None
        self.robot=None
        self.joints=[]
        self.targets={}
        self.step_index=0
        self._caps=BackendCapabilities(
            backend_id="mujoco_cpu_direct_v1",
            source_format="verified_local_urdf",
            step_mode="fixed_step_explicit",
            fixed_timestep_s=PHYSICS_DT,
            supports_verified_urdf=True,
            supports_usd_physics_import=False,
            supports_joint_position_target=True,
            supports_measured_joint_feedback=True,
            supports_collision_contacts=True,
            supports_gpu_batched_worlds=False,
            supports_hardware_actuation=False,
            independently_qualified=False,
        )

    @property
    def capabilities(self):
        return self._caps

    def _require_open(self):
        if self._closed:
            raise RuntimeError("Physics backend is closed")

    def _require_asset(self, asset_id):
        self._require_open()
        if (not isinstance(asset_id,str) or self.robot is None or
                asset_id!=self.robot["asset_id"]):
            raise ValueError("Named verified URDF not loaded in this backend world")

    def import_verified_urdf(self, manifest: str | Path):
        self._require_open()
        if self.model is not None:
            raise ValueError("This bounded MuJoCo world supports one imported URDF")
        doc, inspection=validate_robot_manifest(str(manifest))
        # The audited URDF is parsed into an independent, real MuJoCo model.
        # The source validator rejects XML external entities before this point.
        source=Path(inspection.path)
        root=ET.parse(source).getroot()
        limits={}
        for joint in root.findall("joint"):
            name=joint.get("name")
            kind=joint.get("type")
            if kind not in ("prismatic","revolute","continuous"):
                continue
            entry=joint.find("limit")
            if entry is None:
                raise ValueError("Source URDF has missing joint dynamics limits")
            try:
                effort=float(entry.get("effort","nan"))
                speed=float(entry.get("velocity","nan"))
                lower=float(entry.get("lower","0"))
                upper=float(entry.get("upper","0"))
            except ValueError as exc:
                raise ValueError("Source URDF motor limits are not finite numbers") from exc
            if (not all(map(math.isfinite,(effort,speed,lower,upper))) or
                effort<=0 or speed<=0 or (kind!="continuous" and lower>upper)):
                raise ValueError(f"Invalid source limits for joint {name}")
            limits[name]={"kind":kind,"max_effort":effort,"max_velocity":speed,
                          "lower":lower,"upper":upper}
        if not limits:
            raise ValueError("MuJoCo adapter requires at least one actuated URDF joint")

        try:
            model=self.mj.MjModel.from_xml_path(str(source))
        except Exception as exc:
            raise ValueError("MuJoCo rejected source URDF compilation: "+str(exc)) from exc
        model.opt.timestep=PHYSICS_DT
        model.opt.gravity[:]=[0.,0.,-9.81]
        data=self.mj.MjData(model)
        self.mj.mj_forward(model,data)
        joints=[]
        for name, spec in limits.items():
            jid=int(self.mj.mj_name2id(
                model,self.mj.mjtObj.mjOBJ_JOINT,name
            ))
            if jid<0:
                raise ValueError(f"MuJoCo omitted expected URDF joint: {name}")
            mjtype=model.jnt_type[jid]
            expect=(self.mj.mjtJoint.mjJNT_SLIDE if
                    spec["kind"]=="prismatic" else self.mj.mjtJoint.mjJNT_HINGE)
            if int(mjtype)!=int(expect):
                raise ValueError(f"MuJoCo joint type does not match validated URDF: {name}")
            qa=int(model.jnt_qposadr[jid])
            da=int(model.jnt_dofadr[jid])
            if qa<0 or da<0 or qa>=model.nq or da>=model.nv:
                raise ValueError(f"Invalid MuJoCo joint state indices: {name}")
            joints.append({**spec,"name":name,"joint_index":jid,
                           "qpos_address":qa,"dof_address":da})
        self.model=model
        self.data=data
        self.joints=joints
        self.robot={"asset_id":doc["asset_id"],
                    "source_urdf_sha256":doc["geometry"]["sha256"]}
        self.targets={j["name"]:float(data.qpos[j["qpos_address"]]) for j in joints}
        self.step_index=0
        return {
            "asset_id":doc["asset_id"],
            "source_urdf_sha256":doc["geometry"]["sha256"],
            "joint_count":int(model.njnt),
            "actuated_joint_count":len(joints),
            "verified_source_imported":True,
            "independently_qualified":False,
        }

    def command_joint(self,asset_id: str,name: str,target: float):
        self._require_asset(asset_id)
        spec=next((j for j in self.joints if j["name"]==name),None)
        if spec is None:
            raise ValueError("Joint name not found or ambiguous")
        if isinstance(target,bool) or not isinstance(target,(int,float)) or not math.isfinite(target):
            raise ValueError("Target must be finite numeric")
        if (spec["kind"]!="continuous" and
            (target < spec["lower"]-1e-9 or target>spec["upper"]+1e-9)):
            raise ValueError("Target violates source URDF joint travel bounds")
        if abs(target)>8*math.pi:
            raise ValueError("Target exceeds conservative absolute control bound")
        self.targets[name]=float(target)
        return {"asset_id":asset_id,"joint":name,"target":float(target),
                "units":"meters" if spec["kind"]=="prismatic" else "radians",
                "physically_qualified":False}

    def step(self,count: int=1):
        self._require_open()
        if type(count) is not int or not 1<=count<=50000:
            raise ValueError("Backend step count must be integer in [1,50000]")
        if self.model is None:
            raise RuntimeError("No source-verified MuJoCo model imported")
        for _ in range(count):
            self.data.qfrc_applied[:]=0.
            for j in self.joints:
                dof=j["dof_address"]
                q=float(self.data.qpos[j["qpos_address"]])
                v=float(self.data.qvel[dof])
                target=self.targets[j["name"]]
                # Simulated effort-limited PD; never claim an OEM drive law.
                effort=max(-j["max_effort"],min(j["max_effort"],
                           600.*(target-q)-48.*v))
                self.data.qfrc_applied[dof]=effort
            self.mj.mj_step(self.model,self.data)
            self.step_index+=1
        if any(not math.isfinite(float(self.data.qpos[j["qpos_address"]]))
               for j in self.joints):
            raise ArithmeticError("MuJoCo returned nonfinite joint positions")
        return {"backend_id":self.capabilities.backend_id,
                "step_index":self.step_index,
                "simulated_time_s":self.step_index*PHYSICS_DT,
                "fixed_timestep_s":PHYSICS_DT,"hardware_control_executed":False}

    def read_joint_states(self,asset_id: str):
        self._require_asset(asset_id)
        arr=[]
        for j in self.joints:
            q=float(self.data.qpos[j["qpos_address"]])
            v=float(self.data.qvel[j["dof_address"]])
            force=float(self.data.qfrc_applied[j["dof_address"]])
            if not all(map(math.isfinite,(q,v,force))):
                raise ArithmeticError("MuJoCo returned nonfinite joint state")
            arr.append({
                "name":j["name"],"index":j["joint_index"],
                "unit":"m" if j["kind"]=="prismatic" else "rad",
                "position":q,"velocity":v,
                "solver_reported_effort":force,
                "effort_measurement_semantics":"applied_generalized_force_not_motor_telemetry",
                "source_max_velocity":j["max_velocity"],
                "source_max_effort":j["max_effort"],
            })
        return {
            "asset_id":asset_id,"backend_id":self.capabilities.backend_id,
            "source_urdf_sha256":self.robot["source_urdf_sha256"],
            "step_index":self.step_index,
            "simulated_time_s":self.step_index*PHYSICS_DT,
            "joints":arr,"measured_from_solver":True,
            "physically_calibrated":False,
        }

    def read_contacts(self,asset_id: str,*,maximum: int=32):
        self._require_asset(asset_id)
        if type(maximum) is not int or not 1<=maximum<=256:
            raise ValueError("Contact reporting limit must be integer in [1,256]")
        n=int(self.data.ncon)
        entries=[]
        for i in range(min(n,maximum)):
            c=self.data.contact[i]
            distance=float(c.dist)
            if not math.isfinite(distance):
                raise ArithmeticError("MuJoCo returned nonfinite collision separation")
            entries.append({
                "geom1":int(c.geom1),"geom2":int(c.geom2),
                "signed_separation_m":distance,
            })
        return {
            "asset_id":asset_id,"step_index":self.step_index,
            "contact_count":n,"contacts":entries,"truncated":n>maximum,
            "solver_reported_discrete_contacts":True,
            "swept_collision_detection_qualified":False,
            "self_collision_qualified":False,
        }

    def close(self):
        if not self._closed:
            self.model=None
            self.data=None
            self.robot=None
            self.joints=[]
            self.targets={}
            self._closed=True

    def __enter__(self):
        self._require_open()
        return self

    def __exit__(self,exc_type,exc,traceback):
        self.close()
