"""Native Panda3D OpenGL workcell renderer + hierarchy / transform editor.

This is an actual GL draw pipeline with per-link source CAD geometry
and dynamic measured Bullet transforms, distinct from TinyRenderer.
The shader/lighting are real-time rasterized OpenGL, NOT RTX ray tracing.

Optional dependency: pip install -e ".[gpu]". On CI's Xvfb software
OpenGL is not evidence of a physical RTX 3050 meeting frame targets.
"""
from __future__ import annotations

import math
from pathlib import Path
import tempfile
import time

from .bullet_runtime import BulletPlayground, BulletSettings
from .cad_geometry import extract_verified_urdf_geometry
from .hardware import inspect_device
from .inertial_sensor import IdealBulletInertialProbe
from .render_geometry_cache import render_geometry_fingerprint
from .scene_editor import SceneEditor
from .workcell_acceptance import inspect_workcell


def _cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


def _normal(a,b,c):
    u=[y-x for x,y in zip(a,b)]
    v=[y-x for x,y in zip(a,c)]
    n=_cross(u,v)
    l=math.sqrt(sum(x*x for x in n))
    if l<1e-12:
        return (0.,0.,1.)
    return tuple(x/l for x in n)


def _panda_geometry(part, *, P, geometry_cache=None):
    # Each instance gets a distinct GeomNode. Only an identical immutable
    # Panda3D Geom vertex/index buffer is shared; no CAD or physics is merged.
    key = render_geometry_fingerprint(part) if geometry_cache is not None else None
    if geometry_cache is not None and key in geometry_cache:
        cached, triangles = geometry_cache[key]
        if triangles != len(part.faces):
            raise AssertionError("Render-buffer cache has inconsistent triangle accounting")
        node=P.GeomNode(part.visual_name)
        node.addGeom(cached)
        return node
    if geometry_cache is not None and len(geometry_cache)>=4096:
        raise ValueError("Unique CAD visual geometry cache capacity exceeded")
    vdata=P.GeomVertexData(
        part.visual_name,P.GeomVertexFormat.getV3n3c4(),P.Geom.UHStatic
    )
    vdata.setNumRows(len(part.faces)*3)
    pos=P.GeomVertexWriter(vdata,"vertex")
    norm=P.GeomVertexWriter(vdata,"normal")
    color=P.GeomVertexWriter(vdata,"color")
    triangles=P.GeomTriangles(P.Geom.UHStatic)
    index=0
    for face in part.faces:
        xyz=[part.vertices[i] for i in face]
        n=_normal(*xyz)
        for item in xyz:
            pos.addData3f(*item)
            norm.addData3f(*n)
            color.addData4f(*part.rgba)
            triangles.addVertex(index)
            index+=1
    triangles.closePrimitive()
    geom=P.Geom(vdata)
    geom.addPrimitive(triangles)
    if geometry_cache is not None:
        geometry_cache[key]=(geom,len(part.faces))
    node=P.GeomNode(part.visual_name)
    node.addGeom(geom)
    return node


def _display_bounds(parts):
    mins=[float("inf")]*3
    maxs=[-float("inf")]*3
    for part in parts:
        for v in part.vertices:
            for k in range(3):
                mins[k]=min(mins[k],v[k])
                maxs[k]=max(maxs[k],v[k])
    return {"source_min_m":mins,"source_max_m":maxs}


class GPUStudio:
    """Panda3D native window + Bullet workcell; no fake scene screenshots."""

    def __init__(self, *, headless=False, width=1100, height=760):
        try:
            from panda3d import core as P
            from direct.showbase.ShowBase import ShowBase
        except ImportError as exc:
            raise RuntimeError('GPU viewport dependency missing: pip install -e ".[gpu]"') from exc
        if type(width) is not int or type(height) is not int or not 320<=width<=1920 or not 240<=height<=1080:
            raise ValueError("Viewport must be 320..1920 x 240..1080")
        P.loadPrcFileData("","window-title DSIQ Native OpenGL Workcell Studio")
        P.loadPrcFileData("","win-size "+str(width)+" "+str(height))
        P.loadPrcFileData("","sync-video #f")
        P.loadPrcFileData("","audio-library-name null")
        P.loadPrcFileData("","framebuffer-multisample 1")
        P.loadPrcFileData("","multisamples 4")
        self.P=P
        self.base=ShowBase(windowType="offscreen" if headless else None)
        if not self.base.win:
            self.base.destroy()
            raise RuntimeError("No OpenGL graphics output; GL driver or display unavailable")
        self.base.disableMouse()
        self.base.camLens.setFov(65)
        self.base.camLens.setNearFar(.03,150.)
        self.base.setBackgroundColor(.035,.045,.065,1.)
        self.base.render.setShaderAuto()
        self.world=BulletPlayground(BulletSettings(payload_count=0))
        self.editor=SceneEditor(self.world)
        self._imu_probe=IdealBulletInertialProbe(self.world)
        self._show_imu=False
        self.headless=headless
        self._objects={}
        self._sources={}
        self._source_paths={}
        self._selected=None
        self._paused=True
        self._yaw=35.
        self._pitch=-26.
        self._distance=5.5
        self._center=(0.,0.,.9)
        self._last_mouse=None
        self._frame_ms=[]
        self._last_perf=time.perf_counter()
        self._frame_count=0
        self._triangles=0
        self._geometry_cache={}
        self._visual_draw_instances=0
        self._grid_snap_m=0.0
        self._status="Load a licensed URDF or a DSIQ workcell; data is not OEM qualified."
        self._make_lights()
        self._make_floor()
        self._set_camera()
        self._make_ui()
        if not headless:
            self.base.accept("wheel_up",lambda:self.zoom(-.5))
            self.base.accept("wheel_down",lambda:self.zoom(.5))
            self.base.accept("mouse1",self._start_drag)
            self.base.accept("mouse1-up",self._stop_drag)
            self.base.accept("tab",self.cycle_selection)
            self.base.accept("mouse3",self._pick_cad)
            self.base.accept("arrow_left",lambda:self.move_selected(-.025,0,0))
            self.base.accept("arrow_right",lambda:self.move_selected(.025,0,0))
            self.base.accept("arrow_up",lambda:self.move_selected(0,.025,0))
            self.base.accept("arrow_down",lambda:self.move_selected(0,-.025,0))
            self.base.accept("page_up",lambda:self.move_selected(0,0,.025))
            self.base.accept("page_down",lambda:self.move_selected(0,0,-.025))
            self.base.accept("control-z",self.undo)
            self.base.accept("control-y",self.redo)
            self.base.accept("space",self.toggle_play)
            self.base.accept("control-o",self.import_dialog)
            self.base.accept("g",self.cycle_grid_snap)
            self.base.accept("i",self.toggle_imu_overlay)
        self.base.taskMgr.add(self._tick,"dsiq_opengl_workcell_tick")

    def _make_lights(self):
        P=self.P
        ambient=P.AmbientLight("dsiq_ambient")
        ambient.setColor((.37,.40,.48,1))
        self.base.render.setLight(self.base.render.attachNewNode(ambient))
        for i,dir in enumerate(((-2,1,-3),(.7,-1,-1.5))):
            light=P.DirectionalLight("dsiq_directional_"+str(i))
            light.setColor((.8,.83,.9,1) if i==0 else (.30,.38,.5,1))
            node=self.base.render.attachNewNode(light)
            node.setHpr(35 if i==0 else -110,-65 if i==0 else -45,0)
            self.base.render.setLight(node)

    def _make_floor(self):
        from panda3d.core import CardMaker
        maker=CardMaker("reference_floor_not_oem")
        maker.setFrame(-15,15,-15,15)
        floor=self.base.render.attachNewNode(maker.generate())
        floor.setP(-90)
        floor.setZ(-.02)
        floor.setColor(.10,.13,.19,1)
        floor.setTwoSided(True)

    def _make_ui(self):
        if self.headless:
            self._label=None
            self._imu_label=None
            return
        from direct.gui.DirectGui import DirectLabel,DirectButton
        self._label=DirectLabel(
            text="DSIQ NATIVE | OPENGL CAD EDITOR",
            parent=self.base.aspect2d,
            text_align=self.P.TextNode.ALeft,
            text_scale=.044,
            frameColor=(.03,.04,.07,.90),
            text_fg=(.92,.95,1.,1),
            pos=(-1.31,0,.91),scale=.65,
        )
        self._imu_label=DirectLabel(
            text="IDEAL IMU COM | Paused / no measured acceleration",
            parent=self.base.aspect2d,
            text_align=self.P.TextNode.ALeft,
            text_scale=.033,
            frameColor=(.025,.055,.075,.92),
            text_fg=(.75,.93,1.,1.),
            pos=(-1.31,0,-.51),scale=.65,
        )
        self._imu_label.hide()
        self._hint=DirectLabel(
            parent=self.base.aspect2d,
            text="Ctrl+O Import CAD  | Tab Select  | Arrow/Page keys Pose  | Ctrl+Z/Y Undo/Redo  | Space Physics",
            text_scale=.035,
            frameColor=(.03,.04,.07,.86),
            text_fg=(.92,.95,1.,1),
            pos=(0,0,-.90),scale=1,
        )
        DirectButton(text="IMPORT LICENSED URDF",text_scale=.045,
            pos=(-1.05,0,.74),scale=.50,command=self.import_dialog)
        DirectButton(text="OPEN WORKCELL",text_scale=.045,
            pos=(-.37,0,.74),scale=.50,command=self.workcell_dialog)
        DirectButton(text="UNDO",text_scale=.045,
            pos=(.33,0,.74),scale=.50,command=self.undo)
        DirectButton(text="REDO",text_scale=.045,
            pos=(.73,0,.74),scale=.50,command=self.redo)
        from direct.gui.DirectGui import DirectScrolledFrame
        self._hierarchy=DirectScrolledFrame(
            parent=self.base.aspect2d,
            frameSize=(-.005,.64,-.64,.62),
            canvasSize=(0,.58,-1.2,.05),
            pos=(.66,0,.01),
            frameColor=(.04,.065,.11,.88),
            scrollBarWidth=.035,
        )
        self._hierarchy_controls=[]
        self._property=DirectLabel(
            text="HIERARCHY\nNo imported CAD",
            parent=self.base.aspect2d,
            text_align=self.P.TextNode.ALeft,
            text_scale=.034,
            frameColor=(.03,.045,.075,.9),
            text_fg=(.9,.94,1.,1.),
            pos=(.93,0,-.73),
        )
        for text,delta,pos in (
            ("X-",(-.025,0,0),-.96),("X+",(.025,0,0),-.76),
            ("Y-",(0,-.025,0),-.56),("Y+",(0,.025,0),-.36),
            ("Z-",(0,0,-.025),-.16),("Z+",(0,0,.025),.04),
        ):
            DirectButton(
                parent=self.base.aspect2d,text=text,
                text_scale=.034,pos=(pos,0,-.77),
                scale=.7,command=self.move_selected,
                extraArgs=list(delta),
            )
        DirectButton(
            text="LIVE IMU (I)",text_scale=.040,
            frameColor=(.06,.13,.21,.92),text_fg=(.91,.95,1.,1.),
            pos=(-.43,0,.59),scale=.52,
            command=self.toggle_imu_overlay,
        )
        DirectButton(
            text="GRID SNAP (G)",text_scale=.040,
            frameColor=(.06,.13,.21,.92),text_fg=(.91,.95,1.,1.),
            pos=(-1.03,0,.59),scale=.52,
            command=self.cycle_grid_snap,
        )
        self._refresh_hierarchy()

    def _refresh_hierarchy(self):
        if self.headless or not hasattr(self,"_hierarchy"):
            return
        from direct.gui.DirectGui import DirectButton,DirectLabel
        for widget in self._hierarchy_controls:
            widget.destroy()
        self._hierarchy_controls=[]
        canvas=self._hierarchy.getCanvas()
        row=0
        for aid,model in sorted(self._objects.items()):
            rows=[(aid,aid)]
            rows.extend((aid,"    "+name) for name in sorted(model["links"]))
            for asset,label in rows:
                row+=1
                color=(.5,.8,1.,1.) if asset==self._selected else (.85,.9,.95,1.)
                btn=DirectButton(
                    parent=canvas,text=label,text_scale=.041,
                    text_fg=color,text_align=self.P.TextNode.ALeft,
                    frameColor=(.085,.11,.16,.87),
                    pos=(.015,0,.04-row*.092),
                    command=self._choose,extraArgs=[asset],
                )
                self._hierarchy_controls.append(btn)
        self._hierarchy["canvasSize"]=(0,.58,-max(1.2,row*.092+.2),.05)
        self._refresh_properties()

    def _choose(self,aid):
        if aid in self._objects:
            self._selected=aid
            self._status="Selected actual verified geometry: "+aid
            self._refresh_hierarchy()

    def _refresh_properties(self):
        if self.headless or not hasattr(self,"_property"):
            return
        asset=self._selected
        if not asset:
            self._property["text"]="HIERARCHY\nNo imported CAD"
            return
        value=next((item for item in self.editor.entities()
                    if item["asset_id"]==asset),None)
        pos=value["position_m"] if value else (0,0,0)
        geometry=self._sources.get(asset)
        self._property["text"]=(
            "SOURCE CAD | "+asset+"\n"
            +("GRID SNAP OFF\n" if self._grid_snap_m==0 else
              "GRID SNAP {:.0f} mm\n".format(self._grid_snap_m*1000))

            +("X {:.3f}  Y {:.3f}  Z {:.3f} m\n".format(*pos))
            +str(geometry["triangle_count"] if geometry else 0)
            +" triangles | Not OEM qualified"
        )

    def _pick_cad(self):
        if self.headless:
            return
        watcher=self.base.mouseWatcherNode
        if not watcher.hasMouse():
            return
        mouse=watcher.getMouse()
        width=self.base.win.getXSize()
        height=self.base.win.getYSize()
        eye=tuple(float(v) for v in self.base.camera.getPos(self.base.render))
        f=self.base.camera.getQuat(self.base.render).getForward()
        center=tuple(eye[i]+f[i]*4. for i in range(3))
        pixel=((mouse.x+1.)*.5*width,(1.-mouse.y)*.5*height)
        outcome=self.editor.pick_viewport(
            eye,center,pixel,(width,height),
            fov_degrees=float(self.base.camLens.getFov()[1]),
        )
        if outcome["asset_id"] in self._objects:
            self._choose(outcome["asset_id"])
        return outcome

    def _set_camera(self):
        import math
        a=math.radians(self._yaw)
        b=math.radians(self._pitch)
        x=self._distance*math.cos(b)*math.sin(a)
        y=-self._distance*math.cos(b)*math.cos(a)
        z=-self._distance*math.sin(b)
        self.base.camera.setPos(self._center[0]+x,
                                self._center[1]+y,self._center[2]+z)
        self.base.camera.lookAt(*self._center)

    def _start_drag(self):
        mw=self.base.mouseWatcherNode
        if mw.hasMouse():
            m=mw.getMouse()
            self._last_mouse=(m.x,m.y)

    def _stop_drag(self):
        self._last_mouse=None

    def zoom(self, delta):
        self._distance=max(.3,min(100.,self._distance+delta))
        self._set_camera()

    def cycle_selection(self):
        if not self._objects:
            return
        ids=sorted(self._objects)
        current=ids.index(self._selected) if self._selected in ids else -1
        self._selected=ids[(current+1)%len(ids)]
        self._status="Selected "+self._selected
        self._refresh_hierarchy()

    def _sync_poses(self):
        p=self.world.p
        cid=self.world.client
        for aid,model in self._objects.items():
            root=model["body"]
            for name,entry in model["links"].items():
                index=entry["index"]
                if index<0:
                    pos,q=p.getBasePositionAndOrientation(root,physicsClientId=cid)
                else:
                    state=p.getLinkState(root,index,computeForwardKinematics=True,
                                         physicsClientId=cid)
                    pos,q=state[4],state[5]
                node=entry["node"]
                node.setPos(*pos)
                node.setQuat(self.P.Quat(float(q[3]),float(q[0]),float(q[1]),float(q[2])))

    def render_resource_report(self) -> dict:
        """CPU-side Panda3D geometry sharing counts, never measured VRAM."""
        unique_triangles=sum(tris for geom,tris in self._geometry_cache.values())
        instances=sum(int(part.triangle_count)
                      for source in self._sources.values()
                      for part in source["visual_parts"])
        if instances!=self._triangles or unique_triangles>instances:
            raise AssertionError("GPU draw-geometry cache accounting inconsistent")
        return {
            "schema":"dsiq-opengl-exact-buffer-reuse-v1",
            "total_authored_triangle_instances":instances,
            "unique_geom_buffer_triangles":unique_triangles,
            "reused_triangle_instances":instances-unique_triangles,
            "verified_source_visual_nodes":self._visual_draw_instances,
            "unique_geom_buffer_objects":len(self._geometry_cache),
            "distinct_bullet_assemblies":len(self._objects),
            "render_data_deduplicated_by_exact_geometry_and_rgba":True,
            "physical_colliders_merged":False,
            "materials_altered":False,
            "gpu_allocated_bytes_verified":False,
            "gpu_vram_saving_verified":False,
            "draw_calls_reduced_verified":False,
            "hardware_performance_qualified":False,
        }

    def add_verified_machine(self, manifest_file: str|Path,
                             *, base_position_m=(0.,1.,0.)):
        # All provenance/source triangles checked BEFORE changing Bullet
        # or allocating GPU vertex buffers.
        geometry=extract_verified_urdf_geometry(manifest_file)
        if self._triangles+geometry["triangle_count"]>500_000:
            raise ValueError("GPU scene triangle budget exceeded")
        robot=self.world.import_urdf_robot(
            str(manifest_file),base_position_m=base_position_m
        )
        aid=robot["asset_id"]
        body=robot["body_id"]
        link_indices={
            self.world.p.getJointInfo(
                body,j,physicsClientId=self.world.client
            )[12].decode("utf8","replace"):j
            for j in range(robot["joint_count"])
        }
        link_indices[robot["root_link"]]=-1
        link_nodes={}
        before_cache=set(self._geometry_cache)
        previous_selected=self._selected
        previous_triangles=self._triangles
        previous_visual_instances=self._visual_draw_instances
        try:
            for part in geometry["visual_parts"]:
                if part.link_name not in link_indices:
                    raise ValueError("Verified source visual link absent from actual Bullet")
                if part.link_name not in link_nodes:
                    link_nodes[part.link_name]={
                        "index":link_indices[part.link_name],
                        "node":self.base.render.attachNewNode(
                            "CAD_Link_"+part.link_name
                        ),
                    }
                geom=self.base.render.attachNewNode(
                    _panda_geometry(part,P=self.P,
                                    geometry_cache=self._geometry_cache)
                )
                geom.reparentTo(link_nodes[part.link_name]["node"])
                geom.setTwoSided(True)
            self._objects[aid]={"body":body,"links":link_nodes}
            self._sources[aid]=geometry
            self._source_paths[aid]=str(Path(manifest_file).resolve())
            self._triangles+=geometry["triangle_count"]
            self._visual_draw_instances+=geometry["part_count"]
            self._selected=aid
            self._sync_poses()
            self._refresh_hierarchy()
        except Exception:
            for item in link_nodes.values():
                item["node"].removeNode()
            self.world.p.removeBody(body,physicsClientId=self.world.client)
            self.world.imported_robots.pop(aid,None)
            self._objects.pop(aid,None)
            self._sources.pop(aid,None)
            self._source_paths.pop(aid,None)
            self._triangles=previous_triangles
            self._visual_draw_instances=previous_visual_instances
            self._selected=previous_selected
            for key in set(self._geometry_cache)-before_cache:
                del self._geometry_cache[key]
            raise
        self._status=(
            f"Imported {aid}: {geometry['triangle_count']} actual source "
            "triangles. Source integrity != OEM physical calibration."
        )
        return {"asset_id":aid,"triangles":geometry["triangle_count"],
                "parts":geometry["part_count"],
                "source_mesh_hashes_validated":True,
                "unique_render_buffers_in_scene":len(self._geometry_cache),
                "render_resource_report":self.render_resource_report(),
                "physical_accuracy_calibrated":False}

    def load_workcell(self, path: str|Path) -> dict:
        result=inspect_workcell(path)
        if result["errors"]:
            raise ValueError("Unsafe workcell: "+"; ".join(result["errors"]))
        for component in result["equipment"]:
            extract_verified_urdf_geometry(component["manifest"])
        loaded=[]
        for component in result["equipment"]:
            loaded.append(self.add_verified_machine(
                component["manifest"],
                base_position_m=component["base_position_m"],
            ))
        self._status="Loaded "+result["workcell_id"]+" | "+result["status"]
        return {
            "workcell_id":result["workcell_id"],
            "status":result["status"],
            "assemblies":loaded,
            "industrial_certified":False,
        }

    def _file_dialog(self):
        import tkinter as tk
        from tkinter import filedialog
        root=tk.Tk()
        root.withdraw()
        try:
            return filedialog.askopenfilename(
                title="Select a source-verified DSIQ manifest / workcell",
                filetypes=[("JSON project / manifest","*.json")],
            )
        finally:
            root.destroy()

    def import_dialog(self):
        if self.headless:
            raise RuntimeError("Interactive dialogs require a native display")
        path=self._file_dialog()
        if path:
            self.add_verified_machine(path)

    def workcell_dialog(self):
        if self.headless:
            raise RuntimeError("Interactive dialogs require a native display")
        path=self._file_dialog()
        if path:
            self.load_workcell(path)

    def set_grid_snap(self,step_m: float):
        """Edit-time coordinate grid, never motor trajectory quantization."""
        if isinstance(step_m,bool) or not isinstance(step_m,(int,float)):
            raise ValueError("Grid snap step must be an allowed metric value")
        if float(step_m) not in (0.0,.001,.005,.025,.1):
            raise ValueError("Allowed grid snap: off, 1, 5, 25, 100 millimeters")
        self._grid_snap_m=float(step_m)
        self._status=("Grid snapping off" if step_m==0 else
                      f"Grid snapping: {step_m*1000:.0f} mm")
        self._refresh_properties()
        return {"grid_snap_m":self._grid_snap_m,
                "editor_only":True,"hardware_motion_changed":False}

    def cycle_grid_snap(self):
        choices=(0.0,.025,.005,.001)
        current=choices.index(self._grid_snap_m) if self._grid_snap_m in choices else 0
        return self.set_grid_snap(choices[(current+1)%len(choices)])

    def move_selected(self,x,y,z):
        if self._selected is None:
            return None
        entry=next((e for e in self.editor.entities()
                    if e["asset_id"]==self._selected),None)
        if entry is None:
            raise ValueError("Selected machine not present in physics")
        pos=entry["position_m"]
        if self._grid_snap_m:
            step=self._grid_snap_m
            deltas=(x,y,z)
            target=tuple(
                (round((pos[i] + math.copysign(step,delta))/step)*step)
                if delta else float(pos[i])
                for i,delta in enumerate(deltas)
            )
        else:
            target=(pos[0]+x,pos[1]+y,pos[2]+z)
        value=self.editor.move(self._selected,target)
        self._sync_poses()
        self._refresh_properties()
        return value

    def undo(self):
        result=self.editor.undo()
        self._sync_poses()
        self._refresh_properties()
        return result

    def redo(self):
        result=self.editor.redo()
        self._sync_poses()
        return result

    def toggle_imu_overlay(self):
        """Toggle read-only measured Bullet sensor diagnostics.

        Never commands the robot, changes the source CAD or implies
        physically calibrated hardware. Paused frames show no acceleration.
        """
        self._show_imu=not self._show_imu
        if self._imu_label is not None:
            if self._show_imu:
                self._imu_label.show()
            else:
                self._imu_label.hide()
        return {"enabled":self._show_imu,
                "sensor_calibrated":False,"hardware_command_executed":False}

    def toggle_play(self):
        self._paused=not self._paused

    def _tick(self, task):
        now=time.perf_counter()
        diff=(now-self._last_perf)*1000
        self._last_perf=now
        if self._frame_count>1 and 0<diff<10_000:
            self._frame_ms.append(diff)
            if len(self._frame_ms)>600:
                self._frame_ms=self._frame_ms[-600:]
        self._frame_count+=1
        mw=self.base.mouseWatcherNode
        if self._last_mouse is not None and mw.hasMouse():
            m=mw.getMouse()
            old=self._last_mouse
            self._yaw+=(m.x-old[0])*160.
            self._pitch=max(-85.,min(85.,self._pitch+(m.y-old[1])*110.))
            self._last_mouse=(m.x,m.y)
            self._set_camera()
        if not self._paused:
            self.world.step(4)
            self._sync_poses()
        if self._show_imu and self._imu_label is not None and self._frame_count%12==0:
            sample=self._imu_probe.sample(link_index=5)
            v=sample["angular_velocity_com_rad_s"]
            if sample["acceleration_is_valid"]:
                f=sample["specific_force_com_m_s2"]
                detail="SPECIFIC FORCE [m/s2] {:7.3f} {:7.3f} {:7.3f}".format(*f)
            else:
                detail="SPECIFIC FORCE: no new physics sample"
            self._imu_label["text"]=(
                "BULLET COM IMU REFERENCE | NOT CALIBRATED\n"
                "STEP {} / t={:.3f}s\n".format(
                    sample["physics_step_index"],sample["simulation_time_s"])
                +"GYRO [rad/s] {:7.3f} {:7.3f} {:7.3f}\n".format(*v)
                +detail
            )
        if self._label is not None and self._frame_count%12==0:
            selected=self._selected or "No CAD loaded"
            self._label["text"]=(
                "DSIQ NATIVE | OPENGL CAD EDITOR\n"
                +selected+" | "+str(self._triangles)+" source triangles\n"
                +("PAUSED" if self._paused else "BULLET LIVE")
                +(" | SNAP OFF" if not self._grid_snap_m else
                  " | SNAP {:.0f}mm".format(self._grid_snap_m*1000))
                +" | NOT OEM CALIBRATED"
            )
        return task.cont

    def benchmark(self, *, frames=90, screenshot: str|Path|None=None):
        if type(frames) is not int or not 10<=frames<=600:
            raise ValueError("GPU benchmark must sample 10..600 rendered frames")
        import statistics
        from .hardware import detect_nvidia_gpus
        before=detect_nvidia_gpus()
        self._frame_ms.clear()
        self._last_perf=time.perf_counter()
        start=time.perf_counter()
        for _ in range(frames):
            self.base.taskMgr.step()  # REAL Panda3D GL frame
        elapsed=time.perf_counter()-start
        if not self.base.win or not self.base.win.isValid():
            raise RuntimeError("OpenGL output was invalidated during benchmark")
        if screenshot:
            from panda3d.core import Filename
            target=Path(screenshot).resolve()
            if target.exists():
                raise FileExistsError("Refusing to overwrite existing GL capture")
            target.parent.mkdir(parents=True,exist_ok=True)
            if not self.base.win.saveScreenshot(Filename.fromOsSpecific(str(target))):
                raise RuntimeError("OpenGL window failed to save framebuffer capture")
        samples=self._frame_ms[2:]
        if not samples:
            raise RuntimeError("No real GL frame timing samples available")
        ordered=sorted(samples)
        median=statistics.median(ordered)
        p95=ordered[math.ceil(.95*len(ordered))-1]
        after=detect_nvidia_gpus()
        memory_snapshots=[]
        for device in after:
            previous=next((x for x in before if x.name==device.name),None)
            memory_snapshots.append({
                "name":device.name,"driver":device.driver,
                "vram_total_mib":device.vram_total_mib,
                "vram_used_before_mib":(
                    previous.vram_total_mib-previous.vram_free_mib
                    if previous else None
                ),
                "vram_used_after_mib":device.vram_total_mib-device.vram_free_mib,
                "global_gpu_vram_delta_mib":(
                    (device.vram_total_mib-device.vram_free_mib)-
                    (previous.vram_total_mib-previous.vram_free_mib)
                    if previous else None
                ),
                "note":"Global nvidia-smi device usage, not process-isolated peak VRAM",
            })
        budget=inspect_device().as_dict()
        gsg=self.base.win.getGsg()
        vendor=""
        try:
            vendor=str(gsg.getDriverVendor())
        except (AttributeError,TypeError):
            vendor="UNAVAILABLE"
        return {
            "schema":"dsiq-panda3d-opengl-live-benchmark-v1",
            "renderer":"Panda3D_OpenGL_rasterization",
            "display_backend":str(self.base.win.getPipe().getType().getName()),
            "reported_gl_vendor":vendor,
            "rendered_frames":frames,
            "viewport_pixels":[self.base.win.getXSize(),self.base.win.getYSize()],
            "device_global_vram_snapshots":memory_snapshots,
            "source_asset_classes":sorted({
                item["asset_class"] for item in self._sources.values()
            }),
            "elapsed_wall_seconds":elapsed,
            "observed_fps":frames/elapsed if elapsed>0 else None,
            "interframe_p50_ms":median,
            "interframe_p95_ms":p95,
            "triangles_in_gpu_scene":self._triangles,
            "render_resource_reuse":self.render_resource_report(),
            "referenced_assemblies":len(self._objects),
            "budget_advisory":budget,
            "nvidia_rtx_hardware_qualified":False,
            "rtx_path_tracing_implemented":False,
            "source_cad_visual_integrity_verified":bool(self._objects),
            "photometric_materials_calibrated":False,
            "test_environment":"OpenGL graphics output on current runner",
            "screenshot_file":str(Path(screenshot).resolve()) if screenshot else None,
        }

    def run(self):
        if self.headless:
            raise RuntimeError("Offscreen window is a benchmark; use GUI mode for editor")
        self.base.run()

    def close(self):
        try:
            self.world.close()
        finally:
            self.base.destroy()


def benchmark_exact_geometry_instances(*, copies: int=4, frames: int=48,
                                       width: int=800, height: int=600,
                                       screenshot: str|Path|None=None) -> dict:
    """Run real OpenGL draw measurements on N independent verified gantries.

    Each instance gets its own imported URDF/physical body and separate
    NodePaths. An immutable source-identical Panda3D Geom buffer may be
    shared between instances; the authored triangle/draw-instance count
    always includes every independently positioned copy.
    This does not fabricate a factory, prove VRAM savings, or use Hydra.
    """
    import math
    if type(copies) is not int or not 2<=copies<=4:
        raise ValueError(
            "Geometry reuse benchmark requires 2..4 source machines; "
            "BulletPlayground intentionally limits imported robots to four"
        )
    app=GPUStudio(headless=True,width=width,height=height)
    try:
        from .machine_designer import MachineDesign, save_project
        with tempfile.TemporaryDirectory(prefix="dsiq-gpu-instance-benchmark-") as root:
            columns=math.ceil(math.sqrt(copies))
            rows=math.ceil(copies/columns)
            loaded=[]
            start=time.perf_counter()
            for idx in range(copies):
                asset_id=f"render_reuse_{idx:03d}"
                workcell=save_project(MachineDesign(asset_id=asset_id),
                                      Path(root)/asset_id)
                result=app.add_verified_machine(
                    workcell["manifest"],
                    base_position_m=(float(idx%columns)*2.2,
                                     1.+float(idx//columns)*2.2,0.),
                )
                loaded.append({"asset_id":result["asset_id"],
                               "parts":result["parts"],
                               "source_triangles":result["triangles"]})
            import_seconds=time.perf_counter()-start
            app._center=((columns-1)*1.1,1.+(rows-1)*1.1,.85)
            app._distance=max(5.5,3.3*max(columns,rows))
            app._set_camera()
            resource=app.render_resource_report()
            if (resource["distinct_bullet_assemblies"]!=copies or
                resource["total_authored_triangle_instances"]<=
                    resource["unique_geom_buffer_triangles"] or
                resource["verified_source_visual_nodes"]!=17*copies or
                len({record["body_id"] for record in
                     app.world.imported_robots.values()})!=copies):
                raise AssertionError("Reference render-geometry reuse contract failed")
            report=app.benchmark(frames=frames,screenshot=screenshot)
            report.update({
                "schema":"dsiq-source-identical-geom-reuse-benchmark-v1",
                "fixture":"source_verified_self_authored_xyz_gantries_not_oem",
                "authored_source_instances":loaded,
                "assembly_import_wall_seconds":import_seconds,
                "actual_distinct_Bullet_bodies":copies,
                "source_render_cache":resource,
                "measured_gpu_vram_savings":None,
                "gpu_memory_reuse_not_process_measured":True,
                "hydra_or_vulkan_renderer_implemented":False,
                "real_omni_feature_parity_qualified":False,
            })
            return report
    finally:
        app.close()


def benchmark_gpu_pipeline(*,frames=60,manifest=None,screenshot=None,
                           width=800,height=600):
    """Measured GL pipeline, never a synthetic GPU FPS number."""
    app=GPUStudio(headless=True,width=width,height=height)
    try:
        if manifest:
            model=app.add_verified_machine(manifest)
        else:
            from .machine_designer import MachineDesign,save_project
            with tempfile.TemporaryDirectory() as root:
                machine=save_project(MachineDesign(asset_id="gpu_reference_gantry"),
                                     Path(root)/"machine")
                model=app.add_verified_machine(machine["manifest"])
        result=app.benchmark(frames=frames,screenshot=screenshot)
        result["machine"]=model
        return result
    finally:
        app.close()
