"""Real source-bound gantry actuator time-series across Bullet and MuJoCo.

Capture two INDEPENDENT numerical solvers at the same 240 Hz scene time.
This detects divergent reference servo laws, physical limits and overshoot
rather than claiming that a similar final position proves equivalent physics.
The URDF here is an explicitly self-authored fixture, NOT calibrated OEM CAD.
"""
from __future__ import annotations

from math import isfinite, sqrt
from pathlib import Path
from tempfile import TemporaryDirectory

from .bullet_runtime import PHYSICS_DT
from .machine_designer import MachineDesign, save_project
from .physics_backend import open_backend


def compare_motion_traces(*, joint: str="actuator_x", target: float=.2,
                          steps: int=100, every: int=5,
                          tolerance_m: float=.01) -> dict:
    if joint not in ("actuator_x","actuator_y","actuator_z"):
        raise ValueError("Only named self-authored fixture prismatic joints allowed")
    if isinstance(target,bool) or not isinstance(target,(int,float)) or not isfinite(target):
        raise ValueError("Finite numeric fixture target required")
    if type(steps) is not int or not 24<=steps<=480:
        raise ValueError("Trace step count must be 24..480")
    if type(every) is not int or not 1<=every<=steps:
        raise ValueError("Trace sample cadence must be between 1 and total steps")
    if (isinstance(tolerance_m,bool) or not isinstance(tolerance_m,(int,float))
            or not isfinite(tolerance_m) or not 0<tolerance_m<=.1):
        raise ValueError("Trace tolerance must be within (0,0.1] meters")

    measurements={}
    with TemporaryDirectory(prefix="dsiq-solver-motion-") as temp:
        source=save_project(MachineDesign(asset_id="solver_motion_fixture"),
                            Path(temp)/"gantry")
        expected_hash=None
        for name in ("bullet_cpu_direct_v1","mujoco_cpu_direct_v1"):
            with open_backend(name) as engine:
                loaded=engine.import_verified_urdf(source["manifest"])
                asset=loaded["asset_id"]
                if expected_hash is None:
                    expected_hash=loaded["source_urdf_sha256"]
                elif expected_hash!=loaded["source_urdf_sha256"]:
                    raise AssertionError("Source hash differs across solvers")
                before=engine.read_joint_states(asset)
                axis=next(x for x in before["joints"] if x["name"]==joint)
                if axis["unit"]!="m":
                    raise AssertionError("Only SI-linear fixture joints comparable")
                lower=float(axis["source_lower"])
                upper=float(axis["source_upper"])
                if not lower<=target<=upper:
                    raise ValueError("Fixture target exceeds verified source URDF joint bounds")
                engine.command_joint(asset,joint,target)
                sample=[]
                max_speed=0.
                max_effort=0.
                max_overshoot=0.
                out_of_travel=False
                for step in range(steps):
                    engine.step(1)
                    state=engine.read_joint_states(asset)
                    value=next(x for x in state["joints"] if x["name"]==joint)
                    position=float(value["position"])
                    velocity=float(value["velocity"])
                    effort=float(value["solver_reported_effort"])
                    if not all(isfinite(x) for x in (position,velocity,effort)):
                        raise ArithmeticError("Real solver produced nonfinite telemetry")
                    max_speed=max(max_speed,abs(velocity))
                    max_effort=max(max_effort,abs(effort))
                    max_overshoot=max(max_overshoot,max(0.,position-target))
                    if position<lower-.001 or position>upper+.001:
                        out_of_travel=True
                    if (step+1)%every==0 or step+1==steps:
                        sample.append({"step":step+1,
                                       "simulated_time_s":(step+1)*PHYSICS_DT,
                                       "position_m":position,
                                       "velocity_m_s":velocity,
                                       "solver_reported_effort":effort})
                measurements[name]={
                    "source_hash":loaded["source_urdf_sha256"],
                    "samples":sample,
                    "final_position_m":position,
                    "final_velocity_m_s":velocity,
                    "peak_observed_speed_m_s":max_speed,
                    "source_lower_m":lower,
                    "source_upper_m":upper,
                    "source_max_speed_m_s":axis["source_max_velocity"],
                    "speed_source_envelope_satisfied":
                        max_speed<=axis["source_max_velocity"]+1e-3,
                    "peak_observed_effort":max_effort,
                    "source_max_effort":axis["source_max_effort"],
                    "effort_source_envelope_satisfied":
                        max_effort<=axis["source_max_effort"]+1e-3,
                    "max_position_above_target_m":max_overshoot,
                    "source_travel_bounds_violated":out_of_travel,
                }
    a=measurements["bullet_cpu_direct_v1"]
    b=measurements["mujoco_cpu_direct_v1"]
    if len(a["samples"])!=len(b["samples"]):
        raise AssertionError("Solver snapshots have incompatible time bases")
    diffs=[abs(x["position_m"]-y["position_m"])
           for x,y in zip(a["samples"],b["samples"])]
    rmse=sqrt(sum(v*v for v in diffs)/len(diffs))
    final_delta=abs(a["final_position_m"]-b["final_position_m"])
    return {
        "schema":"dsiq-real-two-solver-motion-trace-v1",
        "source_urdf_sha256":expected_hash,
        "fixture":"self_authored_xyz_gantry_not_oem",
        "joint":joint,"target_m":target,
        "steps":steps,"sample_every_steps":every,
        "fixed_timestep_s":PHYSICS_DT,
        "simulation_duration_s":steps*PHYSICS_DT,
        "trajectory_comparison":{
            "position_rmse_m":rmse,
            "position_peak_difference_m":max(diffs),
            "position_final_difference_m":final_delta,
            "final_delta_tolerance_m":tolerance_m,
            "within_final_tolerance":final_delta<=tolerance_m,
        },
        "backend_measurements":measurements,
        "all_source_speed_and_effort_envelopes_satisfied":all(
            x["speed_source_envelope_satisfied"] and
            x["effort_source_envelope_satisfied"] and
            not x["source_travel_bounds_violated"]
            for x in measurements.values()
        ),
        "comparable_motor_control_laws_calibrated":False,
        "safety_certified":False,
        "note":"Different reference controls or solver settings can produce different traces; no OEM fidelity implied.",
    }
