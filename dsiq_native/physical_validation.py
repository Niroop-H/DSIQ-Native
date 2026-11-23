"""Compare measured external robot-joint observations with Bullet trajectories.

This is an engineer-facing evidence reconciliation tool. It does not create,
invent, or infer manufacturer reference measurements; the caller must supply
an original dated, instrumented measurement file. Passing user-chosen
tolerances does NOT prove that the input is genuinely independent or that
physical hardware is certified.
"""
from __future__ import annotations

import bisect
import hashlib
import json
import math
from pathlib import Path


def compare_measured_robot_motion(simulation_report: str|Path,
                                  physical_observations: str|Path,
                                  *, tolerance: float=.005,
                                  require_uncertainty: bool=False,
                                  require_source_binding: bool=False,
                                  coverage_factor: float=2.0) -> dict:
    if type(tolerance) not in (int,float) or not math.isfinite(tolerance) or not 0<tolerance<=.1:
        raise ValueError("Measured trajectory tolerance must be finite in (0,.1]")
    if type(require_uncertainty) is not bool:
        raise ValueError("require_uncertainty must be a boolean")
    if type(require_source_binding) is not bool:
        raise ValueError("require_source_binding must be a boolean")
    if (type(coverage_factor) not in (int,float) or
            not math.isfinite(coverage_factor) or not 1. <= coverage_factor <= 5.):
        raise ValueError("Uncertainty coverage factor must be within [1,5]")
    simulated=Path(simulation_report).resolve()
    actual=Path(physical_observations).resolve()
    if not simulated.is_file() or not actual.is_file():
        raise FileNotFoundError("Both independent simulation and physical measurement files are required")
    if simulated.stat().st_size>8*1024*1024 or actual.stat().st_size>8*1024*1024:
        raise ValueError("Measurement input exceeds 8 MiB budget")
    data=json.loads(simulated.read_text(encoding="utf8"))
    measured=json.loads(actual.read_text(encoding="utf8"))
    if measured.get("schema")!="dsiq-external-joint-measurements-v1":
        raise ValueError("External measurement schema missing")
    if not measured.get("equipment_identifier") or not measured.get("instrument_identifier"):
        raise ValueError("Physical equipment and measurement instrument must be identified")
    if not isinstance(measured.get("instrument_calibration_reference"),str) or not measured["instrument_calibration_reference"].strip():
        raise ValueError("External instrument calibration reference required")
    source=data.get("tracking",data)
    if source.get("runtime_monitor_passed") is False or source.get("trajectory_completed") is False:
        raise ValueError("Faulted or incomplete Bullet trajectory cannot support physical validation")
    samples=source.get("sampled_reports")
    if not isinstance(samples,list) or len(samples)<3:
        raise ValueError("At least three actual Bullet motion feedback samples required")
    time=[x.get("time_s") for x in samples]
    if any(type(t) not in (int,float) or not math.isfinite(t) for t in time):
        raise ValueError("Simulation timing must be finite")
    if not all(x<y for x,y in zip(time,time[1:])):
        raise ValueError("Simulation feedback times must be monotonic")
    first_states=samples[0].get("measured_joint_states")
    if not isinstance(first_states,dict):
        raise ValueError("Simulation joint feedback must be a numeric mapping")
    names=set(first_states)
    if not names or any(not isinstance(n,str) or not n for n in names):
        raise ValueError("No actual imported robot feedback available")
    for sample in samples:
        states=sample.get("measured_joint_states")
        if not isinstance(states,dict) or set(states)!=names:
            raise ValueError("Inconsistent simulation feedback joint set")
        if any(type(v) not in (float,int) or not math.isfinite(v)
               for v in states.values()):
            raise ValueError("Nonfinite or nonnumeric simulation joint feedback")
    source_joint_units=measured.get("joint_units")
    actual_urdf_units=source.get("joint_units_from_loaded_urdf")
    if actual_urdf_units is not None and (
        not isinstance(actual_urdf_units,dict) or
        set(actual_urdf_units)!=names or
        any(v not in ("m","rad") for v in actual_urdf_units.values())
    ):
        raise ValueError("Simulator source URDF joint unit mapping is incomplete or invalid")
    simulation_source_sha=source.get("source_urdf_sha256")
    provided_source_sha=measured.get("source_urdf_sha256")
    source_hash_matches=(
        isinstance(simulation_source_sha,str) and
        isinstance(provided_source_sha,str) and
        len(simulation_source_sha)==64 and
        all(x in "0123456789abcdef" for x in simulation_source_sha) and
        simulation_source_sha==provided_source_sha
    )
    if provided_source_sha is not None and not source_hash_matches:
        raise ValueError("Physical observation URDF SHA256 does not match simulated source")
    if require_source_binding and (
        not source_hash_matches or actual_urdf_units is None
    ):
        raise ValueError("Strict physical comparison requires a matching source URDF SHA256 and simulator joint unit map")
    if source_joint_units is not None and (
        not isinstance(source_joint_units,dict) or
        set(source_joint_units)!=names or
        any(unit not in ("m","rad") for unit in source_joint_units.values())
    ):
        raise ValueError("External joint unit contract must cover every simulated joint")

    observations=measured.get("observations")
    if not isinstance(observations,list) or len(observations)<3 or len(observations)>30000:
        raise ValueError("External measurements require 3..30000 samples")
    seen=set()
    comparison={}
    encountered_units={}
    any_uncertainty=any(
        isinstance(row,dict) and "standard_uncertainty" in row
        for row in observations
    )
    if require_uncertainty and not any_uncertainty:
        raise ValueError("External standard_uncertainty required for every observation")
    for row in observations:
        if not isinstance(row,dict):
            raise ValueError("Observation must be an object")
        name=row.get("joint")
        t=row.get("time_s")
        value=row.get("measured_position")
        units=row.get("unit")
        if name not in names or units not in ("m","rad"):
            raise ValueError("External joint name or unit does not match declared simulator")
        if actual_urdf_units is not None and units!=actual_urdf_units[name]:
            raise ValueError("Observation joint unit disagrees with loaded source URDF")
        if source_joint_units is not None and units!=source_joint_units[name]:
            raise ValueError("Observation unit differs from declared external source unit contract")
        if name in encountered_units and units!=encountered_units[name]:
            raise ValueError("Mixed units for one physical joint are not permitted")
        encountered_units[name]=units
        if not all(type(x) in (int,float) and math.isfinite(x) for x in (t,value)):
            raise ValueError("External time and position must be finite")
        if not time[0]<=t<=time[-1]:
            raise ValueError("External measurement must lie within actual Bullet sampled interval")
        u=row.get("standard_uncertainty")
        if any_uncertainty:
            if type(u) not in (float,int) or not math.isfinite(u) or not 0 <= u <= .1:
                raise ValueError("Every measurement must have finite nonnegative standard_uncertainty")
            u=float(u)
        key=(name,t)
        if key in seen:
            raise ValueError("Duplicate physical joint/time observation")
        seen.add(key)
        index=bisect.bisect_left(time,t)
        if index==0:
            predicted=float(samples[0]["measured_joint_states"][name])
        else:
            left,right=samples[index-1],samples[index]
            ratio=(t-time[index-1])/(time[index]-time[index-1])
            a=float(left["measured_joint_states"][name])
            b=float(right["measured_joint_states"][name])
            predicted=a+(b-a)*ratio
        deviation=predicted-value
        comparison.setdefault(name,[]).append((t,deviation,units,u))
    if set(comparison)!=names or any(len(rows)<3 for rows in comparison.values()):
        raise ValueError("At least three external measurements for every simulated joint are required")
    result={}
    all_pass=True
    all_conservative_pass=True if any_uncertainty else None
    for name,rows in comparison.items():
        rows.sort(key=lambda x:x[0])
        diffs=[x[1] for x in rows]
        rmse=math.sqrt(sum(v*v for v in diffs)/len(diffs))
        peak=max(abs(v) for v in diffs)
        passed=peak<=tolerance
        all_pass=all_pass and passed
        conservative_peak=(
            max(abs(v)+float(coverage_factor)*float(row[3])
                for v,row in zip(diffs,rows))
            if any_uncertainty else None
        )
        conservative_pass=(
            conservative_peak<=tolerance if any_uncertainty else None
        )
        if any_uncertainty:
            all_conservative_pass=all_conservative_pass and conservative_pass
        result[name]={
            "samples":len(rows),"unit":rows[0][2],
            "mean_signed_error":sum(diffs)/len(diffs),
            "rmse":rmse,"maximum_absolute_error":peak,
            "within_operator_tolerance":passed,
            "conservative_peak_error_plus_uncertainty":conservative_peak,
            "within_uncertainty_adjusted_tolerance":conservative_pass,
        }
    return {
        "schema":"dsiq-measured-bullet-vs-external-v1",
        "external_sha256":hashlib.sha256(actual.read_bytes()).hexdigest(),
        "simulation_sha256":hashlib.sha256(simulated.read_bytes()).hexdigest(),
        "instrument_identifier":measured["instrument_identifier"],
        "equipment_identifier":measured["equipment_identifier"],
        "tolerance_units_per_joint":tolerance,
        "comparison":result,
        "all_operator_tolerances_met":all_pass,
        "all_conservative_uncertainty_bounds_met":all_conservative_pass,
        "standard_uncertainty_complete":any_uncertainty,
        "uncertainty_required":require_uncertainty,
        "coverage_factor":float(coverage_factor),
        "external_joint_unit_contract_present":source_joint_units is not None,
        "simulation_urdf_joint_units_present":actual_urdf_units is not None,
        "simulation_source_sha256_matches_external_metadata":source_hash_matches,
        "source_binding_required":require_source_binding,
        "all_simulated_joint_names_compared":True,
        "completed_simulation_monitor_passed_if_reported":
            source.get("runtime_monitor_passed") is True,
        "measurement_clock_alignment_independently_verified":False,
        "model_uncertainty_included":False,
        "measurement_source_independently_authenticated":False,
        "operator_tolerance_independently_approved":False,
        "industrial_physics_calibrated":False,
        "interpretation":"Operator-provided measurements only. Conservative bounds are abs(residual)+k*reported measurement standard uncertainty, not a confidence guarantee. Source identity, clock alignment, simulation uncertainty and external qualification still require review.",
    }
