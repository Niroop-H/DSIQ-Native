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
                                  *, tolerance: float=.005) -> dict:
    if type(tolerance) not in (int,float) or not math.isfinite(tolerance) or not 0<tolerance<=.1:
        raise ValueError("Measured trajectory tolerance must be finite in (0,.1]")
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
    samples=source.get("sampled_reports")
    if not isinstance(samples,list) or len(samples)<3:
        raise ValueError("At least three actual Bullet motion feedback samples required")
    time=[x.get("time_s") for x in samples]
    if any(type(t) not in (int,float) or not math.isfinite(t) for t in time):
        raise ValueError("Simulation timing must be finite")
    if not all(x<y for x,y in zip(time,time[1:])):
        raise ValueError("Simulation feedback times must be monotonic")
    names=set(samples[0]["measured_joint_states"])
    if not names:
        raise ValueError("No actual imported robot feedback available")
    for sample in samples:
        if set(sample.get("measured_joint_states",{}))!=names:
            raise ValueError("Inconsistent simulation feedback joint set")
    observations=measured.get("observations")
    if not isinstance(observations,list) or len(observations)<3 or len(observations)>30000:
        raise ValueError("External measurements require 3..30000 samples")
    seen=set()
    comparison={}
    for row in observations:
        if not isinstance(row,dict):
            raise ValueError("Observation must be an object")
        name=row.get("joint")
        t=row.get("time_s")
        value=row.get("measured_position")
        units=row.get("unit")
        if name not in names or units not in ("m","rad"):
            raise ValueError("External joint name or unit does not match declared simulator")
        if not all(type(x) in (int,float) and math.isfinite(x) for x in (t,value)):
            raise ValueError("External time and position must be finite")
        if not time[0]<=t<=time[-1]:
            raise ValueError("External measurement must lie within actual Bullet sampled interval")
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
        comparison.setdefault(name,[]).append((t,deviation,units))
    if not comparison or any(len(rows)<3 for rows in comparison.values()):
        raise ValueError("At least three external measurements per joint are required")
    result={}
    all_pass=True
    for name,rows in comparison.items():
        rows.sort(key=lambda x:x[0])
        diffs=[x[1] for x in rows]
        rmse=math.sqrt(sum(v*v for v in diffs)/len(diffs))
        peak=max(abs(v) for v in diffs)
        passed=peak<=tolerance
        all_pass=all_pass and passed
        result[name]={
            "samples":len(rows),"unit":rows[0][2],
            "mean_signed_error":sum(diffs)/len(diffs),
            "rmse":rmse,"maximum_absolute_error":peak,
            "within_operator_tolerance":passed,
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
        "measurement_source_independently_authenticated":False,
        "operator_tolerance_independently_approved":False,
        "industrial_physics_calibrated":False,
        "interpretation":"Comparison with operator-supplied measurements only; external qualification requires traceable physical reference review.",
    }
