"""Actual Bullet link-COM kinematics as idealized IMU-like measurements.

The angular velocity is sampled directly from Bullet. Linear acceleration
is a finite difference of consecutive *measured* COM velocities, and specific
force is acceleration minus authored world gravity, rotated to COM axes.

This is NOT a calibrated or noisy physical IMU, navigation estimator, or
high-rate hardware clock. The first/non-advanced sample cannot estimate
acceleration and is explicitly marked invalid; no zero reading is invented.
"""
from __future__ import annotations

import math
from .bullet_runtime import PHYSICS_DT


class IdealBulletInertialProbe:
    """Persist one snapshot per link to compute sensor-clock differences."""

    def __init__(self, world):
        self.world=world
        self._previous={}

    def sample(self, *, link_index: int=5) -> dict:
        if type(link_index) is not int or not 0<=link_index<=5:
            raise ValueError("Reference robot IMU link index must be in [0,5]")
        p=self.world.p
        cid=self.world.client
        step=self.world.step_index
        values=p.getLinkState(
            self.world.robot,link_index,
            computeLinkVelocity=True,computeForwardKinematics=True,
            physicsClientId=cid,
        )
        if len(values)<8:
            raise RuntimeError("Bullet did not return full COM kinematics")
        quaternion=tuple(float(x) for x in values[1])
        velocity=tuple(float(x) for x in values[6])
        omega=tuple(float(x) for x in values[7])
        if any(not math.isfinite(x) for x in quaternion+velocity+omega):
            raise ArithmeticError("Nonfinite actual Bullet IMU input state")
        matrix=tuple(float(x) for x in p.getMatrixFromQuaternion(quaternion))
        if len(matrix)!=9 or any(not math.isfinite(x) for x in matrix):
            raise ArithmeticError("Invalid Bullet COM orientation matrix")
        def into_com_frame(vector):
            x,y,z=vector
            return [
                matrix[0]*x+matrix[3]*y+matrix[6]*z,
                matrix[1]*x+matrix[4]*y+matrix[7]*z,
                matrix[2]*x+matrix[5]*y+matrix[8]*z,
            ]
        previous=self._previous.get(link_index)
        valid=previous is not None and step>previous[0]
        dt=(step-previous[0])*PHYSICS_DT if valid else None
        acceleration_world=None
        specific_force_body=None
        if valid:
            acceleration_world=[
                (velocity[k]-previous[1][k])/dt for k in range(3)
            ]
            g=self.world.settings.gravity_m_s2
            specific_force_body=into_com_frame((
                acceleration_world[0],
                acceleration_world[1],
                acceleration_world[2]+g,
            ))
            if any(not math.isfinite(x) for x in acceleration_world+specific_force_body):
                raise ArithmeticError("Nonfinite derived finite-difference IMU output")
        if previous is None or step>previous[0] or step<previous[0]:
            self._previous[link_index]=(step,velocity)
        return {
            "schema":"dsiq-bullet-reference-imu-probe-v1",
            "sensor_type":"IDEAL_BULLET_COM_INERTIAL_PROBE",
            "link_index":link_index,
            "physics_step_index":step,
            "simulation_time_s":step*PHYSICS_DT,
            "sample_delta_t_s":dt,
            "linear_velocity_world_m_s":list(velocity),
            "angular_velocity_world_rad_s":list(omega),
            "angular_velocity_com_rad_s":into_com_frame(omega),
            "linear_acceleration_world_m_s2":acceleration_world,
            "specific_force_com_m_s2":specific_force_body,
            "com_orientation_xyzw":list(quaternion),
            "acceleration_is_valid":bool(valid),
            "readout_status":("ADVANCED_PHYSICS_SAMPLE" if valid else
                              "FIRST_OR_UNCHANGED_PHYSICS_STEP"),
            "sensor_noise_added":False,
            "sensor_calibrated":False,
            "simulated_hardware_imu":False,
            "world_gravity_m_s2":[0.,0.,-self.world.settings.gravity_m_s2],
            "note":"Finite-difference COM acceleration; actual Bullet state, not physical IMU commissioning.",
        }
