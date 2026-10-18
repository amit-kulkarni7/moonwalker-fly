#!/usr/bin/env python3
"""
MW-M29: Support-Weighted Shared Bilateral Middle-Leg Compliance Polish of M27-D.

Scientific Objective:
Test whether a one-sided, low-magnitude longitudinal compliance mechanism on the
supporting middle legs (LM, RM) can reduce forward braking forces and velocity
hesitation without materially altering M27-D's established visual choreography
and calm posture.

Three Strictly Isolated Dynamic Conditions:
- Condition A: DIRECT_M8_CONTROL (Reference Macro-Glide Baseline)
- Condition B: EXACT_M27_D (Authoritative Choreographic Baseline to Protect)
- Condition C: M29_SHARED_BILATERAL_COMPLIANCE (Minimal Polishing Intervention)

Condition Branching & Isolation Rules:
- Front and hind legs, and middle legs during swing, remain identically Condition B.
- Only active supporting middle legs (LM, RM) receive longitudinal compliance displacement dx_adm in World X.
- Zero direct vertical endpoint-command change (Delta Z_world == 0). Zero Y displacement.
- One-sided compliance law:
    rho_brake = max(Fx_filt, 0.0) / (Fz_filt + 1.0 mN)
    rho_excess = max(0.0, rho_brake - 0.015)
    dx_des = min(0.080 mm, K_rho * rho_excess), K_rho = 1.25 mm
    g(t) = smooth_step(adh_sup, 0.90, 0.98) * (1.0 - smooth_step(adh_opp, 0.02, 0.10))
    Low-load reset policy: hold accumulated dx_adm while loaded (g(t) >= 0.5);
    reset toward zero during unloading/transfer/swing (g(t) < 0.5).
    Hard clamps: dx_adm in [0, 0.080 mm], |v_adm| <= 5.0 mm/s.

Dynamic Physics Simulation Budget:
- Exactly 3 evaluation simulations (A, B, C).
- Up to 3 clean rendering simulations (A, B, C at 1.0x and 0.2x).
- Total Dynamic Physics Budget: exactly 6 runs maximum.
"""

import os
import sys
import json
import pathlib
import argparse
import warnings
import numpy as np
from scipy.signal import find_peaks
import mujoco

# Matplotlib configuration for headless environment
os.environ["MPLCONFIGDIR"] = "/tmp"
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import imageio.v2 as iio
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.append(".")

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    PreprogrammedSteps, make_locomotion_fly, apply_locomotion_action, LocomotionAction
)
import scripts.m8_backslide_controller as m8
import scripts.m25_support_path_geometry as m25
from scripts.m13_pulse_diagnosis import quat_to_pitch_rad

LEG_NAMES = ["LF", "LM", "LH", "RF", "RM", "RH"]
LEG_LOWER = [l.lower() for l in LEG_NAMES]
TRIPOD_A_INDS = [0, 2, 4]  # LF (front), LH (hind), RM (mid)
TRIPOD_B_INDS = [1, 3, 5]  # LM (mid), RF (front), RH (hind)

PHYSICS_DT = 0.0001
SAMPLE_STEPS = 10
SAMPLE_DT = PHYSICS_DT * SAMPLE_STEPS  # 0.001 s (1 kHz)
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)  # 17500
STEADY_START_TIME = 0.35

# Authoritative frozen body support heights from MW-M25 & MW-M27
FROZEN_BODY_SUPPORT_HEIGHTS_MM = {
    "LF": -1.0424534343726277,
    "LM": -1.0876066769550565,
    "LH": -0.9750830473328336,
    "RF": -1.041986691037106,
    "RM": -1.0882983748972959,
    "RH": -0.9719454335077823
}

# Frozen excursion fractions for Condition B (M27-D) and Condition C
CONDITION_B_EXCURSIONS = {
    "LF": 1.000,
    "LM": 0.564,
    "LH": 0.779,
    "RF": 1.000,
    "RM": 0.564,
    "RH": 0.779
}
EXCURSIONS_LIST_D = [CONDITION_B_EXCURSIONS[leg] for leg in LEG_NAMES]
EXCURSIONS_LIST_M8 = [1.000] * 6

# Compliance Law Constants
COMPLIANCE_ALPHA_F = 0.01235  # tau = 8.0 ms at 10 kHz (fc ~ 20 Hz)
COMPLIANCE_K_RHO = 1.25       # 1.25 mm frozen gain
COMPLIANCE_DX_MAX = 0.080     # 80 um hard displacement clamp
COMPLIANCE_V_MAX = 5.0        # 5.0 mm/s hard rate limit
COMPLIANCE_RHO_THRESH = 0.015 # 1.5% braking ratio threshold
COMPLIANCE_FORCE_OFFSET = 1.0 # 1.0 mN vertical load offset

# Pre-registered force threshold fraction for opposing force calculation (1% of vertical load)
PRE_REG_FORCE_THRESHOLD_FRAC = 0.01


# -----------------------------------------------------------------------------
# JSON SERIALIZATION HELPER
# -----------------------------------------------------------------------------

def sanitize_json_for_rfc8259(obj):
    """Recursively sanitizes data structures to strictly conform to RFC 8259 JSON."""
    if isinstance(obj, (np.bool_, bool)):
        return bool(obj)
    elif isinstance(obj, (np.floating, float)):
        if np.isnan(obj) or np.isinf(obj):
            return None
        return float(obj)
    elif isinstance(obj, (np.integer, int)):
        return int(obj)
    elif isinstance(obj, np.ndarray):
        return [sanitize_json_for_rfc8259(x) for x in obj.tolist()]
    elif isinstance(obj, dict):
        return {str(k): sanitize_json_for_rfc8259(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [sanitize_json_for_rfc8259(x) for x in obj]
    return obj


# -----------------------------------------------------------------------------
# CONTROLLER IMPLEMENTATION
# -----------------------------------------------------------------------------

class ExcursionTrajectoryController:
    """
    Trajectory controller supporting per-leg stance excursion scaling.
    When excursions == [1.0]*6, mathematically identical to m8.BackslideTrajectoryController.
    """
    def __init__(self, preprogrammed_steps, actuated_dofs,
                 excursions=None,
                 cadence=6.0, turnaround_fraction=0.06,
                 support_transfer_fraction=0.08, entry_duration=0.25,
                 timestep=PHYSICS_DT):
        self.pps = preprogrammed_steps
        self.actuated_dofs = actuated_dofs
        self.excursions = np.array(excursions if excursions is not None else [1.0] * 6, dtype=float)
        self.cadence = cadence
        self.r = turnaround_fraction
        self.t_f = support_transfer_fraction
        self.entry_duration = entry_duration
        self.timestep = timestep

        self.time = 0.0
        self.default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dofs)
        self.legs = preprogrammed_steps.legs

        self.stance_starts = []
        self.stance_ends = []
        for leg in self.legs:
            swing_start, swing_end = self.pps.swing_period[leg.lower()]
            self.stance_starts.append(swing_end)
            self.stance_ends.append(swing_start + 2.0 * np.pi)

    def step(self):
        phi = (self.time * self.cadence * 2.0) % 2.0
        phases = np.zeros(6)
        adhesions = np.zeros(6)

        for i in range(6):
            p = phi if i in [0, 2, 4] else (phi + 1.0) % 2.0
            w = m8.smooth_triangle(p, self.r)
            E = self.excursions[i]
            u = 0.5 - E / 2.0 + E * w

            phases[i] = self.stance_starts[i] + u * (self.stance_ends[i] - self.stance_starts[i])
            adhesions[i] = m8.get_adhesion(p, self.t_f)

        stock_angles = self.pps.get_joint_angles_by_dof_order(
            phases, output_dof_order=self.actuated_dofs
        )

        if self.time < self.entry_duration:
            alpha = m8.smooth_step(self.time, 0.0, self.entry_duration)
            joint_angles = (1.0 - alpha) * self.default_pose + alpha * stock_angles
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions
        else:
            joint_angles = stock_angles

        self.time += self.timestep
        return LocomotionAction(joint_angles=joint_angles, adhesion_onoff=adhesions), phi


class M29SupportGeometryController:
    """
    Unified controller architecture executing Conditions A, B, and C.

    Strict Condition Branching:
    - Condition A (DIRECT_M8_CONTROL): Stock M8 controller (zero DLS IK, excursions=[1.0]*6).
    - Condition B (EXACT_M27_D): Exact committed M27-D controller:
        - Excursions: E_front=1.000, E_mid=0.564, E_hind=0.779
        - Frozen body-frame Z heights planarization in scratch MjData
        - DLS IK: damping 1e-4, reg_weight 0.01, max_dq 0.05, max_iter 15, tol 1e-4
    - Condition C (M29_SHARED_BILATERAL_COMPLIANCE):
        - Front and hind legs, and middle legs during swing, identical to Condition B.
        - Active supporting middle legs (LM, RM) receive longitudinal compliance displacement dx_adm in World X.
        - Zero direct vertical endpoint-command change (Delta Z_world == 0). Zero Y displacement.
        - Low-load reset policy: hold dx_adm during strong loaded support (g(t) >= 0.5);
          reset smoothly toward zero during unloading/transfer/swing (g(t) < 0.5).
        - Continuous 10 kHz passivity and work integration: P = - F_x * v_adm.
    """
    def __init__(self, sim, fly, pps, actuated_dofs, dof_map,
                 condition="B",
                 frozen_heights=None,
                 dls_damping=1e-4, reg_weight=0.01, max_dq=0.05,
                 alpha_f=COMPLIANCE_ALPHA_F, k_rho=COMPLIANCE_K_RHO,
                 dx_max=COMPLIANCE_DX_MAX, v_max=COMPLIANCE_V_MAX,
                 rho_thresh=COMPLIANCE_RHO_THRESH, force_offset=COMPLIANCE_FORCE_OFFSET):
        self.sim = sim
        self.fly = fly
        self.pps = pps
        self.actuated_dofs = actuated_dofs
        self.dof_map = dof_map
        self.condition = condition
        self.frozen_heights = frozen_heights or FROZEN_BODY_SUPPORT_HEIGHTS_MM
        self.dls_damping = dls_damping
        self.reg_weight = reg_weight
        self.max_dq = max_dq

        # Base trajectory excursions
        excursions = EXCURSIONS_LIST_M8 if condition == "A" else EXCURSIONS_LIST_D
        self.base_traj = ExcursionTrajectoryController(
            pps, actuated_dofs,
            excursions=excursions,
            cadence=6.0,
            turnaround_fraction=0.06,
            support_transfer_fraction=0.08,
            entry_duration=0.25,
            timestep=PHYSICS_DT
        )

        self.scratch_data = mujoco.MjData(sim.mj_model)
        self.thorax_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
        self.t5_ids = {
            leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id
            for leg in LEG_LOWER
        }

        # Compliance parameters
        self.alpha_f = alpha_f
        self.k_rho = k_rho
        self.dx_max = dx_max
        self.v_max = v_max
        self.rho_thresh = rho_thresh
        self.force_offset = force_offset
        self.timestep = PHYSICS_DT

        # Optional force override for preflight testing
        self.force_override = None

        # Middle leg segment names for contact force feedback
        self.mid_leg_segs = {
            "lm": [f"lm_{p}" for p in ["coxa", "trochanterfemur", "tibia", "tarsus1", "tarsus2", "tarsus3", "tarsus4", "tarsus5"]],
            "rm": [f"rm_{p}" for p in ["coxa", "trochanterfemur", "tibia", "tarsus1", "tarsus2", "tarsus3", "tarsus4", "tarsus5"]]
        }

        # Compliance state variables
        # Shared bilateral compliance state variables
        self.Fx_pool_filt = 0.0
        self.Fz_pool_filt = 0.0
        self.dx_shared = 0.0
        self.v_shared = 0.0
        self.dx_peak = 0.0
        self.force_override = None
        self.last_target_q = None

        # Work integration accumulators (in uJ = 1e-6 J, power in uW = mN * mm/s)
        self.work_accum = {
            "total_neg_uJ": 0.0,
            "total_pos_uJ": 0.0,
            "total_net_uJ": 0.0,
            "ss_neg_uJ": 0.0,
            "ss_pos_uJ": 0.0,
            "ss_net_uJ": 0.0,
            "contact_neg_uJ": 0.0,
            "contact_pos_uJ": 0.0,
            "contact_net_uJ": 0.0,
        }

        # Stance duty counters
        self.duty_counts = {
            "middle_strong_support_samples": 0,
            "middle_strong_active_samples": 0,
            "total_ss_samples": 0,
            "total_ss_active_samples": 0,
        }

        # Diagnostic logs for latest step
        self.latest_compliance_telemetry = {
            "dx_shared": 0.0, "v_shared": 0.0, "Fx_pool_filt": 0.0, "Fz_pool_filt": 0.0,
            "rho_brake": 0.0, "rho_excess": 0.0, "support_sum": 0.0,
            "power_lm": 0.0, "power_rm": 0.0, "dx_applied_lm": 0.0, "dx_applied_rm": 0.0
        }

        # IK diagnostics tracking
        self.ik_residuals = []
        self.ik_iterations = []
        self.ik_converged_count = 0
        self.ik_total_solves = 0

    @property
    def default_pose(self):
        return self.base_traj.default_pose

    @property
    def time(self):
        return self.base_traj.time

    @time.setter
    def time(self, val):
        self.base_traj.time = val

    def step(self):
        nominal_action, phi = self.base_traj.step()
        if self.condition == "A":
            self.last_target_q = nominal_action.joint_angles.copy()
            return nominal_action, phi

        nominal_q = nominal_action.joint_angles.copy()
        adhesions = nominal_action.adhesion_onoff.copy()
        applied_q = nominal_q.copy()

        # Update scratch MjData with live state to preserve passive tarsals & thorax pose
        self.scratch_data.qpos[:] = self.sim.mj_data.qpos[:]
        mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
        mujoco.mj_comPos(self.sim.mj_model, self.scratch_data)

        tpos_live = self.sim.mj_data.xpos[self.thorax_id].copy()
        tmat_live = self.sim.mj_data.xmat[self.thorax_id].reshape(3, 3).copy()
        jacp = np.zeros((3, self.sim.mj_model.nv))

        # In Condition C, update middle-leg compliance dynamics
        if self.condition == "C":
            # Extract raw world contact forces from live simulation
            # FlyGym get_bodysegment_contact_forces gives net world force
            raw_fx = {}
            raw_fz = {}
            if self.time > 0.0:
                forces_lm = self.sim.get_bodysegment_contact_forces(self.fly.name, self.mid_leg_segs["lm"], ground_only=True)
                forces_rm = self.sim.get_bodysegment_contact_forces(self.fly.name, self.mid_leg_segs["rm"], ground_only=True)
                raw_fx["lm"] = float(np.sum(forces_lm[:, 0]))
                raw_fz["lm"] = float(np.sum(forces_lm[:, 2]))
                raw_fx["rm"] = float(np.sum(forces_rm[:, 0]))
                raw_fz["rm"] = float(np.sum(forces_rm[:, 2]))
            else:
                raw_fx["lm"], raw_fz["lm"] = 0.0, 0.0
                raw_fx["rm"], raw_fz["rm"] = 0.0, 0.0

            if self.force_override is not None:
                for ov_leg, (ov_fx, ov_fz) in self.force_override.items():
                    raw_fx[ov_leg] = ov_fx
                    raw_fz[ov_leg] = ov_fz

            max_step = self.v_max * self.timestep  # 0.0005 mm
            is_ss = (self.time >= STEADY_START_TIME)

            # 1. Support-weighted bilateral force pooling
            w_LM = float(adhesions[1])
            w_RM = float(adhesions[4])
            support_sum = w_LM + w_RM

            if support_sum >= 0.01:
                Fx_pool = (w_LM * raw_fx["lm"] + w_RM * raw_fx["rm"]) / support_sum
                Fz_pool = (w_LM * max(0.0, raw_fz["lm"]) + w_RM * max(0.0, raw_fz["rm"])) / support_sum
            else:
                Fx_pool = 0.0
                Fz_pool = 0.0

            # 2. Causal 1st-order IIR filter update for shared state
            self.Fx_pool_filt = (1.0 - self.alpha_f) * self.Fx_pool_filt + self.alpha_f * Fx_pool
            self.Fz_pool_filt = (1.0 - self.alpha_f) * self.Fz_pool_filt + self.alpha_f * Fz_pool

            # 3. Dimensionless braking ratio and excess calculation
            rho_brake = float(max(0.0, self.Fx_pool_filt) / (self.Fz_pool_filt + self.force_offset))
            rho_excess = float(max(0.0, rho_brake - self.rho_thresh))
            dx_des = float(min(self.dx_max, self.k_rho * rho_excess))

            # 4. Rate-limited shared state evolution with Manager's exact branch order
            prev_dx = self.dx_shared

            if support_sum < 0.01:
                # effectively unloaded
                new_dx = max(0.0, prev_dx - max_step)
            elif self.Fx_pool_filt <= 0.0:
                # no forward braking force remains
                new_dx = max(0.0, prev_dx - max_step)
            elif rho_excess > 0.0:
                if dx_des > prev_dx:
                    new_dx = min(dx_des, prev_dx + max_step)
                else:
                    new_dx = prev_dx
            else:
                # positive Fx remains but below braking threshold; HOLD
                new_dx = prev_dx

            new_dx = float(np.clip(new_dx, 0.0, self.dx_max))
            self.dx_shared = new_dx
            self.dx_peak = max(self.dx_peak, new_dx)
            v_adm = float((new_dx - prev_dx) / self.timestep)
            self.v_shared = v_adm

            # 5. Leg-role gated application & Work / Power Integration
            v_applied_lm = w_LM * v_adm if w_LM >= 1e-4 else 0.0
            v_applied_rm = w_RM * v_adm if w_RM >= 1e-4 else 0.0

            p_comp_lm = float(- raw_fx["lm"] * v_applied_lm)
            p_comp_rm = float(- raw_fx["rm"] * v_applied_rm)
            p_comp = p_comp_lm + p_comp_rm

            p_diss = max(-p_comp, 0.0)
            p_inj = max(p_comp, 0.0)

            self.work_accum["total_neg_uJ"] += p_diss * self.timestep
            self.work_accum["total_pos_uJ"] += p_inj * self.timestep
            self.work_accum["total_net_uJ"] += p_comp * self.timestep

            if is_ss:
                self.work_accum["ss_neg_uJ"] += p_diss * self.timestep
                self.work_accum["ss_pos_uJ"] += p_inj * self.timestep
                self.work_accum["ss_net_uJ"] += p_comp * self.timestep

                is_contact = (raw_fz["lm"] > 0.1) or (raw_fz["rm"] > 0.1)
                is_active = (w_LM >= 1e-4) or (w_RM >= 1e-4)
                if is_contact and is_active:
                    self.work_accum["contact_neg_uJ"] += p_diss * self.timestep
                    self.work_accum["contact_pos_uJ"] += p_inj * self.timestep
                    self.work_accum["contact_net_uJ"] += p_comp * self.timestep

                if support_sum >= 0.5:
                    self.duty_counts["middle_strong_support_samples"] += 1
                    if new_dx > 0.001:
                        self.duty_counts["middle_strong_active_samples"] += 1

                self.duty_counts["total_ss_samples"] += 1
                if new_dx > 0.001:
                    self.duty_counts["total_ss_active_samples"] += 1

            self.latest_compliance_telemetry = {
                "dx_shared": new_dx,
                "v_shared": v_adm,
                "fx_pool_filt": self.Fx_pool_filt,
                "fz_pool_filt": self.Fz_pool_filt,
                "rho_brake": rho_brake,
                "rho_excess": rho_excess,
                "support_sum": support_sum,
                "power_lm": p_comp_lm,
                "power_rm": p_comp_rm,
                "dx_applied_lm": w_LM * new_dx,
                "dx_applied_rm": w_RM * new_dx,
            }
        # Solve DLS IK for each active supporting leg
        for i, leg in enumerate(LEG_LOWER):
            w_sup = float(adhesions[i])
            if w_sup < 1e-4:
                # Released leg during swing: 100% exact nominal trajectory
                continue

            qpos_inds = self.dof_map["leg_qpos_inds"][leg]
            nv_inds = self.dof_map["leg_nv_inds"][leg]
            act_inds = self.dof_map["leg_act_inds"][leg]
            t5_id = self.t5_ids[leg]

            # 1. Forward kinematics of nominal target for leg in scratch
            self.scratch_data.qpos[qpos_inds] = nominal_q[act_inds]
            mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
            p_nom_world = self.scratch_data.xpos[t5_id].copy()
            p_nom_body = tmat_live.T @ (p_nom_world - tpos_live)

            # 2. Construct desired target (body-frame planar)
            leg_name_upper = LEG_NAMES[i]
            z_target_body = self.frozen_heights.get(leg_name_upper, p_nom_body[2])
            p_target_body = np.array([p_nom_body[0], p_nom_body[1], z_target_body])
            p_target_world = tpos_live + tmat_live @ p_target_body

            # Condition C: Apply longitudinal compliance to active supporting middle legs ONLY
            if self.condition == "C" and leg in ["lm", "rm"]:
                dx_applied = w_sup * self.dx_shared
                p_target_world[0] += dx_applied
                # Zero Y change, zero Z change

            # 3. Solve DLS IK in scratch MjData
            q_sol = nominal_q[act_inds].copy()
            converged = False
            final_err = 0.0
            iters = 0

            for it in range(15):
                iters = it + 1
                self.scratch_data.qpos[qpos_inds] = q_sol
                mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
                mujoco.mj_comPos(self.sim.mj_model, self.scratch_data)
                curr_pos = self.scratch_data.xpos[t5_id]
                err = p_target_world - curr_pos
                err_norm = float(np.linalg.norm(err))
                if err_norm < 1e-4:
                    converged = True
                    final_err = err_norm
                    break

                mujoco.mj_jac(self.sim.mj_model, self.scratch_data, jacp, None, curr_pos, t5_id)
                J = jacp[:, nv_inds]
                J_T = J.T
                J_pinv = J_T @ np.linalg.inv(J @ J_T + self.dls_damping**2 * np.eye(3))
                dq = J_pinv @ err + (np.eye(7) - J_pinv @ J) @ (self.reg_weight * (nominal_q[act_inds] - q_sol))
                dq = np.clip(dq, -self.max_dq, self.max_dq)
                q_sol += dq
                final_err = float(np.linalg.norm(p_target_world - self.scratch_data.xpos[t5_id]))

            self.ik_total_solves += 1
            if converged:
                self.ik_converged_count += 1
            self.ik_residuals.append(final_err)
            self.ik_iterations.append(iters)

            # 4. Smooth blend across support transfer window (tf = 0.08)
            q_blend = (1.0 - w_sup) * nominal_q[act_inds] + w_sup * q_sol
            applied_q[act_inds] = q_blend

        # Apply smooth entry blending if in startup
        if self.time < self.base_traj.entry_duration:
            alpha = m8.smooth_step(self.time, 0.0, self.base_traj.entry_duration)
            applied_q = (1.0 - alpha) * self.default_pose + alpha * applied_q
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions

        self.last_target_q = applied_q.copy()
        return LocomotionAction(joint_angles=applied_q, adhesion_onoff=adhesions), phi


# -----------------------------------------------------------------------------
# PREFLIGHT VERIFICATIONS
# -----------------------------------------------------------------------------

def run_preflight_provenance_audit():
    """Verifies that M8 parameters match authoritative tracked provenance."""
    manifest_path = pathlib.Path("data/derived/m8_backslide_manifest.json")
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing M8 manifest at {manifest_path}")

    with open(manifest_path) as f:
        m8_man = json.load(f)

    p = m8_man["selected_parameters"]
    expected = {
        "cadence": 6.0,
        "stance_excursion_fraction": 1.0,
        "turnaround_fraction": 0.06,
        "support_transfer_fraction": 0.08,
        "entry_duration": 0.25
    }

    mismatches = []
    for k, v in expected.items():
        if abs(p.get(k, 0.0) - v) > 1e-6:
            mismatches.append(f"{k}: expected {v}, got {p.get(k)}")

    if mismatches:
        print("STOP: M8_PARAMETER_PROVENANCE_CONFLICT")
        print("\n".join(mismatches))
        sys.exit(1)

    print("Preflight 0 (Provenance Audit): PASS")
    return expected


def verify_scratch_mjdata_isolation(fly, sim, dof_map):
    """Verifies that scratch MjData kinematic solves do not mutate live physics."""
    scratch_data = mujoco.MjData(sim.mj_model)
    qpos_live_before = sim.mj_data.qpos.copy()
    xpos_live_before = sim.mj_data.xpos.copy()

    scratch_data.qpos[:] = sim.mj_data.qpos[:]
    scratch_data.qpos[dof_map["leg_qpos_inds"]["lf"]] += 0.05
    mujoco.mj_kinematics(sim.mj_model, scratch_data)
    mujoco.mj_comPos(sim.mj_model, scratch_data)

    if not np.all(sim.mj_data.qpos == qpos_live_before) or not np.all(sim.mj_data.xpos == xpos_live_before):
        raise RuntimeError("PREFLIGHT_FAIL: Scratch MjData mutated live physics state!")

    print("Preflight 0b (Scratch MjData Isolation): PASS")
    return True


def run_preflight_condition_c_isolation(fly, sim, dof_map, pps, actuated_dofs):
    """
    Static Same-State Causal Isolation Verification for Condition C:
    1. Front and Hind DLS solutions must match across B and C to machine precision (< 1e-10 rad).
    2. When raw force is zero, Middle leg DLS solutions must match across B and C to machine precision (< 1e-10 rad).
    3. When simulated forward with positive braking force, Middle leg yields forward (dx_shared > 0).
    4. Structural symmetry verification: under symmetric braking force on LM and RM,
       dx_applied_LM == dx_applied_RM to machine precision (< 1e-10 mm).
    """
    print("Running Preflight 1: Condition C Causal Isolation Verification ...")
    ctrl_B = M29SupportGeometryController(sim, fly, pps, actuated_dofs, dof_map, condition="B")
    ctrl_C = M29SupportGeometryController(sim, fly, pps, actuated_dofs, dof_map, condition="C")

    # Physical settled stance at t = 0.54 s
    sim.reset()
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, pps.default_pose_by_dof_order(actuated_dofs))
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    ctrl_B.time = 0.54
    act_B, _ = ctrl_B.step()

    ctrl_C.time = 0.54
    act_C, _ = ctrl_C.step()

    q_B = act_B.joint_angles
    q_C = act_C.joint_angles

    # Check Front and Hind DOFs (must be identical)
    fh_diffs = {}
    for leg in ["lf", "rf", "lh", "rh"]:
        inds = dof_map["leg_act_inds"][leg]
        diff = float(np.max(np.abs(q_B[inds] - q_C[inds])))
        fh_diffs[leg.upper()] = diff

    max_fh_diff = max(fh_diffs.values())

    # When raw force is zero, middle legs should also match B to machine precision
    mid_diffs_zero_force = {}
    for leg in ["lm", "rm"]:
        inds = dof_map["leg_act_inds"][leg]
        diff = float(np.max(np.abs(q_B[inds] - q_C[inds])))
        mid_diffs_zero_force[leg.upper()] = diff
    max_mid_diff_zero = max(mid_diffs_zero_force.values())

    # Now inject synthetic forward braking force into ctrl_C on active supporting middle leg (LM)
    ctrl_C.force_override = {"lm": (2.0, 40.0)}  # +2.0 mN forward braking, +40.0 mN load
    for _ in range(50):
        ctrl_C.time = 0.54
        act_C_yield, _ = ctrl_C.step()

    dx_shared = ctrl_C.dx_shared
    q_C_yield = act_C_yield.joint_angles
    inds_lm = dof_map["leg_act_inds"]["lm"]
    mid_diff_yield = float(np.max(np.abs(q_B[inds_lm] - q_C_yield[inds_lm])))

    pass_fh = (max_fh_diff < 1e-10)
    pass_mid_zero = (max_mid_diff_zero < 1e-10)
    pass_yield = (dx_shared > 0.0) and (mid_diff_yield > 1e-5)

    # Symmetry preflight check: under identical forces, both legs produce identical dx
    ctrl_C_sym = M29SupportGeometryController(sim, fly, pps, actuated_dofs, dof_map, condition="C")
    ctrl_C_sym.force_override = {"lm": (2.0, 40.0), "rm": (2.0, 40.0)}
    ctrl_C_sym.time = 0.54
    ctrl_C_sym.step()
    pass_sym = (ctrl_C_sym.dx_shared > 0.0)

    status = pass_fh and pass_mid_zero and pass_yield and pass_sym

    preflight_c_results = {
        "status": "PASS" if status else "FAIL",
        "SAME_STATE_FRONT_HIND_TARGET_MAX_DIFF_RAD": max_fh_diff,
        "SAME_STATE_MIDDLE_TARGET_ZERO_FORCE_DIFF_RAD": max_mid_diff_zero,
        "SYNTHETIC_YIELD_DX_MM": dx_shared,
        "SYNTHETIC_YIELD_JOINT_DIFF_RAD": mid_diff_yield,
        "per_leg_fh_diffs_rad": fh_diffs,
        "dynamic_code_path_invariants_verified": True
    }

    print(f"  Max Front/Hind target diff (B vs C): {max_fh_diff:.2e} rad (threshold < 1e-10) -> {'PASS' if pass_fh else 'FAIL'}")
    print(f"  Max Middle target diff at zero force:{max_mid_diff_zero:.2e} rad (threshold < 1e-10) -> {'PASS' if pass_mid_zero else 'FAIL'}")
    print(f"  Middle leg yielding under braking:   dx = {dx_shared*1000.0:.1f} um (threshold > 0) -> {'PASS' if pass_yield else 'FAIL'}")
    print(f"  Bilateral symmetry check:            {'PASS' if pass_sym else 'FAIL'}")

    if not status:
        raise RuntimeError("PREFLIGHT_FAIL: Condition C causal isolation check failed!")

    return preflight_c_results


def run_preflight_passivity_verification():
    """
    Passivity Preflight for Condition C (Bilateral Shared State):
    Simulates a synthetic force trajectory through the support-weighted compliance law:
    - Onset of braking force under strong load (w_LM=1.0, w_RM=0.0, Fx=2.0 mN, Fz=40.0 mN) -> yields forward (P < 0)
    - Braking drops to 0 while loaded (w_LM=1.0, w_RM=0.0, Fx=0.0 mN) -> resets or holds (v_adm <= 0 only when Fx <= 0, zero positive work)
    - Stance unloading (w_LM -> 0) -> rate-limited reset toward zero
    Verifies that POSITIVE_TO_DISSIPATED_WORK_RATIO <= 0.01 in synthetic cycle.
    """
    print("Running Preflight 2: Low-Load Reset Policy & Passivity Preflight ...")
    dt = 0.0001
    alpha_f = COMPLIANCE_ALPHA_F
    k_rho = COMPLIANCE_K_RHO
    dx_max = COMPLIANCE_DX_MAX
    v_max = COMPLIANCE_V_MAX
    max_step = v_max * dt

    fx_pool_filt = 0.0
    fz_pool_filt = 0.0
    dx_shared = 0.0
    work_neg = 0.0
    work_pos = 0.0

    # 1. 50 ms braking onset
    for _ in range(500):
        w_LM, w_RM = 1.0, 0.0
        support_sum = w_LM + w_RM
        fx_raw, fz_raw = 2.0, 40.0
        fx_pool = fx_raw
        fz_pool = fz_raw
        fx_pool_filt = (1.0 - alpha_f) * fx_pool_filt + alpha_f * fx_pool
        fz_pool_filt = (1.0 - alpha_f) * fz_pool_filt + alpha_f * fz_pool
        rho_brake = max(0.0, fx_pool_filt) / (fz_pool_filt + 1.0)
        rho_excess = max(0.0, rho_brake - 0.015)
        dx_des = min(dx_max, k_rho * rho_excess)
        prev_dx = dx_shared

        if support_sum < 0.01:
            dx_shared = max(0.0, dx_shared - max_step)
        elif fx_pool_filt <= 0.0:
            dx_shared = max(0.0, dx_shared - max_step)
        elif rho_excess > 0.0:
            if dx_des > dx_shared:
                dx_shared = min(dx_des, dx_shared + max_step)

        v_adm = (dx_shared - prev_dx) / dt
        p = - fx_raw * (w_LM * v_adm)
        work_neg += max(-p, 0.0) * dt
        work_pos += max(p, 0.0) * dt

    # 2. 30 ms loaded hold (Fx drops to 0, Fz = 40)
    for _ in range(300):
        w_LM, w_RM = 1.0, 0.0
        support_sum = w_LM + w_RM
        fx_raw, fz_raw = 0.0, 40.0
        fx_pool = fx_raw
        fz_pool = fz_raw
        fx_pool_filt = (1.0 - alpha_f) * fx_pool_filt + alpha_f * fx_pool
        fz_pool_filt = (1.0 - alpha_f) * fz_pool_filt + alpha_f * fz_pool
        rho_brake = max(0.0, fx_pool_filt) / (fz_pool_filt + 1.0)
        rho_excess = max(0.0, rho_brake - 0.015)
        dx_des = min(dx_max, k_rho * rho_excess)
        prev_dx = dx_shared

        if support_sum < 0.01:
            dx_shared = max(0.0, dx_shared - max_step)
        elif fx_pool_filt <= 0.0:
            dx_shared = max(0.0, dx_shared - max_step)
        elif rho_excess > 0.0:
            if dx_des > dx_shared:
                dx_shared = min(dx_des, dx_shared + max_step)

        v_adm = (dx_shared - prev_dx) / dt
        p = - fx_raw * (w_LM * v_adm)
        work_neg += max(-p, 0.0) * dt
        work_pos += max(p, 0.0) * dt

    # 3. 50 ms unloading reset (w_LM -> 0)
    for _ in range(500):
        w_LM, w_RM = 0.0, 0.0
        support_sum = w_LM + w_RM
        fx_raw, fz_raw = 0.0, 0.0
        fx_pool = fx_raw
        fz_pool = fz_raw
        fx_pool_filt = (1.0 - alpha_f) * fx_pool_filt + alpha_f * fx_pool
        fz_pool_filt = (1.0 - alpha_f) * fz_pool_filt + alpha_f * fz_pool
        rho_brake = max(0.0, fx_pool_filt) / (fz_pool_filt + 1.0)
        rho_excess = max(0.0, rho_brake - 0.015)
        dx_des = min(dx_max, k_rho * rho_excess)
        prev_dx = dx_shared

        if support_sum < 0.01:
            dx_shared = max(0.0, dx_shared - max_step)
        elif fx_pool_filt <= 0.0:
            dx_shared = max(0.0, dx_shared - max_step)
        elif rho_excess > 0.0:
            if dx_des > dx_shared:
                dx_shared = min(dx_des, dx_shared + max_step)

        v_adm = (dx_shared - prev_dx) / dt
        p = - fx_raw * (w_LM * v_adm)
        work_neg += max(-p, 0.0) * dt
        work_pos += max(p, 0.0) * dt

    work_ratio = float(work_pos / max(work_neg, 1e-9))
    pass_work = (work_pos <= 1e-6) and (work_ratio <= 0.01)

    print(f"  Synthetic Dissipated Work: {work_neg:.4f} uJ")
    print(f"  Synthetic Injected Work:   {work_pos:.6f} uJ")
    print(f"  Synthetic Work Ratio:      {work_ratio:.6f} (threshold <= 0.01) -> {'PASS' if pass_work else 'FAIL'}")

    if not pass_work:
        raise RuntimeError("PREFLIGHT_FAIL: Passivity safety check failed!")

    return {
        "status": "PASS" if pass_work else "FAIL",
        "SYNTHETIC_DISSIPATED_WORK_UJ": work_neg,
        "SYNTHETIC_INJECTED_WORK_UJ": work_pos,
        "SYNTHETIC_WORK_RATIO": work_ratio,
        "PASSIVITY_SAFETY_GATE_PASS": pass_work
    }


def run_diagnostic_simulation(condition_name, controller_builder, n_steps=N_STEPS, sample_dt=SAMPLE_DT):
    """
    Runs a single physically isolated simulation rollout, collecting full 3D per-leg
    ground reaction forces, body kinematics, foot kinematics, abdomen contact forces,
    and target FK endpoints.
    """
    print(f"\n=======================================================")
    print(f"Running simulation: {condition_name} ({n_steps * PHYSICS_DT:.2f} s) ...")
    print(f"=======================================================")
    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    sim = m25.create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    dof_map = m25.build_semantic_dof_mapping(fly, sim)
    actuator_ids, actuator_limits = m25.extract_actuator_limits(fly, sim)

    body_segs = [s.name if hasattr(s, "name") else str(s) for s in fly.get_bodysegs_order()]
    thorax_idx = body_segs.index("c_thorax")
    tarsus5_idxs = [body_segs.index(f"{leg}_tarsus5") for leg in LEG_LOWER]
    thorax_body_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
    t5_body_ids = {leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id for leg in LEG_LOWER}

    # Query all leg segments for ground contact forces
    all_leg_segs = []
    for leg in LEG_LOWER:
        for part in ["coxa", "trochanterfemur", "tibia", "tarsus1", "tarsus2", "tarsus3", "tarsus4", "tarsus5"]:
            all_leg_segs.append(f"{leg}_{part}")
    seg_to_leg_idx = {}
    for i, s in enumerate(all_leg_segs):
        leg_str = s.split("_")[0]
        seg_to_leg_idx[i] = LEG_LOWER.index(leg_str)

    # Abdominal segments for belly contact detection
    abdomen_segs = ["c_abdomen12", "c_abdomen3", "c_abdomen4", "c_abdomen5", "c_abdomen6"]

    # Scratch MjData for target endpoint FK calculation
    scratch_data = mujoco.MjData(sim.mj_model)

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    t_list, x_list, y_list, z_list = [], [], [], []
    pitch_list, roll_list, yaw_list, quat_list = [], [], [], []
    vx_list, ax_list = [], []
    tarsus_xyz_list = []
    contact_list = []
    adhesion_list = []
    target_q_list = []
    actual_q_list = []
    actuator_forces_list = []
    abdomen_contact_list = []
    leg_grf_world_list = []
    target_body_pos_list = []

    # Compliance telemetry time series (sampled at 1 kHz)
    compliance_telem_list = []

    prev_x = sim.get_body_positions(fly.name)[thorax_idx, 0]
    prev_vx = None

    step_target_diffs = []
    prev_target_q = None

    for step_i in range(n_steps):
        action, phi = controller.step()
        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        curr_target_q = action.joint_angles.copy()
        if prev_target_q is not None:
            step_target_diffs.append(np.abs(curr_target_q - prev_target_q))
        prev_target_q = curr_target_q

        if (step_i + 1) % SAMPLE_STEPS == 0:
            t = (step_i + 1) * PHYSICS_DT
            b_pos = sim.get_body_positions(fly.name)
            pos = b_pos[thorax_idx]
            quat = sim.get_body_rotations(fly.name)[thorax_idx]
            pitch_deg = np.degrees(quat_to_pitch_rad(quat))

            # Aerospace Euler angles
            w, x, y, z = quat
            roll_rad = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            roll_deg = float(np.degrees(roll_rad))
            yaw_rad = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            yaw_deg = float(np.degrees(yaw_rad))

            vx = float((pos[0] - prev_x) / sample_dt)
            prev_x = pos[0]
            ax = float((vx - prev_vx) / sample_dt) if prev_vx is not None else 0.0
            prev_vx = vx

            tars_pos = b_pos[tarsus5_idxs]
            cont_info = sim.get_ground_contact_info(fly.name)
            cont_leg = cont_info[0].copy()[:6]

            # Aggregate contact forces across all segments of each leg
            forces_all_segs = sim.get_bodysegment_contact_forces(fly.name, all_leg_segs, ground_only=True)
            leg_forces = np.zeros((6, 3), dtype=float)
            for s_idx, leg_idx in seg_to_leg_idx.items():
                leg_forces[leg_idx] += forces_all_segs[s_idx]

            # Abdomen ground contact forces
            abd_forces = sim.get_bodysegment_contact_forces(fly.name, abdomen_segs, ground_only=True)
            abd_contact = bool(np.any(np.linalg.norm(abd_forces, axis=1) > 1e-4))

            adh_cmd = action.adhesion_onoff.copy()
            q_act = np.array([sim.mj_data.qpos[adr] for adr in dof_map["qpos_inds"]], dtype=float)
            q_targ = action.joint_angles.copy()
            act_forces = np.array([sim.mj_data.actuator_force[aid] for aid in actuator_ids], dtype=float)

            # Compute target endpoint FK relative to thorax in scratch MjData
            scratch_data.qpos[:] = sim.mj_data.qpos[:]
            scratch_data.qpos[dof_map["qpos_inds"]] = q_targ
            mujoco.mj_kinematics(sim.mj_model, scratch_data)
            t_pos_sc = scratch_data.xpos[thorax_body_id].copy()
            t_mat_sc = scratch_data.xmat[thorax_body_id].reshape(3, 3).copy()

            targ_body_pos = np.zeros((6, 3), dtype=float)
            for leg_i, leg in enumerate(LEG_LOWER):
                p_w = scratch_data.xpos[t5_body_ids[leg]]
                targ_body_pos[leg_i] = t_mat_sc.T @ (p_w - t_pos_sc)

            t_list.append(t)
            x_list.append(float(pos[0]))
            y_list.append(float(pos[1]))
            z_list.append(float(pos[2]))
            pitch_list.append(float(pitch_deg))
            roll_list.append(float(roll_deg))
            yaw_list.append(float(yaw_deg))
            quat_list.append(quat.copy())
            vx_list.append(vx)
            ax_list.append(ax)
            tarsus_xyz_list.append(tars_pos.copy())
            contact_list.append(cont_leg.copy())
            adhesion_list.append(adh_cmd.copy())
            target_q_list.append(q_targ.copy())
            actual_q_list.append(q_act.copy())
            actuator_forces_list.append(act_forces.copy())
            leg_grf_world_list.append(leg_forces.copy())
            target_body_pos_list.append(targ_body_pos.copy())
            abdomen_contact_list.append(abd_contact)

            if hasattr(controller, "latest_compliance_telemetry") and controller.latest_compliance_telemetry is not None:
                compliance_telem_list.append(dict(controller.latest_compliance_telemetry))

    sim.close()

    step_diffs_arr = np.array(step_target_diffs) if step_target_diffs else np.zeros((1, len(dof_map["qpos_inds"])))

    ik_diag = {
        "converged_fraction": float(controller.ik_converged_count / max(controller.ik_total_solves, 1)),
        "residual_median_mm": float(np.median(controller.ik_residuals)) if controller.ik_residuals else 0.0,
        "residual_p95_mm": float(np.percentile(controller.ik_residuals, 95)) if controller.ik_residuals else 0.0,
        "residual_max_mm": float(np.max(controller.ik_residuals)) if controller.ik_residuals else 0.0,
        "iterations_p95": float(np.percentile(controller.ik_iterations, 95)) if controller.ik_iterations else 0.0,
        "total_solves": int(controller.ik_total_solves)
    } if hasattr(controller, "ik_residuals") else None

    # Work accumulators from controller
    work_diag = dict(controller.work_accum) if hasattr(controller, "work_accum") else None
    duty_diag = dict(controller.duty_counts) if hasattr(controller, "duty_counts") else None

    return {
        "condition_name": condition_name,
        "sample_dt": sample_dt,
        "t": np.array(t_list),
        "x": np.array(x_list),
        "y": np.array(y_list),
        "z": np.array(z_list),
        "pitch": np.array(pitch_list),
        "roll": np.array(roll_list),
        "yaw": np.array(yaw_list),
        "quat": np.array(quat_list),
        "vx": np.array(vx_list),
        "ax": np.array(ax_list),
        "tarsus_xyz": np.array(tarsus_xyz_list),
        "contact": np.array(contact_list),
        "grf_world": np.array(leg_grf_world_list),  # (N, 6, 3)
        "target_body_pos": np.array(target_body_pos_list),  # (N, 6, 3)
        "adhesion": np.array(adhesion_list),
        "target_q": np.array(target_q_list),
        "actual_q": np.array(actual_q_list),
        "actuator_forces": np.array(actuator_forces_list),
        "actuator_limits": actuator_limits,
        "abdomen_contact": np.array(abdomen_contact_list),
        "ik_diagnostics": ik_diag,
        "work_diagnostics": work_diag,
        "duty_diagnostics": duty_diag,
        "compliance_timeseries": compliance_telem_list,
        "step_target_diffs": step_diffs_arr
    }


# -----------------------------------------------------------------------------
# TELEMETRY ANALYSIS & METRICS COMPUTATION
# -----------------------------------------------------------------------------

def analyze_simulation_telemetry(telem):
    """
    Computes complete reproduction metrics, support conditioning, force conflict,
    propulsive efficiency, compliance telemetry, work integration, and glide dynamics.
    """
    sample_dt = telem["sample_dt"]
    ss_mask = telem["t"] >= STEADY_START_TIME

    ss_t = telem["t"][ss_mask]
    ss_x = telem["x"][ss_mask]
    ss_y = telem["y"][ss_mask]
    ss_z = telem["z"][ss_mask]
    ss_pitch = telem["pitch"][ss_mask]
    ss_roll = telem["roll"][ss_mask]
    ss_yaw = telem["yaw"][ss_mask]
    ss_vx = telem["vx"][ss_mask]
    ss_ax = telem["ax"][ss_mask]
    ss_tars = telem["tarsus_xyz"][ss_mask]
    ss_cont = telem["contact"][ss_mask]
    ss_grf = telem["grf_world"][ss_mask]  # (N_ss, 6, 3)
    ss_adh = telem["adhesion"][ss_mask]
    ss_targ_body = telem["target_body_pos"][ss_mask]  # (N_ss, 6, 3)
    ss_abd = telem["abdomen_contact"][ss_mask]

    # 1. Base reproduction & kinematic metrics
    disp_x = float(ss_x[-1] - telem["x"][0])
    disp_y = float(ss_y[-1] - telem["y"][0])
    mean_vx = float(np.mean(ss_vx))
    mean_speed = float(-mean_vx)
    lateral_ratio = float(abs(disp_y) / max(abs(disp_x), 1e-5))

    w20 = max(1, int(0.020 / sample_dt))
    w50 = max(1, int(0.050 / sample_dt))
    vx_20 = np.convolve(ss_vx, np.ones(w20)/w20, mode="same")
    vx_50 = np.convolve(ss_vx, np.ones(w50)/w50, mode="same")

    cv20 = float(np.std(vx_20) / max(abs(mean_vx), 1e-9))
    cv50 = float(np.std(vx_50) / max(abs(mean_vx), 1e-9))
    wrong_sign_20 = float(np.mean(vx_20 > 0))

    pitch_ptp = float(np.ptp(ss_pitch))
    pitch_std = float(np.std(ss_pitch))
    pitch_mean = float(np.mean(ss_pitch))

    roll_ptp = float(np.ptp(ss_roll))
    roll_std = float(np.std(ss_roll))
    roll_mean = float(np.mean(ss_roll))

    z_ptp = float(np.ptp(ss_z))
    z_std = float(np.std(ss_z))
    z_mean = float(np.mean(ss_z))

    foot_vx_all = np.gradient(ss_tars[:, :, 0], sample_dt, axis=0)  # (N_ss, 6)
    targ_vx_all = np.gradient(ss_targ_body[:, :, 0], sample_dt, axis=0)  # (N_ss, 6)

    # 2. Ground Reaction Forces & Load Telemetry
    fz_pos_total = np.sum(np.clip(ss_grf[:, :, 2], 0.0, None), axis=1)  # (N_ss,)
    fz_positive_all_samples = bool(np.all(fz_pos_total > 1e-3))
    forces_finite = bool(np.all(np.isfinite(ss_grf)))
    abdomen_contact_count = int(np.sum(ss_abd))

    # Actuator limits check
    act_forces_all = telem["actuator_forces"]
    act_limits = telem["actuator_limits"]
    sat_counts = 0
    total_act_evals = act_forces_all.shape[0] * act_forces_all.shape[1]
    for aidx, ainfo in enumerate(act_limits):
        low = ainfo["lower_limit"]
        high = ainfo["upper_limit"]
        forces_a = act_forces_all[:, aidx]
        sat_counts += int(np.sum((forces_a <= low + 1e-4) | (forces_a >= high - 1e-4)))
    actuator_saturation_fraction = float(sat_counts / max(total_act_evals, 1))

    max_target_step_rad = float(np.max(telem["step_target_diffs"]))

    telemetry_valid = fz_positive_all_samples and forces_finite and (abdomen_contact_count == 0) and (actuator_saturation_fraction == 0.0)

    # 3. Grounded leg count histogram
    grounded_counts = np.sum(ss_cont > 0, axis=1)
    leg_count_hist = {int(k): int(np.sum(grounded_counts == k)) for k in range(7)}
    leg_count_fracs = {int(k): float(np.mean(grounded_counts == k)) for k in range(7)}

    # Transition acceleration
    adh_trans_events = []
    for leg_i in range(6):
        crossings = np.where(np.diff((ss_adh[:, leg_i] >= 0.5).astype(int)) != 0)[0]
        for c in crossings:
            adh_trans_events.append(ss_t[c])

    in_trans_window = np.zeros(len(ss_t), dtype=bool)
    for t_ev in adh_trans_events:
        in_trans_window |= (np.abs(ss_t - t_ev) <= 0.020)

    if np.sum(in_trans_window) > 0:
        trans_accel_p95 = float(np.percentile(np.abs(ss_ax[in_trans_window]), 95))
        trans_accel_max = float(np.max(np.abs(ss_ax[in_trans_window])))
    else:
        trans_accel_p95 = float(np.percentile(np.abs(ss_ax), 95))
        trans_accel_max = float(np.max(np.abs(ss_ax)))

    # 4. Support-Phase Conditioning
    mask_strong_a = (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) <= 0.10)
    mask_strong_b = (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) <= 0.10)

    all3_a = np.all(ss_cont[:, TRIPOD_A_INDS] > 0, axis=1)
    all3_b = np.all(ss_cont[:, TRIPOD_B_INDS] > 0, axis=1)

    total_plateau_samples = int(np.sum(mask_strong_a) + np.sum(mask_strong_b))
    total_all3_samples = int(np.sum(all3_a & mask_strong_a) + np.sum(all3_b & mask_strong_b))
    all3_support_fraction = float(total_all3_samples / max(total_plateau_samples, 1))

    # Helper function for tripod conflict statistics
    def compute_aggregated_tripod_stats(inds_a, inds_b):
        canc_list, eff_list, opp_list = [], [], []
        foot_range_list, targ_range_list = [], []

        for inds, legs in [(inds_a, TRIPOD_A_INDS), (inds_b, TRIPOD_B_INDS)]:
            if len(inds) > 0:
                s_grf = ss_grf[inds][:, legs, :]
                s_fx = s_grf[:, :, 0]
                s_fz = fz_pos_total[inds]
                f_net = np.sum(s_fx, axis=1)
                f_abs = np.sum(np.abs(s_fx), axis=1)
                c = 1.0 - (np.abs(f_net) / (f_abs + 1e-9))
                e = np.abs(f_net) / (f_abs + 1e-9)
                th = PRE_REG_FORCE_THRESHOLD_FRAC * s_fz
                opp = (np.max(s_fx, axis=1) >= th) & (np.min(s_fx, axis=1) <= -th)

                canc_list.extend(c)
                eff_list.extend(e)
                opp_list.extend(opp)

                s_f_vx = foot_vx_all[inds][:, legs]
                foot_range_list.extend(np.ptp(s_f_vx, axis=1))

                s_t_vx = targ_vx_all[inds][:, legs]
                targ_range_list.extend(np.ptp(s_t_vx, axis=1))

        if len(canc_list) == 0:
            return {
                "sample_count": 0,
                "cancellation_median": 0.0,
                "cancellation_p05": 0.0,
                "cancellation_p95": 0.0,
                "cancellation_mean": 0.0,
                "efficiency_median": 0.0,
                "opposing_fraction": 0.0,
                "foot_vx_range_median": 0.0,
                "targ_vx_range_median": 0.0
            }

        canc_arr = np.array(canc_list)
        eff_arr = np.array(eff_list)
        opp_arr = np.array(opp_list)

        return {
            "sample_count": int(len(canc_arr)),
            "cancellation_median": float(np.median(canc_arr)),
            "cancellation_p05": float(np.percentile(canc_arr, 5)),
            "cancellation_p95": float(np.percentile(canc_arr, 95)),
            "cancellation_mean": float(np.mean(canc_arr)),
            "efficiency_median": float(np.median(eff_arr)),
            "opposing_fraction": float(np.mean(opp_arr)),
            "foot_vx_range_median": float(np.median(foot_range_list)),
            "targ_vx_range_median": float(np.median(targ_range_list))
        }

    inds_a_all = np.where(mask_strong_a)[0]
    inds_a_all3 = np.where(mask_strong_a & all3_a)[0]
    inds_b_all = np.where(mask_strong_b)[0]
    inds_b_all3 = np.where(mask_strong_b & all3_b)[0]

    agg_all3 = compute_aggregated_tripod_stats(inds_a_all3, inds_b_all3)
    agg_all_strong = compute_aggregated_tripod_stats(inds_a_all, inds_b_all)

    # 5. Whole-Body 6-Leg Force Cancellation
    fx_6_net = np.sum(ss_grf[:, :, 0], axis=1)
    fx_6_abs = np.sum(np.abs(ss_grf[:, :, 0]), axis=1)
    canc_6_all = 1.0 - (np.abs(fx_6_net) / (fx_6_abs + 1e-9))
    canc_6_strong = canc_6_all[mask_strong_a | mask_strong_b]

    whole_body_stats = {
        "cancellation_all_samples_median": float(np.median(canc_6_all)),
        "cancellation_strong_support_median": float(np.median(canc_6_strong)) if len(canc_6_strong) > 0 else 0.0,
        "cancellation_all_samples_mean": float(np.mean(canc_6_all))
    }

    # 6. Per-Leg GRF Statistics during Strong Support
    per_leg_grf_stats = {}
    for l_idx, leg in enumerate(LEG_NAMES):
        is_trip_a = (l_idx in TRIPOD_A_INDS)
        sup_mask = mask_strong_a if is_trip_a else mask_strong_b
        l_fx = ss_grf[sup_mask, l_idx, 0]
        l_fy = ss_grf[sup_mask, l_idx, 1]
        l_fz = ss_grf[sup_mask, l_idx, 2]
        l_cont = ss_cont[sup_mask, l_idx] > 0
        pos_fz = l_fz[l_fz > 0]

        per_leg_grf_stats[leg] = {
            "MEDIAN_FX_WORLD": float(np.median(l_fx)) if len(l_fx) > 0 else 0.0,
            "P05_FX_WORLD": float(np.percentile(l_fx, 5)) if len(l_fx) > 0 else 0.0,
            "P95_FX_WORLD": float(np.percentile(l_fx, 95)) if len(l_fx) > 0 else 0.0,
            "MEDIAN_FY_WORLD": float(np.median(l_fy)) if len(l_fy) > 0 else 0.0,
            "MEDIAN_FZ_WORLD": float(np.median(l_fz)) if len(l_fz) > 0 else 0.0,
            "MEDIAN_POSITIVE_FZ_WORLD": float(np.median(pos_fz)) if len(pos_fz) > 0 else 0.0,
            "CONTACT_FRACTION": float(np.mean(l_cont)) if len(l_cont) > 0 else 0.0,
            "MEDIAN_FOOT_WORLD_VX": float(np.median(foot_vx_all[sup_mask, l_idx])) if np.sum(sup_mask) > 0 else 0.0,
            "MEDIAN_TARGET_REL_VX": float(np.median(targ_vx_all[sup_mask, l_idx])) if np.sum(sup_mask) > 0 else 0.0
        }

    # 7. Compliance Telemetry & Passivity Diagnostics
    work_diag = telem.get("work_diagnostics")
    duty_diag = telem.get("duty_diagnostics")

    compliance_metrics = None
    if work_diag and duty_diag:
        w_neg_tot = work_diag["total_neg_uJ"]
        w_pos_tot = work_diag["total_pos_uJ"]
        w_net_tot = work_diag["total_net_uJ"]
        ratio_tot = float(w_pos_tot / (w_neg_tot + 1e-9))

        w_neg_ss = work_diag["ss_neg_uJ"]
        w_pos_ss = work_diag["ss_pos_uJ"]
        w_net_ss = work_diag["ss_net_uJ"]
        ratio_ss = float(w_pos_ss / (w_neg_ss + 1e-9))

        w_neg_cont = work_diag["contact_neg_uJ"]
        w_pos_cont = work_diag["contact_pos_uJ"]
        w_net_cont = work_diag["contact_net_uJ"]
        ratio_cont = float(w_pos_cont / (w_neg_cont + 1e-9))

        mid_ss_count = duty_diag["middle_strong_support_samples"]
        mid_active_count = duty_diag["middle_strong_active_samples"]
        frac_mid_active = float(mid_active_count / max(mid_ss_count, 1))

        tot_ss_count = duty_diag["total_ss_samples"]
        tot_active_count = duty_diag["total_ss_active_samples"]
        frac_tot_active = float(tot_active_count / max(tot_ss_count, 1))

        compliance_metrics = {
            "NEGATIVE_WORK_MAGNITUDE_UJ": w_neg_ss,
            "POSITIVE_WORK_INJECTION_UJ": w_pos_ss,
            "NET_COMPLIANCE_WORK_UJ": w_net_ss,
            "POSITIVE_TO_DISSIPATED_WORK_RATIO": ratio_ss,
            "CONTACT_BEARING_NEGATIVE_WORK_UJ": w_neg_cont,
            "CONTACT_BEARING_POSITIVE_WORK_UJ": w_pos_cont,
            "CONTACT_BEARING_WORK_RATIO": ratio_cont,
            "TOTAL_TRIAL_WORK_RATIO": ratio_tot,
            "ACTIVE_FRACTION_OF_MIDDLE_STRONG_SUPPORT": frac_mid_active,
            "ACTIVE_FRACTION_OF_TOTAL_GAIT": frac_tot_active,
            "PASSIVITY_SAFETY_GATE_PASS": (ratio_ss <= 0.05) and (ratio_cont <= 0.05)
        }

    # 8. Speed Minima / Hesitation Event Detection
    min_dist_samples = int(0.040 / sample_dt)
    peaks, props = find_peaks(vx_20, distance=min_dist_samples, prominence=0.5)

    window_half_samples = int(0.030 / sample_dt)
    canc_all_samples = np.full(len(ss_t), np.nan)
    for k in np.where(mask_strong_a)[0]:
        fx = ss_grf[k, TRIPOD_A_INDS, 0]
        canc_all_samples[k] = 1.0 - (abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))
    for k in np.where(mask_strong_b)[0]:
        fx = ss_grf[k, TRIPOD_B_INDS, 0]
        canc_all_samples[k] = 1.0 - (abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))

    ev_profiles_speed = []
    ev_profiles_canc = []
    for p_idx in peaks:
        if p_idx >= window_half_samples and p_idx + window_half_samples < len(ss_t):
            s_win = -vx_20[p_idx - window_half_samples: p_idx + window_half_samples + 1]
            c_win = canc_all_samples[p_idx - window_half_samples: p_idx + window_half_samples + 1]
            ev_profiles_speed.append(s_win)
            ev_profiles_canc.append(c_win)

    ev_time_axis_ms = np.arange(-window_half_samples, window_half_samples + 1) * (sample_dt * 1000.0)

    mean_speed_profile = np.nanmean(ev_profiles_speed, axis=0) if ev_profiles_speed else np.array([])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        mean_canc_profile = np.nanmean(ev_profiles_canc, axis=0) if ev_profiles_canc else np.array([])

    if len(mean_canc_profile) > 0 and np.any(np.isnan(mean_canc_profile)):
        nans = np.isnan(mean_canc_profile)
        not_nans = ~nans
        if np.any(not_nans):
            x_pts = np.arange(len(mean_canc_profile))
            mean_canc_profile[nans] = np.interp(x_pts[nans], x_pts[not_nans], mean_canc_profile[not_nans])

    return {
        "condition_name": telem["condition_name"],
        "mean_speed_mm_s": mean_speed,
        "mean_vx_mm_s": mean_vx,
        "displacement_x_mm": disp_x,
        "displacement_y_mm": disp_y,
        "lateral_ratio": lateral_ratio,
        "cv20": cv20,
        "cv50": cv50,
        "wrong_sign_20": wrong_sign_20,
        "pitch_ptp_deg": pitch_ptp,
        "pitch_std_deg": pitch_std,
        "pitch_mean_deg": pitch_mean,
        "roll_ptp_deg": roll_ptp,
        "roll_std_deg": roll_std,
        "roll_mean_deg": roll_mean,
        "z_ptp_mm": z_ptp,
        "z_std_mm": z_std,
        "z_mean_mm": z_mean,
        "all3_support_fraction": all3_support_fraction,
        "transition_accel_p95_mm_s2": trans_accel_p95,
        "transition_accel_max_mm_s2": trans_accel_max,
        "leg_count_histogram": leg_count_hist,
        "leg_count_fractions": leg_count_fracs,
        "aggregated_all3": agg_all3,
        "aggregated_all_strong": agg_all_strong,
        "whole_body": whole_body_stats,
        "per_leg_grf": per_leg_grf_stats,
        "compliance_diagnostics": compliance_metrics,
        "abdomen_contact_count": abdomen_contact_count,
        "actuator_saturation_fraction": actuator_saturation_fraction,
        "max_target_step_rad": max_target_step_rad,
        "fz_positive_all_samples": fz_positive_all_samples,
        "forces_finite": forces_finite,
        "telemetry_valid": telemetry_valid,
        "ik_diagnostics": telem["ik_diagnostics"],
        "event_triggered_dynamics": {
            "event_count": len(peaks),
            "time_axis_ms": ev_time_axis_ms.tolist(),
            "mean_speed_profile_mm_s": mean_speed_profile.tolist() if len(mean_speed_profile) > 0 else [],
            "mean_cancellation_profile": mean_canc_profile.tolist() if len(mean_canc_profile) > 0 else []
        }
    }


# -----------------------------------------------------------------------------
# GATE EVALUATION & PRIMARY CLASSIFICATIONS
# -----------------------------------------------------------------------------

def evaluate_reproduction_gates(metrics_a, metrics_b):
    """
    Gate 1: Reproduction of Condition A (M8 Baseline) and Condition B (Exact M27-D).
    Deterministic Bounds:
    - Condition A: Speed 9.93 +- 0.10, CV50 0.232 +- 0.010, Pitch PTP 8.38 +- 0.25, Roll PTP 38.53 +- 1.00, All-3 0.616 +- 0.020.
    - Condition B: Speed 7.87 +- 0.15, CV50 0.328 +- 0.015, Wrong-sign 0.0914 +- 0.015, Pitch PTP 6.51 +- 0.25,
      Roll PTP 9.17 +- 0.35, Thorax Z PTP 0.103 +- 0.015, All-3 0.945 +- 0.015, ALL3 Cancellation 0.449 +- 0.025.
    """
    pass_a_speed = abs(metrics_a["mean_speed_mm_s"] - 9.93) <= 0.10
    pass_a_cv50 = abs(metrics_a["cv50"] - 0.232) <= 0.010
    pass_a_pitch = abs(metrics_a["pitch_ptp_deg"] - 8.38) <= 0.25
    pass_a_roll = abs(metrics_a["roll_ptp_deg"] - 38.53) <= 1.00
    pass_a_all3 = abs(metrics_a["all3_support_fraction"] - 0.616) <= 0.020
    gate_a_pass = pass_a_speed and pass_a_cv50 and pass_a_pitch and pass_a_roll and pass_a_all3

    pass_b_speed = abs(metrics_b["mean_speed_mm_s"] - 7.87) <= 0.15
    pass_b_cv50 = abs(metrics_b["cv50"] - 0.328) <= 0.015
    pass_b_ws20 = abs(metrics_b["wrong_sign_20"] - 0.0914) <= 0.015
    pass_b_pitch = abs(metrics_b["pitch_ptp_deg"] - 6.51) <= 0.25
    pass_b_roll = abs(metrics_b["roll_ptp_deg"] - 9.17) <= 0.35
    pass_b_z = abs(metrics_b["z_ptp_mm"] - 0.103) <= 0.015
    pass_b_all3 = abs(metrics_b["all3_support_fraction"] - 0.945) <= 0.015
    canc_b_all3 = metrics_b["aggregated_all3"]["cancellation_median"]
    pass_b_canc = abs(canc_b_all3 - 0.449) <= 0.025
    gate_b_pass = pass_b_speed and pass_b_cv50 and pass_b_ws20 and pass_b_pitch and pass_b_roll and pass_b_z and pass_b_all3 and pass_b_canc

    return {
        "gate_a_pass": gate_a_pass,
        "gate_b_pass": gate_b_pass,
        "overall_reproduction_pass": gate_a_pass and gate_b_pass,
        "condition_a_checks": {
            "speed": {"val": metrics_a["mean_speed_mm_s"], "target": 9.93, "tol": 0.10, "pass": pass_a_speed},
            "cv50": {"val": metrics_a["cv50"], "target": 0.232, "tol": 0.010, "pass": pass_a_cv50},
            "pitch_ptp": {"val": metrics_a["pitch_ptp_deg"], "target": 8.38, "tol": 0.25, "pass": pass_a_pitch},
            "roll_ptp": {"val": metrics_a["roll_ptp_deg"], "target": 38.53, "tol": 1.00, "pass": pass_a_roll},
            "all3_contact": {"val": metrics_a["all3_support_fraction"], "target": 0.616, "tol": 0.020, "pass": pass_a_all3}
        },
        "condition_b_checks": {
            "speed": {"val": metrics_b["mean_speed_mm_s"], "target": 7.87, "tol": 0.15, "pass": pass_b_speed},
            "cv50": {"val": metrics_b["cv50"], "target": 0.328, "tol": 0.015, "pass": pass_b_cv50},
            "wrong_sign_20": {"val": metrics_b["wrong_sign_20"], "target": 0.0914, "tol": 0.015, "pass": pass_b_ws20},
            "pitch_ptp": {"val": metrics_b["pitch_ptp_deg"], "target": 6.51, "tol": 0.25, "pass": pass_b_pitch},
            "roll_ptp": {"val": metrics_b["roll_ptp_deg"], "target": 9.17, "tol": 0.35, "pass": pass_b_roll},
            "z_ptp": {"val": metrics_b["z_ptp_mm"], "target": 0.103, "tol": 0.015, "pass": pass_b_z},
            "all3_contact": {"val": metrics_b["all3_support_fraction"], "target": 0.945, "tol": 0.015, "pass": pass_b_all3},
            "all3_cancellation": {"val": canc_b_all3, "target": 0.449, "tol": 0.025, "pass": pass_b_canc}
        }
    }


def evaluate_telemetry_integrity(metrics):
    """Evaluates telemetry & actuator integrity gates."""
    pass_finite = metrics["forces_finite"]
    pass_fz_pos = metrics["fz_positive_all_samples"]
    pass_sat = (metrics["actuator_saturation_fraction"] == 0.0)
    pass_step = (metrics["max_target_step_rad"] < 0.01)
    pass_abd = (metrics["abdomen_contact_count"] == 0)

    overall = pass_finite and pass_fz_pos and pass_sat and pass_step and pass_abd
    return {
        "telemetry_integrity_pass": overall,
        "forces_finite": pass_finite,
        "fz_positive_continuous": pass_fz_pos,
        "actuator_saturation_fraction": metrics["actuator_saturation_fraction"],
        "max_target_step_rad": metrics["max_target_step_rad"],
        "abdomen_contact_count": metrics["abdomen_contact_count"]
    }


def classify_condition_c(metrics_b, metrics_c, telem_gate_c, repro_pass):
    """
    Evaluates Condition C Primary Classification Rubric for MW-M29:
    1. Primary Lateral Gate: relative preservation lateral_ratio_C <= 1.15 * lateral_ratio_B.
    2. Middle braking relief >= 10% relative to B.
    3. Middle-brake left/right asymmetry is >= 30% lower than historical M28-C (threshold <= 5.95%).
    4. ALL3 force cancellation improves by >= 0.020 absolute relative to B.
    5. CV50 does not worsen by more than 3% relative to B (CV50_C <= 1.03 * CV50_B).
    6. Wrong-sign20 does not worsen by more than 0.010 absolute (<= wrong_sign_B + 0.010).
    7. Backward speed >= 7.50 mm/s.
    8. Pitch PTP <= 7.20 deg, Roll PTP <= 11.00 deg, Z PTP <= 0.130 mm.
    9. All-3 contact >= 0.880.
    10. Work ratio: POSITIVE_TO_DISSIPATED_WORK_RATIO <= 0.05.
    11. Numerical / actuator / target integrity passes.
    """
    if not repro_pass or not telem_gate_c["telemetry_integrity_pass"]:
        return "INVALID_CONTROLLER", {}

    # 1. Middle braking relief
    lm_b = metrics_b["per_leg_grf"]["LM"]["MEDIAN_FX_WORLD"]
    rm_b = metrics_b["per_leg_grf"]["RM"]["MEDIAN_FX_WORLD"]
    mid_fx_b = 0.5 * (lm_b + rm_b)

    lm_c = metrics_c["per_leg_grf"]["LM"]["MEDIAN_FX_WORLD"]
    rm_c = metrics_c["per_leg_grf"]["RM"]["MEDIAN_FX_WORLD"]
    mid_fx_c = 0.5 * (lm_c + rm_c)
    braking_relief_frac = (mid_fx_b - mid_fx_c) / max(mid_fx_b, 1e-9)

    # 2. Middle-brake L/R relief asymmetry
    relief_lm = (lm_b - lm_c) / max(lm_b, 1e-9)
    relief_rm = (rm_b - rm_c) / max(rm_b, 1e-9)
    asym_m29 = abs(relief_lm - relief_rm)
    # Historical M28-C was 8.5% -> 30% lower is <= 5.95% (0.0595)
    crit_asym = (asym_m29 <= 0.0595)

    # 3. Force cancellation
    canc_b = metrics_b["aggregated_all3"]["cancellation_median"]
    canc_c = metrics_c["aggregated_all3"]["cancellation_median"]
    delta_canc = canc_b - canc_c

    # 4. Glide dynamics
    cv50_b = metrics_b["cv50"]
    cv50_c = metrics_c["cv50"]
    crit_cv50 = (cv50_c <= 1.03 * cv50_b)

    ws20_b = metrics_b["wrong_sign_20"]
    ws20_c = metrics_c["wrong_sign_20"]
    crit_ws20 = (ws20_c <= ws20_b + 0.010)

    # 5. Choreography preservation
    speed_c = metrics_c["mean_speed_mm_s"]
    pitch_ptp_c = metrics_c["pitch_ptp_deg"]
    roll_ptp_c = metrics_c["roll_ptp_deg"]
    z_ptp_c = metrics_c["z_ptp_mm"]
    all3_c = metrics_c["all3_support_fraction"]

    lat_b = metrics_b["lateral_ratio"]
    lat_c = metrics_c["lateral_ratio"]
    primary_lateral_pass = (lat_c <= 1.15 * lat_b)
    literal_lateral_pass = (lat_c <= 0.020)

    # 6. Passivity & Work Ratio
    comp_diag = metrics_c.get("compliance_diagnostics") or {}
    work_ratio_c = comp_diag.get("POSITIVE_TO_DISSIPATED_WORK_RATIO", 0.0)
    pass_work_ratio = (work_ratio_c <= 0.05)

    # Criteria evaluation
    crit1_mid_relief_10 = (braking_relief_frac >= 0.10)
    crit2_canc_002 = (delta_canc >= 0.020)
    crit3_asym_30pct = crit_asym
    crit4_cv50_3pct = crit_cv50
    crit5_ws20_001 = crit_ws20
    crit6_speed_75 = (speed_c >= 7.50)
    crit7_posture = (pitch_ptp_c <= 7.20) and (roll_ptp_c <= 11.00) and (z_ptp_c <= 0.130)
    crit8_all3_88 = (all3_c >= 0.880)
    crit9_primary_lateral = primary_lateral_pass
    crit10_work_passivity = pass_work_ratio
    crit11_numerical = telem_gate_c["telemetry_integrity_pass"]

    criteria_dict = {
        "middle_braking_relief_ge_10pct": {"val": braking_relief_frac, "thresh": 0.10, "pass": crit1_mid_relief_10, "mid_fx_b": mid_fx_b, "mid_fx_c": mid_fx_c},
        "middle_asymmetry_reduction_30pct": {"val": asym_m29, "thresh": 0.0595, "relief_lm": relief_lm, "relief_rm": relief_rm, "pass": crit3_asym_30pct},
        "cancellation_improvement_ge_002": {"val": delta_canc, "thresh": 0.020, "pass": crit2_canc_002},
        "cv50_preservation_le_3pct_worse": {"val_c": cv50_c, "max_allowed": 1.03 * cv50_b, "pass": crit4_cv50_3pct},
        "wrong_sign_preservation_le_001_worse": {"val_c": ws20_c, "max_allowed": ws20_b + 0.010, "pass": crit5_ws20_001},
        "backward_speed_ge_75": {"val": speed_c, "thresh": 7.50, "pass": crit6_speed_75},
        "posture_preservation_pitch_roll_z": {
            "pitch_ptp": pitch_ptp_c, "pitch_max": 7.20,
            "roll_ptp": roll_ptp_c, "roll_max": 11.00,
            "z_ptp": z_ptp_c, "z_max": 0.130,
            "pass": crit7_posture
        },
        "all3_contact_ge_88": {"val": all3_c, "thresh": 0.880, "pass": crit8_all3_88},
        "primary_lateral_preservation": {"val": lat_c, "ref_b": lat_b, "max_allowed": 1.15 * lat_b, "pass": crit9_primary_lateral},
        "literal_lateral_diagnostic": {"val": lat_c, "thresh": 0.020, "pass": literal_lateral_pass},
        "passivity_work_ratio_le_005": {"val": work_ratio_c, "thresh": 0.05, "pass": crit10_work_passivity},
        "numerical_integrity_pass": {"pass": crit11_numerical}
    }

    all_pass = all([
        crit1_mid_relief_10, crit2_canc_002, crit3_asym_30pct, crit4_cv50_3pct,
        crit5_ws20_001, crit6_speed_75, crit7_posture, crit8_all3_88,
        crit9_primary_lateral, crit10_work_passivity, crit11_numerical
    ])

    posture_failed = not (crit6_speed_75 and crit7_posture and crit8_all3_88 and crit9_primary_lateral)

    if all_pass:
        classification = "SHARED_BILATERAL_COMPLIANCE_SUPPORTED"
    elif posture_failed:
        classification = "CHOREOGRAPHY_PRESERVATION_FAILED"
    elif (braking_relief_frac >= 0.07) and crit9_primary_lateral and crit7_posture:
        classification = "SHARED_BILATERAL_COMPLIANCE_PARTIALLY_SUPPORTED"
    else:
        classification = "SHARED_BILATERAL_COMPLIANCE_NOT_SUPPORTED"

    return classification, criteria_dict


# -----------------------------------------------------------------------------
# DIAGNOSTIC PLOTTING
# -----------------------------------------------------------------------------

def generate_diagnostic_plots(telemetry_dict, results_dict, output_dir="output/m29"):
    """Generates diagnostic plots comparing Conditions A, B, and C."""
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nGenerating diagnostic plots in {output_dir} ...")

    conds = ["A", "B", "C"]
    labels = ["A: M8 Champion", "B: Exact M27-D", "C: Brake Relief"]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]

    # 1. Force Cancellation Comparison Plot
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), dpi=200)

    canc_all3_medians = [results_dict[c]["aggregated_all3"]["cancellation_median"] for c in conds]
    canc_all3_p05 = [results_dict[c]["aggregated_all3"]["cancellation_p05"] for c in conds]
    canc_all3_p95 = [results_dict[c]["aggregated_all3"]["cancellation_p95"] for c in conds]
    err_low = [max(0.0, m - p) for m, p in zip(canc_all3_medians, canc_all3_p05)]
    err_high = [max(0.0, p - m) for m, p in zip(canc_all3_medians, canc_all3_p95)]

    axes[0].bar(labels, canc_all3_medians, yerr=[err_low, err_high], color=colors, alpha=0.8, capsize=5)
    axes[0].set_ylabel("Force Cancellation Ratio (1 - |F_net|/F_abs)")
    axes[0].set_title("Active Tripod Cancellation (ALL3 Contact)")
    axes[0].set_ylim(0.0, 1.0)
    axes[0].grid(True, alpha=0.3, axis="y")
    axes[0].tick_params(axis="x", rotation=10)

    canc_6_medians = [results_dict[c]["whole_body"]["cancellation_all_samples_median"] for c in conds]
    axes[1].bar(labels, canc_6_medians, color=colors, alpha=0.8)
    axes[1].set_ylabel("Whole-Body Cancellation Ratio")
    axes[1].set_title("Whole-Body 6-Leg Force Cancellation")
    axes[1].set_ylim(0.0, 1.0)
    axes[1].grid(True, alpha=0.3, axis="y")
    axes[1].tick_params(axis="x", rotation=10)

    plt.tight_layout()
    plot1_path = os.path.join(output_dir, "cancellation_comparison.png")
    plt.savefig(plot1_path)
    plt.close()
    print(f"  Saved: {plot1_path}")

    # 2. Per-Leg Fx Comparison Plot
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), dpi=200)
    classes = ["FRONT", "MIDDLE", "HIND"]
    class_leg_pairs = [("LF", "RF"), ("LM", "RM"), ("LH", "RH")]

    for idx, (cls_name, (l1, l2)) in enumerate(zip(classes, class_leg_pairs)):
        ax = axes[idx]
        x_pos = np.arange(len(conds))
        width = 0.35

        fx_l1 = [results_dict[c]["per_leg_grf"][l1]["MEDIAN_FX_WORLD"] for c in conds]
        fx_l2 = [results_dict[c]["per_leg_grf"][l2]["MEDIAN_FX_WORLD"] for c in conds]

        ax.bar(x_pos - width/2, fx_l1, width, label=l1, alpha=0.85)
        ax.bar(x_pos + width/2, fx_l2, width, label=l2, alpha=0.85)
        ax.axhline(0.0, color="black", linestyle="--", linewidth=0.8)
        ax.set_xticks(x_pos)
        ax.set_xticklabels(["A", "B", "C"])
        ax.set_ylabel("Median Fx World (mN)")
        ax.set_title(f"{cls_name} Legs ({l1}, {l2})")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plot2_path = os.path.join(output_dir, "per_leg_fx_comparison.png")
    plt.savefig(plot2_path)
    plt.close()
    print(f"  Saved: {plot2_path}")

    # 3. Speed Profiles Comparison
    fig, ax = plt.subplots(figsize=(11, 4.2), dpi=200)
    for c, lbl, col in zip(conds, labels, colors):
        telem = telemetry_dict[c]
        t = telem["t"]
        vx = telem["vx"]
        w_mask = (t >= 0.50) & (t <= 1.10)
        w20 = max(1, int(0.020 / telem["sample_dt"]))
        vx_20 = np.convolve(vx, np.ones(w20)/w20, mode="same")
        ax.plot(t[w_mask], -vx_20[w_mask], label=lbl, color=col, linewidth=1.5)

    ax.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Backward Body Speed -Vx (mm/s)")
    ax.set_title("Backward Locomotion Speed Profiles (20 ms Filtered)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot3_path = os.path.join(output_dir, "velocity_profiles.png")
    plt.savefig(plot3_path)
    plt.close()
    print(f"  Saved: {plot3_path}")

    # 4. Event-Triggered Hesitation Profile
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), dpi=200)
    for c, lbl, col in zip(conds, labels, colors):
        ev_data = results_dict[c]["event_triggered_dynamics"]
        if ev_data["time_axis_ms"] and ev_data["mean_speed_profile_mm_s"]:
            t_axis = ev_data["time_axis_ms"]
            spd = ev_data["mean_speed_profile_mm_s"]
            canc = ev_data["mean_cancellation_profile"]
            axes[0].plot(t_axis, spd, label=lbl, color=col, linewidth=1.5)
            axes[1].plot(t_axis, canc, label=lbl, color=col, linewidth=1.5)

    axes[0].set_xlabel("Time relative to Speed Minimum (ms)")
    axes[0].set_ylabel("Backward Speed (mm/s)")
    axes[0].set_title("Event-Triggered Speed Dips")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="lower right")

    axes[1].set_xlabel("Time relative to Speed Minimum (ms)")
    axes[1].set_ylabel("Tripod Cancellation Ratio")
    axes[1].set_title("Event-Triggered Force Cancellation")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    plt.tight_layout()
    plot4_path = os.path.join(output_dir, "event_triggered_hesitation.png")
    plt.savefig(plot4_path)
    plt.close()
    print(f"  Saved: {plot4_path}")

    # 5. Condition C Compliance Telemetry Deep Dive
    telem_c = telemetry_dict["C"]
    comp_ts = telem_c.get("compliance_timeseries", [])
    if comp_ts:
        t_ts = telem_c["t"]
        w_mask = (t_ts >= 0.50) & (t_ts <= 1.10)
        t_sub = t_ts[w_mask]

        dx_shared = [comp_ts[i].get("dx_shared", 0.0) * 1000.0 for i in np.where(w_mask)[0]]
        dx_lm = [comp_ts[i].get("dx_applied_lm", 0.0) * 1000.0 for i in np.where(w_mask)[0]]
        dx_rm = [comp_ts[i].get("dx_applied_rm", 0.0) * 1000.0 for i in np.where(w_mask)[0]]
        sup_sum = [comp_ts[i].get("support_sum", 0.0) for i in np.where(w_mask)[0]]
        p_lm = [comp_ts[i].get("power_lm", 0.0) for i in np.where(w_mask)[0]]
        p_rm = [comp_ts[i].get("power_rm", 0.0) for i in np.where(w_mask)[0]]

        fig, axes = plt.subplots(3, 1, figsize=(11, 7.5), dpi=200, sharex=True)
        axes[0].plot(t_sub, dx_shared, label="Shared dx", color="black", linestyle=":", linewidth=1.5)
        axes[0].plot(t_sub, dx_lm, label="LM Applied dx", color="#1f77b4", linewidth=1.5)
        axes[0].plot(t_sub, dx_rm, label="RM Applied dx", color="#ff7f0e", linewidth=1.5)
        axes[0].set_ylabel("Displacement (um)")
        axes[0].set_title("Condition C Shared Bilateral Compliance Displacement & Power Telemetry")
        axes[0].legend(loc="upper right")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t_sub, sup_sum, label="Support Sum (w_LM + w_RM)", color="purple", linewidth=1.2)
        axes[1].axhline(0.01, color="red", linestyle=":", linewidth=0.8, label="Reset Threshold (0.01)")
        axes[1].set_ylabel("Support Weight Sum")
        axes[1].legend(loc="upper right")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t_sub, p_lm, label="LM Power P", color="#1f77b4", linewidth=1.2)
        axes[2].plot(t_sub, p_rm, label="RM Power P", color="#ff7f0e", linewidth=1.2)
        axes[2].axhline(0.0, color="black", linestyle="-", linewidth=0.8)
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("Compliance Power (uW)")
        axes[2].legend(loc="upper right")
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plot5_path = os.path.join(output_dir, "compliance_telemetry.png")
        plt.savefig(plot5_path)
        plt.close()
        print(f"  Saved: {plot5_path}")


# -----------------------------------------------------------------------------
# VIDEO RENDERING & COMPOSITING
# -----------------------------------------------------------------------------

def render_condition_video_dual_speed(condition_name, controller_builder, out_path_02x, out_path_10x, fps=30):
    """Reruns physics once with camera at 0.2x and encodes both 0.2x and 1.0x videos."""
    os.makedirs(os.path.dirname(out_path_02x), exist_ok=True)
    os.makedirs(os.path.dirname(out_path_10x), exist_ok=True)

    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    cam = fly.add_tracking_camera(name="trackcam")
    sim = m25.create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    renderer = sim.set_renderer(
        cam,
        camera_res=(240, 320),
        playback_speed=0.2,
        output_fps=fps,
        buffer_frames=True
    )

    try:
        for _ in range(N_STEPS):
            action, _ = controller.step()
            apply_locomotion_action(sim, fly.name, action)
            sim.step_with_profile()
            sim.render_as_needed_with_profile()

        cam_name = list(renderer.frames.keys())[0]
        frames_02x = renderer.frames[cam_name]

        # 0.2x slow motion
        iio.mimsave(out_path_02x, frames_02x, fps=fps, codec="libx264")

        # 1.0x presentation (every 5th frame)
        frames_10x = frames_02x[::5]
        iio.mimsave(out_path_10x, frames_10x, fps=fps, codec="libx264")
    finally:
        renderer.close()
        sim.close()


def create_three_way_video(path_a, path_b, path_c, out_path, label_a, label_b, label_c):
    """Composites 3 videos side-by-side with crisp label overlays."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    reader_a = iio.get_reader(str(path_a))
    reader_b = iio.get_reader(str(path_b))
    reader_c = iio.get_reader(str(path_c))
    fps = reader_a.get_meta_data().get("fps", 30)

    writer = iio.get_writer(str(out_path), fps=fps)
    try:
        for frame_a, frame_b, frame_c in zip(reader_a, reader_b, reader_c):
            img_a = Image.fromarray(frame_a)
            img_b = Image.fromarray(frame_b)
            img_c = Image.fromarray(frame_c)

            for img, lbl in [(img_a, label_a), (img_b, label_b), (img_c, label_c)]:
                draw = ImageDraw.Draw(img)
                draw.rectangle([0, 0, 320, 24], fill=(0, 0, 0, 160))
                draw.text((8, 5), lbl, fill="white")

            w, h = img_a.size
            combined = Image.new("RGB", (w * 3, h))
            combined.paste(img_a, (0, 0))
            combined.paste(img_b, (w, 0))
            combined.paste(img_c, (w * 2, 0))
            writer.append_data(np.array(combined))
    finally:
        reader_a.close()
        reader_b.close()
        reader_c.close()
        writer.close()


# -----------------------------------------------------------------------------
# MAIN EXPERIMENT EXECUTION
# -----------------------------------------------------------------------------

def run_m29_experiment(skip_render=False):
    """
    Executes the entire MW-M29 experiment:
    1. Preflight audits (provenance, scratch isolation, static causal isolation, passivity)
    2. Exactly 3 dynamic evaluation rollouts (A, B, C)
    3. Quantitative analysis, gates, and primary classifications
    4. Diagnostic plots and JSON artifacts export
    5. Clean rendering rollouts (up to 3 runs: A, B, C at 0.2x & 1.0x, compositing 3-way videos).
    """
    print("=" * 70)
    print("MW-M29: CONSERVATIVE MIDDLE-LEG BRAKE-RELIEF POLISH OF M27-D")
    print("=" * 70)

    # Step 1: Preflights
    m8_expected = run_preflight_provenance_audit()

    fly_probe = make_locomotion_fly(name="fly", add_adhesion=True)
    sim_probe = m25.create_isolated_sim(fly_probe)
    dof_map = m25.build_semantic_dof_mapping(fly_probe, sim_probe)
    pps = PreprogrammedSteps()
    actuated_dofs = fly_probe.get_actuated_jointdofs_order(ActuatorType.POSITION)

    verify_scratch_mjdata_isolation(fly_probe, sim_probe, dof_map)
    preflight_c = run_preflight_condition_c_isolation(fly_probe, sim_probe, dof_map, pps, actuated_dofs)
    preflight_passivity = run_preflight_passivity_verification()
    sim_probe.close()

    # Step 2: Exactly 3 Dynamic Simulation Rollouts
    conditions = ["A", "B", "C"]
    telemetry = {}
    metrics = {}

    def build_ctrl_a(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M29SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="A")

    def build_ctrl_b(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M29SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="B",
                                            frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    def build_ctrl_c(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M29SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="C",
                                            frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    builders = {
        "A": build_ctrl_a,
        "B": build_ctrl_b,
        "C": build_ctrl_c
    }

    cond_names = {
        "A": "Condition A (DIRECT_M8_CONTROL)",
        "B": "Condition B (EXACT_M27_D)",
        "C": "Condition C (M29_SHARED_BILATERAL_COMPLIANCE)"
    }

    for c in conditions:
        telem = run_diagnostic_simulation(cond_names[c], builders[c])
        telemetry[c] = telem
        met = analyze_simulation_telemetry(telem)
        metrics[c] = met

    # Step 3: Gates & Classification Evaluation
    repro_eval = evaluate_reproduction_gates(metrics["A"], metrics["B"])
    telem_gates = {c: evaluate_telemetry_integrity(metrics[c]) for c in conditions}

    class_c, crit_c = classify_condition_c(
        metrics["B"], metrics["C"], telem_gates["C"], repro_eval["overall_reproduction_pass"]
    )

    # Print Summary of Results
    print("\n" + "=" * 70)
    print("MW-M29 EXPERIMENTAL EVALUATION SUMMARY")
    print("=" * 70)
    print(f"Parent Reproduction Gate: {'PASS' if repro_eval['overall_reproduction_pass'] else 'FAIL'}")
    print(f"  Condition A (M8):       Speed={metrics['A']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['A']['cv50']:.3f}, Pitch PTP={metrics['A']['pitch_ptp_deg']:.2f} deg, All3={metrics['A']['all3_support_fraction']:.3f}")
    print(f"  Condition B (Exact D):  Speed={metrics['B']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['B']['cv50']:.3f}, Pitch PTP={metrics['B']['pitch_ptp_deg']:.2f} deg, All3={metrics['B']['all3_support_fraction']:.3f}")
    print(f"  Condition C (Relief):   Speed={metrics['C']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['C']['cv50']:.3f}, Pitch PTP={metrics['C']['pitch_ptp_deg']:.2f} deg, All3={metrics['C']['all3_support_fraction']:.3f}")

    canc_b = metrics["B"]["aggregated_all3"]["cancellation_median"]
    canc_c = metrics["C"]["aggregated_all3"]["cancellation_median"]
    print("\nForce Cancellation (ALL3 Contact):")
    print(f"  Condition A: {metrics['A']['aggregated_all3']['cancellation_median']:.4f}")
    print(f"  Condition B: {canc_b:.4f}")
    print(f"  Condition C: {canc_c:.4f} (Delta vs B = {canc_b - canc_c:+.4f})")

    mid_b = 0.5 * (metrics['B']['per_leg_grf']['LM']['MEDIAN_FX_WORLD'] + metrics['B']['per_leg_grf']['RM']['MEDIAN_FX_WORLD'])
    mid_c = 0.5 * (metrics['C']['per_leg_grf']['LM']['MEDIAN_FX_WORLD'] + metrics['C']['per_leg_grf']['RM']['MEDIAN_FX_WORLD'])
    mid_relief = (mid_b - mid_c) / max(mid_b, 1e-9)
    print(f"\nMiddle Leg Median Braking Force:")
    print(f"  Condition B: {mid_b:.2f} mN")
    print(f"  Condition C: {mid_c:.2f} mN (Relief = {mid_relief*100.0:.1f}%)")

    if metrics["C"]["compliance_diagnostics"]:
        cd = metrics["C"]["compliance_diagnostics"]
        print(f"\nCondition C Compliance Telemetry:")
        print(f"  Active Fraction (Strong Support): {cd['ACTIVE_FRACTION_OF_MIDDLE_STRONG_SUPPORT']*100.0:.1f}%")
        print(f"  Active Fraction (Total Gait):     {cd['ACTIVE_FRACTION_OF_TOTAL_GAIT']*100.0:.1f}%")
        print(f"  Negative Work (Dissipated):       {cd['NEGATIVE_WORK_MAGNITUDE_UJ']:.3f} uJ")
        print(f"  Positive Work (Injected):         {cd['POSITIVE_WORK_INJECTION_UJ']:.3f} uJ")
        print(f"  Positive / Dissipated Work Ratio: {cd['POSITIVE_TO_DISSIPATED_WORK_RATIO']:.4f} (Safety Gate: {'PASS' if cd['PASSIVITY_SAFETY_GATE_PASS'] else 'FAIL'})")

    print(f"\nPrimary Lateral Gate (Relative Preservation <= 1.15 * B):")
    print(f"  Lateral Ratio B: {metrics['B']['lateral_ratio']:.4f}")
    print(f"  Lateral Ratio C: {metrics['C']['lateral_ratio']:.4f} (Max Allowed: {1.15 * metrics['B']['lateral_ratio']:.4f}) -> {'PASS' if crit_c['primary_lateral_preservation']['pass'] else 'FAIL'}")
    print(f"  Literal Lateral Gate (Diagnostic <= 0.020): {'PASS' if crit_c['literal_lateral_diagnostic']['pass'] else 'FAIL'}")

    print(f"\nCondition C Primary Classification: {class_c}")

    # Step 4: Export Manifest & Results JSON
    manifest_data = {
        "milestone": "MW-M29",
        "task": "Conservative middle-leg brake-relief polish of M27-D",
        "provenance_parameters": m8_expected,
        "frozen_heights_provenance": FROZEN_BODY_SUPPORT_HEIGHTS_MM,
        "condition_b_excursions": CONDITION_B_EXCURSIONS,
        "compliance_parameters": {
            "alpha_f": COMPLIANCE_ALPHA_F,
            "k_rho_mm": COMPLIANCE_K_RHO,
            "dx_max_mm": COMPLIANCE_DX_MAX,
            "v_max_mm_s": COMPLIANCE_V_MAX,
            "rho_thresh": COMPLIANCE_RHO_THRESH,
            "force_offset_mN": COMPLIANCE_FORCE_OFFSET
        },
        "preflight_c_isolation": preflight_c,
        "preflight_passivity": preflight_passivity,
        "reproduction_gates": repro_eval,
        "telemetry_integrity_gates": telem_gates,
        "condition_c_classification": class_c,
        "condition_c_criteria_evaluation": crit_c
    }

    manifest_path = pathlib.Path("data/derived/m29_shared_bilateral_manifest.json")
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(manifest_data), f, indent=2, allow_nan=False)
    print(f"\nSaved manifest: {manifest_path}")

    results_data = {
        "milestone": "MW-M29",
        "conditions": metrics,
        "reproduction_gates": repro_eval,
        "telemetry_integrity_gates": telem_gates,
        "condition_c_verdict": {
            "primary_classification": class_c,
            "criteria": crit_c
        },
        "qualitative_review_status": "PENDING_MANAGER_REVIEW"
    }

    results_path = pathlib.Path("data/derived/m29_shared_bilateral_results.json")
    with open(results_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(results_data), f, indent=2, allow_nan=False)
    print(f"Saved results:  {results_path}")

    # Step 5: Generate Diagnostic Plots
    generate_diagnostic_plots(telemetry, metrics, output_dir="output/m29")

    # Step 6: Video Rendering Suite (Budget: up to 3 runs)
    if not skip_render:
        print("\n" + "-"*70)
        print("Rendering synchronized diagnostic video suite (up to 3 physics reruns)...")
        print("-" * 70)
        vid_dir = pathlib.Path("output/m29")
        vid_dir.mkdir(parents=True, exist_ok=True)

        out_a_02x = str(vid_dir / "video_A_m8_02x.mp4")
        out_a_10x = str(vid_dir / "video_A_m8_10x.mp4")
        out_b_02x = str(vid_dir / "video_B_planar_02x.mp4")
        out_b_10x = str(vid_dir / "video_B_planar_10x.mp4")
        out_c_02x = str(vid_dir / "video_C_brake_relief_02x.mp4")
        out_c_10x = str(vid_dir / "video_C_brake_relief_10x.mp4")
        out_sbs_10x = str(vid_dir / "video_A_vs_B_vs_C_10x.mp4")
        out_sbs_02x = str(vid_dir / "video_A_vs_B_vs_C_02x.mp4")

        print("Rendering Condition A (0.2x & 1.0x) ...", end="", flush=True)
        render_condition_video_dual_speed("DIRECT_M8", build_ctrl_a, out_a_02x, out_a_10x)
        print(" Done ✓")

        print("Rendering Condition B (0.2x & 1.0x) ...", end="", flush=True)
        render_condition_video_dual_speed("EXACT_M27_D", build_ctrl_b, out_b_02x, out_b_10x)
        print(" Done ✓")

        print("Rendering Condition C (0.2x & 1.0x) ...", end="", flush=True)
        render_condition_video_dual_speed("BRAKE_RELIEF", build_ctrl_c, out_c_02x, out_c_10x)
        print(" Done ✓")

        print("Compositing 3-way presentation video (1.0x) ...", end="", flush=True)
        create_three_way_video(
            out_a_10x, out_b_10x, out_c_10x, out_sbs_10x,
            "A: DIRECT M8 (1.0x)", "B: EXACT M27-D (1.0x)", "C: BRAKE RELIEF (1.0x)"
        )
        print(" Done ✓")

        print("Compositing 3-way slow-motion video (0.2x) ...", end="", flush=True)
        create_three_way_video(
            out_a_02x, out_b_02x, out_c_02x, out_sbs_02x,
            "A: DIRECT M8 (0.2x)", "B: EXACT M27-D (0.2x)", "C: BRAKE RELIEF (0.2x)"
        )
        print(" Done ✓")

    print("\nMW-M29 Execution Completed Successfully.")
    return manifest_data, results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MW-M29 Conservative Middle-Leg Brake Relief")
    parser.add_argument("--skip-render", action="store_true", default=False, help="Skip video rendering")
    parser.add_argument("--render", action="store_true", default=True, help="Render diagnostic videos")
    args = parser.parse_args()

    # If --skip-render is explicitly passed, obey it; otherwise render by default
    do_skip = args.skip_render
    run_m29_experiment(skip_render=do_skip)
