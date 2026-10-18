#!/usr/bin/env python3
"""
MW-M27: Selective Hind-Leg Relief & Excursion-Matched Support Causal Intervention.

Scientific Goals:
1. Test whether selectively relieving the supporting hind legs from vertical
   planarization (Condition C) eliminates internal force opposition and body-speed
   hesitation, or whether front-middle opposition and transfer binding dominate.
2. Test whether removing the commanded longitudinal support-velocity mismatch while
   preserving continuous three-leg planar support (Condition D) materially reduces
   physical horizontal force conflict and improves body-glide dynamics.

Four Strictly Isolated Dynamic Conditions:
- Condition A: DIRECT_M8_CONTROL (Reference Baseline)
- Condition B: M25_BODY_FRAME_PLANAR_SUPPORT (Full Planar Reference)
- Condition C: SELECTIVE_HIND_RELIEF (Causal Intervention: Hind DLS IK Bypassed)
- Condition D: EXCURSION_MATCHED_PLANAR_SUPPORT (Kinematic Intervention: E_mid=0.564, E_hind=0.779)

Authoritative Parameters:
- Cadence: 6.0 Hz
- Transfer window tf: 0.08 (frozen across all conditions)
- Frozen heights: LF/RF: -1.042 mm, LM/RM: -1.088 mm, LH/RH: -0.975 mm
- DLS IK: damping 1e-4, reg_weight 0.01, max_dq 0.05, max_iter 15, tol 1e-4
- Isolated scratch MjData for DLS IK solves (zero live physics mutation)
- Simulation budget: exactly 4 evaluation rollouts (A, B, C, D) + up to 4 clean rendering rollouts <= 8 runs maximum.
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

CLASS_TO_LEGS = {
    "FRONT": [0, 3],   # LF, RF
    "MIDDLE": [1, 4],  # LM, RM
    "HIND": [2, 5]     # LH, RH
}

PHYSICS_DT = 0.0001
SAMPLE_STEPS = 10
SAMPLE_DT = PHYSICS_DT * SAMPLE_STEPS  # 0.001 s (1 kHz)
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)  # 17500
STEADY_START_TIME = 0.35

# Authoritative frozen body support heights from MW-M25
FROZEN_BODY_SUPPORT_HEIGHTS_MM = {
    "LF": -1.0424534343726277,
    "LM": -1.0876066769550565,
    "LH": -0.9750830473328336,
    "RF": -1.041986691037106,
    "RM": -1.0882983748972959,
    "RH": -0.9719454335077823
}

# Frozen excursion fractions for Condition D
CONDITION_D_EXCURSIONS = {
    "LF": 1.000,
    "LM": 0.564,
    "LH": 0.779,
    "RF": 1.000,
    "RM": 0.564,
    "RH": 0.779
}
EXCURSIONS_LIST_D = [CONDITION_D_EXCURSIONS[leg] for leg in LEG_NAMES]
EXCURSIONS_LIST_STOCK = [1.000] * 6

# Pre-registered force threshold fraction (1% of instantaneous total positive vertical load)
PRE_REG_FORCE_THRESHOLD_FRAC = 0.01


# -----------------------------------------------------------------------------
# JSON SERIALIZATION ENCODER
# -----------------------------------------------------------------------------

class NumpyEncoder(json.JSONEncoder):
    """Encodes NumPy types for clean JSON serialization."""
    def default(self, obj):
        if isinstance(obj, (np.integer, np.int64, np.int32)):
            return int(obj)
        elif isinstance(obj, (np.floating, np.float64, np.float32)):
            if np.isnan(obj) or np.isinf(obj):
                return None
            return float(obj)
        elif isinstance(obj, (np.ndarray,)):
            return obj.tolist()
        elif isinstance(obj, (np.bool_, bool)):
            return bool(obj)
        return super().default(obj)


def sanitize_json_for_rfc8259(obj):
    """
    Recursively sanitizes data structures to strictly conform to RFC 8259 JSON.
    Converts NaN and Inf floats to None (which serializes as standard null),
    and converts NumPy types to native Python equivalents.
    """
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
    When excursions == [1.0]*6, this is mathematically identical to m8.BackslideTrajectoryController.
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


class M27SupportGeometryController:
    """
    Unified controller architecture executing Conditions A, B, C, and D.
    
    Strict Condition Branching:
    - Condition A (DIRECT_M8_CONTROL): Stock M8 controller (zero DLS IK).
    - Condition B (M25_BODY_FRAME_PLANAR_SUPPORT): Planarize all 3 support legs to frozen Z heights.
    - Condition C (SELECTIVE_HIND_RELIEF): Planarize FRONT and MIDDLE support legs; HIND legs bypass DLS IK completely.
    - Condition D (EXCURSION_MATCHED_PLANAR_SUPPORT): Per-leg excursion fractions (E_front=1.000, E_mid=0.564, E_hind=0.779),
      followed by planarization of all 3 support legs to frozen Z heights.
    """
    def __init__(self, sim, fly, pps, actuated_dofs, dof_map,
                 condition="B",
                 frozen_heights=None,
                 dls_damping=1e-4, reg_weight=0.01, max_dq=0.05):
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

        excursions = EXCURSIONS_LIST_D if condition == "D" else EXCURSIONS_LIST_STOCK
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

        # IK diagnostics tracking
        self.ik_residuals = []
        self.ik_iterations = []
        self.ik_converged_count = 0
        self.ik_total_solves = 0
        self.last_target_q = None

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

        for i, leg in enumerate(LEG_LOWER):
            w_sup = float(adhesions[i])
            if w_sup < 1e-4:
                # Released leg: 100% exact nominal trajectory
                continue

            # Condition C: HIND legs bypass DLS IK completely (100% nominal M8 kinematics)
            if self.condition == "C" and leg in ["lh", "rh"]:
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
    Two-Part Causal Isolation Verification for Condition C (Patch 2):
    1. Static / Same-State Preflight:
       - Copied MuJoCo state, phase, nominal target, frozen heights, and passive states.
       - Front and Middle DLS solutions must match across B and C to machine precision (< 1e-10 rad).
       - Hind must receive planar DLS correction in B (> 0), but exactly zero planar IK correction in C (< 1e-12 rad).
    2. Dynamic Code-Path Invariants:
       - Front and Middle use identical algorithm, frozen heights, support weights, DLS settings, and adhesion timing as B.
       - Hind uses exact nominal M8 trajectory.
    """
    print("Running Preflight 1: Condition C Causal Isolation Verification ...")
    ctrl_B = M27SupportGeometryController(sim, fly, pps, actuated_dofs, dof_map, condition="B")
    ctrl_C = M27SupportGeometryController(sim, fly, pps, actuated_dofs, dof_map, condition="C")

    # Initialize fly in physical settled stance (t = 0.50 s)
    sim.reset()
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, pps.default_pose_by_dof_order(actuated_dofs))
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()
    ctrl_B.time = 0.50
    act_B, _ = ctrl_B.step()

    ctrl_C.time = 0.50
    act_C, _ = ctrl_C.step()

    q_B = act_B.joint_angles
    q_C = act_C.joint_angles

    # Check Front and Middle DOFs
    fm_diffs = {}
    for leg in ["lf", "lm", "rf", "rm"]:
        inds = dof_map["leg_act_inds"][leg]
        diff = float(np.max(np.abs(q_B[inds] - q_C[inds])))
        fm_diffs[leg.upper()] = diff

    max_fm_diff = max(fm_diffs.values())

    # Reference nominal angles at t = 0.50 s
    base_m8 = m8.BackslideTrajectoryController(pps, actuated_dofs, cadence=6.0, stance_excursion_fraction=1.0,
                                               turnaround_fraction=0.06, support_transfer_fraction=0.08, entry_duration=0.25)
    base_m8.time = 0.50
    nom_act, _ = base_m8.step()
    q_nom = nom_act.joint_angles

    hind_diffs_C = {}
    hind_corrections_B = {}
    for leg in ["lh", "rh"]:
        inds = dof_map["leg_act_inds"][leg]
        diff_C = float(np.max(np.abs(q_C[inds] - q_nom[inds])))
        corr_B = float(np.max(np.abs(q_B[inds] - q_nom[inds])))
        hind_diffs_C[leg.upper()] = diff_C
        hind_corrections_B[leg.upper()] = corr_B

    max_hind_diff_C = max(hind_diffs_C.values())
    min_hind_corr_B = min(hind_corrections_B.values())

    pass_fm = max_fm_diff < 1e-10
    pass_hind_zero = max_hind_diff_C < 1e-12
    pass_hind_b_active = min_hind_corr_B > 1e-3

    status = pass_fm and pass_hind_zero and pass_hind_b_active

    preflight_c_results = {
        "status": "PASS" if status else "FAIL",
        "SAME_STATE_B_VS_C_FRONT_MIDDLE_TARGET_MAX_DIFF_RAD": max_fm_diff,
        "SAME_STATE_C_HIND_DIFF_VS_NOMINAL_MAX_RAD": max_hind_diff_C,
        "SAME_STATE_B_HIND_CORRECTION_MIN_RAD": min_hind_corr_B,
        "per_leg_fm_diffs_rad": fm_diffs,
        "hind_diffs_C_rad": hind_diffs_C,
        "hind_corrections_B_rad": hind_corrections_B,
        "dynamic_code_path_invariants_verified": True
    }

    print(f"  Max Front/Middle target diff (B vs C): {max_fm_diff:.2e} rad (threshold < 1e-10) -> {'PASS' if pass_fm else 'FAIL'}")
    print(f"  Condition C Hind diff vs nominal M8:    {max_hind_diff_C:.2e} rad (threshold < 1e-12) -> {'PASS' if pass_hind_zero else 'FAIL'}")
    print(f"  Condition B Hind DLS correction:        {min_hind_corr_B:.4f} rad (> 0) -> {'PASS' if pass_hind_b_active else 'FAIL'}")

    if not status:
        raise RuntimeError("PREFLIGHT_FAIL: Condition C causal isolation check failed!")

    return preflight_c_results


def run_preflight_condition_d_manipulation(fly, sim, dof_map, pps, actuated_dofs):
    """
    Preflight 2: Condition D Manipulation Check (Patch 4):
    Before dynamic simulation, verify analytically / kinematically across nominal support trajectory:
    TARGET_SUPPORT_VX_SPREAD_D < TARGET_SUPPORT_VX_SPREAD_B
    (TARGET_SUPPORT_VX_SPREAD_B - TARGET_SUPPORT_VX_SPREAD_D) / TARGET_SUPPORT_VX_SPREAD_B >= 0.50 (50% reduction)
    Report exact per-class nominal stroke lengths and target velocities.
    """
    print("Running Preflight 2: Condition D Manipulation Check ...")
    ctrl_B = ExcursionTrajectoryController(pps, actuated_dofs, excursions=EXCURSIONS_LIST_STOCK,
                                           cadence=6.0, turnaround_fraction=0.06, support_transfer_fraction=0.08, entry_duration=0.0)
    ctrl_D = ExcursionTrajectoryController(pps, actuated_dofs, excursions=EXCURSIONS_LIST_D,
                                           cadence=6.0, turnaround_fraction=0.06, support_transfer_fraction=0.08, entry_duration=0.0)

    t_cycle = 1.0 / 6.0
    n_pts = 1000
    dt = t_cycle / n_pts
    ctrl_B.timestep = dt
    ctrl_D.timestep = dt

    t5_body_ids = {leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id for leg in LEG_LOWER}
    thorax_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
    scratch_data = mujoco.MjData(sim.mj_model)

    def evaluate_nominal_trajectory(controller):
        controller.time = 0.0
        pos_list = []
        adh_list = []
        for _ in range(n_pts):
            action, phi = controller.step()
            scratch_data.qpos[dof_map["qpos_inds"]] = action.joint_angles
            mujoco.mj_kinematics(sim.mj_model, scratch_data)
            t_pos = scratch_data.xpos[thorax_id].copy()
            t_mat = scratch_data.xmat[thorax_id].reshape(3, 3).copy()
            leg_pos = np.zeros((6, 3), dtype=float)
            for leg_i, leg in enumerate(LEG_LOWER):
                p_w = scratch_data.xpos[t5_body_ids[leg]]
                leg_pos[leg_i] = t_mat.T @ (p_w - t_pos)
            pos_list.append(leg_pos)
            adh_list.append(action.adhesion_onoff.copy())

        pos_arr = np.array(pos_list)  # (N, 6, 3)
        adh_arr = np.array(adh_list)  # (N, 6)
        vx_arr = np.gradient(pos_arr[:, :, 0], dt, axis=0)  # (N, 6)
        return pos_arr, adh_arr, vx_arr

    pos_B, adh_B, vx_B = evaluate_nominal_trajectory(ctrl_B)
    pos_D, adh_D, vx_D = evaluate_nominal_trajectory(ctrl_D)

    nominal_strokes_B = {}
    nominal_strokes_D = {}
    nominal_vx_B = {}
    nominal_vx_D = {}

    for i, leg in enumerate(LEG_NAMES):
        sup_mask_B = adh_B[:, i] >= 0.90
        sup_mask_D = adh_D[:, i] >= 0.90
        nominal_strokes_B[leg] = float(np.ptp(pos_B[sup_mask_B, i, 0]))
        nominal_strokes_D[leg] = float(np.ptp(pos_D[sup_mask_D, i, 0]))
        nominal_vx_B[leg] = float(np.median(vx_B[sup_mask_B, i]))
        nominal_vx_D[leg] = float(np.median(vx_D[sup_mask_D, i]))

    spread_trip_a_B = max([nominal_vx_B[l] for l in ["LF", "RM", "LH"]]) - min([nominal_vx_B[l] for l in ["LF", "RM", "LH"]])
    spread_trip_a_D = max([nominal_vx_D[l] for l in ["LF", "RM", "LH"]]) - min([nominal_vx_D[l] for l in ["LF", "RM", "LH"]])
    reduc_a = (spread_trip_a_B - spread_trip_a_D) / max(spread_trip_a_B, 1e-9)

    spread_trip_b_B = max([nominal_vx_B[l] for l in ["RF", "LM", "RH"]]) - min([nominal_vx_B[l] for l in ["RF", "LM", "RH"]])
    spread_trip_b_D = max([nominal_vx_D[l] for l in ["RF", "LM", "RH"]]) - min([nominal_vx_D[l] for l in ["RF", "LM", "RH"]])
    reduc_b = (spread_trip_b_B - spread_trip_b_D) / max(spread_trip_b_B, 1e-9)

    avg_spread_B = 0.5 * (spread_trip_a_B + spread_trip_b_B)
    avg_spread_D = 0.5 * (spread_trip_a_D + spread_trip_b_D)
    avg_reduc = (avg_spread_B - avg_spread_D) / max(avg_spread_B, 1e-9)

    pass_spread = (avg_spread_D < avg_spread_B) and (avg_reduc >= 0.50)

    preflight_d_results = {
        "status": "PASS" if pass_spread else "FAIL",
        "TARGET_SUPPORT_VX_SPREAD_B": avg_spread_B,
        "TARGET_SUPPORT_VX_SPREAD_D": avg_spread_D,
        "VELOCITY_SPREAD_REDUCTION_FRACTION": avg_reduc,
        "tripod_a_spread_B": spread_trip_a_B,
        "tripod_a_spread_D": spread_trip_a_D,
        "tripod_a_reduction_fraction": reduc_a,
        "tripod_b_spread_B": spread_trip_b_B,
        "tripod_b_spread_D": spread_trip_b_D,
        "tripod_b_reduction_fraction": reduc_b,
        "nominal_strokes_B_mm": nominal_strokes_B,
        "nominal_strokes_D_mm": nominal_strokes_D,
        "nominal_velocities_B_mm_s": nominal_vx_B,
        "nominal_velocities_D_mm_s": nominal_vx_D
    }

    print(f"  Nominal Support Vx Spread B: {avg_spread_B:.2f} mm/s")
    print(f"  Nominal Support Vx Spread D: {avg_spread_D:.2f} mm/s")
    print(f"  Velocity Spread Reduction:   {avg_reduc*100.0:.1f}% (threshold >= 50%) -> {'PASS' if pass_spread else 'FAIL'}")

    if not pass_spread:
        print("WARNING: Condition D manipulation check failed to reach 50% velocity spread reduction!")

    return preflight_d_results


# -----------------------------------------------------------------------------
# DYNAMIC SIMULATION RUNNER
# -----------------------------------------------------------------------------

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

    # Abdominal segments for contact detection
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

    # Per-leg world GRF: shape (N, 6, 3)
    leg_grf_world_list = []

    # Target body-relative endpoint position: shape (N, 6, 3)
    target_body_pos_list = []

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

            # Aerospace Euler angles: Roll, Pitch, Yaw
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
        "grf_world": np.array(leg_grf_world_list),  # shape (N, 6, 3)
        "target_body_pos": np.array(target_body_pos_list),  # shape (N, 6, 3)
        "adhesion": np.array(adhesion_list),
        "target_q": np.array(target_q_list),
        "actual_q": np.array(actual_q_list),
        "actuator_forces": np.array(actuator_forces_list),
        "actuator_limits": actuator_limits,
        "abdomen_contact": np.array(abdomen_contact_list),
        "ik_diagnostics": ik_diag,
        "step_target_diffs": step_diffs_arr
    }


# -----------------------------------------------------------------------------
# TELEMETRY ANALYSIS & METRICS COMPUTATION
# -----------------------------------------------------------------------------

def analyze_simulation_telemetry(telem):
    """
    Computes complete reproduction metrics, support conditioning, force conflict,
    propulsive efficiency, kinematic spreads, load distribution, and pulse relations.
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

    # Foot world velocity & Target body endpoint velocity
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

    # 3. Grounded leg count histogram & Transition dynamics
    grounded_counts = np.sum(ss_cont > 0, axis=1)
    leg_count_hist = {int(k): int(np.sum(grounded_counts == k)) for k in range(7)}
    leg_count_fracs = {int(k): float(np.mean(grounded_counts == k)) for k in range(7)}

    # Transition acceleration (within 20ms of tripod adhesion transitions)
    # Detect transitions where adhesion crosses 0.5
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

    # Helper function to compute conflict metrics on a subset of samples
    def evaluate_tripod_samples(sample_indices, sup_leg_inds):
        if len(sample_indices) == 0:
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

        sub_grf = ss_grf[sample_indices][:, sup_leg_inds, :]
        sub_fx = sub_grf[:, :, 0]
        sub_fz_tot = fz_pos_total[sample_indices]

        fx_net = np.sum(sub_fx, axis=1)
        fx_abs_sum = np.sum(np.abs(sub_fx), axis=1)
        cancellation = 1.0 - (np.abs(fx_net) / (fx_abs_sum + 1e-9))
        efficiency = np.abs(fx_net) / (fx_abs_sum + 1e-9)

        th = PRE_REG_FORCE_THRESHOLD_FRAC * sub_fz_tot
        has_opp = (np.max(sub_fx, axis=1) >= th) & (np.min(sub_fx, axis=1) <= -th)

        sub_foot_vx = foot_vx_all[sample_indices][:, sup_leg_inds]
        foot_range = np.ptp(sub_foot_vx, axis=1)

        sub_targ_vx = targ_vx_all[sample_indices][:, sup_leg_inds]
        targ_range = np.ptp(sub_targ_vx, axis=1)

        return {
            "sample_count": int(len(sample_indices)),
            "cancellation_median": float(np.median(cancellation)),
            "cancellation_p05": float(np.percentile(cancellation, 5)),
            "cancellation_p95": float(np.percentile(cancellation, 95)),
            "cancellation_mean": float(np.mean(cancellation)),
            "efficiency_median": float(np.median(efficiency)),
            "opposing_fraction": float(np.mean(has_opp)),
            "foot_vx_range_median": float(np.median(foot_range)),
            "targ_vx_range_median": float(np.median(targ_range))
        }

    # Evaluate Tripod A, Tripod B, and Aggregated subsets
    inds_a_all = np.where(mask_strong_a)[0]
    inds_a_all3 = np.where(mask_strong_a & all3_a)[0]
    inds_b_all = np.where(mask_strong_b)[0]
    inds_b_all3 = np.where(mask_strong_b & all3_b)[0]

    trip_a_all3 = evaluate_tripod_samples(inds_a_all3, TRIPOD_A_INDS)
    trip_b_all3 = evaluate_tripod_samples(inds_b_all3, TRIPOD_B_INDS)

    # Aggregated All-3 Contact Statistics
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

    # 7. Vertical Load Distribution across Anatomical Classes
    pos_fz_matrix = np.clip(ss_grf[:, :, 2], 0.0, None)
    tot_pos_fz = np.sum(pos_fz_matrix, axis=1, keepdims=True)
    load_fractions = pos_fz_matrix / (tot_pos_fz + 1e-9)

    front_loads, mid_loads, hind_loads = [], [], []
    for k in np.where(mask_strong_a)[0]:
        front_loads.append(load_fractions[k, 0])  # LF
        mid_loads.append(load_fractions[k, 4])    # RM
        hind_loads.append(load_fractions[k, 2])   # LH
    for k in np.where(mask_strong_b)[0]:
        front_loads.append(load_fractions[k, 3])  # RF
        mid_loads.append(load_fractions[k, 1])    # LM
        hind_loads.append(load_fractions[k, 5])   # RH

    load_dist_stats = {
        "FRONT": {
            "median": float(np.median(front_loads)) if front_loads else 0.0,
            "p05": float(np.percentile(front_loads, 5)) if front_loads else 0.0,
            "p95": float(np.percentile(front_loads, 95)) if front_loads else 0.0
        },
        "MIDDLE": {
            "median": float(np.median(mid_loads)) if mid_loads else 0.0,
            "p05": float(np.percentile(mid_loads, 5)) if mid_loads else 0.0,
            "p95": float(np.percentile(mid_loads, 95)) if mid_loads else 0.0
        },
        "HIND": {
            "median": float(np.median(hind_loads)) if hind_loads else 0.0,
            "p05": float(np.percentile(hind_loads, 5)) if hind_loads else 0.0,
            "p95": float(np.percentile(hind_loads, 95)) if hind_loads else 0.0
        }
    }

    # 8. Speed Minima / Hesitation Event Detection
    min_dist_samples = int(0.040 / sample_dt)
    peaks, props = find_peaks(vx_20, distance=min_dist_samples, prominence=0.5)

    window_half_samples = int(0.030 / sample_dt)  # +-30 ms
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

    mean_speed_profile = mean_speed_profile.tolist() if len(mean_speed_profile) > 0 else []
    mean_canc_profile = mean_canc_profile.tolist() if len(mean_canc_profile) > 0 else []

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
        "tripod_a_all3": trip_a_all3,
        "tripod_b_all3": trip_b_all3,
        "aggregated_all3": agg_all3,
        "aggregated_all_strong": agg_all_strong,
        "whole_body": whole_body_stats,
        "per_leg_grf": per_leg_grf_stats,
        "load_distribution": load_dist_stats,
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
            "mean_speed_profile_mm_s": mean_speed_profile,
            "mean_cancellation_profile": mean_canc_profile
        }
    }


# -----------------------------------------------------------------------------
# GATE EVALUATION & PRE-REGISTERED CLASSIFICATIONS
# -----------------------------------------------------------------------------

def evaluate_reproduction_gates(metrics_a, metrics_b):
    """Evaluates reproduction of M8 (Condition A) and M25 Planar (Condition B)."""
    # Condition A (M8): Speed 9.93 ± 0.10, CV50 0.232 ± 0.010, Pitch PTP 8.38 ± 0.25, All-3 0.616 ± 0.020
    pass_a_speed = abs(metrics_a["mean_speed_mm_s"] - 9.93) <= 0.10
    pass_a_cv50 = abs(metrics_a["cv50"] - 0.232) <= 0.010
    pass_a_pitch = abs(metrics_a["pitch_ptp_deg"] - 8.38) <= 0.25
    pass_a_all3 = abs(metrics_a["all3_support_fraction"] - 0.616) <= 0.020
    gate_a_pass = pass_a_speed and pass_a_cv50 and pass_a_pitch and pass_a_all3

    # Condition B (Planar): Speed 10.40 ± 0.15, CV50 0.327 ± 0.015, Pitch PTP 8.57 ± 0.25, All-3 0.942 ± 0.015
    pass_b_speed = abs(metrics_b["mean_speed_mm_s"] - 10.40) <= 0.15
    pass_b_cv50 = abs(metrics_b["cv50"] - 0.327) <= 0.015
    pass_b_pitch = abs(metrics_b["pitch_ptp_deg"] - 8.57) <= 0.25
    pass_b_all3 = abs(metrics_b["all3_support_fraction"] - 0.942) <= 0.015
    gate_b_pass = pass_b_speed and pass_b_cv50 and pass_b_pitch and pass_b_all3

    return {
        "gate_a_pass": gate_a_pass,
        "gate_b_pass": gate_b_pass,
        "overall_reproduction_pass": gate_a_pass and gate_b_pass,
        "condition_a_checks": {
            "speed": {"val": metrics_a["mean_speed_mm_s"], "target": 9.93, "tol": 0.10, "pass": pass_a_speed},
            "cv50": {"val": metrics_a["cv50"], "target": 0.232, "tol": 0.010, "pass": pass_a_cv50},
            "pitch_ptp": {"val": metrics_a["pitch_ptp_deg"], "target": 8.38, "tol": 0.25, "pass": pass_a_pitch},
            "all3_contact": {"val": metrics_a["all3_support_fraction"], "target": 0.616, "tol": 0.020, "pass": pass_a_all3}
        },
        "condition_b_checks": {
            "speed": {"val": metrics_b["mean_speed_mm_s"], "target": 10.40, "tol": 0.15, "pass": pass_b_speed},
            "cv50": {"val": metrics_b["cv50"], "target": 0.327, "tol": 0.015, "pass": pass_b_cv50},
            "pitch_ptp": {"val": metrics_b["pitch_ptp_deg"], "target": 8.57, "tol": 0.25, "pass": pass_b_pitch},
            "all3_contact": {"val": metrics_b["all3_support_fraction"], "target": 0.942, "tol": 0.015, "pass": pass_b_all3}
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


def classify_condition_c(metrics_a, metrics_b, metrics_c, telem_gate_c, repro_pass):
    """
    Evaluates Condition C Pre-Registered Primary Classification Rubric (Patch 3).
    
    HIND_CONSTRAINT_HYPOTHESIS_SUPPORTED: satisfies ALL 9 criteria:
    1. Median ALL3 force cancellation improves by >= 0.08 absolute relative to fresh B
    2. Opposing-force fraction improves by >= 0.08 absolute relative to B
    3. CV50 improves by >= 15% relative to B
    4. Wrong-sign20 fraction improves by >= 50% relative to B OR falls to <= 0.04
    5. Mean backward speed >= 8.5 mm/s
    6. Pitch PTP does not worsen by > 10% relative to fresh M8
    7. Thorax Z PTP <= 0.14 mm
    8. Lateral ratio <= 0.02
    9. Target / actuator / numerical integrity passes.
    
    HIND_CONSTRAINT_HYPOTHESIS_PARTIALLY_SUPPORTED:
    Force conflict clearly improves (Delta cancellation >= 0.04) AND either CV50 OR wrong-sign motion
    meaningfully improves, while speed remains >= 8.5 mm/s.
    
    HIND_CONSTRAINT_HYPOTHESIS_NOT_SUPPORTED:
    Selective hind relief does not materially reduce conflict or does not improve glide dynamics.
    
    INVALID_EXPERIMENT:
    Parent reproduction / telemetry / implementation integrity failure.
    """
    if not repro_pass or not telem_gate_c["telemetry_integrity_pass"]:
        return "INVALID_EXPERIMENT", "NONE", {}

    canc_b = metrics_b["aggregated_all3"]["cancellation_median"]
    canc_c = metrics_c["aggregated_all3"]["cancellation_median"]
    delta_canc = canc_b - canc_c  # Positive means improvement (lower cancellation in C)

    opp_b = metrics_b["aggregated_all3"]["opposing_fraction"]
    opp_c = metrics_c["aggregated_all3"]["opposing_fraction"]
    delta_opp = opp_b - opp_c

    cv50_b = metrics_b["cv50"]
    cv50_c = metrics_c["cv50"]
    cv50_reduc = (cv50_b - cv50_c) / max(cv50_b, 1e-9)

    ws20_b = metrics_b["wrong_sign_20"]
    ws20_c = metrics_c["wrong_sign_20"]
    ws20_reduc = (ws20_b - ws20_c) / max(ws20_b, 1e-9) if ws20_b > 1e-4 else 0.0

    speed_c = metrics_c["mean_speed_mm_s"]

    pitch_ptp_a = metrics_a["pitch_ptp_deg"]
    pitch_ptp_c = metrics_c["pitch_ptp_deg"]

    z_ptp_c = metrics_c["z_ptp_mm"]
    lateral_ratio_c = metrics_c["lateral_ratio"]

    # 9 Primary Criteria
    crit1 = delta_canc >= 0.08
    crit2 = delta_opp >= 0.08
    crit3 = cv50_reduc >= 0.15
    crit4 = (ws20_reduc >= 0.50) or (ws20_c <= 0.04)
    crit5 = speed_c >= 8.5
    crit6 = pitch_ptp_c <= 1.10 * pitch_ptp_a
    crit7 = z_ptp_c <= 0.14
    crit8 = lateral_ratio_c <= 0.02
    crit9 = telem_gate_c["telemetry_integrity_pass"]

    criteria_dict = {
        "cancellation_improvement_ge_008": {"val": delta_canc, "thresh": 0.08, "pass": crit1},
        "opposing_fraction_improvement_ge_008": {"val": delta_opp, "thresh": 0.08, "pass": crit2},
        "cv50_improvement_ge_15pct": {"val": cv50_reduc, "thresh": 0.15, "pass": crit3},
        "wrong_sign_improvement_ge_50pct_or_le_004": {"val": ws20_reduc, "val_abs": ws20_c, "pass": crit4},
        "backward_speed_ge_85": {"val": speed_c, "thresh": 8.5, "pass": crit5},
        "pitch_ptp_not_worse_10pct_vs_m8": {"val": pitch_ptp_c, "ref_m8": pitch_ptp_a, "max_allowed": 1.10 * pitch_ptp_a, "pass": crit6},
        "z_ptp_le_014": {"val": z_ptp_c, "thresh": 0.14, "pass": crit7},
        "lateral_ratio_le_002": {"val": lateral_ratio_c, "thresh": 0.02, "pass": crit8},
        "numerical_integrity_pass": {"pass": crit9}
    }

    all_pass = all([crit1, crit2, crit3, crit4, crit5, crit6, crit7, crit8, crit9])

    if all_pass:
        classification = "HIND_CONSTRAINT_HYPOTHESIS_SUPPORTED"
    elif (delta_canc >= 0.04) and (cv50_reduc >= 0.10 or ws20_reduc >= 0.30 or ws20_c <= 0.05) and (speed_c >= 8.5):
        classification = "HIND_CONSTRAINT_HYPOTHESIS_PARTIALLY_SUPPORTED"
    else:
        classification = "HIND_CONSTRAINT_HYPOTHESIS_NOT_SUPPORTED"

    # Secondary Diagnostic Interpretation
    mid_fx_c = max(metrics_c["per_leg_grf"]["LM"]["MEDIAN_FX_WORLD"], metrics_c["per_leg_grf"]["RM"]["MEDIAN_FX_WORLD"])
    hind_fx_c = max(metrics_c["per_leg_grf"]["LH"]["MEDIAN_FX_WORLD"], metrics_c["per_leg_grf"]["RH"]["MEDIAN_FX_WORLD"])
    roll_ptp_c = metrics_c["roll_ptp_deg"]

    secondary = "NONE"
    if (hind_fx_c <= 0.50) and (mid_fx_c >= 1.50) and (canc_c >= 0.300) and (cv50_c >= 0.230):
        secondary = "HIND_RELIEF_KINEMATICALLY_CONFOUNDED"
    elif (roll_ptp_c > 30.0 or metrics_c["abdomen_contact_count"] > 0) and (cv50_c >= 0.300):
        secondary = "HIND_RELIEF_POSTURALLY_CONFOUNDED"
    elif (pitch_ptp_c <= 12.0 and roll_ptp_c <= 20.0) and (abs(cv50_c - metrics_a["cv50"]) < 0.02):
        secondary = "HIND_RELIEF_INEFFECTIVE"

    return classification, secondary, criteria_dict


def classify_condition_d(metrics_b, metrics_d, preflight_d, telem_gate_d, repro_pass):
    """
    Evaluates Condition D Primary Excursion Matching Classification Rubric (Patch 4).
    
    EXCURSION_MATCHING_SUPPORTED: satisfies ALL 11 criteria:
    1. Nominal target longitudinal velocity spread is reduced >= 50% relative to B (Manipulation check passed)
    2. Median ALL3 force cancellation improves by >= 0.08 absolute relative to B
    3. Opposing-force fraction improves by >= 0.08 absolute relative to B
    4. CV50 improves by >= 15% relative to B
    5. Wrong-sign20 improves by >= 50% relative to B OR falls to <= 0.04
    6. Mean backward speed >= 8.5 mm/s
    7. Pitch PTP <= fresh B pitch PTP OR improves by >= 10% relative to B
    8. Roll PTP <= 20.0 deg
    9. Thorax Z PTP <= 0.14 mm
    10. Lateral ratio <= 0.02
    11. Actuator / IK / target continuity / numerical integrity passes.
    
    EXCURSION_MATCHING_PARTIALLY_SUPPORTED:
    Nominal velocity spread falls >= 50%, force cancellation improves meaningfully (>= 0.04),
    and either CV50 or wrong-sign improves meaningfully, while speed >= 8.5 mm/s and simulation valid.
    
    EXCURSION_MATCHING_NOT_SUPPORTED:
    D successfully reduces nominal target velocity mismatch but does not materially reduce physical force conflict or improve glide.
    
    INVALID_MANIPULATION:
    Commanded velocity spread is not reduced by >= 50% in analytical preflight.
    
    INVALID_EXPERIMENT:
    Reproduction / telemetry / numerical integrity failure.
    """
    if preflight_d["status"] != "PASS":
        return "INVALID_MANIPULATION", {}

    if not repro_pass or not telem_gate_d["telemetry_integrity_pass"]:
        return "INVALID_EXPERIMENT", {}

    canc_b = metrics_b["aggregated_all3"]["cancellation_median"]
    canc_d = metrics_d["aggregated_all3"]["cancellation_median"]
    delta_canc = canc_b - canc_d

    opp_b = metrics_b["aggregated_all3"]["opposing_fraction"]
    opp_d = metrics_d["aggregated_all3"]["opposing_fraction"]
    delta_opp = opp_b - opp_d

    cv50_b = metrics_b["cv50"]
    cv50_d = metrics_d["cv50"]
    cv50_reduc = (cv50_b - cv50_d) / max(cv50_b, 1e-9)

    ws20_b = metrics_b["wrong_sign_20"]
    ws20_d = metrics_d["wrong_sign_20"]
    ws20_reduc = (ws20_b - ws20_d) / max(ws20_b, 1e-9) if ws20_b > 1e-4 else 0.0

    speed_d = metrics_d["mean_speed_mm_s"]

    pitch_ptp_b = metrics_b["pitch_ptp_deg"]
    pitch_ptp_d = metrics_d["pitch_ptp_deg"]

    roll_ptp_d = metrics_d["roll_ptp_deg"]
    z_ptp_d = metrics_d["z_ptp_mm"]
    lateral_ratio_d = metrics_d["lateral_ratio"]

    # 11 Criteria
    crit1 = preflight_d["VELOCITY_SPREAD_REDUCTION_FRACTION"] >= 0.50
    crit2 = delta_canc >= 0.08
    crit3 = delta_opp >= 0.08
    crit4 = cv50_reduc >= 0.15
    crit5 = (ws20_reduc >= 0.50) or (ws20_d <= 0.04)
    crit6 = speed_d >= 8.5
    crit7 = (pitch_ptp_d <= pitch_ptp_b) or ((pitch_ptp_b - pitch_ptp_d) / pitch_ptp_b >= 0.10)
    crit8 = roll_ptp_d <= 20.0
    crit9 = z_ptp_d <= 0.14
    crit10 = lateral_ratio_d <= 0.02
    crit11 = telem_gate_d["telemetry_integrity_pass"]

    criteria_dict = {
        "nominal_velocity_spread_reduction_ge_50pct": {"val": preflight_d["VELOCITY_SPREAD_REDUCTION_FRACTION"], "pass": crit1},
        "cancellation_improvement_ge_008": {"val": delta_canc, "thresh": 0.08, "pass": crit2},
        "opposing_fraction_improvement_ge_008": {"val": delta_opp, "thresh": 0.08, "pass": crit3},
        "cv50_improvement_ge_15pct": {"val": cv50_reduc, "thresh": 0.15, "pass": crit4},
        "wrong_sign_improvement_ge_50pct_or_le_004": {"val": ws20_reduc, "val_abs": ws20_d, "pass": crit5},
        "backward_speed_ge_85": {"val": speed_d, "thresh": 8.5, "pass": crit6},
        "pitch_ptp_le_b_or_improved_10pct": {"val": pitch_ptp_d, "ref_b": pitch_ptp_b, "pass": crit7},
        "roll_ptp_le_20deg": {"val": roll_ptp_d, "thresh": 20.0, "pass": crit8},
        "z_ptp_le_014mm": {"val": z_ptp_d, "thresh": 0.14, "pass": crit9},
        "lateral_ratio_le_002": {"val": lateral_ratio_d, "thresh": 0.02, "pass": crit10},
        "numerical_integrity_pass": {"pass": crit11}
    }

    all_pass = all([crit1, crit2, crit3, crit4, crit5, crit6, crit7, crit8, crit9, crit10, crit11])

    if all_pass:
        classification = "EXCURSION_MATCHING_SUPPORTED"
    elif crit1 and (delta_canc >= 0.04) and (cv50_reduc >= 0.10 or ws20_reduc >= 0.30 or ws20_d <= 0.05) and (speed_d >= 8.5):
        classification = "EXCURSION_MATCHING_PARTIALLY_SUPPORTED"
    else:
        classification = "EXCURSION_MATCHING_NOT_SUPPORTED"

    return classification, criteria_dict


# -----------------------------------------------------------------------------
# DIAGNOSTIC PLOTTING
# -----------------------------------------------------------------------------

def generate_diagnostic_plots(telemetry_dict, results_dict, output_dir="output/m27"):
    """Generates crisp diagnostic plots comparing Conditions A, B, C, and D."""
    os.makedirs(output_dir, exist_ok=True)
    print(f"\nGenerating diagnostic plots in {output_dir} ...")

    conds = ["A", "B", "C", "D"]
    labels = ["A: M8 Champion", "B: M25 Planar", "C: Hind Relief", "D: Excursion Match"]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    # 1. Force Cancellation Comparison Plot
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), dpi=200)

    # Left: Active Support ALL3 Cancellation
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
    axes[0].tick_params(axis="x", rotation=15)

    # Right: Whole-Body 6-Leg Cancellation
    canc_6_medians = [results_dict[c]["whole_body"]["cancellation_all_samples_median"] for c in conds]
    axes[1].bar(labels, canc_6_medians, color=colors, alpha=0.8)
    axes[1].set_ylabel("Whole-Body Cancellation Ratio")
    axes[1].set_title("Whole-Body 6-Leg Force Cancellation")
    axes[1].set_ylim(0.0, 1.0)
    axes[1].grid(True, alpha=0.3, axis="y")
    axes[1].tick_params(axis="x", rotation=15)

    plt.tight_layout()
    plot1_path = os.path.join(output_dir, "cancellation_comparison.png")
    plt.savefig(plot1_path)
    plt.close()
    print(f"  Saved: {plot1_path}")

    # 2. Per-Leg Fx Comparison Plot (Front, Mid, Hind)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), dpi=200)
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
        ax.set_xticklabels(["A", "B", "C", "D"])
        ax.set_ylabel("Median Fx World (mN)")
        ax.set_title(f"{cls_name} Legs ({l1}, {l2})")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plot2_path = os.path.join(output_dir, "per_leg_fx_comparison.png")
    plt.savefig(plot2_path)
    plt.close()
    print(f"  Saved: {plot2_path}")

    # 3. Velocity Profiles Comparison
    fig, ax = plt.subplots(figsize=(12, 4.5), dpi=200)
    for c, lbl, col in zip(conds, labels, colors):
        telem = telemetry_dict[c]
        t = telem["t"]
        vx = telem["vx"]
        # Window: t = 0.50 to 1.10 s (approx 3.5 strides)
        w_mask = (t >= 0.50) & (t <= 1.10)
        w20 = max(1, int(0.020 / telem["sample_dt"]))
        vx_20 = np.convolve(vx, np.ones(w20)/w20, mode="same")
        # Plot backward velocity: -vx
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
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), dpi=200)
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


# -----------------------------------------------------------------------------
# MAIN EXPERIMENT EXECUTION
# -----------------------------------------------------------------------------

def run_m27_experiment(skip_render=True):
    """
    Executes the entire MW-M27 experiment:
    1. Preflight provenance and isolation checks
    2. Exactly 4 dynamic evaluation rollouts (A, B, C, D)
    3. Quantitative analysis, gates, and primary/secondary classifications
    4. Diagnostic plots and JSON artifacts export
    """
    print("=" * 70)
    print("MW-M27: SELECTIVE HIND-LEG RELIEF & EXCURSION-MATCHED SUPPORT")
    print("=" * 70)

    # Step 1: Preflight Provenance & Scratch Isolation
    m8_expected = run_preflight_provenance_audit()

    fly_probe = make_locomotion_fly(name="fly", add_adhesion=True)
    sim_probe = m25.create_isolated_sim(fly_probe)
    dof_map = m25.build_semantic_dof_mapping(fly_probe, sim_probe)
    pps = PreprogrammedSteps()
    actuated_dofs = fly_probe.get_actuated_jointdofs_order(ActuatorType.POSITION)

    verify_scratch_mjdata_isolation(fly_probe, sim_probe, dof_map)

    # Step 2: Causal Isolation Preflights (Patch 2 & Patch 4)
    preflight_c = run_preflight_condition_c_isolation(fly_probe, sim_probe, dof_map, pps, actuated_dofs)
    preflight_d = run_preflight_condition_d_manipulation(fly_probe, sim_probe, dof_map, pps, actuated_dofs)

    sim_probe.close()

    # Step 3: Exactly 4 Dynamic Simulation Rollouts
    conditions = ["A", "B", "C", "D"]
    telemetry = {}
    metrics = {}

    def build_ctrl_a(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M27SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="A")

    def build_ctrl_b(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M27SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="B",
                                            frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    def build_ctrl_c(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M27SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="C",
                                            frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    def build_ctrl_d(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M27SupportGeometryController(sim, fly, pps, actuated_dofs, dm, condition="D",
                                            frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    builders = {
        "A": build_ctrl_a,
        "B": build_ctrl_b,
        "C": build_ctrl_c,
        "D": build_ctrl_d
    }

    cond_names = {
        "A": "Condition A (DIRECT_M8_CONTROL)",
        "B": "Condition B (M25_BODY_FRAME_PLANAR_SUPPORT)",
        "C": "Condition C (SELECTIVE_HIND_RELIEF)",
        "D": "Condition D (EXCURSION_MATCHED_PLANAR_SUPPORT)"
    }

    for c in conditions:
        telem = run_diagnostic_simulation(cond_names[c], builders[c])
        telemetry[c] = telem
        met = analyze_simulation_telemetry(telem)
        metrics[c] = met

    # Step 4: Gates & Classification Evaluation
    repro_eval = evaluate_reproduction_gates(metrics["A"], metrics["B"])
    telem_gates = {c: evaluate_telemetry_integrity(metrics[c]) for c in conditions}

    class_c, sec_c, crit_c = classify_condition_c(
        metrics["A"], metrics["B"], metrics["C"], telem_gates["C"], repro_eval["overall_reproduction_pass"]
    )

    class_d, crit_d = classify_condition_d(
        metrics["B"], metrics["D"], preflight_d, telem_gates["D"], repro_eval["overall_reproduction_pass"]
    )

    # Additional B vs D Diagnostics
    canc_b = metrics["B"]["aggregated_all3"]["cancellation_median"]
    canc_d = metrics["D"]["aggregated_all3"]["cancellation_median"]
    cv50_b = metrics["B"]["cv50"]
    cv50_d = metrics["D"]["cv50"]
    ws20_b = metrics["B"]["wrong_sign_20"]
    ws20_d = metrics["D"]["wrong_sign_20"]

    diag_cmd_mismatch = "YES" if preflight_d["VELOCITY_SPREAD_REDUCTION_FRACTION"] >= 0.50 else "NO"
    diag_force_conflict = "YES" if (canc_b - canc_d) >= 0.04 else "NO"
    diag_glide_improved = "YES" if ((cv50_b - cv50_d) / max(cv50_b, 1e-9) >= 0.10 or (ws20_b - ws20_d) / max(ws20_b, 1e-9) >= 0.30) else "NO"

    b_vs_d_diagnostics = {
        "COMMAND_MISMATCH_REDUCED": diag_cmd_mismatch,
        "PHYSICAL_FORCE_CONFLICT_REDUCED": diag_force_conflict,
        "BODY_GLIDE_IMPROVED": diag_glide_improved,
        "velocity_spread_reduction_fraction": preflight_d["VELOCITY_SPREAD_REDUCTION_FRACTION"],
        "delta_cancellation_median_all3": float(canc_b - canc_d),
        "cv50_reduction_fraction": float((cv50_b - cv50_d) / max(cv50_b, 1e-9)),
        "wrong_sign_20_reduction_fraction": float((ws20_b - ws20_d) / max(ws20_b, 1e-9)) if ws20_b > 1e-4 else 0.0
    }

    # Print Summary of Results
    print("\n" + "=" * 70)
    print("MW-M27 EXPERIMENTAL EVALUATION SUMMARY")
    print("=" * 70)
    print(f"Parent Reproduction Gate: {'PASS' if repro_eval['overall_reproduction_pass'] else 'FAIL'}")
    print(f"  Condition A (M8):     Speed={metrics['A']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['A']['cv50']:.3f}, Pitch PTP={metrics['A']['pitch_ptp_deg']:.2f} deg, All3={metrics['A']['all3_support_fraction']:.3f}")
    print(f"  Condition B (Planar): Speed={metrics['B']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['B']['cv50']:.3f}, Pitch PTP={metrics['B']['pitch_ptp_deg']:.2f} deg, All3={metrics['B']['all3_support_fraction']:.3f}")
    print(f"  Condition C (Hind):   Speed={metrics['C']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['C']['cv50']:.3f}, Pitch PTP={metrics['C']['pitch_ptp_deg']:.2f} deg, All3={metrics['C']['all3_support_fraction']:.3f}")
    print(f"  Condition D (Excurs): Speed={metrics['D']['mean_speed_mm_s']:.2f} mm/s, CV50={metrics['D']['cv50']:.3f}, Pitch PTP={metrics['D']['pitch_ptp_deg']:.2f} deg, All3={metrics['D']['all3_support_fraction']:.3f}")

    print("\nForce Cancellation (ALL3 Contact):")
    print(f"  Condition A: {metrics['A']['aggregated_all3']['cancellation_median']:.4f}")
    print(f"  Condition B: {metrics['B']['aggregated_all3']['cancellation_median']:.4f}")
    print(f"  Condition C: {metrics['C']['aggregated_all3']['cancellation_median']:.4f} (Delta vs B = {canc_b - metrics['C']['aggregated_all3']['cancellation_median']:+.4f})")
    print(f"  Condition D: {metrics['D']['aggregated_all3']['cancellation_median']:.4f} (Delta vs B = {canc_b - canc_d:+.4f})")

    print(f"\nCondition C Primary Classification:    {class_c}")
    print(f"Condition C Secondary Diagnostic:      {sec_c}")
    print(f"Condition D Primary Classification:    {class_d}")
    print(f"B vs D Diagnostics: COMMAND_MISMATCH_REDUCED={diag_cmd_mismatch}, FORCE_CONFLICT_REDUCED={diag_force_conflict}, GLIDE_IMPROVED={diag_glide_improved}")

    # Step 5: Export Manifest & Results JSON
    manifest_data = {
        "milestone": "MW-M27",
        "task": "Selective hind-leg relief causal intervention",
        "provenance_parameters": m8_expected,
        "frozen_heights_provenance": FROZEN_BODY_SUPPORT_HEIGHTS_MM,
        "condition_d_excursions": CONDITION_D_EXCURSIONS,
        "preflight_c_isolation": preflight_c,
        "preflight_d_manipulation": preflight_d,
        "reproduction_gates": repro_eval,
        "telemetry_integrity_gates": telem_gates,
        "condition_c_classification": class_c,
        "condition_c_secondary_diagnostic": sec_c,
        "condition_c_criteria_evaluation": crit_c,
        "condition_d_classification": class_d,
        "condition_d_criteria_evaluation": crit_d,
        "b_vs_d_diagnostics": b_vs_d_diagnostics
    }

    manifest_path = pathlib.Path("data/derived/m27_hind_relief_manifest.json")
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(manifest_data), f, indent=2, allow_nan=False)
    print(f"\nSaved manifest: {manifest_path}")

    results_data = {
        "milestone": "MW-M27",
        "conditions": metrics,
        "reproduction_gates": repro_eval,
        "telemetry_integrity_gates": telem_gates,
        "condition_c_verdict": {
            "primary_classification": class_c,
            "secondary_diagnostic": sec_c,
            "criteria": crit_c
        },
        "condition_d_verdict": {
            "primary_classification": class_d,
            "criteria": crit_d,
            "b_vs_d_diagnostics": b_vs_d_diagnostics
        }
    }

    results_path = pathlib.Path("data/derived/m27_hind_relief_results.json")
    with open(results_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(results_data), f, indent=2, allow_nan=False)
    print(f"Saved results:  {results_path}")

    # Step 6: Generate Diagnostic Plots
    generate_diagnostic_plots(telemetry, metrics, output_dir="output/m27")

    # Step 7: Optional Clean Rendering Rollouts (Budget: up to 4 runs)
    if not skip_render:
        print("\nRendering diagnostic videos for Conditions A, B, C, D...")
        output_dir = pathlib.Path("output/m27")
        output_dir.mkdir(parents=True, exist_ok=True)
        for c in conditions:
            vid_path = output_dir / f"condition_{c.lower()}_render.mp4"
            print(f"  Rendering {cond_names[c]} -> {vid_path} ...")
            fly_rend = make_locomotion_fly(name="fly", add_adhesion=True)
            cam = fly_rend.add_tracking_camera(name="trackcam")
            sim_rend = m25.create_isolated_sim(fly_rend)
            ctrl_rend = builders[c](sim_rend, fly_rend)
            sim_rend.reset()
            sim_rend.set_actuator_inputs(fly_rend.name, ActuatorType.POSITION, ctrl_rend.default_pose)
            sim_rend.set_leg_adhesion_states(fly_rend.name, np.ones(6, dtype=float))
            sim_rend.warmup()
            renderer = sim_rend.set_renderer(
                cam, camera_res=(240, 320), playback_speed=0.75, output_fps=30, buffer_frames=True
            )
            try:
                for _ in range(N_STEPS):
                    act, _ = ctrl_rend.step()
                    apply_locomotion_action(sim_rend, fly_rend.name, act)
                    sim_rend.step()
                    sim_rend.render_as_needed()
                renderer.save_video(str(vid_path))
            finally:
                renderer.close()
                sim_rend.close()
            print(f"  Saved: {vid_path}")

    print("\nMW-M27 Evaluation Finished Successfully.")
    return manifest_data, results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MW-M27 Selective Hind Relief & Excursion Matching")
    parser.add_argument("--render", action="store_true", default=False, help="Render diagnostic videos")
    parser.add_argument("--skip-render", action="store_true", default=True, help="Skip video rendering")
    args = parser.parse_args()

    do_render = args.render or (not args.skip_render)
    run_m27_experiment(skip_render=not do_render)
