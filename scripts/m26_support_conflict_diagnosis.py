#!/usr/bin/env python3
"""
MW-M26: Diagnose Internal Mechanical Conflict within the Supporting Tripod.

Scientific Question:
"When all three M25-B supporting legs remain planted, do they generate
mutually conflicting horizontal ground reaction forces / kinematics
that mechanically oppose one another through the thorax?"

Evaluates exactly TWO dynamic conditions:
- Condition A: DIRECT_M8_CONTROL (M8 Champion)
- Condition B: M25_BODY_FRAME_PLANAR_SUPPORT (M25 Planar Support)

Reuses the exact implementations, frozen heights, and parameters from M25.
No controller redesign. No parameter search. Exactly 2 dynamic simulation runs.
"""

import os
import sys
import json
import pathlib
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

# Anatomical class mappings
CLASS_TO_LEGS = {
    "FRONT": [0, 3],   # LF, RF
    "MIDDLE": [1, 4],  # LM, RM
    "HIND": [2, 5]     # LH, RH
}

PHYSICS_DT = 0.0001
SAMPLE_STEPS = 10
SAMPLE_DT = PHYSICS_DT * SAMPLE_STEPS
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)
STEADY_START_TIME = 0.35

# Pre-registered thresholds
PRE_REG_FORCE_THRESHOLD_FRAC = 0.01  # 1% of instantaneous total positive vertical load (~1.3 mN)
PRE_REG_EFFECT_THRESHOLD = 0.10     # >= 0.10 absolute increase in median cancellation ratio


# -----------------------------------------------------------------------------
# PREFLIGHT VERIFICATION
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

    return expected


# -----------------------------------------------------------------------------
# SIMULATION WITH COMPREHENSIVE FORCE AND KINEMATIC TELEMETRY
# -----------------------------------------------------------------------------

def run_diagnostic_simulation(condition_name, controller_builder, n_steps=N_STEPS, sample_dt=SAMPLE_DT):
    """
    Runs a single physically isolated simulation rollout, collecting full 3D per-leg
    ground reaction forces, body kinematics, foot kinematics, and target FK endpoints.
    """
    print(f"\nRunning simulation for condition: {condition_name} ({n_steps*PHYSICS_DT:.2f} s) ...")
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

    # Scratch MjData for target endpoint FK calculation
    scratch_data = mujoco.MjData(sim.mj_model)

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    t_list, x_list, y_list, z_list = [], [], [], []
    pitch_list, quat_list = [], []
    vx_list, ax_list = [], []
    tarsus_xyz_list = []
    contact_list = []
    adhesion_list = []
    target_q_list = []
    actual_q_list = []
    actuator_forces_list = []

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
        "ik_diagnostics": ik_diag,
        "step_target_diffs": step_diffs_arr
    }


# -----------------------------------------------------------------------------
# DETAILED FORCE & KINEMATIC CONFLICT ANALYSIS
# -----------------------------------------------------------------------------

def analyze_support_conflict(telem):
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
    ss_quat = telem["quat"][ss_mask]
    ss_vx = telem["vx"][ss_mask]
    ss_ax = telem["ax"][ss_mask]
    ss_tars = telem["tarsus_xyz"][ss_mask]
    ss_cont = telem["contact"][ss_mask]
    ss_grf = telem["grf_world"][ss_mask]  # (N_ss, 6, 3)
    ss_adh = telem["adhesion"][ss_mask]
    ss_targ_body = telem["target_body_pos"][ss_mask]  # (N_ss, 6, 3)

    # 1. Base reproduction metrics
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
    z_ptp = float(np.ptp(ss_z))
    z_std = float(np.std(ss_z))

    # 2. Foot and Target Endpoint Kinematics
    # Foot world velocity (d/dt of tarsus5 world position)
    foot_vx_all = np.gradient(ss_tars[:, :, 0], sample_dt, axis=0)  # (N_ss, 6)
    # Target body endpoint velocity (d/dt of target body-relative X position)
    targ_vx_all = np.gradient(ss_targ_body[:, :, 0], sample_dt, axis=0)  # (N_ss, 6)

    # 3. Ground Reaction Forces: Fx, Fy, Fz per leg
    # World-frame forces acting on the fly:
    # Fx > 0: pushing fly forward (+X). Fx < 0: pushing fly backward (-X, propulsive for moonwalking).
    # Fz > 0: upward support against gravity.
    fz_pos_total = np.sum(np.clip(ss_grf[:, :, 2], 0.0, None), axis=1)  # (N_ss,)

    # Telemetry Integrity Verification
    fz_positive_all_samples = bool(np.all(fz_pos_total > 1e-3))  # Verify continuous positive vertical load
    forces_finite = bool(np.all(np.isfinite(ss_grf)))
    telemetry_valid = fz_positive_all_samples and forces_finite

    # 4. Support-Phase Conditioning
    # Strong support: median support adhesion >= 0.90 AND median opposite adhesion <= 0.10
    tripod_configs = [
        ("TRIPOD_A", TRIPOD_A_INDS, TRIPOD_B_INDS),
        ("TRIPOD_B", TRIPOD_B_INDS, TRIPOD_A_INDS)
    ]

    mask_strong_a = (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) <= 0.10)
    mask_strong_b = (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) <= 0.10)

    # All-3 contact masks
    all3_a = np.all(ss_cont[:, TRIPOD_A_INDS] > 0, axis=1)
    all3_b = np.all(ss_cont[:, TRIPOD_B_INDS] > 0, axis=1)

    all3_support_fraction_a = float(np.mean(all3_a[mask_strong_a])) if np.sum(mask_strong_a) > 0 else 0.0
    all3_support_fraction_b = float(np.mean(all3_b[mask_strong_b])) if np.sum(mask_strong_b) > 0 else 0.0

    total_plateau_samples = int(np.sum(mask_strong_a) + np.sum(mask_strong_b))
    total_all3_samples = int(np.sum(all3_a & mask_strong_a) + np.sum(all3_b & mask_strong_b))
    all3_aggregate_fraction = float(total_all3_samples / max(total_plateau_samples, 1))

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
                "efficiency_p05": 0.0,
                "efficiency_p95": 0.0,
                "opposing_fraction": 0.0,
                "opposing_fraction_01": 0.0,
                "opposing_fraction_05": 0.0,
                "foot_vx_range_median": 0.0,
                "foot_vx_std_median": 0.0,
                "targ_vx_range_median": 0.0,
                "targ_vx_std_median": 0.0,
                "per_leg_fx": {LEG_NAMES[l]: {"median": 0.0, "p05": 0.0, "p95": 0.0} for l in sup_leg_inds}
            }

        sub_grf = ss_grf[sample_indices][:, sup_leg_inds, :]  # (M, 3, 3)
        sub_fx = sub_grf[:, :, 0]  # (M, 3)
        sub_fz_tot = fz_pos_total[sample_indices]  # (M,)

        fx_net = np.sum(sub_fx, axis=1)
        fx_abs_sum = np.sum(np.abs(sub_fx), axis=1)
        cancellation = 1.0 - (np.abs(fx_net) / (fx_abs_sum + 1e-9))
        efficiency = np.abs(fx_net) / (fx_abs_sum + 1e-9)

        # Opposing force fractions at different thresholds
        # Threshold normalized to total positive vertical load
        th_02 = PRE_REG_FORCE_THRESHOLD_FRAC * sub_fz_tot
        th_01 = 0.01 * sub_fz_tot
        th_05 = 0.05 * sub_fz_tot

        has_opp_02 = (np.max(sub_fx, axis=1) >= th_02) & (np.min(sub_fx, axis=1) <= -th_02)
        has_opp_01 = (np.max(sub_fx, axis=1) >= th_01) & (np.min(sub_fx, axis=1) <= -th_01)
        has_opp_05 = (np.max(sub_fx, axis=1) >= th_05) & (np.min(sub_fx, axis=1) <= -th_05)

        # Foot kinematics on these samples
        sub_foot_vx = foot_vx_all[sample_indices][:, sup_leg_inds]  # (M, 3)
        foot_range = np.ptp(sub_foot_vx, axis=1)
        foot_std = np.std(sub_foot_vx, axis=1)

        # Target kinematics on these samples
        sub_targ_vx = targ_vx_all[sample_indices][:, sup_leg_inds]  # (M, 3)
        targ_range = np.ptp(sub_targ_vx, axis=1)
        targ_std = np.std(sub_targ_vx, axis=1)

        per_leg_fx = {}
        for idx_in_trip, leg_idx in enumerate(sup_leg_inds):
            l_name = LEG_NAMES[leg_idx]
            fx_leg = sub_fx[:, idx_in_trip]
            per_leg_fx[l_name] = {
                "median": float(np.median(fx_leg)),
                "p05": float(np.percentile(fx_leg, 5)),
                "p95": float(np.percentile(fx_leg, 95))
            }

        return {
            "sample_count": int(len(sample_indices)),
            "cancellation_median": float(np.median(cancellation)),
            "cancellation_p05": float(np.percentile(cancellation, 5)),
            "cancellation_p95": float(np.percentile(cancellation, 95)),
            "cancellation_mean": float(np.mean(cancellation)),
            "efficiency_median": float(np.median(efficiency)),
            "efficiency_p05": float(np.percentile(efficiency, 5)),
            "efficiency_p95": float(np.percentile(efficiency, 95)),
            "opposing_fraction": float(np.mean(has_opp_02)),
            "opposing_fraction_01": float(np.mean(has_opp_01)),
            "opposing_fraction_05": float(np.mean(has_opp_05)),
            "foot_vx_range_median": float(np.median(foot_range)),
            "foot_vx_std_median": float(np.median(foot_std)),
            "targ_vx_range_median": float(np.median(targ_range)),
            "targ_vx_std_median": float(np.median(targ_std)),
            "per_leg_fx": per_leg_fx
        }

    # Evaluate by Tripod & Contact Topology
    # Tripod A
    inds_a_all = np.where(mask_strong_a)[0]
    inds_a_all3 = np.where(mask_strong_a & all3_a)[0]
    inds_a_lt3 = np.where(mask_strong_a & ~all3_a)[0]

    trip_a_metrics = {
        "ALL_STRONG": evaluate_tripod_samples(inds_a_all, TRIPOD_A_INDS),
        "ALL3_CONTACT": evaluate_tripod_samples(inds_a_all3, TRIPOD_A_INDS),
        "LT3_CONTACT": evaluate_tripod_samples(inds_a_lt3, TRIPOD_A_INDS),
    }

    # Tripod B
    inds_b_all = np.where(mask_strong_b)[0]
    inds_b_all3 = np.where(mask_strong_b & all3_b)[0]
    inds_b_lt3 = np.where(mask_strong_b & ~all3_b)[0]

    trip_b_metrics = {
        "ALL_STRONG": evaluate_tripod_samples(inds_b_all, TRIPOD_B_INDS),
        "ALL3_CONTACT": evaluate_tripod_samples(inds_b_all3, TRIPOD_B_INDS),
        "LT3_CONTACT": evaluate_tripod_samples(inds_b_lt3, TRIPOD_B_INDS),
    }

    # Aggregated metrics (combining samples from both tripods)
    def compute_aggregated_stats(inds_a, inds_b):
        if len(inds_a) == 0 and len(inds_b) == 0:
            return {"sample_count": 0, "cancellation_median": 0.0, "opposing_fraction": 0.0, "efficiency_median": 0.0}

        canc_list = []
        eff_list = []
        opp_list = []
        foot_range_list = []
        foot_std_list = []
        targ_range_list = []
        targ_std_list = []

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
                foot_std_list.extend(np.std(s_f_vx, axis=1))

                s_t_vx = targ_vx_all[inds][:, legs]
                targ_range_list.extend(np.ptp(s_t_vx, axis=1))
                targ_std_list.extend(np.std(s_t_vx, axis=1))

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
            "efficiency_p05": float(np.percentile(eff_arr, 5)),
            "efficiency_p95": float(np.percentile(eff_arr, 95)),
            "opposing_fraction": float(np.mean(opp_arr)),
            "foot_vx_range_median": float(np.median(foot_range_list)),
            "foot_vx_std_median": float(np.median(foot_std_list)),
            "targ_vx_range_median": float(np.median(targ_range_list)),
            "targ_vx_std_median": float(np.median(targ_std_list))
        }

    agg_all3 = compute_aggregated_stats(inds_a_all3, inds_b_all3)
    agg_lt3 = compute_aggregated_stats(inds_a_lt3, inds_b_lt3)
    agg_all_strong = compute_aggregated_stats(inds_a_all, inds_b_all)

    # 5. Full Per-Leg World GRF Statistics during Strong Support
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
            "MEDIAN_ABS_FOOT_WORLD_VX": float(np.median(np.abs(foot_vx_all[sup_mask, l_idx]))) if np.sum(sup_mask) > 0 else 0.0,
            "MEDIAN_TARGET_REL_VX": float(np.median(targ_vx_all[sup_mask, l_idx])) if np.sum(sup_mask) > 0 else 0.0
        }

    # 6. Load Distribution across Anatomical Classes
    # Positive vertical load fraction: max(Fz_i, 0) / sum(max(Fz_j, 0))
    pos_fz_matrix = np.clip(ss_grf[:, :, 2], 0.0, None)  # (N_ss, 6)
    tot_pos_fz = np.sum(pos_fz_matrix, axis=1, keepdims=True)  # (N_ss, 1)
    load_fractions = pos_fz_matrix / (tot_pos_fz + 1e-9)  # (N_ss, 6)

    # For each strong-support sample, identify load fraction of front, mid, hind support legs
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
            "median": float(np.median(front_loads)),
            "p05": float(np.percentile(front_loads, 5)),
            "p95": float(np.percentile(front_loads, 95))
        },
        "MIDDLE": {
            "median": float(np.median(mid_loads)),
            "p05": float(np.percentile(mid_loads, 5)),
            "p95": float(np.percentile(mid_loads, 95))
        },
        "HIND": {
            "median": float(np.median(hind_loads)),
            "p05": float(np.percentile(hind_loads, 5)),
            "p95": float(np.percentile(hind_loads, 95))
        }
    }

    # 7. Speed Minima / Hesitation Event Detection
    # Backward speed is s(t) = -vx_20. Speed minima correspond to peaks of vx_20.
    min_dist_samples = int(0.040 / sample_dt)
    peaks, props = find_peaks(vx_20, distance=min_dist_samples, prominence=0.5)

    speed_minima_events = []
    window_half_samples = int(0.030 / sample_dt)  # +-30 ms

    # Precompute sample-level cancellation ratio across all strong support samples
    canc_all_samples = np.full(len(ss_t), np.nan)
    for k in np.where(mask_strong_a)[0]:
        fx = ss_grf[k, TRIPOD_A_INDS, 0]
        canc_all_samples[k] = 1.0 - (abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))
    for k in np.where(mask_strong_b)[0]:
        fx = ss_grf[k, TRIPOD_B_INDS, 0]
        canc_all_samples[k] = 1.0 - (abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))

    # Event-triggered profiles (length 2*window_half_samples + 1)
    ev_time_axis = np.arange(-window_half_samples, window_half_samples + 1) * (sample_dt * 1000.0)
    ev_vx_traces = []
    ev_canc_traces = []
    ev_fx_front_traces = []
    ev_fx_mid_traces = []
    ev_fx_hind_traces = []
    ev_pitch_traces = []
    ev_contact_count_traces = []

    speed_minima_samples = set()

    for p_idx in peaks:
        t_peak = ss_t[p_idx]
        speed_at_dip = -vx_20[p_idx]

        # Samples within +-10 ms around minima for load comparison
        for offset in range(-int(0.010/sample_dt), int(0.010/sample_dt) + 1):
            idx = p_idx + offset
            if 0 <= idx < len(ss_t):
                speed_minima_samples.add(idx)

        # Full +-30 ms window
        w_start = p_idx - window_half_samples
        w_end = p_idx + window_half_samples + 1
        if w_start >= 0 and w_end <= len(ss_t):
            ev_vx_traces.append(vx_20[w_start:w_end])
            ev_canc_traces.append(canc_all_samples[w_start:w_end])
            ev_pitch_traces.append(ss_pitch[w_start:w_end])

            # Front, mid, hind Fx
            f_fx = np.mean(ss_grf[w_start:w_end, CLASS_TO_LEGS["FRONT"], 0], axis=1)
            m_fx = np.mean(ss_grf[w_start:w_end, CLASS_TO_LEGS["MIDDLE"], 0], axis=1)
            h_fx = np.mean(ss_grf[w_start:w_end, CLASS_TO_LEGS["HIND"], 0], axis=1)
            ev_fx_front_traces.append(f_fx)
            ev_fx_mid_traces.append(m_fx)
            ev_fx_hind_traces.append(h_fx)

            c_count = np.sum(ss_cont[w_start:w_end] > 0, axis=1)
            ev_contact_count_traces.append(c_count)

            speed_minima_events.append({
                "time_s": float(t_peak),
                "backward_speed_mm_s": float(speed_at_dip),
                "peak_vx_20_mm_s": float(vx_20[p_idx]),
                "cancellation_at_dip": float(canc_all_samples[p_idx]) if not np.isnan(canc_all_samples[p_idx]) else None
            })

    speed_minima_indices = sorted(list(speed_minima_samples))

    # Event-triggered averages
    mean_ev_vx = np.mean(ev_vx_traces, axis=0).tolist() if ev_vx_traces else []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        mean_ev_canc = np.nanmean(ev_canc_traces, axis=0).tolist() if ev_canc_traces else []
    mean_ev_pitch = np.mean(ev_pitch_traces, axis=0).tolist() if ev_pitch_traces else []
    mean_ev_fx_front = np.mean(ev_fx_front_traces, axis=0).tolist() if ev_fx_front_traces else []
    mean_ev_fx_mid = np.mean(ev_fx_mid_traces, axis=0).tolist() if ev_fx_mid_traces else []
    mean_ev_fx_hind = np.mean(ev_fx_hind_traces, axis=0).tolist() if ev_fx_hind_traces else []
    mean_ev_cont_count = np.mean(ev_contact_count_traces, axis=0).tolist() if ev_contact_count_traces else []

    # 8. Dominant Load Class during Motion Regimes
    def get_dominant_load_class(subset_indices):
        valid = [idx for idx in subset_indices if (mask_strong_a[idx] or mask_strong_b[idx])]
        if not valid:
            return "NONE", {"FRONT": 0.0, "MIDDLE": 0.0, "HIND": 0.0}

        f_loads, m_loads, h_loads = [], [], []
        for idx in valid:
            if mask_strong_a[idx]:
                f_loads.append(load_fractions[idx, 0])
                m_loads.append(load_fractions[idx, 4])
                h_loads.append(load_fractions[idx, 2])
            else:
                f_loads.append(load_fractions[idx, 3])
                m_loads.append(load_fractions[idx, 1])
                h_loads.append(load_fractions[idx, 5])

        meds = {
            "FRONT": float(np.median(f_loads)),
            "MIDDLE": float(np.median(m_loads)),
            "HIND": float(np.median(h_loads))
        }
        dom = max(meds, key=meds.get)
        return dom, meds

    correct_sign_indices = np.where(vx_20 <= 0)[0]
    wrong_sign_indices = np.where(vx_20 > 0)[0]

    dom_normal, loads_normal = get_dominant_load_class(correct_sign_indices)
    dom_minima, loads_minima = get_dominant_load_class(speed_minima_indices)
    dom_wrong, loads_wrong = get_dominant_load_class(wrong_sign_indices)

    # 9. Wrong-Sign vs Correct-Sign Conditioning
    def evaluate_sign_group(subset_indices):
        valid_a = [i for i in subset_indices if mask_strong_a[i]]
        valid_b = [i for i in subset_indices if mask_strong_b[i]]

        canc_vals, eff_vals, opp_vals = [], [], []
        c_counts = []
        pitches = ss_pitch[subset_indices] if len(subset_indices) > 0 else []

        for inds, legs in [(valid_a, TRIPOD_A_INDS), (valid_b, TRIPOD_B_INDS)]:
            if len(inds) > 0:
                s_grf = ss_grf[inds][:, legs, :]
                s_fx = s_grf[:, :, 0]
                s_fz = fz_pos_total[inds]
                f_net = np.sum(s_fx, axis=1)
                f_abs = np.sum(np.abs(s_fx), axis=1)
                canc_vals.extend(1.0 - (np.abs(f_net) / (f_abs + 1e-9)))
                eff_vals.extend(np.abs(f_net) / (f_abs + 1e-9))
                th = PRE_REG_FORCE_THRESHOLD_FRAC * s_fz
                opp_vals.extend((np.max(s_fx, axis=1) >= th) & (np.min(s_fx, axis=1) <= -th))
                c_counts.extend(np.sum(ss_cont[inds][:, legs] > 0, axis=1))

        if len(canc_vals) == 0:
            return {
                "sample_count": 0,
                "cancellation_median": 0.0,
                "efficiency_median": 0.0,
                "opposing_fraction": 0.0,
                "mean_support_contact_count": 0.0,
                "all3_fraction": 0.0,
                "pitch_median_deg": 0.0,
                "pitch_std_deg": 0.0
            }

        return {
            "sample_count": int(len(canc_vals)),
            "cancellation_median": float(np.median(canc_vals)),
            "cancellation_p05": float(np.percentile(canc_vals, 5)),
            "cancellation_p95": float(np.percentile(canc_vals, 95)),
            "efficiency_median": float(np.median(eff_vals)),
            "opposing_fraction": float(np.mean(opp_vals)),
            "mean_support_contact_count": float(np.mean(c_counts)),
            "all3_fraction": float(np.mean(np.array(c_counts) == 3)),
            "pitch_median_deg": float(np.median(pitches)),
            "pitch_std_deg": float(np.std(pitches))
        }

    correct_sign_stats = evaluate_sign_group(correct_sign_indices)
    wrong_sign_stats = evaluate_sign_group(wrong_sign_indices)

    # Telemetry and integrity checks
    t_diffs = telem["step_target_diffs"]
    max_target_step = float(np.max(t_diffs))
    p99_target_step = float(np.percentile(t_diffs, 99))
    sat_matrix = np.abs(telem["actuator_forces"]) >= 0.999 * telem["actuator_limits"][0]["upper_limit"]
    saturation_frac = float(np.mean(sat_matrix))

    return {
        "condition_name": telem["condition_name"],
        "base_metrics": {
            "MEAN_BACKWARD_SPEED_MM_S": mean_speed,
            "BACKWARD_DISPLACEMENT_MM": disp_x,
            "BACKWARD_SPEED_CV_50MS": cv50,
            "BACKWARD_SPEED_CV_20MS": cv20,
            "PITCH_PEAK_TO_PEAK_DEG": pitch_ptp,
            "PITCH_STD_DEG": pitch_std,
            "THORAX_Z_PEAK_TO_PEAK_MM": z_ptp,
            "THORAX_Z_STD_MM": z_std,
            "LATERAL_RATIO": lateral_ratio,
            "BODY_WRONG_SIGN_FRACTION_20MS": wrong_sign_20,
            "ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE": all3_aggregate_fraction,
            "ALL3_SUPPORT_FRACTION_TRIPOD_A": all3_support_fraction_a,
            "ALL3_SUPPORT_FRACTION_TRIPOD_B": all3_support_fraction_b,
            "PLATEAU_SAMPLES_TOTAL": total_plateau_samples,
            "ALL3_SAMPLES_TOTAL": total_all3_samples,
            "MAX_TARGET_STEP_RAD": max_target_step,
            "ACTUATOR_SATURATION_FRACTION": saturation_frac
        },
        "telemetry_integrity": {
            "TELEMETRY_VALID": telemetry_valid,
            "FORCES_FINITE": forces_finite,
            "FZ_POSITIVE_TOTAL": fz_positive_all_samples,
            "MIN_TOTAL_POSITIVE_FZ_MN": float(np.min(fz_pos_total)),
            "MEDIAN_TOTAL_POSITIVE_FZ_MN": float(np.median(fz_pos_total))
        },
        "support_conflict_by_tripod": {
            "TRIPOD_A": trip_a_metrics,
            "TRIPOD_B": trip_b_metrics
        },
        "support_conflict_aggregate": {
            "ALL3_CONTACT": agg_all3,
            "LT3_CONTACT": agg_lt3,
            "ALL_STRONG": agg_all_strong
        },
        "per_leg_grf_world": per_leg_grf_stats,
        "load_distribution": {
            "support_leg_loads": load_dist_stats,
            "dominant_normal_motion": {"class": dom_normal, "loads": loads_normal},
            "dominant_speed_minima": {"class": dom_minima, "loads": loads_minima},
            "dominant_wrong_sign": {"class": dom_wrong, "loads": loads_wrong}
        },
        "speed_minima_hesitation": {
            "detected_events_count": len(speed_minima_events),
            "events": speed_minima_events,
            "event_triggered_time_ms": ev_time_axis.tolist(),
            "mean_vx_20_mm_s": mean_ev_vx,
            "mean_cancellation_ratio": mean_ev_canc,
            "mean_pitch_deg": mean_ev_pitch,
            "mean_fx_front_mn": mean_ev_fx_front,
            "mean_fx_mid_mn": mean_ev_fx_mid,
            "mean_fx_hind_mn": mean_ev_fx_hind,
            "mean_contact_count": mean_ev_cont_count
        },
        "wrong_sign_conditioning": {
            "CORRECT_SIGN_BODY_MOTION": correct_sign_stats,
            "WRONG_SIGN_BODY_MOTION": wrong_sign_stats
        },
        "ik_diagnostics": telem["ik_diagnostics"]
    }


# -----------------------------------------------------------------------------
# PLOT GENERATION
# -----------------------------------------------------------------------------

def generate_diagnostic_plots(telem_a, telem_b, res_a, res_b, out_dir="output/m26"):
    """Generates the four diagnostic figures requested for MW-M26."""
    p = pathlib.Path(out_dir)
    p.mkdir(parents=True, exist_ok=True)

    # 1. A-vs-B Force Cancellation Distributions during ALL3_CONTACT
    fig, ax = plt.subplots(figsize=(8, 5))
    canc_a = []
    canc_b = []

    # Collect sample-by-sample cancellation for Condition A
    for trip_name, trip_legs in [("A", TRIPOD_A_INDS), ("B", TRIPOD_B_INDS)]:
        mask_strong = (np.median(telem_a["adhesion"][:, trip_legs], axis=1) >= 0.90) & (telem_a["t"] >= STEADY_START_TIME)
        all3 = np.all(telem_a["contact"][:, trip_legs] > 0, axis=1)
        valid = np.where(mask_strong & all3)[0]
        if len(valid) > 0:
            fx = telem_a["grf_world"][valid][:, trip_legs, 0]
            c = 1.0 - (np.abs(np.sum(fx, axis=1)) / (np.sum(np.abs(fx), axis=1) + 1e-9))
            canc_a.extend(c)

    for trip_name, trip_legs in [("A", TRIPOD_A_INDS), ("B", TRIPOD_B_INDS)]:
        mask_strong = (np.median(telem_b["adhesion"][:, trip_legs], axis=1) >= 0.90) & (telem_b["t"] >= STEADY_START_TIME)
        all3 = np.all(telem_b["contact"][:, trip_legs] > 0, axis=1)
        valid = np.where(mask_strong & all3)[0]
        if len(valid) > 0:
            fx = telem_b["grf_world"][valid][:, trip_legs, 0]
            c = 1.0 - (np.abs(np.sum(fx, axis=1)) / (np.sum(np.abs(fx), axis=1) + 1e-9))
            canc_b.extend(c)

    bins = np.linspace(0, 1.0, 31)
    if canc_a:
        ax.hist(canc_a, bins=bins, alpha=0.5, density=True, label=f"Condition A (M8) - Med: {np.median(canc_a):.3f}", color="blue")
    if canc_b:
        ax.hist(canc_b, bins=bins, alpha=0.5, density=True, label=f"Condition B (Planar) - Med: {np.median(canc_b):.3f}", color="red")

    ax.set_xlabel("Force Cancellation Ratio (1 - |Net Fx| / Sum |Fx|)")
    ax.set_ylabel("Probability Density")
    ax.set_title("Distribution of Internal Horizontal Force Cancellation (ALL3 Contact)")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(p / "cancellation_distributions.png", dpi=150)
    plt.close(fig)

    # 2. Event-Triggered Minima Dynamics (+-30 ms around speed dips)
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    t_ev = res_b["speed_minima_hesitation"]["event_triggered_time_ms"]
    if t_ev:
        axes[0].plot(t_ev, res_b["speed_minima_hesitation"]["mean_vx_20_mm_s"], "k-", lw=2, label="Body Vx (20ms)")
        axes[0].axvline(0, color="gray", linestyle="--", alpha=0.7)
        axes[0].set_ylabel("Body Vx (mm/s)\n(<0 is backward)")
        axes[0].set_title("Event-Triggered Averages around Condition B Speed Minima (Dip at t=0)")
        axes[0].legend(loc="upper left")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(t_ev, res_b["speed_minima_hesitation"]["mean_cancellation_ratio"], "r-", lw=2, label="Force Cancellation Ratio")
        axes[1].axvline(0, color="gray", linestyle="--", alpha=0.7)
        axes[1].set_ylabel("Cancellation Ratio")
        axes[1].legend(loc="upper left")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(t_ev, res_b["speed_minima_hesitation"]["mean_fx_front_mn"], "b-", label="Front Legs Fx")
        axes[2].plot(t_ev, res_b["speed_minima_hesitation"]["mean_fx_mid_mn"], "g-", label="Middle Legs Fx")
        axes[2].plot(t_ev, res_b["speed_minima_hesitation"]["mean_fx_hind_mn"], "m-", label="Hind Legs Fx")
        axes[2].axvline(0, color="gray", linestyle="--", alpha=0.7)
        axes[2].axhline(0, color="black", linestyle=":", alpha=0.5)
        axes[2].set_xlabel("Time Relative to Backward-Speed Minimum (ms)")
        axes[2].set_ylabel("Ground Fx (mN)\n(<0 backward, >0 forward)")
        axes[2].legend(loc="upper left")
        axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(p / "event_triggered_minima.png", dpi=150)
    plt.close(fig)

    # 3. Body Vx vs Force Cancellation Scatter / Phase Map
    fig, ax = plt.subplots(figsize=(8, 6))
    ss_mask_b = telem_b["t"] >= STEADY_START_TIME
    vx_b = telem_b["vx"][ss_mask_b]
    # Compute 20ms boxcar
    w20 = max(1, int(0.020 / telem_b["sample_dt"]))
    vx_b_20 = np.convolve(vx_b, np.ones(w20)/w20, mode="same")

    canc_b_all = []
    vx_b_valid = []
    for k, idx in enumerate(np.where(ss_mask_b)[0]):
        adh = telem_b["adhesion"][idx]
        is_a = (np.median(adh[TRIPOD_A_INDS]) >= 0.90) and (np.median(adh[TRIPOD_B_INDS]) <= 0.10)
        is_b = (np.median(adh[TRIPOD_B_INDS]) >= 0.90) and (np.median(adh[TRIPOD_A_INDS]) <= 0.10)
        if is_a or is_b:
            legs = TRIPOD_A_INDS if is_a else TRIPOD_B_INDS
            fx = telem_b["grf_world"][idx, legs, 0]
            c = 1.0 - (np.abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))
            canc_b_all.append(c)
            vx_b_valid.append(vx_b_20[k])

    if canc_b_all:
        ax.scatter(-np.array(vx_b_valid), canc_b_all, alpha=0.4, color="crimson", edgecolors="none", s=20)
        ax.set_xlabel("Backward Speed (-Vx_20, mm/s)")
        ax.set_ylabel("Force Cancellation Ratio")
        ax.set_title("Condition B: Instantaneous Cancellation vs Backward Speed")
        ax.axvline(0, color="black", linestyle="--", alpha=0.6, label="Speed = 0 (Wrong-Sign Boundary)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(p / "vx_vs_cancellation.png", dpi=150)
    plt.close(fig)

    # 4. Force Cancellation vs Normalized Support Phase
    fig, ax = plt.subplots(figsize=(8, 5))
    phase_all = (telem_b["t"] * 6.0 * 2.0) % 2.0  # 0 to 2 for complete stride
    n_bins = 20
    bin_edges = np.linspace(0, 1.0, n_bins + 1)
    phase_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])

    phase_canc_a = [[] for _ in range(n_bins)]
    phase_canc_b = [[] for _ in range(n_bins)]

    for telem, phase_canc in [(telem_a, phase_canc_a), (telem_b, phase_canc_b)]:
        ss_m = telem["t"] >= STEADY_START_TIME
        for idx in np.where(ss_m)[0]:
            adh = telem["adhesion"][idx]
            is_a = (np.median(adh[TRIPOD_A_INDS]) >= 0.90) and (np.median(adh[TRIPOD_B_INDS]) <= 0.10)
            is_b = (np.median(adh[TRIPOD_B_INDS]) >= 0.90) and (np.median(adh[TRIPOD_A_INDS]) <= 0.10)
            if is_a or is_b:
                legs = TRIPOD_A_INDS if is_a else TRIPOD_B_INDS
                fx = telem["grf_world"][idx, legs, 0]
                c = 1.0 - (np.abs(np.sum(fx)) / (np.sum(np.abs(fx)) + 1e-9))
                # Local support phase within tripod cycle (0 to 1)
                t_val = telem["t"][idx]
                norm_p = (t_val * 6.0 * 2.0) % 1.0
                bin_i = int(np.clip(norm_p * n_bins, 0, n_bins - 1))
                phase_canc[bin_i].append(c)

    med_canc_a = [np.median(vals) if vals else np.nan for vals in phase_canc_a]
    med_canc_b = [np.median(vals) if vals else np.nan for vals in phase_canc_b]

    ax.plot(phase_centers, med_canc_a, "b-o", lw=2, label="Condition A (M8)")
    ax.plot(phase_centers, med_canc_b, "r-s", lw=2, label="Condition B (Planar Support)")
    ax.set_xlabel("Normalized Support Stance Phase (0: Start of Stance, 1: End)")
    ax.set_ylabel("Median Force Cancellation Ratio")
    ax.set_title("Force Cancellation Ratio Across Stance Phase")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(p / "cancellation_vs_phase.png", dpi=150)
    plt.close(fig)

    print(f"Generated 4 diagnostic plots in {out_dir}/ ✓")


# -----------------------------------------------------------------------------
# HYPOTHESIS EVALUATION & CLASSIFICATION
# -----------------------------------------------------------------------------

def evaluate_diagnostic_hypothesis(res_a, res_b, repro_passed):
    """
    Evaluates the primary diagnostic hypothesis against pre-registered criteria:
    Hypothesis:
    "M25-B's enforced three-leg support increases internal horizontal force
    conflict compared with M8, and this conflict is associated with its
    worse velocity pulsation / wrong-sign motion."
    """
    a_med_canc = res_a["support_conflict_aggregate"]["ALL3_CONTACT"]["cancellation_median"]
    b_med_canc = res_b["support_conflict_aggregate"]["ALL3_CONTACT"]["cancellation_median"]

    a_opp_frac = res_a["support_conflict_aggregate"]["ALL3_CONTACT"]["opposing_fraction"]
    b_opp_frac = res_b["support_conflict_aggregate"]["ALL3_CONTACT"]["opposing_fraction"]

    b_ws_canc = res_b["wrong_sign_conditioning"]["WRONG_SIGN_BODY_MOTION"]["cancellation_median"]
    b_cs_canc = res_b["wrong_sign_conditioning"]["CORRECT_SIGN_BODY_MOTION"]["cancellation_median"]

    # Per-tripod cancellation differences
    canc_a_trip_a = res_a["support_conflict_by_tripod"]["TRIPOD_A"]["ALL3_CONTACT"]["cancellation_median"]
    canc_b_trip_a = res_b["support_conflict_by_tripod"]["TRIPOD_A"]["ALL3_CONTACT"]["cancellation_median"]
    canc_a_trip_b = res_a["support_conflict_by_tripod"]["TRIPOD_B"]["ALL3_CONTACT"]["cancellation_median"]
    canc_b_trip_b = res_b["support_conflict_by_tripod"]["TRIPOD_B"]["ALL3_CONTACT"]["cancellation_median"]

    tripod_a_elevated = (canc_b_trip_a >= canc_a_trip_a + 0.05)
    tripod_b_elevated = (canc_b_trip_b >= canc_a_trip_b + 0.05)
    both_tripods_elevated = tripod_a_elevated and tripod_b_elevated

    # Criteria evaluation
    crit_1_canc_increase = bool(b_med_canc >= a_med_canc + PRE_REG_EFFECT_THRESHOLD)
    crit_2_opp_increase = bool(b_opp_frac >= a_opp_frac + PRE_REG_EFFECT_THRESHOLD)
    crit_3_pulse_association = bool(b_ws_canc >= b_cs_canc + 0.05)
    crit_4_both_tripods = bool(both_tripods_elevated)
    crit_5_integrity = bool(
        repro_passed and
        res_a["telemetry_integrity"]["TELEMETRY_VALID"] and
        res_b["telemetry_integrity"]["TELEMETRY_VALID"]
    )

    criteria = {
        "1_cancellation_meaningfully_higher_all3": crit_1_canc_increase,
        "2_opposing_force_fraction_higher": crit_2_opp_increase,
        "3_wrong_sign_cancellation_elevated": crit_3_pulse_association,
        "4_both_tripods_show_conflict": crit_4_both_tripods,
        "5_reproduction_and_integrity_pass": crit_5_integrity
    }

    if not crit_5_integrity:
        classification = "INVALID_DIAGNOSTIC"
    elif crit_1_canc_increase and crit_2_opp_increase and crit_3_pulse_association and crit_4_both_tripods:
        classification = "SUPPORT_CONFLICT_SUPPORTED"
    elif crit_1_canc_increase:
        classification = "SUPPORT_CONFLICT_PARTIALLY_SUPPORTED"
    else:
        classification = "SUPPORT_CONFLICT_NOT_SUPPORTED"

    # Identify dominant conflict leg or class
    # Compare per-leg median Fx in Condition B during strong support
    fx_b = res_b["per_leg_grf_world"]
    # Legs pushing forward (+Fx, resisting backward moonwalk)
    opposing_legs = [leg for leg, st in fx_b.items() if st["MEDIAN_FX_WORLD"] > 0]

    return {
        "classification": classification,
        "criteria": criteria,
        "delta_cancellation_all3": float(b_med_canc - a_med_canc),
        "delta_opposing_fraction_all3": float(b_opp_frac - a_opp_frac),
        "b_wrong_vs_correct_canc_diff": float(b_ws_canc - b_cs_canc),
        "tripod_a_result": f"A: {canc_a_trip_a:.3f} -> B: {canc_b_trip_a:.3f} (Δ={canc_b_trip_a - canc_a_trip_a:+.3f})",
        "tripod_b_result": f"A: {canc_a_trip_b:.3f} -> B: {canc_b_trip_b:.3f} (Δ={canc_b_trip_b - canc_a_trip_b:+.3f})",
        "dominant_opposing_legs": opposing_legs
    }


# -----------------------------------------------------------------------------
# MAIN EXPERIMENT EXECUTION
# -----------------------------------------------------------------------------

def main():
    print("=" * 80)
    print("MW-M26: SUPPORT FORCE CONFLICT DIAGNOSTIC EXPERIMENT")
    print("=" * 80)

    # 1. Preflight Audits
    m8_params = run_preflight_provenance_audit()
    print("M8 Authoritative Parameter Provenance Verified ✓")

    fly_pre = make_locomotion_fly("fly_pre", add_adhesion=True)
    sim_pre = m25.create_isolated_sim(fly_pre)
    dof_map_pre = m25.build_semantic_dof_mapping(fly_pre, sim_pre)
    sim_pre.reset()
    sim_pre.warmup()
    m25.verify_scratch_mjdata_isolation(fly_pre, sim_pre, dof_map_pre)
    print("Scratch MjData Physics State Non-Mutation Verified: NO live mutation ✓")
    sim_pre.close()

    pps = PreprogrammedSteps()

    # 2. RUN CONDITION A: DIRECT_M8_CONTROL
    print("\n" + "-"*80)
    print("STEP 1: RUNNING CONDITION A (DIRECT_M8_CONTROL)...")
    print("-" * 80)
    def build_ctrl_a(sim, fly):
        act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
        return m8.BackslideTrajectoryController(pps, act_dofs, **m8_params)

    telem_a = run_diagnostic_simulation("DIRECT_M8_CONTROL", build_ctrl_a)
    res_a = analyze_support_conflict(telem_a)

    base_a = res_a["base_metrics"]
    print(f"\nCondition A Reproduction Checks:")
    print(f"  Speed:        {base_a['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s (expected ~9.93)")
    print(f"  CV50:         {base_a['BACKWARD_SPEED_CV_50MS']:.3f} (expected ~0.230)")
    print(f"  Pitch PTP:    {base_a['PITCH_PEAK_TO_PEAK_DEG']:.2f}° (expected ~8.38°)")
    print(f"  All-3 Contact:{base_a['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} (expected ~0.616)")

    repro_a_checks = {
        "speed_in_bounds": (9.60 <= base_a["MEAN_BACKWARD_SPEED_MM_S"] <= 10.30),
        "cv50_in_bounds": (0.200 <= base_a["BACKWARD_SPEED_CV_50MS"] <= 0.260),
        "pitch_in_bounds": (7.80 <= base_a["PITCH_PEAK_TO_PEAK_DEG"] <= 9.00),
        "all3_in_bounds": (0.570 <= base_a["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"] <= 0.660)
    }
    repro_a_pass = all(repro_a_checks.values())
    if not repro_a_pass:
        print("\nSTOP: M26_PARENT_REPRODUCTION_FAILED (Condition A failed reproduction)")
        print("Checks:", repro_a_checks)
        sys.exit(1)
    print("Condition A Reproduction: PASSED ✓")

    # 3. DERIVE AND FREEZE SUPPORT HEIGHTS FROM FRESH CONDITION A
    print("\n" + "-"*80)
    print("STEP 2: DERIVING AND FREEZING SUPPORT HEIGHTS FROM CONDITION A...")
    print("-" * 80)
    height_data = m25.derive_fresh_condition_a_support_heights(telem_a)
    frozen_body_heights = height_data["body_support_height_median_mm"]

    # 4. RUN CONDITION B: M25_BODY_FRAME_PLANAR_SUPPORT
    print("\n" + "-"*80)
    print("STEP 3: RUNNING CONDITION B (M25_BODY_FRAME_PLANAR_SUPPORT)...")
    print("-" * 80)
    def build_ctrl_b(sim, fly):
        act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
        dof_map = m25.build_semantic_dof_mapping(fly, sim)
        return m25.M25SupportGeometryController(
            sim, fly, pps, act_dofs, dof_map,
            mode="BODY_FRAME_PLANAR",
            frozen_heights=frozen_body_heights,
            m8_params=m8_params
        )

    telem_b = run_diagnostic_simulation("M25_BODY_FRAME_PLANAR_SUPPORT", build_ctrl_b)
    res_b = analyze_support_conflict(telem_b)

    base_b = res_b["base_metrics"]
    print(f"\nCondition B Reproduction Checks:")
    print(f"  Speed:        {base_b['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s (expected ~10.40)")
    print(f"  CV50:         {base_b['BACKWARD_SPEED_CV_50MS']:.3f} (expected ~0.328)")
    print(f"  Pitch PTP:    {base_b['PITCH_PEAK_TO_PEAK_DEG']:.2f}° (expected ~8.57°)")
    print(f"  All-3 Contact:{base_b['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} (expected ~0.942)")

    repro_b_checks = {
        "speed_in_bounds": (10.00 <= base_b["MEAN_BACKWARD_SPEED_MM_S"] <= 10.80),
        "cv50_in_bounds": (0.290 <= base_b["BACKWARD_SPEED_CV_50MS"] <= 0.370),
        "pitch_in_bounds": (8.00 <= base_b["PITCH_PEAK_TO_PEAK_DEG"] <= 9.20),
        "all3_in_bounds": (0.900 <= base_b["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"] <= 0.980)
    }
    repro_b_pass = all(repro_b_checks.values())
    if not repro_b_pass:
        print("\nSTOP: M26_PARENT_REPRODUCTION_FAILED (Condition B failed reproduction)")
        print("Checks:", repro_b_checks)
        sys.exit(1)
    print("Condition B Reproduction: PASSED ✓")

    repro_overall = repro_a_pass and repro_b_pass

    # 5. DIAGNOSTIC EVALUATION
    print("\n" + "-"*80)
    print("STEP 4: EVALUATING SUPPORT FORCE CONFLICT HYPOTHESIS...")
    print("-" * 80)
    diag_eval = evaluate_diagnostic_hypothesis(res_a, res_b, repro_overall)

    # 6. PLOT GENERATION
    print("\n" + "-"*80)
    print("STEP 5: GENERATING DIAGNOSTIC PLOTS...")
    print("-" * 80)
    generate_diagnostic_plots(telem_a, telem_b, res_a, res_b, "output/m26")

    # 7. ASSEMBLE RESULTS PAYLOAD
    results_payload = {
        "milestone": "MW-M26",
        "description": "Diagnose internal mechanical conflict within the supporting tripod",
        "provenance_check": "PASSED",
        "reproduction": {
            "overall_status": "PASS" if repro_overall else "FAIL",
            "condition_a_checks": repro_a_checks,
            "condition_b_checks": repro_b_checks
        },
        "pre_registered_thresholds": {
            "force_cancellation_increase": PRE_REG_EFFECT_THRESHOLD,
            "opposing_force_load_fraction": PRE_REG_FORCE_THRESHOLD_FRAC
        },
        "condition_a": res_a,
        "condition_b": res_b,
        "diagnostic_hypothesis_evaluation": diag_eval
    }

    manifest_payload = {
        "milestone": "MW-M26",
        "task": "Diagnose internal mechanical conflict within the supporting tripod",
        "reproduction_gate": "PASS" if repro_overall else "FAIL",
        "force_telemetry_valid": "YES" if (res_a["telemetry_integrity"]["TELEMETRY_VALID"] and res_b["telemetry_integrity"]["TELEMETRY_VALID"]) else "NO",
        "A_MEDIAN_CANCELLATION_ALL3": res_a["support_conflict_aggregate"]["ALL3_CONTACT"]["cancellation_median"],
        "B_MEDIAN_CANCELLATION_ALL3": res_b["support_conflict_aggregate"]["ALL3_CONTACT"]["cancellation_median"],
        "A_OPPOSING_FORCE_FRACTION": res_a["support_conflict_aggregate"]["ALL3_CONTACT"]["opposing_fraction"],
        "B_OPPOSING_FORCE_FRACTION": res_b["support_conflict_aggregate"]["ALL3_CONTACT"]["opposing_fraction"],
        "B_WRONG_SIGN_CANCELLATION": res_b["wrong_sign_conditioning"]["WRONG_SIGN_BODY_MOTION"]["cancellation_median"],
        "B_CORRECT_SIGN_CANCELLATION": res_b["wrong_sign_conditioning"]["CORRECT_SIGN_BODY_MOTION"]["cancellation_median"],
        "TRIPOD_A_RESULT": diag_eval["tripod_a_result"],
        "TRIPOD_B_RESULT": diag_eval["tripod_b_result"],
        "DOMINANT_CONFLICT_LEG_OR_CLASS": ", ".join(diag_eval["dominant_opposing_legs"]) if diag_eval["dominant_opposing_legs"] else "NONE",
        "FINAL_CLASSIFICATION": diag_eval["classification"]
    }

    derived_dir = pathlib.Path("data/derived")
    derived_dir.mkdir(parents=True, exist_ok=True)
    with open(derived_dir / "m26_support_conflict_results.json", "w") as f:
        json.dump(results_payload, f, indent=2)
    with open(derived_dir / "m26_support_conflict_manifest.json", "w") as f:
        json.dump(manifest_payload, f, indent=2)

    print(f"\nResults written to: {derived_dir / 'm26_support_conflict_results.json'}")
    print(f"Manifest written to: {derived_dir / 'm26_support_conflict_manifest.json'}")

    print("\n" + "="*80)
    print("FINAL SUMMARY REPORT:")
    print("=" * 80)
    print(f"M26_REPRODUCTION: {'PASS' if repro_overall else 'FAIL'}")
    print(f"FORCE_TELEMETRY_VALID: {'YES' if manifest_payload['force_telemetry_valid'] == 'YES' else 'NO'}")
    print(f"A_MEDIAN_CANCELLATION_ALL3: {manifest_payload['A_MEDIAN_CANCELLATION_ALL3']:.4f}")
    print(f"B_MEDIAN_CANCELLATION_ALL3: {manifest_payload['B_MEDIAN_CANCELLATION_ALL3']:.4f}")
    print(f"A_OPPOSING_FORCE_FRACTION: {manifest_payload['A_OPPOSING_FORCE_FRACTION']:.4f}")
    print(f"B_OPPOSING_FORCE_FRACTION: {manifest_payload['B_OPPOSING_FORCE_FRACTION']:.4f}")
    print(f"B_WRONG_SIGN_CANCELLATION: {manifest_payload['B_WRONG_SIGN_CANCELLATION']:.4f}")
    print(f"B_CORRECT_SIGN_CANCELLATION: {manifest_payload['B_CORRECT_SIGN_CANCELLATION']:.4f}")
    print(f"TRIPOD_A_RESULT: {manifest_payload['TRIPOD_A_RESULT']}")
    print(f"TRIPOD_B_RESULT: {manifest_payload['TRIPOD_B_RESULT']}")
    print(f"DOMINANT_CONFLICT_LEG_OR_CLASS: {manifest_payload['DOMINANT_CONFLICT_LEG_OR_CLASS']}")
    print(f"FINAL_CLASSIFICATION: {manifest_payload['FINAL_CLASSIFICATION']}")
    print("=" * 80)


if __name__ == "__main__":
    main()
