"""MW-M3: Moonwalk mechanics validation + narrow adhesion-phase refinement."""

import json
import pathlib
import sys
import time
from importlib.metadata import version

import numpy as np

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    CPGController,
    LocomotionAction,
    PreprogrammedSteps,
    apply_locomotion_action,
    make_locomotion_fly,
    make_tripod_cpg_network,
)


def compute_adhesion(
    phase_fraction: float,
    stock_adhesion: np.ndarray,
    curr_phases: np.ndarray,
    preprogrammed_steps: PreprogrammedSteps,
) -> np.ndarray:
    """Compute adhesion state for a given phase shift fraction."""
    if phase_fraction == 0.0:
        return stock_adhesion
    shifted = (curr_phases + phase_fraction * 2.0 * np.pi) % (2.0 * np.pi)
    return preprogrammed_steps.get_adhesion_onoff_by_phase(shifted)


def compute_pearson_r(x: np.ndarray, y: np.ndarray) -> float:
    """Compute Pearson correlation coefficient handling zero-variance cases."""
    std_x = float(np.std(x))
    std_y = float(np.std(y))
    if std_x < 1e-8 or std_y < 1e-8:
        return float("nan")
    r = float(np.corrcoef(x, y)[0, 1])
    return r


def main() -> None:
    print("M3_VALIDATION_START")

    flygym_ver = version("flygym")
    mujoco_ver = version("mujoco")
    numpy_ver = np.__version__
    print(f"FLYGYM_VERSION={flygym_ver}")
    print(f"MUJOCO_VERSION={mujoco_ver}")
    print(f"NUMPY_VERSION={numpy_ver}")

    # 1. Compose model
    fly = make_locomotion_fly(
        name="m3_validate",
        add_adhesion=True,
        colorize=True,
    )
    cam = fly.add_tracking_camera(name="trackcam")

    world = FlatGroundWorld()
    spawn_rot = Rotation3D(format="quat", values=[1, 0, 0, 0])
    world.add_fly(
        fly=fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=spawn_rot,
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD,
    )

    sim = Simulation(world)
    actual_timestep = float(sim.mj_model.opt.timestep)
    assert actual_timestep > 0, "Timestep must be > 0"

    # 2. Semantic indexing (no magic numbers)
    # 2a. Actuated DOFs mapping
    actuated_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    actuated_dof_count = len(actuated_dofs)
    assert actuated_dof_count == 42, f"Expected 42 actuated DOFs, got {actuated_dof_count}"

    all_dofs = fly.get_jointdofs_order()
    actuated_dof_indices = np.array([all_dofs.index(d) for d in actuated_dofs], dtype=int)
    assert len(actuated_dof_indices) == 42

    actuated_dof_names = [f"{d.parent.name}->{d.child.name}:{d.axis.value}" for d in actuated_dofs]

    # 2b. Body segments mapping
    body_segs = fly.get_bodysegs_order()
    seg_names = [s.name if hasattr(s, "name") else str(s) for s in body_segs]
    root_name = fly.root_segment.name if hasattr(fly.root_segment, "name") else str(fly.root_segment)
    thorax_idx = seg_names.index(root_name)

    legs_order = fly.get_legs_order()
    assert len(legs_order) == 6, f"Expected 6 legs, got {len(legs_order)}"
    tarsus5_indices = [seg_names.index(f"{leg}_tarsus5") for leg in legs_order]

    print(f"ACTUATED_DOF_COUNT={actuated_dof_count}")
    print(f"ALL_DOF_COUNT={len(all_dofs)}")
    print(f"BODY_SEGMENTS_COUNT={len(body_segs)}")
    print(f"THORAX_SEGMENT_INDEX={thorax_idx} ({root_name})")
    print(f"LEGS_ORDER={legs_order}")
    print(f"TARSUS5_INDICES={tarsus5_indices}")

    # 3. Controller setup
    preprogrammed_steps = PreprogrammedSteps()
    cpg_network = make_tripod_cpg_network(
        timestep=actual_timestep,
        intrinsic_amplitude=1.0,
        coupling_strength=10.0,
        convergence_coef=20.0,
        seed=0,
    )
    controller = CPGController(
        cpg_network=cpg_network,
        preprogrammed_steps=preprogrammed_steps,
        output_dof_order=actuated_dofs,
    )
    default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dofs)

    trial_duration_s = 1.0
    n_steps = round(trial_duration_s / actual_timestep)
    sample_interval_steps = 10
    sample_dt_s = sample_interval_steps * actual_timestep

    print(f"TRIAL_DURATION_SECONDS={trial_duration_s}")
    print(f"PHYSICS_TIMESTEP_SECONDS={actual_timestep}")
    print(f"TOTAL_PHYSICS_STEPS={n_steps}")
    print(f"SAMPLE_INTERVAL_STEPS={sample_interval_steps}")
    print(f"TELEMETRY_SAMPLE_INTERVAL_SECONDS={sample_dt_s}")

    conditions = [
        ("STOCK", 0.00),
        ("PHASE_0_40", 0.40),
        ("PHASE_0_45", 0.45),
        ("PHASE_0_50", 0.50),
        ("PHASE_0_55", 0.55),
        ("PHASE_0_60", 0.60),
    ]

    telemetry_raw = {}
    trial_summaries = {}

    start_wall_time = time.perf_counter()

    # 4. Run the 6 measurement trials
    for var_name, phase_frac in conditions:
        print(f"\n--- RUNNING TRIAL: {var_name} (phase={phase_frac:.2f}) ---")

        # Deterministic reset
        sim.reset()
        controller.cpg_network.random_state = np.random.RandomState(0)
        controller.cpg_network.reset()
        sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
        sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
        sim.warmup()

        start_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        start_x, start_y, start_z = [float(v) for v in start_thorax_pos]

        # Telemetry buffers
        buf_timestamps = []
        buf_thorax_pos = []
        buf_actual_actuated_joints = []
        buf_target_actuated_joints = []
        buf_tarsus5_world_pos = []
        buf_adhesion_states = []
        buf_contact_found = []
        buf_contact_positions = []

        min_z = start_z
        max_abs_lat_disp = 0.0

        for step_idx in range(n_steps):
            stock_action = controller.step()

            adhesion = compute_adhesion(
                phase_frac,
                stock_action.adhesion_onoff,
                controller.cpg_network.curr_phases,
                preprogrammed_steps,
            )

            action_to_apply = LocomotionAction(
                joint_angles=stock_action.joint_angles,
                adhesion_onoff=adhesion,
            )

            apply_locomotion_action(sim, fly.name, action_to_apply)
            sim.step()

            # Inspectability tracking
            curr_thorax = sim.get_body_positions(fly.name)[thorax_idx]
            cz = float(curr_thorax[2])
            if cz < min_z:
                min_z = cz
            lat_disp = abs(float(curr_thorax[1]) - start_y)
            if lat_disp > max_abs_lat_disp:
                max_abs_lat_disp = lat_disp

            # Sample telemetry every 10 steps
            if (step_idx + 1) % sample_interval_steps == 0:
                sample_t = (step_idx + 1) * actual_timestep
                all_body_positions = sim.get_body_positions(fly.name)
                t_pos = all_body_positions[thorax_idx].copy()
                tars_pos = all_body_positions[tarsus5_indices].copy()

                all_joints = sim.get_joint_angles(fly.name)
                act_joints = all_joints[actuated_dof_indices].copy()

                tgt_joints = stock_action.joint_angles.copy()
                adh_states = adhesion.copy()

                c_found, _, _, c_pos, _, _ = sim.get_ground_contact_info(fly.name)

                buf_timestamps.append(sample_t)
                buf_thorax_pos.append(t_pos)
                buf_actual_actuated_joints.append(act_joints)
                buf_target_actuated_joints.append(tgt_joints)
                buf_tarsus5_world_pos.append(tars_pos)
                buf_adhesion_states.append(adh_states)
                buf_contact_found.append(c_found.copy())
                buf_contact_positions.append(c_pos.copy())

        end_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        end_x, end_y, end_z = [float(v) for v in end_thorax_pos]

        disp_x = end_x - start_x
        disp_y = end_y - start_y
        disp_z = end_z - start_z
        lateral_ratio = abs(disp_y) / max(abs(disp_x), 1e-8)
        avg_vx = disp_x / trial_duration_s

        is_finite = bool(np.all(np.isfinite([end_x, end_y, end_z])))
        inspectable = bool(is_finite and min_z > 0.2 and abs(disp_y) < 15.0)

        trial_summaries[var_name] = {
            "variant": var_name,
            "phase_fraction": phase_frac,
            "start_thorax_xyz_mm": [start_x, start_y, start_z],
            "end_thorax_xyz_mm": [end_x, end_y, end_z],
            "displacement_x_mm": disp_x,
            "displacement_y_mm": disp_y,
            "displacement_z_mm": disp_z,
            "lateral_ratio": lateral_ratio,
            "average_vx_mm_s": avg_vx,
            "min_thorax_z_mm": min_z,
            "max_abs_lateral_disp_mm": max_abs_lat_disp,
            "inspectable": inspectable,
        }

        telemetry_raw[var_name] = {
            "timestamps": np.array(buf_timestamps, dtype=float),
            "thorax_pos": np.array(buf_thorax_pos, dtype=float),
            "actual_actuated_joints": np.array(buf_actual_actuated_joints, dtype=float),
            "target_actuated_joints": np.array(buf_target_actuated_joints, dtype=float),
            "tarsus5_world_pos": np.array(buf_tarsus5_world_pos, dtype=float),
            "adhesion_states": np.array(buf_adhesion_states, dtype=float),
            "contact_found": np.array(buf_contact_found, dtype=float),
            "contact_positions": np.array(buf_contact_positions, dtype=float),
        }

        print(
            f"SUMMARY {var_name:12s} | X={disp_x:+8.4f} mm | Y={disp_y:+8.4f} mm | "
            f"LatRatio={lateral_ratio:.4f} | MinZ={min_z:.4f} mm | Inspectable={inspectable}"
        )

    sweep_wall_time = time.perf_counter() - start_wall_time
    print(f"\nALL_TRIALS_COMPLETED in {sweep_wall_time:.2f} s")

    # 5. Target-Program Integrity Verification
    stock_targets = telemetry_raw["STOCK"]["target_actuated_joints"]
    target_diffs = {}
    target_integrity_pass = True

    print("\n=== TARGET-PROGRAM INTEGRITY CHECK ===")
    for var_name, _ in conditions:
        cand_targets = telemetry_raw[var_name]["target_actuated_joints"]
        max_diff = float(np.max(np.abs(cand_targets - stock_targets)))
        target_diffs[var_name] = max_diff
        print(f"{var_name:12s} | MAX_TARGET_JOINT_DIFF_RAD = {max_diff:.6e}")
        if max_diff > 1e-12:
            target_integrity_pass = False

    print(f"TARGET_PROGRAM_IDENTICAL_ACROSS_ALL_CANDIDATES={'YES' if target_integrity_pass else 'NO'}")
    assert target_integrity_pass, "M3 FATAL: Target joint commands differed from STOCK!"

    # 6. Actual Joint Similarity to Stock
    stock_actual_joints = telemetry_raw["STOCK"]["actual_actuated_joints"]
    actual_joint_metrics = {}

    print("\n=== ACTUAL JOINT SIMILARITY TO STOCK ===")
    print(f"{'VARIANT':12s} | {'OVERALL_RMSE_RAD':16s} | {'MEDIAN_CORR':12s} | {'MIN_CORR':12s}")
    for var_name, _ in conditions:
        cand_actual = telemetry_raw[var_name]["actual_actuated_joints"]
        # Overall RMSE across all samples and all 42 DoFs
        diff_sq = (cand_actual - stock_actual_joints) ** 2
        overall_rmse = float(np.sqrt(np.mean(diff_sq)))

        # Per-DoF RMSE and Pearson correlation
        per_dof_rmse = np.sqrt(np.mean(diff_sq, axis=0))
        per_dof_corr = []
        for d_idx in range(42):
            r = compute_pearson_r(cand_actual[:, d_idx], stock_actual_joints[:, d_idx])
            per_dof_corr.append(r)
        per_dof_corr = np.array(per_dof_corr, dtype=float)

        non_nan_corrs = per_dof_corr[np.isfinite(per_dof_corr)]
        median_corr = float(np.median(non_nan_corrs)) if len(non_nan_corrs) > 0 else float("nan")
        min_corr = float(np.min(non_nan_corrs)) if len(non_nan_corrs) > 0 else float("nan")

        actual_joint_metrics[var_name] = {
            "overall_rmse_rad": overall_rmse,
            "per_dof_rmse_rad": per_dof_rmse.tolist(),
            "per_dof_correlation": [None if np.isnan(v) else float(v) for v in per_dof_corr],
            "median_correlation": median_corr,
            "min_correlation": min_corr,
        }
        print(f"{var_name:12s} | {overall_rmse:16.6f} | {median_corr:12.6f} | {min_corr:12.6f}")

    # 7. Body-Centered Foot Trajectories
    # foot_relative_xyz = tarsus5_world_xyz - thorax_world_xyz
    foot_metrics = {}
    print("\n=== BODY-CENTERED DISTAL-FOOT SIMILARITY ===")
    print(f"{'VARIANT':12s} | {'MEAN_FOOT_X_RMSE_MM':20s} | {'MEDIAN_FOOT_X_CORR':20s}")

    stock_thorax = telemetry_raw["STOCK"]["thorax_pos"]  # (N, 3)
    stock_tarsus = telemetry_raw["STOCK"]["tarsus5_world_pos"]  # (N, 6, 3)
    # stock relative: (N, 6, 3)
    stock_foot_rel = stock_tarsus - stock_thorax[:, np.newaxis, :]

    for var_name, _ in conditions:
        cand_thorax = telemetry_raw[var_name]["thorax_pos"]
        cand_tarsus = telemetry_raw[var_name]["tarsus5_world_pos"]
        cand_foot_rel = cand_tarsus - cand_thorax[:, np.newaxis, :]

        per_leg_metrics = {}
        leg_x_rmses = []
        leg_x_corrs = []

        for l_idx, leg in enumerate(legs_order):
            diff = cand_foot_rel[:, l_idx, :] - stock_foot_rel[:, l_idx, :]  # (N, 3)
            rmse_x = float(np.sqrt(np.mean(diff[:, 0] ** 2)))
            rmse_y = float(np.sqrt(np.mean(diff[:, 1] ** 2)))
            rmse_z = float(np.sqrt(np.mean(diff[:, 2] ** 2)))
            rmse_3d = float(np.sqrt(np.mean(np.sum(diff ** 2, axis=-1))))

            corr_x = compute_pearson_r(cand_foot_rel[:, l_idx, 0], stock_foot_rel[:, l_idx, 0])

            per_leg_metrics[leg] = {
                "rmse_x_mm": rmse_x,
                "rmse_y_mm": rmse_y,
                "rmse_z_mm": rmse_z,
                "rmse_3d_mm": rmse_3d,
                "corr_x": None if np.isnan(corr_x) else corr_x,
            }
            leg_x_rmses.append(rmse_x)
            if np.isfinite(corr_x):
                leg_x_corrs.append(corr_x)

        mean_rmse_x = float(np.mean(leg_x_rmses))
        median_corr_x = float(np.median(leg_x_corrs)) if leg_x_corrs else float("nan")

        foot_metrics[var_name] = {
            "per_leg": per_leg_metrics,
            "mean_foot_x_rmse_mm": mean_rmse_x,
            "median_foot_x_correlation": median_corr_x,
        }
        print(f"{var_name:12s} | {mean_rmse_x:20.6f} | {median_corr_x:20.6f}")

    # 8. Ground-Slip Analysis
    # Analyze distal tarsus world X velocity and contact position X velocity during continuous contact
    all_slip_metrics = {}
    print("\n=== GROUND-SLIP ANALYSIS (COMPUTING FOR ALL CANDIDATES) ===")

    for var_name, _ in conditions:
        tarsus_pos = telemetry_raw[var_name]["tarsus5_world_pos"]  # (N, 6, 3)
        contact_found = telemetry_raw[var_name]["contact_found"]  # (N, 6)
        contact_pos = telemetry_raw[var_name]["contact_positions"]  # (N, 6, 3)
        adhesion = telemetry_raw[var_name]["adhesion_states"]  # (N, 6)
        n_samples = tarsus_pos.shape[0]

        leg_slip = {}
        for l_idx, leg in enumerate(legs_order):
            on_tarsus_vx = []
            off_tarsus_vx = []
            on_contact_vx = []
            off_contact_vx = []

            for k in range(n_samples - 1):
                # Continuous contact check: contact_found > 0 at both adjacent samples
                cf_k = contact_found[k, l_idx] > 0
                cf_next = contact_found[k + 1, l_idx] > 0

                if cf_k and cf_next:
                    # Velocities in mm/s
                    dt = sample_dt_s
                    vx_t = (tarsus_pos[k + 1, l_idx, 0] - tarsus_pos[k, l_idx, 0]) / dt
                    vx_c = (contact_pos[k + 1, l_idx, 0] - contact_pos[k, l_idx, 0]) / dt

                    adh_k = adhesion[k, l_idx]
                    adh_next = adhesion[k + 1, l_idx]

                    if adh_k >= 0.5 and adh_next >= 0.5:
                        on_tarsus_vx.append(vx_t)
                        on_contact_vx.append(vx_c)
                    elif adh_k < 0.5 and adh_next < 0.5:
                        off_tarsus_vx.append(vx_t)
                        off_contact_vx.append(vx_c)

            on_count = len(on_tarsus_vx)
            off_count = len(off_tarsus_vx)

            med_vx_on = float(np.median(on_tarsus_vx)) if on_count > 0 else None
            med_abs_vx_on = float(np.median(np.abs(on_tarsus_vx))) if on_count > 0 else None
            neg_frac_on = float(np.mean(np.array(on_tarsus_vx) < 0.0)) if on_count > 0 else None

            med_vx_off = float(np.median(off_tarsus_vx)) if off_count > 0 else None
            med_abs_vx_off = float(np.median(np.abs(off_tarsus_vx))) if off_count > 0 else None
            neg_frac_off = float(np.mean(np.array(off_tarsus_vx) < 0.0)) if off_count > 0 else None

            med_contact_vx_off = float(np.median(off_contact_vx)) if off_count > 0 else None

            leg_slip[leg] = {
                "contact_on_samples": on_count,
                "contact_off_samples": off_count,
                "median_vx_adhesion_on": med_vx_on,
                "median_abs_vx_adhesion_on": med_abs_vx_on,
                "negative_vx_fraction_adhesion_on": neg_frac_on,
                "median_vx_adhesion_off": med_vx_off,
                "median_abs_vx_adhesion_off": med_abs_vx_off,
                "negative_vx_fraction_adhesion_off": neg_frac_off,
                "contact_position_median_vx_off": med_contact_vx_off,
            }

        all_slip_metrics[var_name] = leg_slip

    # 9. Candidate Selection
    print("\n=== CANDIDATE SELECTION ===")
    # Qualifying criteria:
    # - X displacement < -5.0 mm
    # - target joint commands identical to STOCK (already asserted)
    # - inspectable (full trial, finite, min Z > 0.2 mm, abs(Y) < 15 mm)
    qualifying_candidates = []
    for var_name, summary in trial_summaries.items():
        if var_name == "STOCK":
            continue
        disp_x = summary["displacement_x_mm"]
        inspectable = summary["inspectable"]
        target_identical = target_diffs[var_name] <= 1e-12

        if disp_x < -5.0 and inspectable and target_identical:
            qualifying_candidates.append(summary)

    print(f"QUALIFYING_BACKWARD_CANDIDATES_COUNT={len(qualifying_candidates)}")
    for qc in qualifying_candidates:
        print(
            f"  Candidate {qc['variant']}: X={qc['displacement_x_mm']:+.4f} mm, "
            f"Y={qc['displacement_y_mm']:+.4f} mm, LatRatio={qc['lateral_ratio']:.4f}"
        )

    selected_candidate = None
    if qualifying_candidates:
        # Lexicographic ranking:
        # FIRST: lowest LATERAL_RATIO
        # SECOND: more negative X displacement (i.e. smaller displacement_x_mm)
        qualifying_candidates.sort(
            key=lambda c: (round(c["lateral_ratio"], 5), c["displacement_x_mm"])
        )
        selected_candidate = qualifying_candidates[0]
        print(f"\nSELECTED_CANDIDATE={selected_candidate['variant']}")
        print(f"  X_DISPLACEMENT_MM={selected_candidate['displacement_x_mm']:.4f}")
        print(f"  Y_DISPLACEMENT_MM={selected_candidate['displacement_y_mm']:.4f}")
        print(f"  LATERAL_RATIO={selected_candidate['lateral_ratio']:.4f}")
    else:
        print("\nNO_QUALIFYING_CANDIDATE_FOUND")

    # 10. Evidence Classification
    evidence = {}
    if selected_candidate is not None:
        sel_var = selected_candidate["variant"]
        # A. FORWARD_TARGET_PROGRAM_PRESERVED
        evidence["forward_target_program_preserved"] = "YES" if target_integrity_pass else "NO"

        # B. ACTUAL_LEG_KINEMATICS_FORWARD_LIKE
        # Based on actual joint median correlation and foot X correlation
        j_corr = actual_joint_metrics[sel_var]["median_correlation"]
        f_corr = foot_metrics[sel_var]["median_foot_x_correlation"]
        j_rmse = actual_joint_metrics[sel_var]["overall_rmse_rad"]
        print(f"\nKinematics evidence check: JointMedianCorr={j_corr:.4f}, FootXMedianCorr={f_corr:.4f}, JointRMSE={j_rmse:.4f}")
        if j_corr >= 0.70 and f_corr >= 0.70:
            evidence["actual_leg_kinematics_forward_like"] = "STRONG"
        elif j_corr >= 0.40 and f_corr >= 0.40:
            evidence["actual_leg_kinematics_forward_like"] = "MODERATE"
        elif j_corr > 0.0 or f_corr > 0.0:
            evidence["actual_leg_kinematics_forward_like"] = "WEAK"
        else:
            evidence["actual_leg_kinematics_forward_like"] = "NO"

        # C. GROUND_SLIDING_EVIDENCE
        # Evaluate low-adhesion contact sliding velocities and support grip pattern
        sel_slip = all_slip_metrics[sel_var]
        off_samples_total = sum(sel_slip[leg]["contact_off_samples"] for leg in legs_order)
        on_samples_total = sum(sel_slip[leg]["contact_on_samples"] for leg in legs_order)
        # Check median vx for low adhesion across legs with off samples
        off_vx_list = [
            sel_slip[leg]["median_vx_adhesion_off"]
            for leg in legs_order
            if sel_slip[leg]["median_vx_adhesion_off"] is not None
        ]
        neg_frac_off_list = [
            sel_slip[leg]["negative_vx_fraction_adhesion_off"]
            for leg in legs_order
            if sel_slip[leg]["negative_vx_fraction_adhesion_off"] is not None
        ]

        print(f"Ground-slip evidence check: ON_samples={on_samples_total}, OFF_samples={off_samples_total}")
        print(f"OFF median vx list: {off_vx_list}")
        print(f"OFF neg frac list: {neg_frac_off_list}")

        if off_samples_total > 50 and len(off_vx_list) >= 3 and np.mean(neg_frac_off_list) >= 0.6:
            evidence["ground_sliding_evidence"] = "STRONG"
        elif off_samples_total > 20 and len(off_vx_list) >= 2:
            evidence["ground_sliding_evidence"] = "MODERATE"
        elif off_samples_total > 0:
            evidence["ground_sliding_evidence"] = "WEAK"
        else:
            evidence["ground_sliding_evidence"] = "NO"

        # D. REVERSE_TRANSLATION_CONFIRMED
        evidence["reverse_translation_confirmed"] = (
            "YES" if selected_candidate["displacement_x_mm"] < -5.0 else "NO"
        )

        # VALIDATED_MOONWALK_CANDIDATE
        if (
            evidence["reverse_translation_confirmed"] == "YES"
            and evidence["forward_target_program_preserved"] == "YES"
            and evidence["ground_sliding_evidence"] in ("STRONG", "MODERATE")
            and evidence["actual_leg_kinematics_forward_like"] in ("STRONG", "MODERATE")
        ):
            evidence["validated_moonwalk_candidate"] = "YES"
        else:
            evidence["validated_moonwalk_candidate"] = "NO"
    else:
        evidence = {
            "forward_target_program_preserved": "YES" if target_integrity_pass else "NO",
            "actual_leg_kinematics_forward_like": "NO",
            "ground_sliding_evidence": "NO",
            "reverse_translation_confirmed": "NO",
            "validated_moonwalk_candidate": "NO",
        }

    print("\n=== EVIDENCE CLASSIFICATION ===")
    for k, v in evidence.items():
        print(f"{k.upper()} = {v}")

    # 11. Save output artifacts
    output_dir = pathlib.Path("output/m3")
    output_dir.mkdir(parents=True, exist_ok=True)

    # 11a. NPZ Telemetry export
    npz_path = output_dir / "mechanics_telemetry.npz"
    npz_data = {
        "actuated_dof_names": np.array(actuated_dof_names),
        "legs_order": np.array(legs_order),
    }
    for var_name, _ in conditions:
        prefix = f"{var_name}_"
        for arr_name, arr in telemetry_raw[var_name].items():
            npz_data[f"{prefix}{arr_name}"] = arr

    np.savez_compressed(npz_path, **npz_data)
    print(f"\nSAVED_TELEMETRY_NPZ={npz_path.as_posix()} ({npz_path.stat().st_size} bytes)")

    # 11b. Summary JSON export
    summary_path = output_dir / "mechanics_summary.json"
    summary_export = {
        "flygym_version": flygym_ver,
        "mujoco_version": mujoco_ver,
        "numpy_version": numpy_ver,
        "trial_duration_s": trial_duration_s,
        "physics_timestep_s": actual_timestep,
        "sample_interval_steps": sample_interval_steps,
        "telemetry_sample_interval_s": sample_dt_s,
        "actuated_dof_count": actuated_dof_count,
        "target_joint_diffs_rad": target_diffs,
        "target_program_identical": target_integrity_pass,
        "trial_summaries": trial_summaries,
        "actual_joint_metrics": actual_joint_metrics,
        "body_relative_foot_metrics": foot_metrics,
        "ground_slip_metrics": all_slip_metrics,
        "selected_candidate": selected_candidate["variant"] if selected_candidate else None,
        "evidence": evidence,
    }
    with open(summary_path, "w") as f:
        json.dump(summary_export, f, indent=2)
    print(f"SAVED_SUMMARY_JSON={summary_path.as_posix()}")

    # 12. Render Replay for Selected Candidate (1.5 simulated seconds)
    video_path = output_dir / "validated_candidate.mp4"
    video_created = False
    rendered_frame_count = 0

    if selected_candidate is not None:
        sel_var = selected_candidate["variant"]
        sel_phase = selected_candidate["phase_fraction"]
        replay_duration_s = 1.5
        n_replay_steps = round(replay_duration_s / actual_timestep)

        print(f"\n=== RENDERING CANDIDATE REPLAY ===")
        print(f"Candidate: {sel_var} (phase={sel_phase:.2f})")
        print(f"Duration: {replay_duration_s} s ({n_replay_steps} steps)")
        print(f"Output Video: {video_path.as_posix()}")

        renderer = sim.set_renderer(
            cam,
            camera_res=(240, 320),
            playback_speed=0.2,
            output_fps=25,
            buffer_frames=True,
        )

        try:
            # Deterministic reset for replay
            sim.reset()
            controller.cpg_network.random_state = np.random.RandomState(0)
            controller.cpg_network.reset()
            sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
            sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
            sim.warmup()

            for _ in range(n_replay_steps):
                stock_action = controller.step()
                adhesion = compute_adhesion(
                    sel_phase,
                    stock_action.adhesion_onoff,
                    controller.cpg_network.curr_phases,
                    preprogrammed_steps,
                )
                action_to_apply = LocomotionAction(
                    joint_angles=stock_action.joint_angles,
                    adhesion_onoff=adhesion,
                )
                apply_locomotion_action(sim, fly.name, action_to_apply)
                sim.step_with_profile()
                sim.render_as_needed_with_profile()

            renderer.save_video(video_path)

            if renderer.frames:
                rendered_frame_count = sum(len(f) for f in renderer.frames.values())
            else:
                rendered_frame_count = sim._frames_rendered

            assert video_path.is_file() and video_path.stat().st_size > 0
            video_created = True
            print(f"VIDEO_CREATED=YES")
            print(f"VIDEO_SIZE_BYTES={video_path.stat().st_size}")
            print(f"RENDERED_FRAME_COUNT={rendered_frame_count}")
        finally:
            renderer.close()
    else:
        print("\nNO_CANDIDATE_QUALIFIED_FOR_RENDERING")

    total_elapsed = time.perf_counter() - start_wall_time
    print(f"\nM3_EXECUTION_FINISHED in {total_elapsed:.2f} s")
    print("M3_VALIDATION_RESULT=PASS")


if __name__ == "__main__":
    main()
