import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
"""MW-M8: Dedicated Backslide Controller & Reproducible Grid Search."""

import json
import pathlib
from importlib.metadata import version

import numpy as np
import imageio.v2 as iio

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    LocomotionAction,
    PreprogrammedSteps,
    apply_locomotion_action,
    make_locomotion_fly,
    make_tripod_cpg_network
)

from m7_moonwalk_choreography import MoonwalkChoreographyController


def smooth_step(x, edge0, edge1):
    t = np.clip((x - edge0) / (edge1 - edge0 + 1e-9), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def smooth_triangle(phi, r):
    phi = phi % 2.0
    if phi > 1.0:
        phi = 2.0 - phi

    v_max = 1.0 / (1.0 - r + 1e-9)

    if phi <= r:
        if r < 1e-9:
            return 0.0
        x = phi / r
        return r * v_max * (x**3 - 0.5 * x**4)
    elif phi <= 1.0 - r:
        return 0.5 * r * v_max + v_max * (phi - r)
    else:
        if r < 1e-9:
            return 1.0
        psi = 1.0 - phi
        x = psi / r
        return 1.0 - r * v_max * (x**3 - 0.5 * x**4)


def get_adhesion(phi, t_f):
    phi = phi % 2.0
    d1 = phi - 1.0
    if -t_f/2 <= d1 <= t_f/2:
        return smooth_step(d1, -t_f/2, t_f/2)

    d0 = phi
    if phi > 1.5:
        d0 = phi - 2.0
    if -t_f/2 <= d0 <= t_f/2:
        return 1.0 - smooth_step(d0, -t_f/2, t_f/2)

    if 0 < phi < 1:
        return 0.0
    return 1.0


class BackslideTrajectoryController:
    def __init__(self, preprogrammed_steps, actuated_dofs,
                 cadence=4.0, stance_excursion_fraction=0.8,
                 turnaround_fraction=0.1, support_transfer_fraction=0.1,
                 entry_duration=0.25, timestep=0.0001):
        self.pps = preprogrammed_steps
        self.actuated_dofs = actuated_dofs
        self.cadence = cadence
        self.E = stance_excursion_fraction
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
            if i in [0, 2, 4]:
                p = phi
            else:
                p = (phi + 1.0) % 2.0

            w = smooth_triangle(p, self.r)
            u = 0.5 - self.E/2.0 + self.E * w

            phases[i] = self.stance_starts[i] + u * (self.stance_ends[i] - self.stance_starts[i])
            adhesions[i] = get_adhesion(p, self.t_f)

        stock_angles = self.pps.get_joint_angles_by_dof_order(
            phases, output_dof_order=self.actuated_dofs
        )

        if self.time < self.entry_duration:
            alpha = smooth_step(self.time, 0.0, self.entry_duration)
            joint_angles = (1.0 - alpha) * self.default_pose + alpha * stock_angles
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions
        else:
            joint_angles = stock_angles

        self.time += self.timestep

        return LocomotionAction(joint_angles=joint_angles, adhesion_onoff=adhesions), phi


def evaluate_candidate(sim, fly, controller, params, n_steps, sample_dt, thorax_idx, tarsus5_indices, is_m7=False, return_telemetry=False):
    sim.reset()
    if is_m7:
        controller.cpg_network.random_state = np.random.RandomState(0)
        controller.cpg_network.reset()
        controller.time = 0.0
    else:
        controller.time = 0.0

    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    start_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()

    t_list, x_list, y_list, z_list = [], [], [], []
    v_x_list = []
    adhesion_list, tarsus_x_list, contact_list = [], [], []

    prev_x = start_pos[0]
    tripod_a = [0, 2, 4]
    tripod_b = [1, 3, 5]
    tripod_states = []

    entry_duration = params.get('entry_duration', 0.25)

    for i in range(n_steps):
        if is_m7:
            action = controller.step()
        else:
            action, _ = controller.step()

        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        if (i + 1) % 10 == 0:
            t = (i + 1) * sim.mj_model.opt.timestep
            pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
            t_list.append(t)
            x_list.append(pos[0])
            y_list.append(pos[1])
            z_list.append(pos[2])
            v_x_list.append((pos[0] - prev_x) / sample_dt)
            prev_x = pos[0]

            adhesion_list.append(action.adhesion_onoff.copy())
            tarsus_pos = sim.get_body_positions(fly.name)[tarsus5_indices].copy()
            tarsus_x_list.append(tarsus_pos[:, 0])

            contact_info = sim.get_ground_contact_info(fly.name)
            contact_list.append(contact_info[0].copy())

            a_sup = np.mean(action.adhesion_onoff[tripod_a]) > 0.5
            b_sup = np.mean(action.adhesion_onoff[tripod_b]) > 0.5
            if a_sup and not b_sup:
                tripod_states.append('A')
            elif b_sup and not a_sup:
                tripod_states.append('B')
            else:
                tripod_states.append('T')

    steady_idx = 0
    for i, t in enumerate(t_list):
        if t > entry_duration + 0.1:
            steady_idx = i
            break

    if steady_idx >= len(t_list) - 5:
        record = {
            "parameters": params,
            "status": {
                "simulation_completed": False,
                "feasible": False,
                "rejection_reason": "simulation did not reach steady state"
            },
            "metrics": None
        }
        if return_telemetry:
            return record, None
        return record

    steady_x = np.array(x_list[steady_idx:])
    steady_y = np.array(y_list[steady_idx:])
    steady_z = np.array(z_list[steady_idx:])
    steady_vx = np.array(v_x_list[steady_idx:])

    disp_x = steady_x[-1] - x_list[0]
    disp_y = steady_y[-1] - y_list[0]

    mean_vx = np.mean(steady_vx)

    window_20ms = int(0.020 / sample_dt)
    if len(steady_vx) >= window_20ms:
        steady_vx_20ms = np.convolve(steady_vx, np.ones(window_20ms)/window_20ms, mode='valid')
        backward_speed_cv_20ms = float(np.std(steady_vx_20ms) / max(abs(mean_vx), 1e-9))
    else:
        backward_speed_cv_20ms = float(np.std(steady_vx) / max(abs(mean_vx), 1e-9))

    window_50ms = int(0.050 / sample_dt)
    if len(steady_vx) >= window_50ms:
        steady_vx_50ms = np.convolve(steady_vx, np.ones(window_50ms)/window_50ms, mode='valid')
        backward_speed_cv_50ms = float(np.std(steady_vx_50ms) / max(abs(mean_vx), 1e-9))
    else:
        backward_speed_cv_50ms = float(np.std(steady_vx) / max(abs(mean_vx), 1e-9))

    backward_speed_cv_raw = float(np.std(steady_vx) / max(abs(mean_vx), 1e-9))
    lateral_ratio = abs(disp_y) / max(abs(disp_x), 1e-5)
    z_ptp = np.ptp(steady_z)

    steady_adh = np.array(adhesion_list[steady_idx:])
    steady_tarsus_x = np.array(tarsus_x_list[steady_idx:])
    steady_contact = np.array(contact_list[steady_idx:])

    support_vx = []
    slide_vx = []
    slide_negative_count = 0
    slide_total_count = 0

    for k in range(1, len(steady_adh)):
        vxs = (steady_tarsus_x[k] - steady_tarsus_x[k-1]) / sample_dt
        adh = steady_adh[k]
        cont = steady_contact[k]
        cont_prev = steady_contact[k-1]

        for leg_idx in range(6):
            if cont[leg_idx] > 0 and cont_prev[leg_idx] > 0:
                if adh[leg_idx] > 0.5:
                    support_vx.append(abs(vxs[leg_idx]))
                else:
                    slide_vx.append(vxs[leg_idx])
                    slide_total_count += 1
                    if vxs[leg_idx] < 0:
                        slide_negative_count += 1

    med_support_vx = float(np.median(support_vx)) if support_vx else 0.0
    med_slide_vx = float(np.median(slide_vx)) if slide_vx else 0.0
    slide_neg_frac = float(slide_negative_count / slide_total_count) if slide_total_count > 0 else 0.0
    slide_ratio = abs(med_slide_vx) / max(abs(mean_vx), 1e-9)

    exchanges = 0
    curr_state = None
    for state in tripod_states[steady_idx:]:
        if state in ('A', 'B'):
            if curr_state is not None and state != curr_state:
                exchanges += 1
            curr_state = state

    a_frac = sum(1 for s in tripod_states[steady_idx:] if s == 'A') / len(tripod_states[steady_idx:])
    b_frac = sum(1 for s in tripod_states[steady_idx:] if s == 'B') / len(tripod_states[steady_idx:])

    rejection_reasons = []
    if mean_vx > -0.1:
        rejection_reasons.append(f"mean_vx ({mean_vx:.2f}) > -0.1 mm/s")
    if disp_x > -5.0:
        rejection_reasons.append(f"displacement_x ({disp_x:.2f}) > -5.0 mm")
    if exchanges < 2:
        rejection_reasons.append(f"tripod_role_exchanges ({exchanges}) < 2")
    if not np.all(np.isfinite(steady_x)):
        rejection_reasons.append("non-finite steady position")
    if np.min(steady_z) < 0.15:
        rejection_reasons.append(f"min_steady_z ({np.min(steady_z):.2f}) < 0.15 mm")
    if lateral_ratio > 0.5:
        rejection_reasons.append(f"lateral_ratio ({lateral_ratio:.2f}) > 0.5")
    if a_frac < 0.1 or b_frac < 0.1:
        rejection_reasons.append(f"asymmetric tripod duty (A={a_frac:.2f}, B={b_frac:.2f})")
    if slide_neg_frac < 0.5:
        rejection_reasons.append(f"slide_neg_frac ({slide_neg_frac:.2f}) < 0.5")

    feasible = len(rejection_reasons) == 0
    rejection_reason = "; ".join(rejection_reasons) if rejection_reasons else None

    per_leg = {}
    for leg_idx in range(6):
        svx = []
        slvx = []
        sl_neg = 0
        for k in range(1, len(steady_adh)):
            vxs = (steady_tarsus_x[k] - steady_tarsus_x[k-1]) / sample_dt
            adh = steady_adh[k]
            cont = steady_contact[k]
            cont_prev = steady_contact[k-1]
            if cont[leg_idx] > 0 and cont_prev[leg_idx] > 0:
                if adh[leg_idx] > 0.5:
                    svx.append(abs(vxs[leg_idx]))
                else:
                    slvx.append(vxs[leg_idx])
                    if vxs[leg_idx] < 0:
                        sl_neg += 1
        per_leg[str(leg_idx)] = {
            "support_samples": len(svx),
            "slide_samples": len(slvx),
            "support_median_abs_vx": float(np.median(svx)) if svx else 0.0,
            "slide_median_vx": float(np.median(slvx)) if slvx else 0.0,
            "slide_negative_fraction": float(sl_neg / len(slvx)) if slvx else 0.0
        }

    metrics = {
        "backward_displacement_mm": float(disp_x),
        "mean_backward_speed_mm_s": float(abs(mean_vx)),
        "backward_speed_cv_raw": backward_speed_cv_raw,
        "backward_speed_cv_20ms": backward_speed_cv_20ms,
        "backward_speed_cv_50ms": backward_speed_cv_50ms,
        "lateral_ratio": float(lateral_ratio),
        "thorax_z_peak_to_peak_mm": float(z_ptp),
        "contacting_support_median_abs_vx": med_support_vx,
        "contacting_support_median_abs_vx_mm_s": med_support_vx,
        "contacting_slide_median_vx": med_slide_vx,
        "contacting_slide_median_vx_mm_s": med_slide_vx,
        "contacting_slide_negative_fraction": slide_neg_frac,
        "contacting_slide_to_body_speed_ratio": slide_ratio,
        "tripod_role_exchanges": exchanges,
        "tripod_a_support_fraction": float(a_frac),
        "tripod_b_support_fraction": float(b_frac),
        "per_leg": per_leg
    }

    record = {
        "parameters": params,
        "status": {
            "simulation_completed": True,
            "feasible": feasible,
            "rejection_reason": rejection_reason
        },
        "metrics": metrics
    }

    if return_telemetry:
        return record, {
            "t": np.array(t_list),
            "x": np.array(x_list),
            "y": np.array(y_list),
            "z": np.array(z_list),
            "vx": np.array(v_x_list),
            "adhesion": np.array(adhesion_list)
        }
    return record


def render_video(sim, fly, controller, n_steps, filepath, cam, is_m7=False):
    sim.reset()
    if is_m7:
        controller.cpg_network.random_state = np.random.RandomState(0)
        controller.cpg_network.reset()
        controller.time = 0.0
    else:
        controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    renderer = sim.set_renderer(
        cam,
        camera_res=(240, 320),
        playback_speed=0.75,
        output_fps=30,
        buffer_frames=True,
    )

    try:
        for _ in range(n_steps):
            if is_m7:
                action = controller.step()
            else:
                action, _ = controller.step()
            apply_locomotion_action(sim, fly.name, action)
            sim.step_with_profile()
            sim.render_as_needed_with_profile()

        renderer.save_video(filepath)
    finally:
        renderer.close()


def write_side_by_side(video_left_path, video_right_path, output_path):
    from PIL import Image, ImageDraw
    reader_l = iio.get_reader(str(video_left_path))
    reader_r = iio.get_reader(str(video_right_path))
    fps = reader_l.get_meta_data().get("fps", 30)

    writer = iio.get_writer(str(output_path), fps=fps)
    try:
        for frame_l, frame_r in zip(reader_l, reader_r):
            img_l = Image.fromarray(frame_l)
            img_r = Image.fromarray(frame_r)

            draw_l = ImageDraw.Draw(img_l)
            draw_l.text((10, 10), "M7 — ADHESION-TIMING CONTROL", fill="white")
            draw_r = ImageDraw.Draw(img_r)
            draw_r.text((10, 10), "M8 — DEDICATED BACKSLIDE", fill="white")

            w_l, h_l = img_l.size
            w_r, h_r = img_r.size
            combined = Image.new("RGB", (w_l + w_r, max(h_l, h_r)))
            combined.paste(img_l, (0, 0))
            combined.paste(img_r, (w_l, 0))
            writer.append_data(np.array(combined))
    finally:
        reader_l.close()
        reader_r.close()
        writer.close()


def main():
    print("M8_START")
    output_dir = pathlib.Path("output/m8")
    output_dir.mkdir(parents=True, exist_ok=True)
    derived_dir = pathlib.Path("data/derived")
    derived_dir.mkdir(parents=True, exist_ok=True)

    fly = make_locomotion_fly(name="m8_fly", add_adhesion=True)
    cam = fly.add_tracking_camera(name="trackcam")

    world = FlatGroundWorld()
    world.add_fly(
        fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )
    sim = Simulation(world)

    body_segs = fly.get_bodysegs_order()
    seg_names = [s.name if hasattr(s, "name") else str(s) for s in body_segs]
    thorax_idx = seg_names.index(fly.root_segment.name)
    legs_order = fly.get_legs_order()
    tarsus5_indices = [seg_names.index(f"{leg}_tarsus5") for leg in legs_order]
    actuated_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)

    timestep = float(sim.mj_model.opt.timestep)
    trial_duration = 1.75
    n_steps = int(trial_duration / timestep)
    sample_dt = 10 * timestep

    pps = PreprogrammedSteps()

    print("Evaluating M7_ADHESION_ONLY_CONTROL (Apples-to-Apples)...")
    m7_params = {"cadence": 10.0, "phase_shift": 0.55, "transfer_width": 0.5, "entry_duration": 0.25}
    cpg_m7 = make_tripod_cpg_network(timestep=timestep)
    ctrl_m7 = MoonwalkChoreographyController(cpg_m7, pps, actuated_dofs, **m7_params)
    m7_record, m7_telemetry = evaluate_candidate(
        sim, fly, ctrl_m7, m7_params, n_steps, sample_dt, thorax_idx, tarsus5_indices, is_m7=True, return_telemetry=True
    )
    print("M7 METRICS:", json.dumps(m7_record["metrics"], indent=2))

    print("\nRunning complete deterministic search for M8...")
    cadences = [3.0, 4.0, 5.0, 6.0]
    stance_excursion_fractions = [0.60, 0.80, 1.00]
    turnaround_fractions = [0.06, 0.12]
    support_transfer_fractions = [0.08, 0.15]
    entry_duration = 0.25

    candidate_grid = {
        "cadence": cadences,
        "stance_excursion_fraction": stance_excursion_fractions,
        "turnaround_fraction": turnaround_fractions,
        "support_transfer_fraction": support_transfer_fractions,
        "entry_duration": entry_duration
    }

    all_candidate_records = []

    # 4 * 3 * 2 * 2 = 48 candidates
    idx = 0
    for cad in cadences:
        for E in stance_excursion_fractions:
            for r in turnaround_fractions:
                for tf in support_transfer_fractions:
                    idx += 1
                    p = {
                        "cadence": cad,
                        "stance_excursion_fraction": E,
                        "turnaround_fraction": r,
                        "support_transfer_fraction": tf,
                        "entry_duration": entry_duration
                    }
                    print(f"[{idx}/48] Testing {p}...")
                    c = BackslideTrajectoryController(pps, actuated_dofs, **p)
                    record = evaluate_candidate(sim, fly, c, p, n_steps, sample_dt, thorax_idx, tarsus5_indices)
                    all_candidate_records.append(record)

    # Search completeness assertion
    canonical_tuples = [
        (r["parameters"]["cadence"],
         r["parameters"]["stance_excursion_fraction"],
         r["parameters"]["turnaround_fraction"],
         r["parameters"]["support_transfer_fraction"])
        for r in all_candidate_records
    ]
    unique_tuples = set(canonical_tuples)

    print(f"\nEXPECTED_CANDIDATES=48")
    print(f"ACTUAL_CANDIDATES={len(all_candidate_records)}")
    print(f"UNIQUE_PARAMETER_TUPLES={len(unique_tuples)}")

    assert len(all_candidate_records) == 48, f"Expected 48 candidates, got {len(all_candidate_records)}"
    assert len(unique_tuples) == 48, f"Expected 48 unique tuples, got {len(unique_tuples)}"

    feasible_candidates = [r for r in all_candidate_records if r["status"]["feasible"]]
    print(f"FEASIBLE_CANDIDATES={len(feasible_candidates)}")

    if not feasible_candidates:
        raise RuntimeError("No feasible candidates found in grid search.")

    def candidate_ranking_key(c):
        m = c["metrics"]
        p = c["parameters"]
        return (
            m["backward_speed_cv_20ms"],
            m["lateral_ratio"],
            m["thorax_z_peak_to_peak_mm"],
            m["contacting_support_median_abs_vx"],
            p["cadence"],
            p["stance_excursion_fraction"],
            p["turnaround_fraction"],
            p["support_transfer_fraction"],
        )

    feasible_candidates.sort(key=candidate_ranking_key)
    best_candidate = feasible_candidates[0]
    best_p = best_candidate["parameters"]

    print("\nSELECTED CANDIDATE PARAMETERS:")
    print(json.dumps(best_p, indent=2))
    print("\nSELECTED CANDIDATE METRICS:")
    print(json.dumps(best_candidate["metrics"], indent=2))

    ranking_method = "lexicographic: lowest backward_speed_cv_20ms, lowest lateral_ratio, lowest thorax_z_peak_to_peak_mm, lowest contacting_support_median_abs_vx"

    search_results_payload = {
        "schema_version": "1.0.0",
        "flygym_version": version("flygym"),
        "mujoco_version": version("mujoco"),
        "candidate_grid": candidate_grid,
        "expected_candidate_count": 48,
        "actual_candidate_count": len(all_candidate_records),
        "feasible_candidate_count": len(feasible_candidates),
        "ranking_method": ranking_method,
        "all_candidates": all_candidate_records,
        "selected_candidate": {
            "parameters": best_p,
            "metrics": best_candidate["metrics"]
        }
    }

    # Write to data/derived and output/m8
    with open(derived_dir / "m8_search_results.json", "w") as f:
        json.dump(search_results_payload, f, indent=2)
    with open(output_dir / "search_results.json", "w") as f:
        json.dump(search_results_payload, f, indent=2)

    print("\nRendering performance rollout (M8 selected)...")
    ctrl_best = BackslideTrajectoryController(pps, actuated_dofs, **best_p)
    render_steps = int(2.75 / timestep)

    ctrl_tele = BackslideTrajectoryController(pps, actuated_dofs, **best_p)
    rollout_record, best_telemetry = evaluate_candidate(
        sim, fly, ctrl_tele, best_p, render_steps, sample_dt, thorax_idx, tarsus5_indices, return_telemetry=True
    )

    render_video(sim, fly, ctrl_best, render_steps, output_dir / "backslide_v1.mp4", cam)

    print("Rendering M7 for comparison...")
    cpg_m7_2 = make_tripod_cpg_network(timestep=timestep)
    ctrl_m7_2 = MoonwalkChoreographyController(cpg_m7_2, pps, actuated_dofs, **m7_params)
    render_video(sim, fly, ctrl_m7_2, render_steps, output_dir / "m7.mp4", cam, is_m7=True)

    print("Compositing side-by-side video...")
    write_side_by_side(output_dir / "m7.mp4", output_dir / "backslide_v1.mp4", output_dir / "m7_vs_m8.mp4")

    print("Saving telemetry...")
    np.savez_compressed(
        output_dir / "backslide_telemetry.npz",
        m7_t=m7_telemetry['t'], m7_vx=m7_telemetry['vx'], m7_z=m7_telemetry['z'],
        best_t=best_telemetry['t'], best_vx=best_telemetry['vx'], best_z=best_telemetry['z'],
    )

    print("Updating manifest...")
    manifest = {
        "milestone": "M8",
        "flygym_version": version("flygym"),
        "mujoco_version": version("mujoco"),
        "human_reference": {
            "DOI": "10.1590/1806-9126-RBEF-2024-0319",
            "qualitative_principles_used": "one foot supports while the other slides backward, roles exchange.",
            "human_absolute_scale_transferred": False,
            "note": "Human absolute speeds were NOT transferred to the fly."
        },
        "candidate_grid": candidate_grid,
        "expected_candidates": 48,
        "evaluated_candidates": len(all_candidate_records),
        "feasible_candidates": len(feasible_candidates),
        "ranking_method": ranking_method,
        "selected_parameters": best_p,
        "selected_search_metrics": best_candidate["metrics"],
        "final_rollout_metrics": rollout_record["metrics"],
        "m7_control_metrics": m7_record["metrics"],
        "contact_aware_definitions": {
            "contact_criterion": "contact_found > 0 at both adjacent telemetry samples from sim.get_ground_contact_info",
            "support_criterion": "contacting and commanded adhesion > 0.5",
            "slide_criterion": "contacting and commanded adhesion <= 0.5"
        },
        "cv_filter_definitions": {
            "backward_speed_cv_raw": "std(vx) / |mean(vx)| on raw 1ms samples",
            "backward_speed_cv_20ms": "std(vx_20ms) / |mean(vx)| using 20ms moving average",
            "backward_speed_cv_50ms": "std(vx_50ms) / |mean(vx)| using 50ms moving average"
        },
        "controller": {
            "architecture": "BackslideTrajectoryController",
            "trajectory_basis": "PreprogrammedSteps stance-like interval",
            "stance_interval_definition": "complement of swing_period [swing_end, swing_start+2pi]",
            "trajectory_direction_calibration": {
                "all_legs": "increasing u moves tarsus5 BACKWARD relative to body. SUPPORT requires DECREASING u (moves body backward)."
            },
            "tripod_definitions": {
                "TRIPOD_A": ["LF", "LH", "RM"],
                "TRIPOD_B": ["LM", "RF", "RH"]
            },
            "adhesion_semantics": "CONTINUOUS_ADHESION_SUPPORTED"
        },
        "scientific_boundary": "a six-legged backslide choreography built from FlyGym-derived leg trajectory shapes, not biological Drosophila backward walking."
    }

    with open(derived_dir / "m8_backslide_manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)

    print("M8_COMPLETE")


if __name__ == "__main__":
    main()
