"""MW-M7: Moonwalk Choreography."""

import json
import pathlib
import time
from importlib.metadata import version

import numpy as np
import imageio

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

def smooth_step(x, edge0, edge1):
    t = np.clip((x - edge0) / (edge1 - edge0 + 1e-9), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)

def get_smooth_adhesion(phase, swing_start, swing_end, phase_shift_frac, transfer_width):
    shift = phase_shift_frac * 2.0 * np.pi
    p = (phase + shift) % (2.0 * np.pi)
    
    d_start = (p - swing_start + np.pi) % (2 * np.pi) - np.pi
    d_end = (p - swing_end + np.pi) % (2 * np.pi) - np.pi
    
    val = 1.0
    if -transfer_width/2 <= d_start <= transfer_width/2:
        val = 1.0 - smooth_step(d_start, -transfer_width/2, transfer_width/2)
    elif -transfer_width/2 <= d_end <= transfer_width/2:
        val = smooth_step(d_end, -transfer_width/2, transfer_width/2)
    elif swing_start + transfer_width/2 < p < swing_end - transfer_width/2:
        val = 0.0
    return val

class MoonwalkChoreographyController:
    def __init__(self, cpg_network, preprogrammed_steps, actuated_dofs,
                 cadence=12.0, phase_shift=0.55, transfer_width=0.4, entry_duration=0.25):
        self.cpg_network = cpg_network
        self.cpg_network.intrinsic_frequencies = np.ones(6) * cadence
        self.preprogrammed_steps = preprogrammed_steps
        self.actuated_dofs = actuated_dofs
        
        self.phase_shift = phase_shift
        self.transfer_width = transfer_width
        self.entry_duration = entry_duration
        self.time = 0.0
        
        self.default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dofs)
        self.legs = preprogrammed_steps.legs

    def step(self):
        self.cpg_network.step()
        curr_phases = self.cpg_network.curr_phases
        
        stock_angles = self.preprogrammed_steps.get_joint_angles_by_dof_order(
            curr_phases, output_dof_order=self.actuated_dofs
        )
        
        adhesion = np.zeros(6)
        for i, leg in enumerate(self.legs):
            swing_start, swing_end = self.preprogrammed_steps.swing_period[leg.lower()]
            adhesion[i] = get_smooth_adhesion(curr_phases[i], swing_start, swing_end, self.phase_shift, self.transfer_width)
            
        if self.time < self.entry_duration:
            alpha = self.time / self.entry_duration
            alpha_smooth = smooth_step(alpha, 0.0, 1.0)
            joint_angles = (1.0 - alpha_smooth) * self.default_pose + alpha_smooth * stock_angles
            
            # smoothly transition adhesion from full grip (1.0) to moonwalk pattern
            adhesion = (1.0 - alpha_smooth) * np.ones(6) + alpha_smooth * adhesion
        else:
            joint_angles = stock_angles
            
        self.time += self.cpg_network.timestep
        
        return LocomotionAction(joint_angles=joint_angles, adhesion_onoff=adhesion)


def compute_pearson_r(x, y):
    std_x, std_y = float(np.std(x)), float(np.std(y))
    if std_x < 1e-8 or std_y < 1e-8:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])

def evaluate_candidate(sim, fly, controller, params, n_steps, sample_dt, thorax_idx, tarsus5_indices, return_telemetry=False):
    # Deterministic reset
    sim.reset()
    controller.cpg_network.random_state = np.random.RandomState(0)
    controller.cpg_network.reset()
    controller.time = 0.0
    
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()
    
    start_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
    
    t_list, x_list, y_list, z_list = [], [], [], []
    v_x_list = []
    adhesion_list, tarsus_x_list = [], []
    
    # Orientation arrays
    yaw_list, pitch_list, roll_list = [], [], []
    
    prev_x = start_pos[0]
    
    tripod_a = [0, 2, 4] # LF, LH, RM
    tripod_b = [1, 3, 5] # LM, RF, RH
    
    tripod_states = []
    
    for i in range(n_steps):
        action = controller.step()
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
            
            # Fallback orientation logic if FlyGym API changes, 
            # root is usually thorax. M3 didn't use orientation natively.
            # We'll skip orientation metric for search to be safe and use Z & Lat
            
            adhesion_list.append(action.adhesion_onoff.copy())
            tarsus_pos = sim.get_body_positions(fly.name)[tarsus5_indices].copy()
            tarsus_x_list.append(tarsus_pos[:, 0])
            
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
        if t > params['entry_duration'] + 0.1:
            steady_idx = i
            break
            
    if steady_idx >= len(t_list) - 5:
        return None
        
    steady_x = np.array(x_list[steady_idx:])
    steady_y = np.array(y_list[steady_idx:])
    steady_z = np.array(z_list[steady_idx:])
    steady_vx = np.array(v_x_list[steady_idx:])
    
    disp_x = steady_x[-1] - x_list[0]
    disp_y = steady_y[-1] - y_list[0]
    
    mean_vx = np.mean(steady_vx)
    if mean_vx >= -0.1: # Must move backward meaningfully
        return None
        
    backward_speed_cv = np.std(steady_vx) / abs(mean_vx)
    lateral_ratio = abs(disp_y) / max(abs(disp_x), 1e-5)
    
    z_ptp = np.ptp(steady_z)
    
    steady_adh = np.array(adhesion_list[steady_idx:])
    steady_tarsus_x = np.array(tarsus_x_list[steady_idx:])
    
    support_vx = []
    slide_vx = []
    for k in range(1, len(steady_adh)):
        vxs = (steady_tarsus_x[k] - steady_tarsus_x[k-1]) / sample_dt
        adh = steady_adh[k]
        for leg_idx in range(6):
            if adh[leg_idx] > 0.5:
                support_vx.append(abs(vxs[leg_idx]))
            else:
                slide_vx.append(-vxs[leg_idx])
                
    med_support_vx = float(np.median(support_vx)) if support_vx else 0.0
    med_slide_vx = float(np.median(slide_vx)) if slide_vx else 0.0
    slide_ratio = med_slide_vx / abs(mean_vx) if mean_vx != 0 else 0.0
    
    exchanges = 0
    curr_state = None
    for state in tripod_states[steady_idx:]:
        if state in ('A', 'B'):
            if curr_state is not None and state != curr_state:
                exchanges += 1
            curr_state = state
            
    a_frac = sum(1 for s in tripod_states[steady_idx:] if s == 'A') / len(tripod_states[steady_idx:])
    b_frac = sum(1 for s in tripod_states[steady_idx:] if s == 'B') / len(tripod_states[steady_idx:])
    
    feasible = (disp_x < -5.0) and (exchanges > 2) and np.all(np.isfinite(steady_x)) and (np.min(steady_z) > 0.15) and (lateral_ratio < 0.5)
    
    res = {
        "params": params,
        "feasible": bool(feasible),
        "backward_displacement_mm": float(disp_x),
        "mean_backward_speed_mm_s": float(abs(mean_vx)),
        "backward_speed_cv": float(backward_speed_cv),
        "lateral_ratio": float(lateral_ratio),
        "thorax_z_peak_to_peak_mm": float(z_ptp),
        "support_foot_median_abs_vx_mm_s": float(med_support_vx),
        "slide_foot_median_vx_mm_s": float(med_slide_vx),
        "slide_to_body_speed_ratio": float(slide_ratio),
        "tripod_role_exchanges": exchanges,
        "tripod_a_support_fraction": float(a_frac),
        "tripod_b_support_fraction": float(b_frac)
    }
    
    if return_telemetry:
        return res, {
            "t": np.array(t_list),
            "x": np.array(x_list),
            "y": np.array(y_list),
            "z": np.array(z_list),
            "vx": np.array(v_x_list),
            "adhesion": np.array(adhesion_list)
        }
    return res

def render_video(sim, fly, controller, n_steps, filepath, cam):
    sim.reset()
    controller.cpg_network.random_state = np.random.RandomState(0)
    controller.cpg_network.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()
    
    renderer = sim.set_renderer(
        cam,
        camera_res=(240, 320),
        playback_speed=0.6,
        output_fps=30,
        buffer_frames=True,
    )
    
    try:
        for _ in range(n_steps):
            action = controller.step()
            apply_locomotion_action(sim, fly.name, action)
            sim.step_with_profile()
            sim.render_as_needed_with_profile()
            
        renderer.save_video(filepath)
    finally:
        frames = renderer.frames.get("trackcam", [])
        renderer.close()
    return frames

def write_side_by_side(frames_left, frames_right, filepath):
    import imageio.v3 as iio
    from PIL import Image, ImageDraw, ImageFont

    out_frames = []
    min_len = min(len(frames_left), len(frames_right))
    for i in range(min_len):
        img_l = Image.fromarray(frames_left[i])
        img_r = Image.fromarray(frames_right[i])
        
        # Add text
        draw_l = ImageDraw.Draw(img_l)
        draw_l.text((10, 10), "M3 PROOF OF CONCEPT", fill="white")
        draw_r = ImageDraw.Draw(img_r)
        draw_r.text((10, 10), "M7 SMOOTH CHOREOGRAPHY", fill="white")
        
        w_l, h_l = img_l.size
        w_r, h_r = img_r.size
        combined = Image.new("RGB", (w_l + w_r, max(h_l, h_r)))
        combined.paste(img_l, (0, 0))
        combined.paste(img_r, (w_l, 0))
        out_frames.append(np.array(combined))
        
    iio.imwrite(filepath, out_frames, fps=30)

def main():
    print("M7_START")
    output_dir = pathlib.Path("output/m7")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    fly = make_locomotion_fly(name="m7_fly", add_adhesion=True)
    cam = fly.add_tracking_camera(name="trackcam")
    
    world = FlatGroundWorld()
    world.add_fly(fly, spawn_position=[0, 0, 0.5], spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]), bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD)
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
    
    print("Evaluating BASELINE_M3_STYLE...")
    baseline_params = {"cadence": 12.0, "phase_shift": 0.55, "transfer_width": 0.0, "entry_duration": 0.0}
    cpg_base = make_tripod_cpg_network(timestep=timestep)
    ctrl_base = MoonwalkChoreographyController(cpg_base, pps, actuated_dofs, **baseline_params)
    base_res, base_telemetry = evaluate_candidate(sim, fly, ctrl_base, baseline_params, n_steps, sample_dt, thorax_idx, tarsus5_indices, return_telemetry=True)
    print("BASELINE METRICS:", json.dumps(base_res, indent=2))
    
    print("\nRunning deterministic search...")
    candidates = []
    
    cadences = [10.0, 12.0]
    phase_shifts = [0.55, 0.58]
    transfer_widths = [0.3, 0.5]
    entry_durations = [0.25]
    
    for cad in cadences:
        for ps in phase_shifts:
            for tw in transfer_widths:
                for ed in entry_durations:
                    p = {"cadence": cad, "phase_shift": ps, "transfer_width": tw, "entry_duration": ed}
                    print(f"Testing {p}...")
                    cpg = make_tripod_cpg_network(timestep=timestep)
                    c = MoonwalkChoreographyController(cpg, pps, actuated_dofs, **p)
                    r = evaluate_candidate(sim, fly, c, p, n_steps, sample_dt, thorax_idx, tarsus5_indices)
                    if r and r['feasible']:
                        candidates.append(r)

    print(f"\nFound {len(candidates)} feasible candidates.")
    if not candidates:
        print("NO FEASIBLE CANDIDATES FOUND.")
        return
        
    # Rank candidates
    # 1. LOWER backward-speed coefficient of variation
    # 2. LOWER lateral ratio
    # 3. LOWER vertical body oscillation
    candidates.sort(key=lambda x: (x['backward_speed_cv'], x['lateral_ratio'], x['thorax_z_peak_to_peak_mm']))
    best_candidate = candidates[0]
    
    print("\nSELECTED CANDIDATE METRICS:")
    print(json.dumps(best_candidate, indent=2))
    
    # Save search results
    with open(output_dir / "search_results.json", "w") as f:
        json.dump({"baseline": base_res, "selected": best_candidate, "all_candidates": candidates}, f, indent=2)
        
    print("\nRendering performance rollout...")
    best_p = best_candidate['params']
    cpg_best = make_tripod_cpg_network(timestep=timestep)
    ctrl_best = MoonwalkChoreographyController(cpg_best, pps, actuated_dofs, **best_p)
    render_steps = int(2.5 / timestep)
    
    # Re-run for telemetry
    cpg_tele = make_tripod_cpg_network(timestep=timestep)
    ctrl_tele = MoonwalkChoreographyController(cpg_tele, pps, actuated_dofs, **best_p)
    _, best_telemetry = evaluate_candidate(sim, fly, ctrl_tele, best_p, n_steps, sample_dt, thorax_idx, tarsus5_indices, return_telemetry=True)
    
    # Render Best
    frames_right = render_video(sim, fly, ctrl_best, render_steps, output_dir / "smooth_moonwalk.mp4", cam)
    
    print("Rendering baseline for comparison...")
    # Render Baseline
    cpg_base2 = make_tripod_cpg_network(timestep=timestep)
    ctrl_base2 = MoonwalkChoreographyController(cpg_base2, pps, actuated_dofs, **baseline_params)
    frames_left = render_video(sim, fly, ctrl_base2, render_steps, output_dir / "baseline.mp4", cam)
    
    print("Compositing side-by-side video...")
    write_side_by_side(frames_left, frames_right, output_dir / "baseline_vs_smooth.mp4")
    
    print("Saving telemetry...")
    np.savez_compressed(
        output_dir / "choreography_telemetry.npz",
        baseline_t=base_telemetry['t'],
        baseline_vx=base_telemetry['vx'],
        baseline_z=base_telemetry['z'],
        best_t=best_telemetry['t'],
        best_vx=best_telemetry['vx'],
        best_z=best_telemetry['z'],
    )
    
    print("Saving manifest...")
    manifest = {
        "milestone": "M7",
        "flygym_version": version("flygym"),
        "mujoco_version": version("mujoco"),
        "human_reference": {
            "DOI": "10.1590/1806-9126-RBEF-2024-0319",
            "qualitative_principles_used": "Smooth backward glide, clear support/slide role exchange, relatively stable head/body translation.",
            "human_absolute_scale_transferred": False,
            "note": "Human absolute speeds were NOT transferred to the fly."
        },
        "controller": {
            "selected_parameters": best_p,
            "tripod_definitions": {
                "TRIPOD_A": ["LF", "LH", "RM"],
                "TRIPOD_B": ["LM", "RF", "RH"]
            },
            "adhesion_semantics": "CONTINUOUS_ADHESION_SUPPORTED",
            "joint_trajectory_basis": "STOCK_DERIVED",
            "target_trajectory_modifications": "NONE_BEYOND_TIMING"
        },
        "metrics": {
            "baseline": base_res,
            "selected_candidate": best_candidate
        },
        "scientific_boundary": "This is an engineered six-legged choreography inspired by moonwalk kinematics, not biological fly locomotion."
    }
    
    pathlib.Path("data/derived").mkdir(parents=True, exist_ok=True)
    with open("data/derived/m7_choreography_manifest.json", "w") as f:
        json.dump(manifest, f, indent=2)
        
    print("M7_COMPLETE")

if __name__ == "__main__":
    main()
