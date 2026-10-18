#!/usr/bin/env python3
"""
Full-colour visual foundation pipeline for Moonwalker Fly (MW-M31):
- Drosophila melanogaster V2 colorization and procedural forest soil texture
- Three-point directional daylight illumination
- Frozen M27-D controller with bit-exact physics equivalence verification
- 60 FPS video production (1.0x and 0.2x playback) and side-by-side comparison
"""

import os
import sys
import json
import pathlib
import argparse
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import gaussian_filter
import mujoco as mj

# Matplotlib configuration for headless environment
os.environ["MPLCONFIGDIR"] = "/tmp"
import matplotlib
matplotlib.use("Agg")

import imageio.v2 as iio

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.append(".")

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym.utils.mjcf import add_texture, add_material
from flygym_demo.complex_terrain import (
    PreprogrammedSteps, make_locomotion_fly, apply_locomotion_action
)
import scripts.m25_support_path_geometry as m25
import scripts.m30_tripod_transfer_relief as m30

# -----------------------------------------------------------------------------
# GLOBAL CONSTANTS & PROVENANCE
# -----------------------------------------------------------------------------

PHYSICS_DT = 0.0001
TRIAL_DURATION = 1.75
STEADY_START_TIME = 0.35
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)  # 17500 steps

DEFAULT_VISUALS_YAML = "data/visuals/visuals_drosophila_melanogaster_v2.yaml"
DEFAULT_TEXTURE_PATH = "data/visuals/forest_floor_soil_v2.png"

OUTPUT_DIR = pathlib.Path("output/m31")
DERIVED_DIR = pathlib.Path("data/derived")


def ensure_procedural_soil_texture(texture_path=DEFAULT_TEXTURE_PATH, size=512, seed=101):
    """
    Synthesizes a high-resolution organic forest-floor soil texture with
    deep loam base, peat grain, and restrained natural moss accents.
    """
    path = pathlib.Path(texture_path)
    if path.exists():
        return str(path)

    path.parent.mkdir(parents=True, exist_ok=True)
    np.random.seed(seed)

    noise_broad = gaussian_filter(np.random.randn(size, size), sigma=32.0)
    noise_med = gaussian_filter(np.random.randn(size, size), sigma=10.0)
    noise_fine = gaussian_filter(np.random.randn(size, size), sigma=3.0)

    noise_broad = (noise_broad - noise_broad.min()) / (noise_broad.max() - noise_broad.min())
    noise_med = (noise_med - noise_med.min()) / (noise_med.max() - noise_med.min())
    noise_fine = (noise_fine - noise_fine.min()) / (noise_fine.max() - noise_fine.min())

    r = 42 + 22 * noise_broad + 10 * noise_med + 6 * noise_fine
    g = 30 + 16 * noise_broad + 8 * noise_med + 5 * noise_fine
    b = 20 + 10 * noise_broad + 5 * noise_med + 3 * noise_fine

    moss_field = gaussian_filter(np.random.randn(size, size), sigma=20.0)
    moss_field = (moss_field - moss_field.min()) / (moss_field.max() - moss_field.min())
    moss_mask = np.clip((moss_field * 0.7 + noise_broad * 0.3 - 0.48) * 3.5, 0, 1)

    r = r * (1.0 - 0.55 * moss_mask) + (38 + 10 * noise_med) * (0.55 * moss_mask)
    g = g * (1.0 - 0.75 * moss_mask) + (58 + 14 * noise_med) * (0.75 * moss_mask)
    b = b * (1.0 - 0.40 * moss_mask) + (26 + 8 * noise_med) * (0.40 * moss_mask)

    fleck_mask = (np.random.rand(size, size) > 0.992).astype(float)
    r += fleck_mask * 12
    g += fleck_mask * 9
    b += fleck_mask * 5

    rgb = np.clip(np.stack([r, g, b], axis=-1), 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(path)
    return str(path)


def create_natural_habitat_world(texture_path=DEFAULT_TEXTURE_PATH):
    """
    Constructs the natural habitat world:
    - 100% physically preserved flat ground plane (z=0, identical contact mechanics).
    - Matte organic earthy forest-floor soil texture and material.
    - Balanced daylight lighting with soft cast shadows and generous fill.
    """
    world = FlatGroundWorld()
    tex_path = ensure_procedural_soil_texture(texture_path)

    # Attach organic soil texture and matte earthy material
    add_texture(
        world.mjcf_root,
        name="soil_tex",
        type="2d",
        file=tex_path
    )
    add_material(
        world.mjcf_root,
        name="soil_mat",
        texture="soil_tex",
        texrepeat=(90, 90),
        reflectance=0.0,
        specular=0.03,
        shininess=0.04,
        roughness=0.92
    )
    world.ground_geom.material = "soil_mat"

    # Multi-source directional daylight lighting
    world.mjcf_root.worldbody.add_light(
        name="sunlight",
        pos=[2.0, -10.0, 22.0],
        dir=[-0.05, 0.40, -0.91],
        type=mj.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=1,
        diffuse=[0.72, 0.68, 0.60],
        specular=[0.18, 0.16, 0.14],
        ambient=[0.32, 0.32, 0.32]
    )

    world.mjcf_root.worldbody.add_light(
        name="skylight",
        pos=[-4.0, 12.0, 18.0],
        dir=[0.15, -0.45, -0.87],
        type=mj.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=0,
        diffuse=[0.38, 0.42, 0.46],
        specular=[0.04, 0.04, 0.04]
    )

    world.mjcf_root.worldbody.add_light(
        name="rimlight",
        pos=[-15.0, 0.0, 10.0],
        dir=[0.8, 0.0, -0.6],
        type=mj.mjtLightType.mjLIGHT_DIRECTIONAL,
        castshadow=0,
        diffuse=[0.22, 0.22, 0.20],
        specular=[0.06, 0.06, 0.05]
    )

    return world


def build_colorized_fly(visuals_yaml=DEFAULT_VISUALS_YAML):
    """Creates the NeuroMechFly with refined Drosophila melanogaster colorization."""
    fly = make_locomotion_fly(name="fly", add_adhesion=True, colorize=False)
    fly.colorize(visuals_yaml)
    return fly


def build_monochrome_fly():
    """Creates the standard monochrome NeuroMechFly."""
    return make_locomotion_fly(name="fly", add_adhesion=True, colorize=False)


def add_hero_camera(fly):
    """Attaches the authoritative hero tracking camera."""
    return fly.add_tracking_camera(
        name="herocam",
        pos_offset=(-0.65, -6.6, 3.4),
        rotation=Rotation3D("xyaxes", (1.0, 0.0, 0.0, 0.0, 0.50, 0.866)),
        fovy=29.0
    )


def overlay_header_banner(frame, text):
    """Overlays a clean semi-transparent header banner with descriptive text."""
    img = Image.fromarray(frame)
    w, h = img.size
    draw = ImageDraw.Draw(img)

    banner_h = 36
    overlay = Image.new("RGBA", (w, banner_h), (0, 0, 0, 160))
    img.paste(overlay, (0, 0), overlay)

    draw.text((16, 10), text, fill=(240, 240, 240))
    return np.array(img)


def run_full_phase2_production():
    """
    Executes the complete Phase 2 video and asset production pipeline:
    1. Simulates and captures Monochrome M27-D.
    2. Simulates and captures Full-Colour M27-D (V2).
    3. Verifies bit-exact 0.0 discrepancy everywhere.
    4. Encodes 1.0x and 0.2x standalone color videos at 60 FPS.
    5. Encodes 1.0x and 0.2x synchronized side-by-side comparison videos at 60 FPS.
    6. Saves manifest and results JSON.
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    print("Starting full-colour visual foundation production...")

    camera_res = (720, 960)  # (H, W) per pane
    fps = 60
    playback_speed = 0.2  # Records 300 fps of simulation time

    # -------------------------------------------------------------------------
    # 1. RUN MONOCHROME M27-D BASELINE SIMULATION
    # -------------------------------------------------------------------------
    print("\n[1/3] Simulating Monochrome M27-D Baseline (1.75 s / 17,500 steps) ...")
    fly_mono = build_monochrome_fly()
    cam_mono = add_hero_camera(fly_mono)
    world_mono = FlatGroundWorld()
    world_mono.add_fly(
        fly_mono,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )
    sim_mono = Simulation(world_mono)
    dof_map_mono = m25.build_semantic_dof_mapping(fly_mono, sim_mono)
    pps_mono = m30.PreprogrammedSteps()
    act_dofs_mono = fly_mono.get_actuated_jointdofs_order(ActuatorType.POSITION)
    ctrl_mono = m30.M30TransferReliefController(sim_mono, fly_mono, pps_mono, act_dofs_mono, dof_map_mono, condition="A")

    renderer_mono = sim_mono.set_renderer(
        cam_mono,
        camera_res=camera_res,
        playback_speed=playback_speed,
        output_fps=fps,
        buffer_frames=True
    )

    sim_mono.reset()
    ctrl_mono.time = 0.0
    sim_mono.set_actuator_inputs(fly_mono.name, ActuatorType.POSITION, ctrl_mono.default_pose)
    sim_mono.set_leg_adhesion_states(fly_mono.name, np.ones(6, dtype=float))
    sim_mono.warmup()

    # Store qpos history at 1 kHz for rigorous bit-exact verification
    qpos_mono_history = []
    qvel_mono_history = []

    for step_i in range(N_STEPS):
        action, _ = ctrl_mono.step()
        apply_locomotion_action(sim_mono, fly_mono.name, action)
        sim_mono.step_with_profile()
        sim_mono.render_as_needed_with_profile()

        if (step_i + 1) % 10 == 0:
            qpos_mono_history.append(sim_mono.mj_data.qpos.copy())
            qvel_mono_history.append(sim_mono.mj_data.qvel.copy())

    frames_mono_raw = list(renderer_mono.frames["fly/herocam"])
    renderer_mono.close()
    sim_mono.close()
    print(f"  Monochrome capture complete. Frames captured: {len(frames_mono_raw)}")

    # -------------------------------------------------------------------------
    # 2. RUN FULL-COLOUR M27-D (V2) SIMULATION
    # -------------------------------------------------------------------------
    print("\n[2/3] Simulating Full-Colour M27-D (V2) in Natural Habitat ...")
    fly_color = build_colorized_fly(DEFAULT_VISUALS_YAML)
    cam_color = add_hero_camera(fly_color)
    world_color = create_natural_habitat_world(DEFAULT_TEXTURE_PATH)
    world_color.add_fly(
        fly_color,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )
    sim_color = Simulation(world_color)
    dof_map_color = m25.build_semantic_dof_mapping(fly_color, sim_color)
    pps_color = m30.PreprogrammedSteps()
    act_dofs_color = fly_color.get_actuated_jointdofs_order(ActuatorType.POSITION)
    ctrl_color = m30.M30TransferReliefController(sim_color, fly_color, pps_color, act_dofs_color, dof_map_color, condition="A")

    renderer_color = sim_color.set_renderer(
        cam_color,
        camera_res=camera_res,
        playback_speed=playback_speed,
        output_fps=fps,
        buffer_frames=True
    )

    sim_color.reset()
    ctrl_color.time = 0.0
    sim_color.set_actuator_inputs(fly_color.name, ActuatorType.POSITION, ctrl_color.default_pose)
    sim_color.set_leg_adhesion_states(fly_color.name, np.ones(6, dtype=float))
    sim_color.warmup()

    max_qpos_diff = 0.0
    max_qvel_diff = 0.0

    for step_i in range(N_STEPS):
        action, _ = ctrl_color.step()
        apply_locomotion_action(sim_color, fly_color.name, action)
        sim_color.step_with_profile()
        sim_color.render_as_needed_with_profile()

        if (step_i + 1) % 10 == 0:
            sample_idx = (step_i + 1) // 10 - 1
            diff_qpos = float(np.max(np.abs(sim_color.mj_data.qpos - qpos_mono_history[sample_idx])))
            diff_qvel = float(np.max(np.abs(sim_color.mj_data.qvel - qvel_mono_history[sample_idx])))
            if diff_qpos > max_qpos_diff:
                max_qpos_diff = diff_qpos
            if diff_qvel > max_qvel_diff:
                max_qvel_diff = diff_qvel

    frames_color_raw = list(renderer_color.frames["fly/herocam"])
    renderer_color.close()
    sim_color.close()
    print(f"  Full-Colour capture complete. Frames captured: {len(frames_color_raw)}")

    print(f"\n[Physics Equivalence Audit]")
    print(f"  Max qpos discrepancy: {max_qpos_diff:.2e}")
    print(f"  Max qvel discrepancy: {max_qvel_diff:.2e}")
    assert max_qpos_diff == 0.0, f"Physics discrepancy in qpos! {max_qpos_diff}"
    assert max_qvel_diff == 0.0, f"Physics discrepancy in qvel! {max_qvel_diff}"
    print("  STATUS: BIT-EXACT 0.0 LOCOMOTION PRESERVATION VERIFIED.")

    # -------------------------------------------------------------------------
    # 3. VIDEO PROCESSING & ENCODING
    # -------------------------------------------------------------------------
    print("\n[3/3] Encoding 60 FPS Deliverable Videos ...")

    # Steady-state frame slicing:
    # At playback_speed=0.2 and output_fps=60, render interval = 1/300 s.
    # Steady state starts at t = 0.35 s -> start index = 0.35 * 300 = 105 frames.
    start_frame_idx = int(STEADY_START_TIME / (playback_speed / fps))
    total_raw_frames = len(frames_color_raw)

    print(f"  Total raw frames: {total_raw_frames}")
    print(f"  Steady-state start frame: {start_frame_idx} (t={STEADY_START_TIME:.2f}s)")

    # Steady-state 0.2x frames (all 300 frames/sec for 0.35-1.75s)
    # Total steady duration = 1.40s * 300 frames/s = 420 frames (played at 60 FPS -> 7.0 seconds of video)
    color_steady_02x = frames_color_raw[start_frame_idx:]
    mono_steady_02x = frames_mono_raw[start_frame_idx:]

    # Steady-state 1.0x frames (every 5th frame -> 60 frames/sec)
    # Total steady frames = 420 / 5 = 84 frames (played at 60 FPS -> 1.40 seconds of real-time video)
    color_steady_10x = color_steady_02x[::5]
    mono_steady_10x = mono_steady_02x[::5]

    # Full trial 1.0x frames (for optional review)
    color_full_10x = frames_color_raw[::5]

    # Output paths
    p_color_10x = OUTPUT_DIR / "video_m27d_color_10x.mp4"
    p_color_02x = OUTPUT_DIR / "video_m27d_color_02x.mp4"
    p_cmp_10x = OUTPUT_DIR / "video_monochrome_vs_color_10x.mp4"
    p_cmp_02x = OUTPUT_DIR / "video_monochrome_vs_color_02x.mp4"
    p_screenshot = OUTPUT_DIR / "screenshot_m27d_color_v2.png"

    # Save hero screenshot from steady-state (t = 0.40s)
    # At t=0.40s, frame index in raw is int(0.40 * 300) = 120
    hero_frame = frames_color_raw[int(0.40 / (playback_speed / fps))]
    Image.fromarray(hero_frame).save(p_screenshot)
    print(f"  Hero Screenshot: {p_screenshot} ({hero_frame.shape[1]}x{hero_frame.shape[0]})")

    # Encode 1. `video_m27d_color_10x.mp4`
    print(f"  Encoding {p_color_10x} ({len(color_steady_10x)} frames at 60 FPS) ...")
    iio.mimsave(p_color_10x, color_steady_10x, fps=fps, codec="libx264", quality=8)

    # Encode 2. `video_m27d_color_02x.mp4`
    print(f"  Encoding {p_color_02x} ({len(color_steady_02x)} frames at 60 FPS) ...")
    iio.mimsave(p_color_02x, color_steady_02x, fps=fps, codec="libx264", quality=8)

    # Encode 3. `video_monochrome_vs_color_10x.mp4` (Side-by-side synchronized comparison)
    print(f"  Assembling side-by-side 1.0x comparison ({len(color_steady_10x)} frames) ...")
    cmp_frames_10x = []
    for f_m, f_c in zip(mono_steady_10x, color_steady_10x):
        pane_m = overlay_header_banner(f_m, "MONOCHROME BASELINE | M27-D MOONWALK")
        pane_c = overlay_header_banner(f_c, "FULL-COLOUR HABITAT (V2) | 1.0x REAL-TIME (60 FPS)")
        combined = np.concatenate([pane_m, pane_c], axis=1)
        cmp_frames_10x.append(combined)

    print(f"  Encoding {p_cmp_10x} (Resolution: {cmp_frames_10x[0].shape[1]}x{cmp_frames_10x[0].shape[0]} at 60 FPS) ...")
    iio.mimsave(p_cmp_10x, cmp_frames_10x, fps=fps, codec="libx264", quality=8)

    # Encode 4. `video_monochrome_vs_color_02x.mp4` (Side-by-side slow-motion comparison)
    print(f"  Assembling side-by-side 0.2x comparison ({len(color_steady_02x)} frames) ...")
    cmp_frames_02x = []
    for f_m, f_c in zip(mono_steady_02x, color_steady_02x):
        pane_m = overlay_header_banner(f_m, "MONOCHROME BASELINE | M27-D MOONWALK")
        pane_c = overlay_header_banner(f_c, "FULL-COLOUR HABITAT (V2) | 0.2x SLOW-MOTION (60 FPS)")
        combined = np.concatenate([pane_m, pane_c], axis=1)
        cmp_frames_02x.append(combined)

    print(f"  Encoding {p_cmp_02x} ...")
    iio.mimsave(p_cmp_02x, cmp_frames_02x, fps=fps, codec="libx264", quality=8)

    # -------------------------------------------------------------------------
    # 4. WRITE MANIFEST & RESULTS JSON
    # -------------------------------------------------------------------------
    manifest_data = {
        "milestone": "MW-M31",
        "task": "Full-colour, realistic visual foundation for Moonwalker Fly",
        "status": "PHASE_2_COMPLETED",
        "controller": {
            "name": "M27_D_EXCURSION_MATCHED_PLANAR_SUPPORT",
            "provenance": "scripts/m27_selective_hind_relief.py / scripts/m30_tripod_transfer_relief.py (Condition A)",
            "cadence_hz": 6.0,
            "turnaround_fraction": 0.06,
            "support_transfer_fraction": 0.08,
            "entry_duration_s": 0.25,
            "trial_duration_s": 1.75,
            "steady_start_time_s": 0.35,
            "excursion_fractions": {
                "LF": 1.000, "LM": 0.564, "LH": 0.779,
                "RF": 1.000, "RM": 0.564, "RH": 0.779
            }
        },
        "appearance_configuration": {
            "visuals_yaml": DEFAULT_VISUALS_YAML,
            "soil_texture_png": DEFAULT_TEXTURE_PATH,
            "ground_material": {
                "reflectance": 0.0,
                "specular": 0.03,
                "shininess": 0.04,
                "roughness": 0.92,
                "texrepeat": [90, 90]
            },
            "lighting": {
                "sunlight": {"type": "directional", "diffuse": [0.72, 0.68, 0.60], "castshadow": True},
                "skylight": {"type": "directional", "diffuse": [0.38, 0.42, 0.46], "castshadow": False},
                "rimlight": {"type": "directional", "diffuse": [0.22, 0.22, 0.20], "castshadow": False}
            },
            "camera": {
                "name": "herocam",
                "pos_offset": [-0.65, -6.6, 3.4],
                "rotation_xyaxes": [1.0, 0.0, 0.0, 0.0, 0.50, 0.866],
                "fovy_deg": 29.0
            }
        },
        "rendering_specifications": {
            "resolution_per_pane": [720, 960],
            "combined_resolution": [720, 1920],
            "output_fps": 60,
            "playback_speeds": ["1.0x", "0.2x"],
            "steady_state_window_s": [0.35, 1.75]
        },
        "physics_preservation_verification": {
            "max_qpos_discrepancy": max_qpos_diff,
            "max_qvel_discrepancy": max_qvel_diff,
            "status": "PASS_BIT_EXACT_EQUIVALENCE"
        },
        "output_files": {
            "screenshot": str(p_screenshot),
            "video_color_10x": str(p_color_10x),
            "video_color_02x": str(p_color_02x),
            "video_comparison_10x": str(p_cmp_10x),
            "video_comparison_02x": str(p_cmp_02x)
        }
    }

    manifest_path = DERIVED_DIR / "m31_visual_foundation_manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest_data, f, indent=2)
    print(f"\nSaved manifest to: {manifest_path}")

    results_data = {
        "milestone": "MW-M31",
        "task": "Full-colour visual foundation",
        "summary": "Full-colour realistic Drosophila melanogaster and natural habitat successfully produced with bit-exact preservation of M27-D moonwalk.",
        "physics_verified": True,
        "max_qpos_diff": max_qpos_diff,
        "max_qvel_diff": max_qvel_diff,
        "steady_state_duration_s": 1.40,
        "deliverables": {
            "screenshot_png": str(p_screenshot),
            "video_color_10x_mp4": str(p_color_10x),
            "video_color_02x_mp4": str(p_color_02x),
            "video_comparison_10x_mp4": str(p_cmp_10x),
            "video_comparison_02x_mp4": str(p_cmp_02x)
        }
    }

    results_path = DERIVED_DIR / "m31_visual_foundation_results.json"
    with open(results_path, "w") as f:
        json.dump(results_data, f, indent=2)
    print(f"Saved results to: {results_path}")

    print("Full-colour visual foundation production complete.")
    return manifest_data


def main():
    parser = argparse.ArgumentParser(description="MW-M31 Visual Foundation Pipeline")
    parser.add_argument("--screenshot-only", action="store_true", default=False,
                        help="Render hero screenshot only")
    parser.add_argument("--verify-equivalence", action="store_true", default=False,
                        help="Run full 17,500-step physics equivalence check")
    parser.add_argument("--output", type=str, default="output/m31/screenshot_m27d_color_v2.png",
                        help="Output path for the screenshot")
    args = parser.parse_args()

    if args.screenshot_only:
        p_screenshot = OUTPUT_DIR / "screenshot_m27d_color_v2.png"
        render_screenshot(out_path=str(p_screenshot))
    elif args.verify_equivalence:
        verify_full_rollout_physics_equivalence()
    else:
        run_full_phase2_production()


if __name__ == "__main__":
    main()
