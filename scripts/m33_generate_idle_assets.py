#!/usr/bin/env python3
"""
Widescreen habitat and idle motion asset generator for MW-M33:
- video_moonwalk_16x9.mp4: 16:9 widescreen render of M27-D in natural habitat
- video_idle_16x9.mp4: 360 frames (6.0 s at 60 FPS) posture oscillation and stance micro-stepping
- Modular idle clips (rest, reposition, settle)
- poster_habitat_16x9.png: initial still frame matching qpos[105]
"""

import os
import sys
import json
import pathlib
import numpy as np
from PIL import Image
import mujoco as mj
import imageio.v2 as iio

os.environ["MPLCONFIGDIR"] = "/tmp"

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.append(".")

from flygym import Simulation
from flygym.compose import FlatGroundWorld
from flygym.utils.math import Rotation3D
import scripts.m31_visual_foundation as m31

TRAJECTORY_PATH = pathlib.Path("data/derived/m33_m27d_trajectory.npz")
OUTPUT_DIR = pathlib.Path("output/m33")
WEB_ASSETS_DIR = pathlib.Path("web/assets")
DERIVED_DIR = pathlib.Path("data/derived")

OUTPUT_FPS = 60
CAMERA_RES = (720, 1280)  # (H, W) -> 16:9 widescreen


def quat_mult(q1, q2):
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)


def rpy_to_quat(r, p, y):
    """Convert small roll, pitch, yaw angles (radians) to quaternion [w, x, y, z]."""
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ], dtype=float)


def generate_idle_qpos(q105, n_frames=360, duration=6.0):
    """
    Synthesizes physically grounded, anatomically natural idle poses
    for the Drosophila fruit fly around the M27-D starting pose q105.
    """
    times = np.linspace(0.0, duration, n_frames, endpoint=False)
    idle_qpos = np.tile(q105, (n_frames, 1))

    # Posture oscillation: 4 cycles over 6 seconds
    omega_resp = 2.0 * np.pi * (4.0 / duration)

    for i, t in enumerate(times):
        # 1. Posture oscillation (subtle rhythmic vertical motion & pitch)
        env_resp = np.sin(np.pi * (t / duration)) ** 0.5
        z_resp = 0.014 * np.sin(omega_resp * t) * env_resp
        pitch_resp = 0.004 * np.sin(omega_resp * t) * env_resp

        # 2. Alert Head/Body Scan (t in [0.7, 1.8]s)
        yaw_scan = 0.014 * np.exp(-((t - 1.25) / 0.25) ** 2)
        roll_scan = 0.005 * np.exp(-((t - 1.25) / 0.25) ** 2)

        # 3. Natural Stance Repositioning & Small Steps (t in [2.0, 3.8]s)
        # Right Front (RF) micro-step: t in [2.0, 2.7]s
        rf_lift = 0.0
        rf_advance = 0.0
        if 2.0 <= t <= 2.7:
            p = (t - 2.0) / 0.7
            rf_lift = np.sin(np.pi * p)
            rf_advance = 0.5 * (1.0 - np.cos(np.pi * p))
        elif 2.7 < t <= 4.6:
            rf_advance = 1.0
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            rf_advance = 0.5 * (1.0 + np.cos(np.pi * p))

        # Left Front (LF) micro-step: t in [2.8, 3.5]s
        lf_lift = 0.0
        lf_advance = 0.0
        if 2.8 <= t <= 3.5:
            p = (t - 2.8) / 0.7
            lf_lift = np.sin(np.pi * p)
            lf_advance = 0.5 * (1.0 - np.cos(np.pi * p))
        elif 3.5 < t <= 4.6:
            lf_advance = 1.0
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            lf_advance = 0.5 * (1.0 + np.cos(np.pi * p))

        # Thorax subtle weight shift forward (0.10 mm)
        x_shift = 0.0
        if 2.2 <= t <= 3.6:
            p = (t - 2.2) / 1.4
            x_shift = 0.10 * (0.5 * (1.0 - np.cos(np.pi * p)))
        elif 3.6 < t <= 4.6:
            x_shift = 0.10
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            x_shift = 0.10 * (0.5 * (1.0 + np.cos(np.pi * p)))

        # 4. Grooming / alert twitch (t in [3.8, 4.5]s)
        head_dip = -0.006 * np.exp(-((t - 4.15) / 0.20) ** 2)

        # Apply root position & orientation
        idle_qpos[i, 0] = q105[0] + x_shift
        idle_qpos[i, 2] = q105[2] + z_resp

        dq = rpy_to_quat(roll_scan, pitch_resp + head_dip, yaw_scan)
        idle_qpos[i, 3:7] = quat_mult(q105[3:7], dq)

        # Apply RF leg offsets
        idle_qpos[i, 40] = q105[40] - 0.05 * rf_advance  # yaw forward
        idle_qpos[i, 41] = q105[41] - 0.15 * rf_lift - 0.02 * rf_advance  # pitch
        idle_qpos[i, 43] = q105[43] - 0.09 * rf_lift - 0.015 * rf_advance  # femur
        idle_qpos[i, 45] = q105[45] - 0.06 * rf_lift  # tibia

        # Apply LF leg offsets
        idle_qpos[i, 7] = q105[7] + 0.05 * lf_advance
        idle_qpos[i, 8] = q105[8] - 0.14 * lf_lift - 0.02 * lf_advance
        idle_qpos[i, 10] = q105[10] - 0.08 * lf_lift - 0.015 * lf_advance
        idle_qpos[i, 12] = q105[12] - 0.05 * lf_lift

    return idle_qpos


def main():
    print("Generating widescreen habitat and idle motion assets...")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    WEB_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    # 1. Load trajectory
    if not TRAJECTORY_PATH.exists():
        raise FileNotFoundError(f"Trajectory file {TRAJECTORY_PATH} not found!")

    traj = np.load(TRAJECTORY_PATH)
    times = traj["time"]
    qpos_arr = traj["qpos"]
    q105 = qpos_arr[105].copy()

    # 2. Setup natural habitat simulation & 16:9 widescreen camera
    print("\nBuilding approved natural habitat scene (16:9 Widescreen 1280x720) ...")
    fly = m31.build_colorized_fly()
    cam = m31.add_hero_camera(fly)
    world = m31.create_natural_habitat_world()
    world.add_fly(
        fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0])
    )

    sim = Simulation(world)
    renderer = sim.set_renderer(
        cam,
        camera_res=CAMERA_RES,
        output_fps=OUTPUT_FPS,
        buffer_frames=False
    )
    sim.reset()
    cam_id = renderer._cameras_names2id["fly/herocam"]

    # 3. Render and save initial poster (qpos[105])
    print("\nRendering 16:9 poster still frame at qpos[105] ...")
    sim.mj_data.qpos[:] = q105
    mj.mj_forward(sim.mj_model, sim.mj_data)
    renderer.mj_renderer.update_scene(sim.mj_data, cam_id, renderer.scene_option)
    f_poster = renderer.mj_renderer.render()

    p_poster_web = WEB_ASSETS_DIR / "poster_habitat_16x9.png"
    p_poster_out = OUTPUT_DIR / "poster_habitat_16x9.png"
    Image.fromarray(f_poster).save(p_poster_web)
    Image.fromarray(f_poster).save(p_poster_out)
    print(f"  Saved poster to {p_poster_web} and {p_poster_out}")

    # 4. Render 16:9 Moonwalk Video (86 frames at 60 FPS)
    frame_indices = list(range(105, 531, 5))
    n_moonwalk_frames = len(frame_indices)
    print(f"\nRendering 16:9 Moonwalk Video ({n_moonwalk_frames} frames from M27-D trajectory) ...")

    moonwalk_frames = []
    for i, idx in enumerate(frame_indices):
        sim.mj_data.qpos[:] = qpos_arr[idx]
        mj.mj_forward(sim.mj_model, sim.mj_data)
        renderer.mj_renderer.update_scene(sim.mj_data, cam_id, renderer.scene_option)
        rgb = renderer.mj_renderer.render()
        moonwalk_frames.append(rgb)

    p_moonwalk_web = WEB_ASSETS_DIR / "video_moonwalk_16x9.mp4"
    p_moonwalk_out = OUTPUT_DIR / "video_m27d_moonwalk_16x9.mp4"
    print(f"  Encoding {p_moonwalk_web} ...")
    iio.mimsave(p_moonwalk_web, moonwalk_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    iio.mimsave(p_moonwalk_out, moonwalk_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    print(f"  Saved moonwalk video ({p_moonwalk_web.stat().st_size / 1024:.1f} KB)")

    # 5. Render 16:9 Idle Videos (360 frames = 6.0 s at 60 FPS)
    n_idle_frames = 360
    print(f"\nSynthesizing & Rendering 16:9 Idle Video ({n_idle_frames} frames = 6.0 s) ...")
    idle_qpos = generate_idle_qpos(q105, n_frames=n_idle_frames, duration=6.0)

    idle_frames = []
    for i in range(n_idle_frames):
        sim.mj_data.qpos[:] = idle_qpos[i]
        mj.mj_forward(sim.mj_model, sim.mj_data)
        renderer.mj_renderer.update_scene(sim.mj_data, cam_id, renderer.scene_option)
        rgb = renderer.mj_renderer.render()
        idle_frames.append(rgb)
        if (i + 1) % 60 == 0:
            print(f"  Rendered {i + 1}/{n_idle_frames} idle frames")

    # Encode master continuous idle loop
    p_idle_web = WEB_ASSETS_DIR / "video_idle_16x9.mp4"
    p_idle_out = OUTPUT_DIR / "video_idle_16x9.mp4"
    print(f"  Encoding master idle loop {p_idle_web} ...")
    iio.mimsave(p_idle_web, idle_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    iio.mimsave(p_idle_out, idle_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    print(f"  Saved master idle video ({p_idle_web.stat().st_size / 1024:.1f} KB)")

    # Also save individual modular idle clips for playlist support
    p_clip1 = OUTPUT_DIR / "video_idle_1_rest.mp4"
    p_clip2 = OUTPUT_DIR / "video_idle_2_reposition.mp4"
    p_clip3 = OUTPUT_DIR / "video_idle_3_settle.mp4"
    iio.mimsave(p_clip1, idle_frames[0:120], fps=OUTPUT_FPS, codec="libx264", quality=8)
    iio.mimsave(p_clip2, idle_frames[120:240], fps=OUTPUT_FPS, codec="libx264", quality=8)
    iio.mimsave(p_clip3, idle_frames[240:360], fps=OUTPUT_FPS, codec="libx264", quality=8)
    print(f"  Saved modular idle clips: clip1 ({p_clip1.stat().st_size / 1024:.1f} KB), clip2 ({p_clip2.stat().st_size / 1024:.1f} KB), clip3 ({p_clip3.stat().st_size / 1024:.1f} KB)")

    renderer.close()
    sim.close()

    # 6. Save manifest
    manifest = {
        "milestone": "MW-M33",
        "phase": "IMMERSIVE_EXPERIENCE_FOUNDATION",
        "resolution": [1280, 720],
        "aspect_ratio": "16:9",
        "fps": OUTPUT_FPS,
        "assets": {
            "poster_habitat_16x9": str(p_poster_web),
            "video_moonwalk_16x9": str(p_moonwalk_web),
            "video_idle_16x9": str(p_idle_web),
            "modular_clips": [str(p_clip1), str(p_clip2), str(p_clip3)]
        },
        "moonwalk_duration_source_sec": n_moonwalk_frames / OUTPUT_FPS,
        "moonwalk_duration_05x_presentation_sec": (n_moonwalk_frames / OUTPUT_FPS) / 0.5,
        "idle_duration_sec": n_idle_frames / OUTPUT_FPS,
        "first_frame_diff_vs_q105": float(np.max(np.abs(idle_qpos[0] - q105))),
        "loop_continuity_diff": float(np.max(np.abs(idle_qpos[-1] - idle_qpos[0])))
    }

    manifest_path = DERIVED_DIR / "m33_idle_assets_manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nSaved manifest to {manifest_path}")
    print("Widescreen habitat and idle motion asset generation complete.")


if __name__ == "__main__":
    main()
