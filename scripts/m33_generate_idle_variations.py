#!/usr/bin/env python3
"""
Precomputed idle variations generator for Drosophila fly habitat.

Generates compatible 6.0 s idle variations anchored at qpos[105] to ensure
continuous loop transitions without pose discontinuity:
- Variation 2: Left-front lead step and lateral yaw scan
- Variation 3: Observational posture with curiosity pitch and subtle weight shift
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
import scripts.m33_generate_idle_assets as m33_idle

TRAJECTORY_PATH = pathlib.Path("data/derived/m33_m27d_trajectory.npz")
OUTPUT_DIR = pathlib.Path("output/m33")
WEB_ASSETS_DIR = pathlib.Path("web/assets")

OUTPUT_FPS = 60
CAMERA_RES = (720, 1280)  # (H, W)


def generate_idle_variation_2(q105, n_frames=360, duration=6.0):
    """
    Variation 2: Left-front lead step, lateral yaw scan to left then right,
    settling seamlessly back to qpos[105].
    """
    times = np.linspace(0.0, duration, n_frames, endpoint=False)
    idle_qpos = np.tile(q105, (n_frames, 1))

    omega_resp = 2.0 * np.pi * (4.0 / duration)

    for i, t in enumerate(times):
        env_resp = np.sin(np.pi * (t / duration)) ** 0.5
        z_resp = 0.012 * np.sin(omega_resp * t + np.pi / 6.0) * env_resp
        pitch_resp = 0.003 * np.sin(omega_resp * t) * env_resp

        # Lateral alert scan: yaw left at t=1.1s, yaw right at t=4.2s
        yaw_scan_left = -0.016 * np.exp(-((t - 1.1) / 0.26) ** 2)
        yaw_scan_right = 0.014 * np.exp(-((t - 4.2) / 0.28) ** 2)
        yaw_scan = yaw_scan_left + yaw_scan_right

        roll_scan = -0.006 * np.exp(-((t - 1.1) / 0.26) ** 2) + 0.005 * np.exp(-((t - 4.2) / 0.28) ** 2)

        # LF lead micro-step: t in [1.9, 2.6]s
        lf_lift = 0.0
        lf_advance = 0.0
        if 1.9 <= t <= 2.6:
            p = (t - 1.9) / 0.7
            lf_lift = np.sin(np.pi * p)
            lf_advance = 0.5 * (1.0 - np.cos(np.pi * p))
        elif 2.6 < t <= 4.6:
            lf_advance = 1.0
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            lf_advance = 0.5 * (1.0 + np.cos(np.pi * p))

        # RF secondary micro-step: t in [2.7, 3.4]s
        rf_lift = 0.0
        rf_advance = 0.0
        if 2.7 <= t <= 3.4:
            p = (t - 2.7) / 0.7
            rf_lift = np.sin(np.pi * p)
            rf_advance = 0.5 * (1.0 - np.cos(np.pi * p))
        elif 3.4 < t <= 4.6:
            rf_advance = 1.0
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            rf_advance = 0.5 * (1.0 + np.cos(np.pi * p))

        # Thorax subtle forward weight shift (0.08 mm)
        x_shift = 0.0
        if 2.1 <= t <= 3.5:
            p = (t - 2.1) / 1.4
            x_shift = 0.08 * (0.5 * (1.0 - np.cos(np.pi * p)))
        elif 3.5 < t <= 4.6:
            x_shift = 0.08
        elif 4.6 < t <= 5.4:
            p = (t - 4.6) / 0.8
            x_shift = 0.08 * (0.5 * (1.0 + np.cos(np.pi * p)))

        # Root pos & orientation
        idle_qpos[i, 0] = q105[0] + x_shift
        idle_qpos[i, 2] = q105[2] + z_resp

        dq = m33_idle.rpy_to_quat(roll_scan, pitch_resp, yaw_scan)
        idle_qpos[i, 3:7] = m33_idle.quat_mult(q105[3:7], dq)

        # Apply leg offsets
        idle_qpos[i, 7] = q105[7] + 0.05 * lf_advance
        idle_qpos[i, 8] = q105[8] - 0.14 * lf_lift - 0.02 * lf_advance
        idle_qpos[i, 10] = q105[10] - 0.08 * lf_lift - 0.015 * lf_advance
        idle_qpos[i, 12] = q105[12] - 0.05 * lf_lift

        idle_qpos[i, 40] = q105[40] - 0.05 * rf_advance
        idle_qpos[i, 41] = q105[41] - 0.14 * rf_lift - 0.02 * rf_advance
        idle_qpos[i, 43] = q105[43] - 0.08 * rf_lift - 0.015 * rf_advance
        idle_qpos[i, 45] = q105[45] - 0.06 * rf_lift

    return idle_qpos


def generate_idle_variation_3(q105, n_frames=360, duration=6.0):
    """
    Variation 3: Quiet observational pause with curiosity pitch,
    subtle weight-shift across middle tarsi, and gentle settle to qpos[105].
    """
    times = np.linspace(0.0, duration, n_frames, endpoint=False)
    idle_qpos = np.tile(q105, (n_frames, 1))

    omega_resp = 2.0 * np.pi * (4.0 / duration)

    for i, t in enumerate(times):
        env_resp = np.sin(np.pi * (t / duration)) ** 0.5
        z_resp = 0.015 * np.sin(omega_resp * t) * env_resp
        pitch_resp = 0.005 * np.sin(omega_resp * t) * env_resp

        # Curiosity head/body tilt upward at t=2.0s, downward at t=3.8s
        pitch_curiosity = 0.008 * np.exp(-((t - 2.0) / 0.35) ** 2) - 0.006 * np.exp(-((t - 3.8) / 0.30) ** 2)
        yaw_curiosity = 0.008 * np.exp(-((t - 2.0) / 0.35) ** 2)

        # Subtle forward/backward weight transfer without full foot lift
        x_shift = 0.0
        if 1.8 <= t <= 3.2:
            p = (t - 1.8) / 1.4
            x_shift = 0.05 * np.sin(np.pi * p)

        # Root pos & orientation
        idle_qpos[i, 0] = q105[0] + x_shift
        idle_qpos[i, 2] = q105[2] + z_resp

        dq = m33_idle.rpy_to_quat(0.0, pitch_resp + pitch_curiosity, yaw_curiosity)
        idle_qpos[i, 3:7] = m33_idle.quat_mult(q105[3:7], dq)

        # Subtle middle leg tarsus flex adjustments
        tarsus_adjust = 0.03 * np.sin(np.pi * np.clip((t - 1.8) / 1.4, 0, 1))
        idle_qpos[i, 20] = q105[20] - tarsus_adjust
        idle_qpos[i, 53] = q105[53] - tarsus_adjust

    return idle_qpos


def render_idle_sequence(sim, renderer, cam_id, qpos_seq, output_path, fps=60):
    frames = []
    n_frames = len(qpos_seq)
    for i in range(n_frames):
        sim.mj_data.qpos[:] = qpos_seq[i]
        mj.mj_forward(sim.mj_model, sim.mj_data)
        renderer.mj_renderer.update_scene(sim.mj_data, cam_id, renderer.scene_option)
        rgb = renderer.mj_renderer.render()
        frames.append(rgb)
    iio.mimsave(output_path, frames, fps=fps, codec="libx264", quality=8)
    print(f"  Encoded {output_path} ({len(frames)} frames, {output_path.stat().st_size / 1024:.1f} KB)")


def main():
    print("Rendering idle variations...")

    traj = np.load(TRAJECTORY_PATH)
    q105 = traj["qpos"][105].copy()

    # Build habitat simulation
    fly = m31.build_colorized_fly()
    cam = m31.add_hero_camera(fly)
    world = m31.create_natural_habitat_world()
    world.add_fly(fly, spawn_position=[0, 0, 0.5], spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]))

    sim = Simulation(world)
    renderer = sim.set_renderer(cam, camera_res=CAMERA_RES, output_fps=OUTPUT_FPS, buffer_frames=False)
    sim.reset()
    cam_id = renderer._cameras_names2id["fly/herocam"]

    # 1. Generate & Render Variation 2
    print("\nRendering Idle Variation 2 (Left-front lead & lateral scanning) ...")
    qpos_var2 = generate_idle_variation_2(q105)
    diff0_v2 = np.max(np.abs(qpos_var2[0] - q105))
    difflast_v2 = np.max(np.abs(qpos_var2[-1] - qpos_var2[0]))
    print(f"  Var 2 diff frame 0 vs q105: {diff0_v2:.6e}, loop closure: {difflast_v2:.6e}")
    p_var2_web = WEB_ASSETS_DIR / "video_idle_var2_16x9.mp4"
    p_var2_out = OUTPUT_DIR / "video_idle_var2_16x9.mp4"
    render_idle_sequence(sim, renderer, cam_id, qpos_var2, p_var2_web)
    render_idle_sequence(sim, renderer, cam_id, qpos_var2, p_var2_out)

    # 2. Generate & Render Variation 3
    print("\nRendering Idle Variation 3 (Curiosity pitch & subtle stance tuning) ...")
    qpos_var3 = generate_idle_variation_3(q105)
    diff0_v3 = np.max(np.abs(qpos_var3[0] - q105))
    difflast_v3 = np.max(np.abs(qpos_var3[-1] - qpos_var3[0]))
    print(f"  Var 3 diff frame 0 vs q105: {diff0_v3:.6e}, loop closure: {difflast_v3:.6e}")
    p_var3_web = WEB_ASSETS_DIR / "video_idle_var3_16x9.mp4"
    p_var3_out = OUTPUT_DIR / "video_idle_var3_16x9.mp4"
    render_idle_sequence(sim, renderer, cam_id, qpos_var3, p_var3_web)
    render_idle_sequence(sim, renderer, cam_id, qpos_var3, p_var3_out)

    renderer.close()
    sim.close()

    print("Idle variations generation complete.")


if __name__ == "__main__":
    main()
