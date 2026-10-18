#!/usr/bin/env python3
"""
Renders 16:9 widescreen (1280x720) theatrical stage performance video:
- Resolution: 1280x720 matching habitat video
- Camera framing: pos_offset=(-0.65, -6.6, 3.4), fovy=29.0 deg
- 86 frames at 60 FPS (duration 1.433 s, presentation duration 2.867 s at 0.5x)
- Replays M27-D trajectory from data/derived/m33_m27d_trajectory.npz
- Visual-only 3D fedora attached to fly head
- Theatrical lighting: dark ebony deck, spotlight pool, key, fill, and rim lights
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
from flygym.anatomy import ContactBodiesPreset
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import make_locomotion_fly

import scripts.m33_render_stage as m33_stage

TRAJECTORY_PATH = pathlib.Path("data/derived/m33_m27d_trajectory.npz")
OUTPUT_DIR = pathlib.Path("output/m33")
WEB_ASSETS_DIR = pathlib.Path("web/assets")
DERIVED_DIR = pathlib.Path("data/derived")

OUTPUT_FPS = 60
CAMERA_RES = (720, 1280)  # (H, W) -> 16:9 widescreen


def main():
    print("Rendering 16:9 theatrical stage video (1280x720)...")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    WEB_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    # 1. Load authoritative M27-D trajectory
    if not TRAJECTORY_PATH.exists():
        raise FileNotFoundError(f"Trajectory file {TRAJECTORY_PATH} not found!")

    traj = np.load(TRAJECTORY_PATH)
    times = traj["time"]
    qpos_arr = traj["qpos"]

    frame_indices = list(range(105, 531, 5))
    n_frames = len(frame_indices)
    print(f"Presentation window frame mapping: {n_frames} frames from index {frame_indices[0]} to {frame_indices[-1]}")

    # 2. Build theatrical stage world & attach fedora
    print("\nBuilding approved theatrical stage scene (16:9 Widescreen) ...")
    fly = make_locomotion_fly(name="fly", add_adhesion=True, colorize=False)
    fly.colorize(m33_stage.VISUALS_YAML)
    m33_stage.attach_fedora_to_fly(fly)

    cam = fly.add_tracking_camera(
        name="herocam",
        pos_offset=m33_stage.HERO_CAM_POS_OFFSET,
        rotation=m33_stage.HERO_CAM_ROTATION,
        fovy=m33_stage.HERO_CAM_FOVY
    )

    world = m33_stage.build_performance_stage_world()
    world.add_fly(
        fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
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

    # 3. Render stage frames via forward kinematics
    print(f"\nRendering {n_frames} performance frames via forward kinematics ...")
    stage_frames = []
    for i, traj_idx in enumerate(frame_indices):
        sim.mj_data.qpos[:] = qpos_arr[traj_idx]
        mj.mj_forward(sim.mj_model, sim.mj_data)
        renderer.mj_renderer.update_scene(sim.mj_data, cam_id, renderer.scene_option)
        rgb = renderer.mj_renderer.render()
        stage_frames.append(rgb)

        if (i + 1) % 20 == 0 or (i + 1) == n_frames:
            print(f"  Rendered {i + 1}/{n_frames} frames ({times[traj_idx]:.2f} s)")

    renderer.close()
    sim.close()

    # 4. Save video deliverables
    p_stage_web = WEB_ASSETS_DIR / "video_stage_16x9.mp4"
    p_stage_out = OUTPUT_DIR / "video_stage_16x9.mp4"
    print(f"\nEncoding {p_stage_web} ({n_frames} frames at {OUTPUT_FPS} FPS) ...")
    iio.mimsave(p_stage_web, stage_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    iio.mimsave(p_stage_out, stage_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)
    print(f"  Saved stage video: {p_stage_web} ({p_stage_web.stat().st_size / 1024:.1f} KB)")

    # Save initial poster frame for stage
    p_stage_poster = WEB_ASSETS_DIR / "poster_stage_16x9.png"
    Image.fromarray(stage_frames[0]).save(p_stage_poster)
    print(f"  Saved stage initial poster: {p_stage_poster}")

    print("Stage video production complete.")


if __name__ == "__main__":
    main()
