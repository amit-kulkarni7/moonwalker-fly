#!/usr/bin/env python3
"""
Renders portrait (720x1280, 9:16) video assets for mobile viewports:
- Habitat idle variations 1, 2, 3
- Theatrical 3D fedora performance stage (M27-D playback)
- Initial still posters

Camera framing:
- pos_offset = (-0.75, -8.25, 4.25)
- rotation = xyaxes (1.0, 0.0, 0.0, 0.0, 0.50, 0.866)
- fovy = 50.0 deg
- camera_res = (1280, 720) -> Height=1280, Width=720
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
from flygym.compose import FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import make_locomotion_fly

import scripts.m31_visual_foundation as m31
import scripts.m33_render_stage as m33_stage
import scripts.m33_generate_idle_assets as m33_idle
import scripts.m33_generate_idle_variations as m33_variations

TRAJECTORY_PATH = pathlib.Path("data/derived/m33_m27d_trajectory.npz")
OUTPUT_DIR = pathlib.Path("output/m33")
WEB_ASSETS_DIR = pathlib.Path("web/assets")
DERIVED_DIR = pathlib.Path("data/derived")

OUTPUT_FPS = 60
PORTRAIT_CAMERA_RES = (1280, 720)  # (Height, Width)
PORTRAIT_CAM_POS_OFFSET = (-0.75, -8.25, 4.25)
PORTRAIT_CAM_ROTATION = Rotation3D("xyaxes", (1.0, 0.0, 0.0, 0.0, 0.50, 0.866))
PORTRAIT_CAM_FOVY = 50.0


def render_qpos_sequence(sim, renderer, cam_id, qpos_seq, output_path, fps=60):
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
    return frames


def main():
    print("Rendering portrait assets (720x1280) for mobile viewports...")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    WEB_ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    if not TRAJECTORY_PATH.exists():
        raise FileNotFoundError(f"Trajectory file {TRAJECTORY_PATH} not found!")

    traj = np.load(TRAJECTORY_PATH)
    times = traj["time"]
    qpos_arr = traj["qpos"]
    q105 = qpos_arr[105].copy()

    # --------------------------------------------------------------------------
    # 1. THEATRICAL FEDORA STAGE (PORTRAIT)
    # --------------------------------------------------------------------------
    print("\n--- 1. Rendering Theatrical Stage Performance (Portrait 720x1280) ---")
    fly_stage = make_locomotion_fly(name="fly", add_adhesion=True, colorize=False)
    fly_stage.colorize(m33_stage.VISUALS_YAML)
    m33_stage.attach_fedora_to_fly(fly_stage)

    cam_stage = fly_stage.add_tracking_camera(
        name="herocam",
        pos_offset=PORTRAIT_CAM_POS_OFFSET,
        rotation=PORTRAIT_CAM_ROTATION,
        fovy=PORTRAIT_CAM_FOVY
    )

    world_stage = m33_stage.build_performance_stage_world()
    world_stage.add_fly(
        fly_stage,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )

    sim_stage = Simulation(world_stage)
    renderer_stage = sim_stage.set_renderer(
        cam_stage,
        camera_res=PORTRAIT_CAMERA_RES,
        output_fps=OUTPUT_FPS,
        buffer_frames=False
    )
    sim_stage.reset()
    cam_id_stage = renderer_stage._cameras_names2id["fly/herocam"]

    frame_indices = list(range(105, 531, 5))
    stage_qpos_seq = [qpos_arr[idx] for idx in frame_indices]

    p_stage_web = WEB_ASSETS_DIR / "video_stage_portrait.mp4"
    p_stage_out = OUTPUT_DIR / "video_stage_portrait.mp4"
    stage_frames = render_qpos_sequence(sim_stage, renderer_stage, cam_id_stage, stage_qpos_seq, p_stage_web, fps=OUTPUT_FPS)
    iio.mimsave(p_stage_out, stage_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)

    # Save stage portrait poster
    p_stage_poster_web = WEB_ASSETS_DIR / "poster_stage_portrait.png"
    p_stage_poster_out = OUTPUT_DIR / "poster_stage_portrait.png"
    Image.fromarray(stage_frames[0]).save(p_stage_poster_web)
    Image.fromarray(stage_frames[0]).save(p_stage_poster_out)
    print(f"  Saved stage poster: {p_stage_poster_web}")

    renderer_stage.close()
    sim_stage.close()

    # --------------------------------------------------------------------------
    # 2. NATURAL HABITAT IDLE SEQUENCES (PORTRAIT)
    # --------------------------------------------------------------------------
    print("\n--- 2. Rendering Natural Habitat Idle Variations (Portrait 720x1280) ---")
    fly_hab = m31.build_colorized_fly()
    cam_hab = fly_hab.add_tracking_camera(
        name="herocam",
        pos_offset=PORTRAIT_CAM_POS_OFFSET,
        rotation=PORTRAIT_CAM_ROTATION,
        fovy=PORTRAIT_CAM_FOVY
    )

    world_hab = m31.create_natural_habitat_world()
    world_hab.add_fly(
        fly_hab,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0])
    )

    sim_hab = Simulation(world_hab)
    renderer_hab = sim_hab.set_renderer(
        cam_hab,
        camera_res=PORTRAIT_CAMERA_RES,
        output_fps=OUTPUT_FPS,
        buffer_frames=False
    )
    sim_hab.reset()
    cam_id_hab = renderer_hab._cameras_names2id["fly/herocam"]

    # Variation 1
    print("\n  Generating & rendering Idle Variation 1 (RF lead step & forward shift) ...")
    qpos_var1 = m33_idle.generate_idle_qpos(q105)
    p_var1_web = WEB_ASSETS_DIR / "video_idle_portrait.mp4"
    p_var1_out = OUTPUT_DIR / "video_idle_portrait.mp4"
    var1_frames = render_qpos_sequence(sim_hab, renderer_hab, cam_id_hab, qpos_var1, p_var1_web, fps=OUTPUT_FPS)
    iio.mimsave(p_var1_out, var1_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)

    # Save habitat portrait poster
    p_hab_poster_web = WEB_ASSETS_DIR / "poster_habitat_portrait.png"
    p_hab_poster_out = OUTPUT_DIR / "poster_habitat_portrait.png"
    Image.fromarray(var1_frames[0]).save(p_hab_poster_web)
    Image.fromarray(var1_frames[0]).save(p_hab_poster_out)
    print(f"  Saved habitat poster: {p_hab_poster_web}")

    # Variation 2
    print("\n  Generating & rendering Idle Variation 2 (LF lead step & lateral scan) ...")
    qpos_var2 = m33_variations.generate_idle_variation_2(q105)
    p_var2_web = WEB_ASSETS_DIR / "video_idle_var2_portrait.mp4"
    p_var2_out = OUTPUT_DIR / "video_idle_var2_portrait.mp4"
    var2_frames = render_qpos_sequence(sim_hab, renderer_hab, cam_id_hab, qpos_var2, p_var2_web, fps=OUTPUT_FPS)
    iio.mimsave(p_var2_out, var2_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)

    # Variation 3
    print("\n  Generating & rendering Idle Variation 3 (Curiosity pitch & quiet pause) ...")
    qpos_var3 = m33_variations.generate_idle_variation_3(q105)
    p_var3_web = WEB_ASSETS_DIR / "video_idle_var3_portrait.mp4"
    p_var3_out = OUTPUT_DIR / "video_idle_var3_portrait.mp4"
    var3_frames = render_qpos_sequence(sim_hab, renderer_hab, cam_id_hab, qpos_var3, p_var3_web, fps=OUTPUT_FPS)
    iio.mimsave(p_var3_out, var3_frames, fps=OUTPUT_FPS, codec="libx264", quality=8)

    renderer_hab.close()
    sim_hab.close()

    print("Finished rendering portrait assets.")


if __name__ == "__main__":
    main()
