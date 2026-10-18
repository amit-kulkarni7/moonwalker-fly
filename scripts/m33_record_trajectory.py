#!/usr/bin/env python3
"""
Trajectory recorder for M27-D locomotion.

Executes a deterministic M27-D simulation rollout (1.75 s, dt=0.0001 s)
and records generalized coordinates (qpos) and velocities (qvel)
at FlyGym render frame intervals (300 Hz sampling).
"""

import os
import sys
import json
import pathlib
import numpy as np
import mujoco as mj

os.environ["MPLCONFIGDIR"] = "/tmp"

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.append(".")

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    make_locomotion_fly, apply_locomotion_action
)
import scripts.m25_support_path_geometry as m25
import scripts.m30_tripod_transfer_relief as m30

PHYSICS_DT = 0.0001
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)  # 17500 steps
RENDER_FPS = 60
PLAYBACK_SPEED = 0.2
SECS_PER_RENDER = PLAYBACK_SPEED / RENDER_FPS  # 1/300 s ~= 0.00333333 s

DERIVED_DIR = pathlib.Path("data/derived")


def record_m27d_trajectory(output_path=DERIVED_DIR / "m33_m27d_trajectory.npz"):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    print("Recording M27-D trajectory...")
    print(f"Trial duration: {TRIAL_DURATION:.2f} s ({N_STEPS} physics steps)")
    print(f"Render sampling rate: {RENDER_FPS / PLAYBACK_SPEED:.1f} Hz (every {SECS_PER_RENDER*1e3:.3f} ms)")

    fly = make_locomotion_fly(name="fly", add_adhesion=True, colorize=False)
    world = FlatGroundWorld()
    world.add_fly(
        fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )

    cam = fly.add_tracking_camera(
        name="sync_cam",
        pos_offset=(-0.65, -6.6, 3.4),
        rotation=Rotation3D("xyaxes", (1.0, 0.0, 0.0, 0.0, 0.50, 0.866)),
        fovy=29.0
    )

    sim = Simulation(world)
    renderer = sim.set_renderer(
        cam,
        camera_res=(1, 1),
        playback_speed=PLAYBACK_SPEED,
        output_fps=RENDER_FPS,
        buffer_frames=False
    )

    dof_map = m25.build_semantic_dof_mapping(fly, sim)
    pps = m30.PreprogrammedSteps()
    act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    ctrl = m30.M30TransferReliefController(sim, fly, pps, act_dofs, dof_map, condition="A")

    sim.reset()
    ctrl.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, ctrl.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    # Telemetry storage matching FlyGym renderer frames exactly
    time_records = []
    qpos_records = []
    qvel_records = []
    step_indices = []

    print("\nRunning deterministic simulation with FlyGym frame synchronization ...")
    for step_i in range(N_STEPS):
        action, _ = ctrl.step()
        apply_locomotion_action(sim, fly.name, action)
        sim.step_with_profile()

        # Capture state whenever FlyGym renderer triggers a video frame
        if renderer.render_as_needed(sim.mj_data):
            time_records.append(float(sim.mj_data.time))
            qpos_records.append(sim.mj_data.qpos.copy())
            qvel_records.append(sim.mj_data.qvel.copy())
            step_indices.append(step_i)

        if (step_i + 1) % 3500 == 0:
            pct = (step_i + 1) / N_STEPS * 100
            print(f"  Step {step_i + 1}/{N_STEPS} ({pct:.0f}%) - sim time {sim.mj_data.time:.2f} s - frames: {len(qpos_records)}")

    renderer.close()
    sim.close()

    time_arr = np.array(time_records)
    qpos_arr = np.array(qpos_records)
    qvel_arr = np.array(qvel_records)
    step_indices_arr = np.array(step_indices)

    print(f"\nTrajectory recorded successfully:")
    print(f"  Total frames: {len(time_arr)}")
    print(f"  Time range: {time_arr[0]:.4f} s to {time_arr[-1]:.4f} s")
    print(f"  qpos shape: {qpos_arr.shape}")
    print(f"  qvel shape: {qvel_arr.shape}")
    print(f"  Simulation step range: {step_indices_arr[0]} to {step_indices_arr[-1]}")

    np.savez_compressed(
        output_path,
        time=time_arr,
        qpos=qpos_arr,
        qvel=qvel_arr,
        step_indices=step_indices_arr,
        physics_dt=PHYSICS_DT,
        trial_duration=TRIAL_DURATION,
        render_fps=RENDER_FPS,
        playback_speed=PLAYBACK_SPEED,
        controller="M27_D_EXCURSION_MATCHED_PLANAR_SUPPORT",
        condition="A"
    )
    print(f"Saved trajectory file: {output_path} ({output_path.stat().st_size / 1024:.1f} KB)")
    return output_path


if __name__ == "__main__":
    record_m27d_trajectory()
