"""MW-M1: Stock forward locomotion baseline using FlyGym CPG controller."""

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
    PreprogrammedSteps,
    apply_locomotion_action,
    make_locomotion_fly,
    make_tripod_cpg_network,
)


def main() -> None:
    print("M1_FORWARD_START")

    flygym_ver = version("flygym")
    mujoco_ver = version("mujoco")
    numpy_ver = np.__version__
    print(f"FLYGYM_VERSION={flygym_ver}")
    print(f"MUJOCO_VERSION={mujoco_ver}")
    print(f"NUMPY_VERSION={numpy_ver}")

    # 1. Compose stock locomotion fly
    fly = make_locomotion_fly(
        name="m1_forward",
        add_adhesion=True,
        colorize=True,
    )

    # 2. Add body tracking camera (lateral / three-quarter view)
    cam = fly.add_tracking_camera(name="trackcam")

    # 3. FlatGroundWorld and spawn fly
    world = FlatGroundWorld()
    spawn_rot = Rotation3D(format="quat", values=[1, 0, 0, 0])
    world.add_fly(
        fly=fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=spawn_rot,
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD,
    )

    # 4. Instantiate simulation
    sim = Simulation(world)
    actual_timestep = float(sim.mj_model.opt.timestep)
    assert actual_timestep > 0, "Timestep must be > 0"

    # 5. Stock CPG controller setup
    preprogrammed_steps = PreprogrammedSteps()
    actuated_dof_order = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    actuated_dof_count = len(actuated_dof_order)

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
        output_dof_order=actuated_dof_order,
    )

    intrinsic_freq_hz = float(cpg_network.intrinsic_freqs[0])
    step_cycle_freq_hz = float(preprogrammed_steps.step_cycle_frequency_hz)

    print(f"ACTUATED_DOF_COUNT={actuated_dof_count}")
    print(f"CPG_INTRINSIC_FREQUENCY_HZ={intrinsic_freq_hz:.2f}")
    print(f"STEP_CYCLE_FREQUENCY_HZ={step_cycle_freq_hz:.2f}")

    assert actuated_dof_count == 42, (
        f"Expected 42 actuated DOFs, got {actuated_dof_count}"
    )

    # 6. Renderer setup
    renderer = sim.set_renderer(
        cam,
        camera_res=(240, 320),
        playback_speed=0.2,
        output_fps=25,
        buffer_frames=True,
    )

    try:
        # 7. Initialization & Warmup
        sim.reset()
        default_pose = preprogrammed_steps.default_pose_by_dof_order(
            actuated_dof_order
        )
        sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
        sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
        sim.warmup()

        # 8. Record starting thorax position
        body_segments = fly.get_bodysegs_order()
        thorax_idx = body_segments.index(fly.root_segment)
        start_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        start_x, start_y, start_z = [float(v) for v in start_thorax_pos]

        run_time_seconds = 2.0
        n_steps = round(run_time_seconds / actual_timestep)
        assert n_steps > 0, "Steps must be > 0"

        print(f"RUN_TIME_SECONDS={run_time_seconds}")
        print(f"PHYSICS_TIMESTEP_SECONDS={actual_timestep}")
        print(f"PHYSICS_STEPS={n_steps}")

        # 9. Locomotion rollout
        t0 = time.perf_counter()
        for _ in range(n_steps):
            action = controller.step()
            apply_locomotion_action(sim, fly.name, action)
            sim.step_with_profile()
            sim.render_as_needed_with_profile()
        wall_time_seconds = time.perf_counter() - t0

        # 10. Record ending thorax position and compute metrics
        end_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        end_x, end_y, end_z = [float(v) for v in end_thorax_pos]

        forward_disp = end_x - start_x
        lateral_disp = end_y - start_y
        vertical_disp = end_z - start_z
        avg_forward_speed = forward_disp / run_time_seconds

        final_mujoco_time = float(sim.mj_data.time)

        print(f"START_THORAX_X_MM={start_x:.4f}")
        print(f"START_THORAX_Y_MM={start_y:.4f}")
        print(f"START_THORAX_Z_MM={start_z:.4f}")
        print(f"END_THORAX_X_MM={end_x:.4f}")
        print(f"END_THORAX_Y_MM={end_y:.4f}")
        print(f"END_THORAX_Z_MM={end_z:.4f}")
        print(f"FORWARD_DISPLACEMENT_MM={forward_disp:.4f}")
        print(f"LATERAL_DISPLACEMENT_MM={lateral_disp:.4f}")
        print(f"VERTICAL_DISPLACEMENT_MM={vertical_disp:.4f}")
        print(f"AVERAGE_FORWARD_SPEED_MM_S={avg_forward_speed:.4f}")
        print(f"FINAL_MUJOCO_TIME_SECONDS={final_mujoco_time:.6f}")

        # Frame count check
        if renderer.frames:
            rendered_frame_count = sum(len(f) for f in renderer.frames.values())
        else:
            rendered_frame_count = sim._frames_rendered
        print(f"RENDERED_FRAME_COUNT={rendered_frame_count}")
        assert rendered_frame_count > 0, "No frames were buffered by renderer"

        # Engineering acceptance checks
        assert forward_disp > 5.0, (
            f"Forward displacement {forward_disp:.2f} mm <= 5.0 mm"
        )
        assert abs(lateral_disp) < abs(forward_disp), (
            f"Lateral displacement {abs(lateral_disp):.2f} mm >= forward displacement {abs(forward_disp):.2f} mm"
        )

        # Performance
        steps_per_sec = (
            n_steps / wall_time_seconds if wall_time_seconds > 0 else 0.0
        )
        realtime_factor = (
            (n_steps * actual_timestep) / wall_time_seconds
            if wall_time_seconds > 0
            else 0.0
        )

        print(f"WALL_TIME_SECONDS={wall_time_seconds:.4f}")
        print(f"REALTIME_FACTOR={realtime_factor:.4f}")
        print(f"PHYSICS_STEPS_PER_SECOND={steps_per_sec:.2f}")
        print("PERFORMANCE_CLASSIFICATION=DIAGNOSTIC_NOT_BENCHMARK")

        # 11. Save MP4 video
        output_dir = pathlib.Path("output")
        output_dir.mkdir(parents=True, exist_ok=True)
        video_path = output_dir / "m1_forward.mp4"
        renderer.save_video(video_path)

        assert video_path.is_file(), f"Video file not found at {video_path}"
        video_size_bytes = video_path.stat().st_size
        assert video_size_bytes > 0, f"Video file {video_path} is empty"

        print(f"VIDEO_PATH={video_path.as_posix()}")
        print(f"VIDEO_SIZE_BYTES={video_size_bytes}")

        # 12. Save machine-readable metrics JSON
        metrics = {
            "flygym_version": flygym_ver,
            "mujoco_version": mujoco_ver,
            "simulation_duration_s": run_time_seconds,
            "timestep_s": actual_timestep,
            "physics_steps": n_steps,
            "actuated_dof_count": actuated_dof_count,
            "start_thorax_position_mm": [start_x, start_y, start_z],
            "end_thorax_position_mm": [end_x, end_y, end_z],
            "forward_displacement_mm": forward_disp,
            "lateral_displacement_mm": lateral_disp,
            "vertical_displacement_mm": vertical_disp,
            "average_forward_speed_mm_s": avg_forward_speed,
            "rendered_frame_count": rendered_frame_count,
        }
        metrics_path = output_dir / "m1_forward_metrics.json"
        with open(metrics_path, "w") as f:
            json.dump(metrics, f, indent=2)

        assert metrics_path.is_file(), f"Metrics file not found at {metrics_path}"
        print(f"METRICS_JSON_PATH={metrics_path.as_posix()}")

        sim.print_performance_report()

        print("M1_FORWARD_RESULT=PASS")
    finally:
        renderer.close()


if __name__ == "__main__":
    main()
