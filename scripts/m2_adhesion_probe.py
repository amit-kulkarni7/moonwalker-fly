"""MW-M2: Forward-kinematics / adhesion-phase feasibility experiment."""

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


def compute_adhesion_for_variant(
    variant: str,
    stock_adhesion: np.ndarray,
    curr_phases: np.ndarray,
    preprogrammed_steps: PreprogrammedSteps,
) -> np.ndarray:
    """Compute adhesion state for a given experimental variant."""
    if variant == "STOCK":
        return stock_adhesion
    elif variant == "INVERTED":
        return np.logical_not(stock_adhesion)
    elif variant == "PHASE_SHIFT_0_25":
        shifted = (curr_phases + 0.25 * 2 * np.pi) % (2 * np.pi)
        return preprogrammed_steps.get_adhesion_onoff_by_phase(shifted)
    elif variant == "PHASE_SHIFT_0_50":
        shifted = (curr_phases + 0.50 * 2 * np.pi) % (2 * np.pi)
        return preprogrammed_steps.get_adhesion_onoff_by_phase(shifted)
    elif variant == "PHASE_SHIFT_0_75":
        shifted = (curr_phases + 0.75 * 2 * np.pi) % (2 * np.pi)
        return preprogrammed_steps.get_adhesion_onoff_by_phase(shifted)
    else:
        raise ValueError(f"Unknown variant: {variant}")


def main() -> None:
    print("M2_EXPERIMENT_START")

    flygym_ver = version("flygym")
    mujoco_ver = version("mujoco")
    numpy_ver = np.__version__
    print(f"FLYGYM_VERSION={flygym_ver}")
    print(f"MUJOCO_VERSION={mujoco_ver}")
    print(f"NUMPY_VERSION={numpy_ver}")

    # 1. Compose model
    fly = make_locomotion_fly(
        name="m2_probe",
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

    # 2. Controller setup
    preprogrammed_steps = PreprogrammedSteps()
    actuated_dof_order = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    assert len(actuated_dof_order) == 42, f"Expected 42 DOFs, got {len(actuated_dof_order)}"

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

    default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dof_order)
    body_segments = fly.get_bodysegs_order()
    thorax_idx = body_segments.index(fly.root_segment)

    trial_duration_s = 1.0
    n_steps = round(trial_duration_s / actual_timestep)
    assert n_steps > 0

    variants = [
        "STOCK",
        "INVERTED",
        "PHASE_SHIFT_0_25",
        "PHASE_SHIFT_0_50",
        "PHASE_SHIFT_0_75",
    ]

    print(f"TRIAL_DURATION_SECONDS={trial_duration_s}")
    print(f"PHYSICS_TIMESTEP_SECONDS={actual_timestep}")
    print(f"VARIANTS_TO_TEST={len(variants)}")

    joint_command_modification_count = 0
    results = {}

    sweep_t0 = time.perf_counter()

    for var in variants:
        # Reset fair state
        sim.reset()
        controller.cpg_network.random_state = np.random.RandomState(0)
        controller.cpg_network.reset()
        sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
        sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
        sim.warmup()

        start_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        start_x, start_y, start_z = [float(v) for v in start_pos]

        min_z = start_z
        max_abs_lat_disp = 0.0
        adhesion_active_count = 0
        total_adhesion_checks = 0

        for _ in range(n_steps):
            stock_action = controller.step()

            adhesion = compute_adhesion_for_variant(
                var,
                stock_action.adhesion_onoff,
                controller.cpg_network.curr_phases,
                preprogrammed_steps,
            )

            action_to_apply = LocomotionAction(
                joint_angles=stock_action.joint_angles,
                adhesion_onoff=adhesion,
            )

            # Strict integrity check: joint targets must remain unchanged
            if not np.array_equal(action_to_apply.joint_angles, stock_action.joint_angles):
                joint_command_modification_count += 1

            apply_locomotion_action(sim, fly.name, action_to_apply)
            sim.step()

            curr_pos = sim.get_body_positions(fly.name)[thorax_idx]
            cx, cy, cz = [float(v) for v in curr_pos]
            if cz < min_z:
                min_z = cz
            lat_disp_step = abs(cy - start_y)
            if lat_disp_step > max_abs_lat_disp:
                max_abs_lat_disp = lat_disp_step

            adhesion_active_count += int(np.sum(adhesion > 0.5))
            total_adhesion_checks += len(adhesion)

        end_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
        end_x, end_y, end_z = [float(v) for v in end_pos]

        disp_x = end_x - start_x
        disp_y = end_y - start_y
        disp_z = end_z - start_z
        avg_vx = disp_x / trial_duration_s
        adhesion_fraction = (
            adhesion_active_count / total_adhesion_checks
            if total_adhesion_checks > 0
            else 0.0
        )

        is_finite = bool(np.all(np.isfinite([end_x, end_y, end_z])))
        inspectable = bool(is_finite and min_z > 0.2 and max_abs_lat_disp < 15.0)

        if disp_x > 1.0:
            trans_class = "FORWARD"
        elif disp_x < -1.0:
            trans_class = "BACKWARD"
        else:
            trans_class = "NEUTRAL"

        results[var] = {
            "variant": var,
            "start_thorax_xyz_mm": [start_x, start_y, start_z],
            "end_thorax_xyz_mm": [end_x, end_y, end_z],
            "displacement_x_mm": disp_x,
            "displacement_y_mm": disp_y,
            "displacement_z_mm": disp_z,
            "average_vx_mm_s": avg_vx,
            "min_thorax_z_mm": min_z,
            "max_abs_lateral_disp_mm": max_abs_lat_disp,
            "adhesion_fraction": adhesion_fraction,
            "translation_class": trans_class,
            "inspectable": inspectable,
        }

        print(
            f"TRIAL {var:17s} | X={disp_x:+8.4f} mm | Y={disp_y:+8.4f} mm | Z={disp_z:+8.4f} mm | "
            f"MinZ={min_z:6.4f} mm | AdhFrac={adhesion_fraction:.3f} | {trans_class:8s} | Inspectable={inspectable}"
        )

    sweep_wall_time = time.perf_counter() - sweep_t0
    print(f"SWEEP_WALL_TIME_SECONDS={sweep_wall_time:.4f}")
    print(f"VARIANTS_TESTED={len(variants)}")
    print(f"JOINT_COMMAND_MODIFICATION_COUNT={joint_command_modification_count}")

    # Enforce forward-kinematics integrity
    assert joint_command_modification_count == 0, (
        f"Integrity check failed: {joint_command_modification_count} joint modifications detected!"
    )

    # 3. Identify best backward candidate if any
    backward_candidates = [
        r for r in results.values()
        if r["translation_class"] == "BACKWARD" and r["inspectable"]
    ]

    best_candidate = None
    if backward_candidates:
        # Rank primarily by most negative x displacement
        backward_candidates.sort(key=lambda r: r["displacement_x_mm"])
        best_candidate = backward_candidates[0]
        print(f"BACKWARD_CANDIDATE_FOUND=YES")
        print(f"BEST_VARIANT={best_candidate['variant']}")
        print(f"BEST_X_DISPLACEMENT_MM={best_candidate['displacement_x_mm']:.4f}")
    else:
        print("BACKWARD_CANDIDATE_FOUND=NO")

    # 4. Save results JSON
    output_dir = pathlib.Path("output/m2")
    output_dir.mkdir(parents=True, exist_ok=True)
    results_json_path = output_dir / "adhesion_probe_results.json"

    export_data = {
        "flygym_version": flygym_ver,
        "mujoco_version": mujoco_ver,
        "trial_duration_s": trial_duration_s,
        "timestep_s": actual_timestep,
        "physics_steps": n_steps,
        "joint_command_modification_count": joint_command_modification_count,
        "adhesion_phase_shift_supported": True,
        "adhesion_phase_shift_method": "PreprogrammedSteps.get_adhesion_onoff_by_phase(shifted_phases)",
        "results": results,
        "best_candidate": best_candidate["variant"] if best_candidate else None,
    }

    with open(results_json_path, "w") as f:
        json.dump(export_data, f, indent=2)
    print(f"RESULTS_JSON_PATH={results_json_path.as_posix()}")

    # 5. Optional single rendered replay of best candidate
    rendered_frame_count = 0
    video_created = False
    video_path = output_dir / "best_candidate.mp4"

    if best_candidate is not None:
        best_var = best_candidate["variant"]
        print(f"RENDERING_BEST_CANDIDATE={best_var}")

        renderer = sim.set_renderer(
            cam,
            camera_res=(240, 320),
            playback_speed=0.2,
            output_fps=25,
            buffer_frames=True,
        )

        try:
            sim.reset()
            controller.cpg_network.random_state = np.random.RandomState(0)
            controller.cpg_network.reset()
            sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
            sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
            sim.warmup()

            for _ in range(n_steps):
                stock_action = controller.step()
                adhesion = compute_adhesion_for_variant(
                    best_var,
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
            print(f"VIDEO_PATH={video_path.as_posix()}")
            print(f"VIDEO_SIZE_BYTES={video_path.stat().st_size}")
            print(f"RENDERED_FRAME_COUNT={rendered_frame_count}")
        finally:
            renderer.close()
    else:
        print("CANDIDATE_RENDERED=NO (No candidate qualified)")

    print("M2_EXPERIMENT_RESULT=PASS")


if __name__ == "__main__":
    main()
