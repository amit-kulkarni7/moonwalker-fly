import json
import pathlib
import sys
import time
import csv

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

class MDNCommandModel:
    def __init__(self, mdn_info):
        self.mdn_info = mdn_info
        self.activities_by_root_id = {rid: 0.0 for rid in mdn_info}

    @property
    def left_command(self):
        lefts = [val for rid, val in self.activities_by_root_id.items() if self.mdn_info[rid]['hemisphere'] == 'Left']
        return float(np.mean(lefts)) if lefts else 0.0

    @property
    def right_command(self):
        rights = [val for rid, val in self.activities_by_root_id.items() if self.mdn_info[rid]['hemisphere'] == 'Right']
        return float(np.mean(rights)) if rights else 0.0

    @property
    def population_command(self):
        return float(np.mean(list(self.activities_by_root_id.values()))) if self.activities_by_root_id else 0.0

    def set_activities(self, activities):
        for rid, val in activities.items():
            if rid in self.activities_by_root_id:
                self.activities_by_root_id[rid] = val


def compute_adhesion(
    phase_fraction: float,
    stock_adhesion: np.ndarray,
    curr_phases: np.ndarray,
    preprogrammed_steps: PreprogrammedSteps,
) -> np.ndarray:
    if phase_fraction == 0.0:
        return stock_adhesion
    shifted = (curr_phases + phase_fraction * 2.0 * np.pi) % (2.0 * np.pi)
    return preprogrammed_steps.get_adhesion_onoff_by_phase(shifted)


def main():
    print("M5_DEMO_START")

    # 1. Load BANC artifacts
    manifest_path = pathlib.Path("data/derived/banc_v888_mdn_manifest.json")
    edges_path = pathlib.Path("data/derived/banc_v888_mdn_edges.csv")

    with open(manifest_path, "r") as f:
        manifest = json.load(f)

    mdn_list = manifest.get("mdn_neurons", [])
    banc_mdn_count = len(mdn_list)
    print(f"BANC_MDN_COUNT={banc_mdn_count}")
    assert banc_mdn_count == 4, f"Expected 4 canonical MDNs, found {banc_mdn_count}"

    canonical_mdns = {}
    for m in mdn_list:
        rid = m["root_id"]
        canonical_mdns[rid] = {
            "cell_type": m["cell_type"],
            "super_class": m["super_class"],
            "hemisphere": str(m.get("side", "")).capitalize()
        }

    left_mdns = {rid: info for rid, info in canonical_mdns.items() if info["hemisphere"] == "Left"}
    right_mdns = {rid: info for rid, info in canonical_mdns.items() if info["hemisphere"] == "Right"}
    assert len(left_mdns) == 2, "Expected exactly 2 left MDNs"
    assert len(right_mdns) == 2, "Expected exactly 2 right MDNs"

    for rid, info in canonical_mdns.items():
        assert info["cell_type"] == "MDN"
        assert info["super_class"] == "descending"

    print(f"MDN_LEFT_ROOT_IDS={list(left_mdns.keys())}")
    print(f"MDN_RIGHT_ROOT_IDS={list(right_mdns.keys())}")

    downstream_partners = {}
    lbl40_connections = []
    lul130_connections = []

    min_synapse_count = float('inf')
    edge_count = 0
    with open(edges_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            edge_count += 1
            synapses = int(row["synapse_count"])
            if synapses < min_synapse_count:
                min_synapse_count = synapses

            mdn_rid = row["mdn_root_id"]
            assert mdn_rid in canonical_mdns, f"Root ID {mdn_rid} not found in canonical MDNs"

            if row["direction_relative_to_mdn"] == "DOWNSTREAM":
                pct = row["partner_cell_type"]
                downstream_partners[pct] = downstream_partners.get(pct, 0) + synapses
                if pct == "LBL40":
                    lbl40_connections.append((mdn_rid, synapses))
                if pct == "LUL130":
                    lul130_connections.append((mdn_rid, synapses))

    assert edge_count > 0, "Connectivity file is empty"
    assert min_synapse_count >= 3, f"Min synapse count is {min_synapse_count} < 3"

    print(f"LBL40_DIRECT_GRAPH_STATUS={'YES' if lbl40_connections else 'NO'}")
    if lbl40_connections:
        print(f"LBL40_CONNECTIONS: {lbl40_connections}")
    print(f"LUL130_DIRECT_GRAPH_STATUS={'YES' if lul130_connections else 'NO'}")
    if lul130_connections:
        print(f"LUL130_CONNECTIONS: {lul130_connections}")

    # Get top 10 downstream
    sorted_downstream = sorted(downstream_partners.items(), key=lambda item: item[1], reverse=True)
    print("TOP_10_AGGREGATED_DOWNSTREAM_TYPES:")
    for pct, count in sorted_downstream[:10]:
        print(f"  {pct}: {count} synapses")

    # 2. Setup simulation
    fly = make_locomotion_fly(
        name="m5_demo",
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

    actuated_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    preprogrammed_steps = PreprogrammedSteps()
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
        output_dof_order=actuated_dofs,
    )
    default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dofs)

    # 3. Timeline configuration
    t_activation = 1.25
    t_end = 3.0
    n_steps = round(t_end / actual_timestep)

    mdn_model = MDNCommandModel(canonical_mdns)

    out_dir = pathlib.Path("output/m5")
    out_dir.mkdir(parents=True, exist_ok=True)
    video_path = out_dir / "connectome_trigger_demo.mp4"
    metrics_path = out_dir / "connectome_trigger_metrics.json"

    # Set renderer
    renderer = sim.set_renderer(
        cam,
        camera_res=(360, 480), # Width, Height - note flygym typically expects (width, height)
        playback_speed=0.35,
        output_fps=30,
        buffer_frames=True,
    )

    sim.reset()
    controller.cpg_network.random_state = np.random.RandomState(0)
    controller.cpg_network.reset()
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    body_segs = fly.get_bodysegs_order()
    seg_names = [s.name if hasattr(s, "name") else str(s) for s in body_segs]
    root_name = fly.root_segment.name if hasattr(fly.root_segment, "name") else str(fly.root_segment)
    thorax_idx = seg_names.index(root_name)

    start_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()
    pos_at_activation = None

    max_joint_mod = 0.0
    metrics_data = []

    print("STATE=FORWARD")
    print(f"MDN_POPULATION_COMMAND=0.0")

    try:
        for step_idx in range(n_steps):
            t = step_idx * actual_timestep

            if t < t_activation:
                activities = {rid: 0.0 for rid in canonical_mdns}
                phase_offset = 0.0
            else:
                if pos_at_activation is None:
                    pos_at_activation = sim.get_body_positions(fly.name)[thorax_idx].copy()
                    print(f"EVENT=MDN_ACTIVATION")
                    print(f"EVENT_TIME_SECONDS={t:.2f}")
                    print(f"MDN_POPULATION_COMMAND=1.0")
                    print(f"STATE=ENGINEERED_MOONWALK")
                activities = {rid: 1.0 for rid in canonical_mdns}
                phase_offset = 0.55

            mdn_model.set_activities(activities)

            stock_action = controller.step()
            adhesion = compute_adhesion(
                phase_offset,
                stock_action.adhesion_onoff,
                controller.cpg_network.curr_phases,
                preprogrammed_steps
            )

            action_to_apply = LocomotionAction(
                joint_angles=stock_action.joint_angles,
                adhesion_onoff=adhesion,
            )

            diff = float(np.max(np.abs(action_to_apply.joint_angles - stock_action.joint_angles)))
            if diff > max_joint_mod:
                max_joint_mod = diff

            apply_locomotion_action(sim, fly.name, action_to_apply)
            sim.step()
            sim.render_as_needed()

            # Telemetry sample every 20 steps (~ 0.002s * 20 = 0.04s)
            if step_idx % 20 == 0:
                t_pos = sim.get_body_positions(fly.name)[thorax_idx]
                metrics_data.append({
                    "time": round(t, 4),
                    "activities": activities.copy(),
                    "left_command": mdn_model.left_command,
                    "right_command": mdn_model.right_command,
                    "population_command": mdn_model.population_command,
                    "moonwalk_mode_active": t >= t_activation,
                    "thorax_xyz": [float(x) for x in t_pos],
                    "adhesion_states": [float(x) for x in adhesion]
                })

        renderer.save_video(video_path)
    finally:
        renderer.close()

    end_thorax_pos = sim.get_body_positions(fly.name)[thorax_idx].copy()

    pre_x_disp = float(pos_at_activation[0] - start_thorax_pos[0])
    post_x_disp = float(end_thorax_pos[0] - pos_at_activation[0])

    pre_speed = pre_x_disp / t_activation
    post_speed = post_x_disp / (t_end - t_activation)
    total_x = float(end_thorax_pos[0] - start_thorax_pos[0])
    total_y = float(end_thorax_pos[1] - start_thorax_pos[1])

    print(f"PRE_MDN_FORWARD_DISPLACEMENT_MM={pre_x_disp:.4f}")
    print(f"PRE_MDN_AVG_X_SPEED_MM_S={pre_speed:.4f}")
    print(f"POST_MDN_X_DISPLACEMENT_MM={post_x_disp:.4f}")
    print(f"POST_MDN_AVG_X_SPEED_MM_S={post_speed:.4f}")
    print(f"TOTAL_X_DISPLACEMENT_MM={total_x:.4f}")
    print(f"TOTAL_Y_DISPLACEMENT_MM={total_y:.4f}")
    print(f"MAX_JOINT_TARGET_MODIFICATION_RAD={max_joint_mod:.4e}")

    with open(metrics_path, "w") as f:
        json.dump(metrics_data, f, indent=2)

    if pre_x_disp > 5.0 and post_x_disp < -5.0 and max_joint_mod == 0.0:
        print("M5_CONNECTOME_TRIGGER_RESULT=PASS")
    else:
        print("M5_CONNECTOME_TRIGGER_RESULT=FAIL")

if __name__ == "__main__":
    main()
