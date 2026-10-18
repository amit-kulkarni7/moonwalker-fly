#!/usr/bin/env python3
"""
MW-M25: Support-path geometry falsification test on the exact M8 champion.

Testing ONE narrow question:
"Does correcting the vertical geometry of the SUPPORTING tripod's trajectory
materially improve M8 contact persistence, pitch stability and body-glide
smoothness while preserving M8's high backward speed?"

Evaluates exactly THREE dynamic conditions:
- Condition A: DIRECT_M8_CONTROL (Fresh baseline)
- Condition B: BODY_FRAME_PLANAR_SUPPORT
- Condition C: WORLD_GROUND_CONSISTENT_SUPPORT

Strict causal isolation:
- No new slider trajectory
- No FTi postural cues
- No new topology / cadence / adhesion timing
- Support heights derived from fresh Condition A and frozen per leg
- DLS IK solves in dedicated scratch MjData (zero live physics mutation)
- Maximum dynamic physics runs: 6 (3 evaluation + 3 rendering).
"""

import sys, os, json, pathlib, argparse
from importlib.metadata import version
import numpy as np
import mujoco
import imageio.v2 as iio
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.append(".")

from flygym import Simulation
from flygym.anatomy import ContactBodiesPreset
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    PreprogrammedSteps, make_locomotion_fly, apply_locomotion_action, LocomotionAction
)
import scripts.m8_backslide_controller as m8
from scripts.m13_pulse_diagnosis import quat_to_pitch_rad

LEG_NAMES = ["LF", "LM", "LH", "RF", "RM", "RH"]
LEG_LOWER = [l.lower() for l in LEG_NAMES]
TRIPOD_A_INDS = [0, 2, 4]  # LF, LH, RM
TRIPOD_B_INDS = [1, 3, 5]  # LM, RF, RH

PHYSICS_DT = 0.0001
SAMPLE_STEPS = 10
SAMPLE_DT = PHYSICS_DT * SAMPLE_STEPS
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)
STEADY_START_TIME = 0.35

# -----------------------------------------------------------------------------
# DOF MAPPING & SEMANTIC AUDIT
# -----------------------------------------------------------------------------

def build_semantic_dof_mapping(fly, sim):
    """Explicitly map named actuated DOFs to leg groups and MuJoCo addresses."""
    actuated_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
    qpos_inds = []
    nv_inds = []
    for dof in actuated_dofs:
        jnt_id = mujoco.mj_name2id(sim.mj_model, mujoco.mjtObj.mjOBJ_JOINT, dof.name)
        if jnt_id == -1:
            jnt_id = mujoco.mj_name2id(sim.mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"{fly.name}/{dof.name}")
        if jnt_id == -1:
            raise KeyError(f"Joint '{dof.name}' not found in MuJoCo model.")
        qpos_inds.append(int(sim.mj_model.jnt_qposadr[jnt_id]))
        nv_inds.append(int(sim.mj_model.jnt_dofadr[jnt_id]))

    leg_act_inds = {}
    leg_nv_inds = {}
    leg_qpos_inds = {}
    dof_names_per_leg = {}

    for leg in LEG_LOWER:
        matching = [
            i for i, d in enumerate(actuated_dofs)
            if f"-{leg}_" in d.name.lower() or d.name.lower().startswith(f"{leg}_")
        ]
        if len(matching) != 7:
            raise ValueError(f"Expected exactly 7 DOFs for leg {leg}, got {len(matching)}: {[actuated_dofs[i].name for i in matching]}")
        leg_act_inds[leg] = matching
        leg_nv_inds[leg] = [nv_inds[i] for i in matching]
        leg_qpos_inds[leg] = [qpos_inds[i] for i in matching]
        dof_names_per_leg[leg] = [actuated_dofs[i].name for i in matching]

    return {
        "actuated_dofs": actuated_dofs,
        "qpos_inds": qpos_inds,
        "nv_inds": nv_inds,
        "leg_act_inds": leg_act_inds,
        "leg_nv_inds": leg_nv_inds,
        "leg_qpos_inds": leg_qpos_inds,
        "dof_names_per_leg": dof_names_per_leg
    }


def extract_actuator_limits(fly, sim):
    """Extract force limits and addresses for all 42 leg actuators."""
    actuator_info = []
    actuator_ids = []
    for dof in fly.get_actuated_jointdofs_order(ActuatorType.POSITION):
        act_name = f"{fly.name}/{dof.name}-position"
        act_id = mujoco.mj_name2id(sim.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
        if act_id == -1:
            act_name = f"{dof.name}-position"
            act_id = mujoco.mj_name2id(sim.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
        if act_id == -1:
            raise RuntimeError(f"Actuator '{act_name}' not found.")
        actuator_ids.append(act_id)
        frange = sim.mj_model.actuator_forcerange[act_id]
        actuator_info.append({
            "dof": dof.name,
            "actuator_id": int(act_id),
            "lower_limit": float(frange[0]),
            "upper_limit": float(frange[1])
        })
    return actuator_ids, actuator_info


# -----------------------------------------------------------------------------
# PREFLIGHT VERIFICATION
# -----------------------------------------------------------------------------

def run_preflight_provenance_audit():
    """Verifies that M8 parameters match authoritative tracked provenance."""
    manifest_path = pathlib.Path("data/derived/m8_backslide_manifest.json")
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing M8 manifest at {manifest_path}")

    with open(manifest_path) as f:
        m8_man = json.load(f)

    p = m8_man["selected_parameters"]
    expected = {
        "cadence": 6.0,
        "stance_excursion_fraction": 1.0,
        "turnaround_fraction": 0.06,
        "support_transfer_fraction": 0.08,
        "entry_duration": 0.25
    }

    mismatches = []
    for k, v in expected.items():
        if abs(p.get(k, 0.0) - v) > 1e-6:
            mismatches.append(f"{k}: expected {v}, got {p.get(k)}")

    if mismatches:
        print("STOP: M8_PARAMETER_PROVENANCE_CONFLICT")
        print("\n".join(mismatches))
        sys.exit(1)

    return expected


def verify_scratch_mjdata_isolation(fly, sim, dof_map):
    """Verifies that scratch MjData kinematic solves do not mutate live physics."""
    scratch_data = mujoco.MjData(sim.mj_model)
    qpos_live_before = sim.mj_data.qpos.copy()
    xpos_live_before = sim.mj_data.xpos.copy()

    # Modify scratch
    scratch_data.qpos[:] = sim.mj_data.qpos[:]
    scratch_data.qpos[dof_map["leg_qpos_inds"]["lf"]] += 0.05
    mujoco.mj_kinematics(sim.mj_model, scratch_data)
    mujoco.mj_comPos(sim.mj_model, scratch_data)

    if not np.all(sim.mj_data.qpos == qpos_live_before) or not np.all(sim.mj_data.xpos == xpos_live_before):
        raise RuntimeError("PREFLIGHT_FAIL: Scratch MjData mutated live physics state!")

    return "NO"


# -----------------------------------------------------------------------------
# CONTROLLER ARCHITECTURE
# -----------------------------------------------------------------------------

class M25SupportGeometryController:
    """
    Controller supporting Conditions B & C:
    - Retains M8 horizontal support progression
    - Replaces varying vertical trajectory with frozen support heights
    - Solves DLS IK in dedicated scratch MjData (zero live mutation)
    - Controlled marker: origin of body {leg}_tarsus5 conditioned on live passive tarsals
    - Released legs remain exact M8 when w_support == 0
    - Smooth blend across support transfer window (tf = 0.08).
    """
    def __init__(self, sim, fly, pps, actuated_dofs, dof_map,
                 mode="BODY_FRAME_PLANAR",
                 frozen_heights=None,
                 m8_params=None,
                 dls_damping=1e-4, reg_weight=0.01, max_dq=0.05):
        self.sim = sim
        self.fly = fly
        self.pps = pps
        self.actuated_dofs = actuated_dofs
        self.dof_map = dof_map
        self.mode = mode
        self.frozen_heights = frozen_heights or {}
        self.dls_damping = dls_damping
        self.reg_weight = reg_weight
        self.max_dq = max_dq

        self.m8_params = m8_params or {
            "cadence": 6.0, "stance_excursion_fraction": 1.0,
            "turnaround_fraction": 0.06, "support_transfer_fraction": 0.08,
            "entry_duration": 0.25, "timestep": PHYSICS_DT
        }

        self.base_m8 = m8.BackslideTrajectoryController(
            pps, actuated_dofs, **self.m8_params
        )

        self.scratch_data = mujoco.MjData(sim.mj_model)

        self.thorax_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
        self.t5_ids = {
            leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id
            for leg in LEG_LOWER
        }

        # IK diagnostics tracking
        self.ik_residuals = []
        self.ik_iterations = []
        self.ik_converged_count = 0
        self.ik_total_solves = 0

        self.last_target_q = None

    @property
    def default_pose(self):
        return self.base_m8.default_pose

    @property
    def time(self):
        return self.base_m8.time

    @time.setter
    def time(self, val):
        self.base_m8.time = val

    def step(self):
        nominal_action, phi = self.base_m8.step()
        nominal_q = nominal_action.joint_angles.copy()
        adhesions = nominal_action.adhesion_onoff.copy()

        applied_q = nominal_q.copy()

        # Update scratch with live state to preserve passive tarsals & thorax pose
        self.scratch_data.qpos[:] = self.sim.mj_data.qpos[:]
        mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
        mujoco.mj_comPos(self.sim.mj_model, self.scratch_data)

        # Measured thorax pose
        tpos_live = self.sim.mj_data.xpos[self.thorax_id].copy()
        tmat_live = self.sim.mj_data.xmat[self.thorax_id].reshape(3, 3).copy()

        jacp = np.zeros((3, self.sim.mj_model.nv))

        for i, leg in enumerate(LEG_LOWER):
            w_sup = float(adhesions[i])
            if w_sup < 1e-4:
                # Released leg: 100% exact M8
                continue

            qpos_inds = self.dof_map["leg_qpos_inds"][leg]
            nv_inds = self.dof_map["leg_nv_inds"][leg]
            act_inds = self.dof_map["leg_act_inds"][leg]
            t5_id = self.t5_ids[leg]

            # 1. Forward kinematics of nominal target for leg in scratch
            self.scratch_data.qpos[qpos_inds] = nominal_q[act_inds]
            mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
            p_nom_world = self.scratch_data.xpos[t5_id].copy()
            p_nom_body = tmat_live.T @ (p_nom_world - tpos_live)

            # 2. Construct desired target
            leg_name_upper = LEG_NAMES[i]
            if self.mode == "BODY_FRAME_PLANAR":
                z_target_body = self.frozen_heights.get(leg_name_upper, p_nom_body[2])
                p_target_body = np.array([p_nom_body[0], p_nom_body[1], z_target_body])
                p_target_world = tpos_live + tmat_live @ p_target_body
            elif self.mode == "WORLD_GROUND_CONSISTENT":
                z_target_world = self.frozen_heights.get(leg_name_upper, p_nom_world[2])
                p_target_world = np.array([p_nom_world[0], p_nom_world[1], z_target_world])
            else:
                p_target_world = p_nom_world

            # 3. Solve DLS IK in scratch MjData
            q_sol = nominal_q[act_inds].copy()
            converged = False
            final_err = 0.0
            iters = 0

            for it in range(15):
                iters = it + 1
                self.scratch_data.qpos[qpos_inds] = q_sol
                mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
                mujoco.mj_comPos(self.sim.mj_model, self.scratch_data)
                curr_pos = self.scratch_data.xpos[t5_id]
                err = p_target_world - curr_pos
                err_norm = float(np.linalg.norm(err))
                if err_norm < 1e-4:
                    converged = True
                    final_err = err_norm
                    break

                mujoco.mj_jac(self.sim.mj_model, self.scratch_data, jacp, None, curr_pos, t5_id)
                J = jacp[:, nv_inds]
                J_T = J.T
                J_pinv = J_T @ np.linalg.inv(J @ J_T + self.dls_damping**2 * np.eye(3))
                dq = J_pinv @ err + (np.eye(7) - J_pinv @ J) @ (self.reg_weight * (nominal_q[act_inds] - q_sol))
                dq = np.clip(dq, -self.max_dq, self.max_dq)
                q_sol += dq
                final_err = float(np.linalg.norm(p_target_world - self.scratch_data.xpos[t5_id]))

            self.ik_total_solves += 1
            if converged:
                self.ik_converged_count += 1
            self.ik_residuals.append(final_err)
            self.ik_iterations.append(iters)

            # 4. Smooth blend across support transfer window
            q_blend = (1.0 - w_sup) * nominal_q[act_inds] + w_sup * q_sol
            applied_q[act_inds] = q_blend

        # Apply smooth entry blending if in startup
        if self.time < self.base_m8.entry_duration:
            alpha = m8.smooth_step(self.time, 0.0, self.base_m8.entry_duration)
            applied_q = (1.0 - alpha) * self.default_pose + alpha * applied_q
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions

        self.last_target_q = applied_q.copy()
        return LocomotionAction(joint_angles=applied_q, adhesion_onoff=adhesions), phi


# -----------------------------------------------------------------------------
# SIMULATION ENGINE
# -----------------------------------------------------------------------------

def create_isolated_sim(fly):
    world = FlatGroundWorld()
    world.add_fly(
        fly,
        spawn_position=[0, 0, 0.5],
        spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]),
        bodysegs_with_ground_contact=ContactBodiesPreset.LEGS_THORAX_ABDOMEN_HEAD
    )
    return Simulation(world)


def run_isolated_simulation(condition_name, controller_builder, n_steps=N_STEPS, sample_dt=SAMPLE_DT):
    """Runs a single physically isolated simulation rollout."""
    print(f"Running simulation for condition: {condition_name} ({n_steps*PHYSICS_DT:.2f} s) ...")
    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    sim = create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    dof_map = build_semantic_dof_mapping(fly, sim)
    actuator_ids, actuator_limits = extract_actuator_limits(fly, sim)

    body_segs = [s.name if hasattr(s, "name") else str(s) for s in fly.get_bodysegs_order()]
    thorax_idx = body_segs.index("c_thorax")
    tarsus5_idxs = [body_segs.index(f"{leg}_tarsus5") for leg in LEG_LOWER]
    tarsus_seg_names = [f"{leg}_tarsus5" for leg in LEG_LOWER]

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    t_list, x_list, y_list, z_list = [], [], [], []
    pitch_list, quat_list = [], []
    vx_list, ax_list = [], []
    tarsus_xyz_list = []
    contact_list = []
    fz_list = []
    adhesion_list = []
    target_q_list = []
    actual_q_list = []
    actuator_forces_list = []

    prev_x = sim.get_body_positions(fly.name)[thorax_idx, 0]
    prev_vx = None

    step_target_diffs = []
    prev_target_q = None
    step_boundary_indices = []
    prev_phi = None

    for step_i in range(n_steps):
        action, phi = controller.step()
        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        curr_target_q = action.joint_angles.copy()
        if prev_target_q is not None:
            step_target_diffs.append(np.abs(curr_target_q - prev_target_q))
            # Detect role exchange boundaries
            if prev_phi is not None:
                if (prev_phi < 1.0 <= phi) or (prev_phi < 2.0 and phi < prev_phi):
                    step_boundary_indices.append(len(step_target_diffs) - 1)
        prev_target_q = curr_target_q
        prev_phi = phi

        if (step_i + 1) % SAMPLE_STEPS == 0:
            t = (step_i + 1) * PHYSICS_DT
            b_pos = sim.get_body_positions(fly.name)
            pos = b_pos[thorax_idx]
            quat = sim.get_body_rotations(fly.name)[thorax_idx]
            pitch_deg = np.degrees(quat_to_pitch_rad(quat))

            vx = float((pos[0] - prev_x) / sample_dt)
            prev_x = pos[0]
            ax = float((vx - prev_vx) / sample_dt) if prev_vx is not None else 0.0
            prev_vx = vx

            tars_pos = b_pos[tarsus5_idxs]
            cont_info = sim.get_ground_contact_info(fly.name)
            cont_leg = cont_info[0].copy()[:6]

            forces = sim.get_bodysegment_contact_forces(fly.name, tarsus_seg_names, ground_only=True)
            fz_leg = forces[:, 2].copy()

            adh_cmd = action.adhesion_onoff.copy()

            q_act = np.array([sim.mj_data.qpos[adr] for adr in dof_map["qpos_inds"]], dtype=float)
            q_targ = action.joint_angles.copy()
            act_forces = np.array([sim.mj_data.actuator_force[aid] for aid in actuator_ids], dtype=float)

            t_list.append(t)
            x_list.append(float(pos[0]))
            y_list.append(float(pos[1]))
            z_list.append(float(pos[2]))
            pitch_list.append(float(pitch_deg))
            quat_list.append(quat.copy())
            vx_list.append(vx)
            ax_list.append(ax)
            tarsus_xyz_list.append(tars_pos.copy())
            contact_list.append(cont_leg.copy())
            fz_list.append(fz_leg.copy())
            adhesion_list.append(adh_cmd.copy())
            target_q_list.append(q_targ.copy())
            actual_q_list.append(q_act.copy())
            actuator_forces_list.append(act_forces.copy())

    sim.close()

    step_diffs_arr = np.array(step_target_diffs) if step_target_diffs else np.zeros((1, len(dof_map["qpos_inds"])))

    ik_diag = {
        "converged_fraction": float(controller.ik_converged_count / max(controller.ik_total_solves, 1)),
        "residual_median_mm": float(np.median(controller.ik_residuals)) if controller.ik_residuals else 0.0,
        "residual_p95_mm": float(np.percentile(controller.ik_residuals, 95)) if controller.ik_residuals else 0.0,
        "residual_max_mm": float(np.max(controller.ik_residuals)) if controller.ik_residuals else 0.0,
        "iterations_p95": float(np.percentile(controller.ik_iterations, 95)) if controller.ik_iterations else 0.0,
        "total_solves": int(controller.ik_total_solves)
    } if hasattr(controller, "ik_residuals") else None

    return {
        "condition_name": condition_name,
        "sample_dt": sample_dt,
        "t": np.array(t_list),
        "x": np.array(x_list),
        "y": np.array(y_list),
        "z": np.array(z_list),
        "pitch": np.array(pitch_list),
        "quat": np.array(quat_list),
        "vx": np.array(vx_list),
        "ax": np.array(ax_list),
        "tarsus_xyz": np.array(tarsus_xyz_list),
        "contact": np.array(contact_list),
        "fz": np.array(fz_list),
        "adhesion": np.array(adhesion_list),
        "target_q": np.array(target_q_list),
        "actual_q": np.array(actual_q_list),
        "actuator_forces": np.array(actuator_forces_list),
        "actuator_limits": actuator_limits,
        "ik_diagnostics": ik_diag,
        "step_target_diffs": step_diffs_arr,
        "step_boundary_indices": step_boundary_indices
    }


# -----------------------------------------------------------------------------
# SUPPORT HEIGHT DERIVATION FROM FRESH CONDITION A
# -----------------------------------------------------------------------------

def derive_fresh_condition_a_support_heights(telem_a):
    """
    Derives frozen per-leg body and world support heights from fresh Condition A.
    Samples must satisfy:
    - strong support plateau: support tripod adh >= 0.90, opposite tripod adh <= 0.10
    - all three support legs in physical contact
    - individual leg in physical contact.
    """
    print("Deriving frozen per-leg support heights from fresh Condition A ...")
    ss_mask = telem_a["t"] >= STEADY_START_TIME

    adh = telem_a["adhesion"][ss_mask]
    cont = telem_a["contact"][ss_mask]
    tars_xyz = telem_a["tarsus_xyz"][ss_mask]
    x_pos = telem_a["x"][ss_mask]
    y_pos = telem_a["y"][ss_mask]
    z_pos = telem_a["z"][ss_mask]
    quats = telem_a["quat"][ss_mask]

    body_heights = {}
    world_heights = {}
    valid_sample_counts = {}

    for i, leg in enumerate(LEG_NAMES):
        is_tripod_a = (i in TRIPOD_A_INDS)
        sup_trip = TRIPOD_A_INDS if is_tripod_a else TRIPOD_B_INDS
        opp_trip = TRIPOD_B_INDS if is_tripod_a else TRIPOD_A_INDS

        sup_adh = np.median(adh[:, sup_trip], axis=1)
        opp_adh = np.median(adh[:, opp_trip], axis=1)
        all3_contact = np.all(cont[:, sup_trip] > 0, axis=1)
        leg_contact = cont[:, i] > 0

        valid_mask = (sup_adh >= 0.90) & (opp_adh <= 0.10) & all3_contact & leg_contact
        n_valid = int(np.sum(valid_mask))
        valid_sample_counts[leg] = n_valid

        if n_valid < 100:
            print(f"STOP: SUPPORT_HEIGHT_PROVENANCE_FAILURE for {leg} (found {n_valid} < 100 samples)")
            sys.exit(1)

        valid_wz = tars_xyz[valid_mask, i, 2]
        world_heights[leg] = float(np.median(valid_wz))

        # Compute body-relative Z for valid samples
        rel_z_list = []
        for idx in np.where(valid_mask)[0]:
            th_pos = np.array([x_pos[idx], y_pos[idx], z_pos[idx]])
            q = quats[idx]
            # Rotation matrix from quaternion (w, x, y, z)
            w, x, y, z = q[0], q[1], q[2], q[3]
            R = np.array([
                [1.0 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
                [2*(x*y + z*w),     1.0 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                [2*(x*z - y*w),     2*(y*z + x*w),     1.0 - 2*(x**2 + y**2)]
            ])
            p_rel = R.T @ (tars_xyz[idx, i] - th_pos)
            rel_z_list.append(p_rel[2])

        body_heights[leg] = float(np.median(rel_z_list))

    print("Frozen Six-Leg Derived Support Heights:")
    for leg in LEG_NAMES:
        print(f"  {leg}: World Z = {world_heights[leg]:.5f} mm | Body Z = {body_heights[leg]:.5f} mm (N={valid_sample_counts[leg]})")

    return {
        "body_support_height_median_mm": body_heights,
        "world_support_height_median_mm": world_heights,
        "valid_samples_per_leg": valid_sample_counts
    }


# -----------------------------------------------------------------------------
# ANALYTICS & HYPOTHESIS EVALUATION
# -----------------------------------------------------------------------------

def compute_condition_analytics(telem):
    """Computes full suite of body, contact, load, foot, and integrity metrics."""
    ss_mask = telem["t"] >= STEADY_START_TIME
    sample_dt = telem["sample_dt"]

    ss_t = telem["t"][ss_mask]
    ss_x = telem["x"][ss_mask]
    ss_y = telem["y"][ss_mask]
    ss_z = telem["z"][ss_mask]
    ss_pitch = telem["pitch"][ss_mask]
    ss_quat = telem["quat"][ss_mask]
    ss_vx = telem["vx"][ss_mask]
    ss_ax = telem["ax"][ss_mask]
    ss_tars = telem["tarsus_xyz"][ss_mask]
    ss_cont = telem["contact"][ss_mask]
    ss_fz = telem["fz"][ss_mask]
    ss_adh = telem["adhesion"][ss_mask]

    disp_x = float(ss_x[-1] - telem["x"][0])
    disp_y = float(ss_y[-1] - telem["y"][0])
    mean_vx = float(np.mean(ss_vx))
    mean_speed = float(-mean_vx)
    lateral_ratio = float(abs(disp_y) / max(abs(disp_x), 1e-5))

    # Boxcar filtered velocities
    w20 = max(1, int(0.020 / sample_dt))
    w50 = max(1, int(0.050 / sample_dt))
    vx_20 = np.convolve(ss_vx, np.ones(w20)/w20, mode="valid") if len(ss_vx) >= w20 else ss_vx
    vx_50 = np.convolve(ss_vx, np.ones(w50)/w50, mode="valid") if len(ss_vx) >= w50 else ss_vx

    cv20 = float(np.std(vx_20) / max(abs(mean_vx), 1e-9))
    cv50 = float(np.std(vx_50) / max(abs(mean_vx), 1e-9))

    wrong_sign_20 = float(np.mean(vx_20 > 0))
    pitch_ptp = float(np.ptp(ss_pitch))
    pitch_std = float(np.std(ss_pitch))
    z_ptp = float(np.ptp(ss_z))
    z_std = float(np.std(ss_z))

    # Euler roll and detrended yaw
    w = ss_quat[:, 0]; x = ss_quat[:, 1]; y = ss_quat[:, 2]; z = ss_quat[:, 3]
    roll_deg = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    roll_ptp = float(np.ptp(roll_deg))

    yaw_raw_rad = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    yaw_unwrapped = np.unwrap(yaw_raw_rad)
    lin_trend = np.polyval(np.polyfit(ss_t, yaw_unwrapped, 1), ss_t)
    yaw_detrended_deg = np.degrees(yaw_unwrapped - lin_trend)
    yaw_detrended_ptp = float(np.ptp(yaw_detrended_deg))

    # Steady body acceleration (20ms filtered)
    ss_pos = np.column_stack([ss_x, ss_y, ss_z])
    v_3d = np.diff(ss_pos, axis=0) / sample_dt
    kernel_20 = np.ones(w20) / float(w20)
    v_filt = np.column_stack([np.convolve(v_3d[:, d], kernel_20, mode="valid") for d in range(3)])
    a_filt = np.diff(v_filt, axis=0) / sample_dt
    accel_filt_p95 = float(np.percentile(np.linalg.norm(a_filt, axis=1), 95))

    # Transition dynamics (within +-20ms of role exchanges)
    phi_all = (telem["t"] * 6.0 * 2.0) % 2.0
    exchange_events = np.where((phi_all[:-1] < 1.0) & (phi_all[1:] >= 1.0))[0].tolist()
    exchange_events += np.where((phi_all[:-1] < 2.0) & (phi_all[1:] < phi_all[:-1]))[0].tolist()
    trans_window = int(0.020 / sample_dt)
    trans_indices = set()
    for e_idx in exchange_events:
        for offset in range(-trans_window, trans_window + 1):
            idx = e_idx + offset
            if 0 <= idx < len(ss_ax):
                trans_indices.add(idx)
    trans_indices = sorted(list(trans_indices))
    trans_accel_p95 = float(np.percentile(np.abs(ss_ax[trans_indices]), 95)) if trans_indices else 0.0

    # Support Contact Conditioning (M13C.1 style)
    # Strong support plateau: sup_adh >= 0.90 and opp_adh <= 0.10
    tripod_metrics = {}
    all3_samples_total = 0
    plateau_samples_total = 0

    for name, sup_inds, opp_inds in [("TRIPOD_A", TRIPOD_A_INDS, TRIPOD_B_INDS), ("TRIPOD_B", TRIPOD_B_INDS, TRIPOD_A_INDS)]:
        sup_adh = np.median(ss_adh[:, sup_inds], axis=1)
        opp_adh = np.median(ss_adh[:, opp_inds], axis=1)
        is_plateau = (sup_adh >= 0.90) & (opp_adh <= 0.10)
        n_plat = int(np.sum(is_plateau))
        plateau_samples_total += n_plat

        if n_plat > 0:
            c_sup = ss_cont[is_plateau][:, sup_inds] > 0
            front_frac = float(np.mean(c_sup[:, 0]))  # LF or LM
            hind_frac = float(np.mean(c_sup[:, 1]))   # LH or RF
            mid_frac = float(np.mean(c_sup[:, 2]))    # RM or RH
            all3_frac = float(np.mean(np.all(c_sup, axis=1)))
            all3_samples_total += int(np.sum(np.all(c_sup, axis=1)))
        else:
            front_frac, mid_frac, hind_frac, all3_frac = 0.0, 0.0, 0.0, 0.0

        tripod_metrics[name] = {
            "FRONT_CONTACT_FRACTION": front_frac,
            "MIDDLE_CONTACT_FRACTION": mid_frac,
            "HIND_CONTACT_FRACTION": hind_frac,
            "ALL3_SUPPORT_CONTACT_FRACTION": all3_frac,
            "PLATEAU_SAMPLES": n_plat
        }

    all3_agg = float(all3_samples_total / max(plateau_samples_total, 1))

    # Contiguous all3 intervals
    all3_mask = np.zeros(len(ss_t), dtype=bool)
    for trip_inds, opp_inds in [(TRIPOD_A_INDS, TRIPOD_B_INDS), (TRIPOD_B_INDS, TRIPOD_A_INDS)]:
        s_adh = np.median(ss_adh[:, trip_inds], axis=1)
        o_adh = np.median(ss_adh[:, opp_inds], axis=1)
        plat = (s_adh >= 0.90) & (o_adh <= 0.10)
        all3 = np.all(ss_cont[:, trip_inds] > 0, axis=1)
        all3_mask |= (plat & all3)

    diff = np.diff(np.pad(all3_mask.astype(int), (1, 1), 'constant'))
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]
    contiguous_intervals = []
    for s_idx, e_idx in zip(starts, ends):
        dur_ms = (e_idx - s_idx) * (sample_dt * 1000.0)
        if dur_ms >= 10.0:
            contiguous_intervals.append({
                "duration_ms": dur_ms,
                "dx_mm": float(ss_x[min(e_idx, len(ss_x)-1)] - ss_x[s_idx])
            })

    # Load Telemetry (M18S-style load qualification)
    pos_fz = np.clip(ss_fz, 0.0, None)
    tot_pos_fz = np.sum(pos_fz, axis=1)
    valid_fz_mask = tot_pos_fz >= 1e-6
    load_fractions = np.zeros_like(pos_fz)
    load_fractions[valid_fz_mask] = pos_fz[valid_fz_mask] / tot_pos_fz[valid_fz_mask, None]

    load_qualified_matrix = (ss_cont > 0) & (load_fractions >= 0.05)
    load_qual_counts = np.sum(load_qualified_matrix, axis=1)
    p05_load_qual_contacts = float(np.percentile(load_qual_counts, 5))

    # Per anatomical class load statistics during strong support
    class_indices = {"FRONT": [0, 3], "MIDDLE": [1, 4], "HIND": [2, 5]}
    load_class_metrics = {}
    for c_name, inds in class_indices.items():
        c_pos_fz = pos_fz[:, inds].flatten()
        c_load_frac = load_fractions[:, inds].flatten()
        c_cont = (ss_cont[:, inds] > 0).flatten()
        load_class_metrics[c_name] = {
            "MEDIAN_POSITIVE_FZ_MN": float(np.median(c_pos_fz[c_pos_fz > 0])) if np.sum(c_pos_fz > 0) > 0 else 0.0,
            "MEDIAN_LOAD_FRACTION": float(np.median(c_load_frac)),
            "P05_LOAD_FRACTION": float(np.percentile(c_load_frac, 5)),
            "CONTACT_FRACTION": float(np.mean(c_cont))
        }

    # Foot Mechanics
    tars_vx = np.diff(ss_tars[:, :, 0], axis=0) / sample_dt
    tars_vx = np.vstack([tars_vx[0:1], tars_vx])  # Match shape

    anchor_vx_all = []
    slider_vx_all = []
    slider_neg_count = 0
    slider_tot_count = 0
    slider_cont_count = 0

    foot_mechanics_per_leg = {}
    for i, leg in enumerate(LEG_NAMES):
        adh_i = ss_adh[:, i]
        cont_i = ss_cont[:, i] > 0
        vx_i = tars_vx[:, i]

        is_act = (adh_i >= 0.90) & cont_i
        is_rel = adh_i <= 0.10

        act_vx = np.abs(vx_i[is_act])
        anchor_vx_all.extend(act_vx)

        rel_grounded = is_rel & cont_i
        rel_vx = vx_i[rel_grounded]
        slider_vx_all.extend(rel_vx)
        slider_neg_count += np.sum(rel_vx < 0)
        slider_tot_count += len(rel_vx)
        slider_cont_count += np.sum(cont_i[is_rel])

        foot_mechanics_per_leg[leg] = {
            "ANCHOR_MEDIAN_ABS_VX_MM_S": float(np.median(act_vx)) if len(act_vx) > 0 else 0.0,
            "SLIDER_MEDIAN_VX_MM_S": float(np.median(rel_vx)) if len(rel_vx) > 0 else 0.0,
            "SLIDER_NEGATIVE_FRACTION": float(np.mean(rel_vx < 0)) if len(rel_vx) > 0 else 0.0,
            "SLIDER_CONTACT_FRACTION": float(np.mean(cont_i[is_rel])) if np.sum(is_rel) > 0 else 0.0
        }

    # Target & Actuator Integrity
    target_q = telem["target_q"]
    step_diffs = telem.get("step_target_diffs", np.abs(np.diff(target_q, axis=0)))
    step_bound_inds = telem.get("step_boundary_indices", [])

    max_target_step = float(np.max(step_diffs))
    p99_target_step = float(np.percentile(step_diffs, 99))
    max_boundary_step = float(np.max(step_diffs[step_bound_inds, :])) if len(step_bound_inds) > 0 else max_target_step
    boundary_ratio = float(max_boundary_step / max(p99_target_step, 1e-6))

    target_finite = bool(np.all(np.isfinite(target_q)))
    hidden_teleport = bool(max_target_step > 0.050)

    # Actuator force & tracking telemetry
    act_forces = telem["actuator_forces"]
    act_limits = telem["actuator_limits"]
    sat_matrix = np.zeros_like(act_forces, dtype=bool)
    for a_i, lim in enumerate(act_limits):
        f_i = act_forces[:, a_i]
        sat_matrix[:, a_i] = (f_i <= 0.98 * lim["lower_limit"]) | (f_i >= 0.98 * lim["upper_limit"])

    tracking_errors = np.abs(telem["target_q"] - telem["actual_q"])

    return {
        "MEAN_BACKWARD_SPEED_MM_S": mean_speed,
        "BACKWARD_DISPLACEMENT_MM": disp_x,
        "BACKWARD_SPEED_CV_50MS": cv50,
        "BACKWARD_SPEED_CV_20MS": cv20,
        "PITCH_PEAK_TO_PEAK_DEG": pitch_ptp,
        "PITCH_STD_DEG": pitch_std,
        "THORAX_Z_PEAK_TO_PEAK_MM": z_ptp,
        "THORAX_Z_STD_MM": z_std,
        "LATERAL_RATIO": lateral_ratio,
        "BODY_WRONG_SIGN_FRACTION_20MS": wrong_sign_20,
        "STEADY_BODY_ACCEL_P95_MM_S2": accel_filt_p95,
        "TRANSITION_ACCEL_P95_MM_S2": trans_accel_p95,
        "ROLL_PTP_DEG": roll_ptp,
        "YAW_DETRENDED_PTP_DEG": yaw_detrended_ptp,
        "P05_LOAD_QUALIFIED_CONTACT_COUNT": p05_load_qual_contacts,
        "ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE": all3_agg,
        "TRIPOD_SUPPORT_METRICS": tripod_metrics,
        "CONTIGUOUS_ALL3_INTERVALS_COUNT": len(contiguous_intervals),
        "CONTIGUOUS_ALL3_MEAN_DURATION_MS": float(np.mean([i["duration_ms"] for i in contiguous_intervals])) if contiguous_intervals else 0.0,
        "LOAD_CLASS_METRICS": load_class_metrics,
        "ANCHOR_MEDIAN_ABS_WORLD_VX_ALL_MM_S": float(np.median(anchor_vx_all)) if anchor_vx_all else 0.0,
        "SLIDER_MEDIAN_WORLD_VX_ALL_MM_S": float(np.median(slider_vx_all)) if slider_vx_all else 0.0,
        "SLIDER_NEGATIVE_VX_FRACTION_ALL": float(slider_neg_count / max(slider_tot_count, 1)),
        "SLIDER_CONTACT_FRACTION_ALL": float(slider_cont_count / max(np.sum(ss_adh <= 0.10), 1)),
        "FOOT_MECHANICS_PER_LEG": foot_mechanics_per_leg,
        "TARGET_INTEGRITY": {
            "MAX_TARGET_STEP_RAD": max_target_step,
            "P99_TARGET_STEP_RAD": p99_target_step,
            "MAX_BOUNDARY_TARGET_STEP_RAD": max_boundary_step,
            "BOUNDARY_TO_P99_TARGET_STEP_RATIO": boundary_ratio,
            "TARGET_FINITE": target_finite,
            "HIDDEN_TARGET_TELEPORT": "NO" if not hidden_teleport else "YES"
        },
        "ACTUATOR_INTEGRITY": {
            "MAX_ABSOLUTE_FORCE_MN": float(np.max(np.abs(act_forces))),
            "SATURATION_FRACTION": float(np.mean(sat_matrix)),
            "TRACKING_ERROR_P95_RAD": float(np.percentile(tracking_errors, 95))
        },
        "IK_DIAGNOSTICS": telem["ik_diagnostics"]
    }


def evaluate_hypothesis_gates(m_cand, m_baseline):
    """Evaluates the 8 pre-registered primary hypothesis criteria against fresh M8."""
    gates = {}

    gates["speed_ge_8"] = bool(m_cand["MEAN_BACKWARD_SPEED_MM_S"] >= 8.0)

    base_all3 = m_baseline["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"]
    cand_all3 = m_cand["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"]
    gates["all3_contact_progress"] = bool((cand_all3 >= base_all3 + 0.15) or (cand_all3 >= 0.80))

    base_pitch = m_baseline["PITCH_PEAK_TO_PEAK_DEG"]
    cand_pitch = m_cand["PITCH_PEAK_TO_PEAK_DEG"]
    gates["pitch_improved_20pct"] = bool(cand_pitch <= 0.80 * base_pitch)

    base_cv50 = m_baseline["BACKWARD_SPEED_CV_50MS"]
    cand_cv50 = m_cand["BACKWARD_SPEED_CV_50MS"]
    gates["cv50_improved_10pct"] = bool(cand_cv50 <= 0.90 * base_cv50)

    base_z_ptp = m_baseline["THORAX_Z_PEAK_TO_PEAK_MM"]
    cand_z_ptp = m_cand["THORAX_Z_PEAK_TO_PEAK_MM"]
    gates["z_ptp_not_worsened_25pct"] = bool(cand_z_ptp <= 1.25 * base_z_ptp)

    gates["lateral_ratio_le_002"] = bool(m_cand["LATERAL_RATIO"] <= 0.02)

    base_ws20 = m_baseline["BODY_WRONG_SIGN_FRACTION_20MS"]
    cand_ws20 = m_cand["BODY_WRONG_SIGN_FRACTION_20MS"]
    gates["wrong_sign_preserved"] = bool(cand_ws20 <= base_ws20 + 0.02)

    t_integ = m_cand["TARGET_INTEGRITY"]
    a_integ = m_cand["ACTUATOR_INTEGRITY"]
    ik_diag = m_cand["IK_DIAGNOSTICS"]
    ik_valid = (ik_diag is None) or (ik_diag["residual_max_mm"] <= 1e-3)
    gates["actuator_and_continuity_safe"] = bool(
        t_integ["TARGET_FINITE"] and
        (t_integ["HIDDEN_TARGET_TELEPORT"] == "NO") and
        (t_integ["MAX_TARGET_STEP_RAD"] <= 0.020) and
        (t_integ["BOUNDARY_TO_P99_TARGET_STEP_RATIO"] <= 3.0) and
        (a_integ["SATURATION_FRACTION"] == 0.0) and
        ik_valid
    )

    all_pass = all(gates.values())
    return gates, all_pass


def classify_condition(gates, m_cand, m_baseline):
    """Assigns formal interpretation class."""
    if not m_cand["TARGET_INTEGRITY"]["TARGET_FINITE"] or (m_cand["IK_DIAGNOSTICS"] and m_cand["IK_DIAGNOSTICS"]["residual_max_mm"] > 1e-3):
        return "INVALID_IMPLEMENTATION"

    all_pass = all(gates.values())
    if all_pass:
        return "SUPPORTED"

    base_all3 = m_baseline["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"]
    cand_all3 = m_cand["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"]
    base_pitch = m_baseline["PITCH_PEAK_TO_PEAK_DEG"]
    cand_pitch = m_cand["PITCH_PEAK_TO_PEAK_DEG"]
    base_cv50 = m_baseline["BACKWARD_SPEED_CV_50MS"]
    cand_cv50 = m_cand["BACKWARD_SPEED_CV_50MS"]

    all3_improved = (cand_all3 >= base_all3 + 0.10)
    pitch_or_cv_improved = (cand_pitch <= 0.90 * base_pitch) or (cand_cv50 <= 0.90 * base_cv50)
    speed_ok = (m_cand["MEAN_BACKWARD_SPEED_MM_S"] >= 8.0)

    if all3_improved and pitch_or_cv_improved and speed_ok and gates["actuator_and_continuity_safe"]:
        return "PARTIALLY_SUPPORTED"

    return "SUPPORT_PLANARIZATION_FAILED"


def adjudicate_b_vs_c(m_b, class_b, m_c, class_c):
    """Deterministic lexicographic adjudication between Condition B and Condition C."""
    rank = {"SUPPORTED": 3, "PARTIALLY_SUPPORTED": 2, "SUPPORT_PLANARIZATION_FAILED": 1, "INVALID_IMPLEMENTATION": 0}
    r_b = rank.get(class_b, 0)
    r_c = rank.get(class_c, 0)

    if r_b > r_c:
        return "YES", "NO", "NO", "Condition B achieved strictly higher classification tier"
    elif r_c > r_b:
        return "NO", "YES", "NO", "Condition C achieved strictly higher classification tier"

    # Same tier: lexicographic comparison
    # 1. ALL3 support contact aggregate (diff > 0.02)
    d_all3 = m_c["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"] - m_b["ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE"]
    if abs(d_all3) > 0.02:
        return ("NO", "YES", "NO", f"C had higher all-3 contact ({m_c['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} vs {m_b['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f})") if d_all3 > 0 else ("YES", "NO", "NO", f"B had higher all-3 contact ({m_b['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} vs {m_c['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f})")

    # 2. Pitch PTP (diff > 0.2 deg)
    d_pitch = m_b["PITCH_PEAK_TO_PEAK_DEG"] - m_c["PITCH_PEAK_TO_PEAK_DEG"]
    if abs(d_pitch) > 0.20:
        return ("NO", "YES", "NO", f"C had lower pitch PTP ({m_c['PITCH_PEAK_TO_PEAK_DEG']:.2f}° vs {m_b['PITCH_PEAK_TO_PEAK_DEG']:.2f}°)") if d_pitch > 0 else ("YES", "NO", "NO", f"B had lower pitch PTP ({m_b['PITCH_PEAK_TO_PEAK_DEG']:.2f}° vs {m_c['PITCH_PEAK_TO_PEAK_DEG']:.2f}°)")

    # 3. CV50 (diff > 0.010)
    d_cv50 = m_b["BACKWARD_SPEED_CV_50MS"] - m_c["BACKWARD_SPEED_CV_50MS"]
    if abs(d_cv50) > 0.010:
        return ("NO", "YES", "NO", f"C had lower CV50 ({m_c['BACKWARD_SPEED_CV_50MS']:.3f} vs {m_b['BACKWARD_SPEED_CV_50MS']:.3f})") if d_cv50 > 0 else ("YES", "NO", "NO", f"B had lower CV50 ({m_b['BACKWARD_SPEED_CV_50MS']:.3f} vs {m_c['BACKWARD_SPEED_CV_50MS']:.3f})")

    # 4. Backward speed (diff > 0.20 mm/s)
    d_speed = m_c["MEAN_BACKWARD_SPEED_MM_S"] - m_b["MEAN_BACKWARD_SPEED_MM_S"]
    if abs(d_speed) > 0.20:
        return ("NO", "YES", "NO", f"C had higher backward speed ({m_c['MEAN_BACKWARD_SPEED_MM_S']:.2f} vs {m_b['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s)") if d_speed > 0 else ("YES", "NO", "NO", f"B had higher backward speed ({m_b['MEAN_BACKWARD_SPEED_MM_S']:.2f} vs {m_c['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s)")

    # 5. Thorax Z PTP (diff > 0.010 mm)
    d_z = m_b["THORAX_Z_PEAK_TO_PEAK_MM"] - m_c["THORAX_Z_PEAK_TO_PEAK_MM"]
    if abs(d_z) > 0.010:
        return ("NO", "YES", "NO", f"C had lower thorax Z PTP ({m_c['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} vs {m_b['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm)") if d_z > 0 else ("YES", "NO", "NO", f"B had lower thorax Z PTP ({m_b['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} vs {m_c['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm)")

    # 6. Actuator tracking error P95 (diff > 0.005 rad)
    d_track = m_b["ACTUATOR_INTEGRITY"]["TRACKING_ERROR_P95_RAD"] - m_c["ACTUATOR_INTEGRITY"]["TRACKING_ERROR_P95_RAD"]
    if abs(d_track) > 0.005:
        return ("NO", "YES", "NO", "C had lower tracking error") if d_track > 0 else ("YES", "NO", "NO", "B had lower tracking error")

    return "NO", "NO", "YES", "All primary differences within deterministic noise tolerances"


# -----------------------------------------------------------------------------
# VIDEO RENDERING & COMPOSITING
# -----------------------------------------------------------------------------

def render_condition_video_dual_speed(condition_name, controller_builder, out_path_02x, out_path_10x, fps=30):
    """Reruns physics once with camera at 0.2x and encodes both 0.2x and 1.0x videos."""
    os.makedirs(os.path.dirname(out_path_02x), exist_ok=True)
    os.makedirs(os.path.dirname(out_path_10x), exist_ok=True)

    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    cam = fly.add_tracking_camera(name="trackcam")
    sim = create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    renderer = sim.set_renderer(
        cam,
        camera_res=(240, 320),
        playback_speed=0.2,
        output_fps=fps,
        buffer_frames=True
    )

    try:
        for _ in range(N_STEPS):
            action, _ = controller.step()
            apply_locomotion_action(sim, fly.name, action)
            sim.step_with_profile()
            sim.render_as_needed_with_profile()

        cam_name = list(renderer.frames.keys())[0]
        frames_02x = renderer.frames[cam_name]

        # 0.2x slow motion
        iio.mimsave(out_path_02x, frames_02x, fps=fps, codec="libx264")

        # 1.0x presentation (every 5th frame)
        frames_10x = frames_02x[::5]
        iio.mimsave(out_path_10x, frames_10x, fps=fps, codec="libx264")
    finally:
        renderer.close()
        sim.close()


def create_three_way_video(path_a, path_b, path_c, out_path, label_a, label_b, label_c):
    """Composites 3 videos side-by-side with crisp label overlays."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    reader_a = iio.get_reader(str(path_a))
    reader_b = iio.get_reader(str(path_b))
    reader_c = iio.get_reader(str(path_c))
    fps = reader_a.get_meta_data().get("fps", 30)

    writer = iio.get_writer(str(out_path), fps=fps)
    try:
        for frame_a, frame_b, frame_c in zip(reader_a, reader_b, reader_c):
            img_a = Image.fromarray(frame_a)
            img_b = Image.fromarray(frame_b)
            img_c = Image.fromarray(frame_c)

            for img, lbl in [(img_a, label_a), (img_b, label_b), (img_c, label_c)]:
                draw = ImageDraw.Draw(img)
                draw.rectangle([0, 0, 320, 24], fill=(0, 0, 0, 160))
                draw.text((8, 5), lbl, fill="white")

            w, h = img_a.size
            combined = Image.new("RGB", (w * 3, h))
            combined.paste(img_a, (0, 0))
            combined.paste(img_b, (w, 0))
            combined.paste(img_c, (w * 2, 0))
            writer.append_data(np.array(combined))
    finally:
        reader_a.close()
        reader_b.close()
        reader_c.close()
        writer.close()


# -----------------------------------------------------------------------------
# MAIN EXPERIMENT ORCHESTRATOR
# -----------------------------------------------------------------------------

def main():
    print("=" * 80)
    print("MW-M25: SUPPORT-PATH GEOMETRY FALSIFICATION TEST ON EXACT M8 CHAMPION")
    print("=" * 80)

    # 1. Preflight Audits
    m8_params = run_preflight_provenance_audit()
    print("M8 Authoritative Parameter Provenance Verified ✓")

    fly_pre = make_locomotion_fly("fly_pre", add_adhesion=True)
    sim_pre = create_isolated_sim(fly_pre)
    dof_map_pre = build_semantic_dof_mapping(fly_pre, sim_pre)
    sim_pre.reset()
    sim_pre.warmup()
    verify_scratch_mjdata_isolation(fly_pre, sim_pre, dof_map_pre)
    print("Scratch MjData Physics State Non-Mutation Verified: NO live mutation ✓")
    sim_pre.close()

    pps = PreprogrammedSteps()

    # 2. RUN CONDITION A: DIRECT_M8_CONTROL
    print("\n" + "-"*80)
    print("STEP 1: EVALUATING CONDITION A (DIRECT_M8_CONTROL)...")
    print("-"*80)
    def build_ctrl_a(sim, fly):
        act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
        return m8.BackslideTrajectoryController(pps, act_dofs, **m8_params)

    telem_a = run_isolated_simulation("DIRECT_M8_CONTROL", build_ctrl_a)
    metrics_a = compute_condition_analytics(telem_a)

    print(f"\nCondition A Results:")
    print(f"  Speed:        {metrics_a['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s (hist: ~9.93)")
    print(f"  Displacement: {metrics_a['BACKWARD_DISPLACEMENT_MM']:.2f} mm (hist: ~-14.94)")
    print(f"  CV50:         {metrics_a['BACKWARD_SPEED_CV_50MS']:.3f} (hist: ~0.233)")
    print(f"  CV20:         {metrics_a['BACKWARD_SPEED_CV_20MS']:.3f} (hist: ~0.463)")
    print(f"  Pitch PTP:    {metrics_a['PITCH_PEAK_TO_PEAK_DEG']:.2f}° (hist: ~8.49°)")
    print(f"  Thorax Z PTP: {metrics_a['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm (hist: ~0.106)")
    print(f"  Lateral:      {metrics_a['LATERAL_RATIO']:.4f} (hist: ~0.005)")
    print(f"  All-3 Contact:{metrics_a['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f}")

    # Reproduction Gate Check
    repro_checks = {
        "speed_in_bounds": (9.63 <= metrics_a['MEAN_BACKWARD_SPEED_MM_S'] <= 10.23),
        "disp_in_bounds": (-15.44 <= metrics_a['BACKWARD_DISPLACEMENT_MM'] <= -14.44),
        "cv50_in_bounds": (0.208 <= metrics_a['BACKWARD_SPEED_CV_50MS'] <= 0.258),
        "pitch_in_bounds": (7.99 <= metrics_a['PITCH_PEAK_TO_PEAK_DEG'] <= 8.99),
        "z_ptp_in_bounds": (0.091 <= metrics_a['THORAX_Z_PEAK_TO_PEAK_MM'] <= 0.121),
        "lateral_in_bounds": (metrics_a['LATERAL_RATIO'] <= 0.010),
        "continuity_safe": (metrics_a['TARGET_INTEGRITY']['MAX_TARGET_STEP_RAD'] <= 0.020)
    }

    if not all(repro_checks.values()):
        print("\nSTOP: M8_REPRODUCTION_FAILED")
        print("M8 Reproduction checks failed:", repro_checks)
        sys.exit(1)
    print("\nM8 Reproduction Gate: PASSED ✓")

    # 3. DERIVE AND FREEZE SUPPORT HEIGHTS FROM FRESH CONDITION A
    print("\n" + "-"*80)
    print("STEP 2: DERIVING AND FREEZING PER-LEG SUPPORT HEIGHTS...")
    print("-"*80)
    height_data = derive_fresh_condition_a_support_heights(telem_a)
    frozen_body_heights = height_data["body_support_height_median_mm"]
    frozen_world_heights = height_data["world_support_height_median_mm"]

    # 4. RUN CONDITION B: BODY_FRAME_PLANAR_SUPPORT
    print("\n" + "-"*80)
    print("STEP 3: EVALUATING CONDITION B (BODY_FRAME_PLANAR_SUPPORT)...")
    print("-"*80)
    def build_ctrl_b(sim, fly):
        act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
        dof_map = build_semantic_dof_mapping(fly, sim)
        return M25SupportGeometryController(
            sim, fly, pps, act_dofs, dof_map,
            mode="BODY_FRAME_PLANAR",
            frozen_heights=frozen_body_heights,
            m8_params=m8_params
        )

    telem_b = run_isolated_simulation("BODY_FRAME_PLANAR_SUPPORT", build_ctrl_b)
    metrics_b = compute_condition_analytics(telem_b)
    gates_b, all_pass_b = evaluate_hypothesis_gates(metrics_b, metrics_a)
    class_b = classify_condition(gates_b, metrics_b, metrics_a)

    print(f"\nCondition B Results:")
    print(f"  Speed:        {metrics_b['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s")
    print(f"  CV50:         {metrics_b['BACKWARD_SPEED_CV_50MS']:.3f} (M8: {metrics_a['BACKWARD_SPEED_CV_50MS']:.3f})")
    print(f"  Pitch PTP:    {metrics_b['PITCH_PEAK_TO_PEAK_DEG']:.2f}° (M8: {metrics_a['PITCH_PEAK_TO_PEAK_DEG']:.2f}°)")
    print(f"  Thorax Z PTP: {metrics_b['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm (M8: {metrics_a['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm)")
    print(f"  All-3 Contact:{metrics_b['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} (M8: {metrics_a['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f})")
    print(f"  Classification: {class_b}")

    # 5. RUN CONDITION C: WORLD_GROUND_CONSISTENT_SUPPORT
    print("\n" + "-"*80)
    print("STEP 4: EVALUATING CONDITION C (WORLD_GROUND_CONSISTENT_SUPPORT)...")
    print("-"*80)
    def build_ctrl_c(sim, fly):
        act_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)
        dof_map = build_semantic_dof_mapping(fly, sim)
        return M25SupportGeometryController(
            sim, fly, pps, act_dofs, dof_map,
            mode="WORLD_GROUND_CONSISTENT",
            frozen_heights=frozen_world_heights,
            m8_params=m8_params
        )

    telem_c = run_isolated_simulation("WORLD_GROUND_CONSISTENT_SUPPORT", build_ctrl_c)
    metrics_c = compute_condition_analytics(telem_c)
    gates_c, all_pass_c = evaluate_hypothesis_gates(metrics_c, metrics_a)
    class_c = classify_condition(gates_c, metrics_c, metrics_a)

    print(f"\nCondition C Results:")
    print(f"  Speed:        {metrics_c['MEAN_BACKWARD_SPEED_MM_S']:.2f} mm/s")
    print(f"  CV50:         {metrics_c['BACKWARD_SPEED_CV_50MS']:.3f} (M8: {metrics_a['BACKWARD_SPEED_CV_50MS']:.3f})")
    print(f"  Pitch PTP:    {metrics_c['PITCH_PEAK_TO_PEAK_DEG']:.2f}° (M8: {metrics_a['PITCH_PEAK_TO_PEAK_DEG']:.2f}°)")
    print(f"  Thorax Z PTP: {metrics_c['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm (M8: {metrics_a['THORAX_Z_PEAK_TO_PEAK_MM']:.3f} mm)")
    print(f"  All-3 Contact:{metrics_c['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f} (M8: {metrics_a['ALL3_SUPPORT_CONTACT_FRACTION_AGGREGATE']:.3f})")
    print(f"  Classification: {class_c}")

    # 6. DETERMINISTIC B-vs-C ADJUDICATION
    b_better, c_better, no_diff, adj_reason = adjudicate_b_vs_c(metrics_b, class_b, metrics_c, class_c)
    print("\n" + "-"*80)
    print("STEP 5: DETERMINISTIC B-vs-C COMPARISON")
    print("-"*80)
    print(f"  BODY_FRAME_PLANAR_BETTER:       {b_better}")
    print(f"  WORLD_GROUND_CONSISTENT_BETTER: {c_better}")
    print(f"  NO_CLEAR_DIFFERENCE:            {no_diff}")
    print(f"  Rationale:                      {adj_reason}")

    # 7. RENDERING VIDEO SUITE (3 clean reruns, total budget = 6 runs)
    print("\n" + "-"*80)
    print("STEP 6: RENDERING VIDEO SUITE (3 Physics Reruns)...")
    print("-"*80)
    vid_dir = pathlib.Path("output/m25")
    vid_dir.mkdir(parents=True, exist_ok=True)

    out_a_02x = str(vid_dir / "video_A_m8_02x.mp4")
    out_a_10x = str(vid_dir / "video_A_m8_10x.mp4")
    out_b_02x = str(vid_dir / "video_B_body_planar_02x.mp4")
    out_b_10x = str(vid_dir / "video_B_body_planar_10x.mp4")
    out_c_02x = str(vid_dir / "video_C_world_ground_02x.mp4")
    out_c_10x = str(vid_dir / "video_C_world_ground_10x.mp4")
    out_sbs_10x = str(vid_dir / "video_A_vs_B_vs_C_10x.mp4")
    out_sbs_02x = str(vid_dir / "video_A_vs_B_vs_C_02x.mp4")

    print("Rendering Condition A (0.2x & 1.0x) ...", end="", flush=True)
    render_condition_video_dual_speed("DIRECT_M8", build_ctrl_a, out_a_02x, out_a_10x)
    print(" Done ✓")

    print("Rendering Condition B (0.2x & 1.0x) ...", end="", flush=True)
    render_condition_video_dual_speed("BODY_FRAME_PLANAR", build_ctrl_b, out_b_02x, out_b_10x)
    print(" Done ✓")

    print("Rendering Condition C (0.2x & 1.0x) ...", end="", flush=True)
    render_condition_video_dual_speed("WORLD_GROUND_CONSISTENT", build_ctrl_c, out_c_02x, out_c_10x)
    print(" Done ✓")

    print("Compositing 3-way side-by-side presentation video (1.0x) ...", end="", flush=True)
    create_three_way_video(
        out_a_10x, out_b_10x, out_c_10x, out_sbs_10x,
        "A: DIRECT M8 (1.0x)", "B: BODY PLANAR (1.0x)", "C: WORLD GROUND (1.0x)"
    )
    print(" Done ✓")

    print("Compositing 3-way side-by-side diagnostic video (0.2x) ...", end="", flush=True)
    create_three_way_video(
        out_a_02x, out_b_02x, out_c_02x, out_sbs_02x,
        "A: DIRECT M8 (0.2x)", "B: BODY PLANAR (0.2x)", "C: WORLD GROUND (0.2x)"
    )
    print(" Done ✓")

    # 8. ASSEMBLE RESULTS PAYLOAD & MANIFEST
    results_payload = {
        "milestone": "MW-M25",
        "description": "Support-path geometry falsification test on exact M8 champion",
        "flygym_version": version("flygym"),
        "mujoco_version": version("mujoco"),
        "provenance_check": "PASSED",
        "preflight_live_state_mutation": "NO",
        "frozen_support_heights": height_data,
        "conditions": {
            "CONDITION_A": {
                "name": "DIRECT_M8_CONTROL",
                "metrics": metrics_a,
                "reproduction_gate_checks": repro_checks,
                "reproduction_passed": True
            },
            "CONDITION_B": {
                "name": "BODY_FRAME_PLANAR_SUPPORT",
                "metrics": metrics_b,
                "hypothesis_gates": gates_b,
                "all_gates_pass": all_pass_b,
                "classification": class_b
            },
            "CONDITION_C": {
                "name": "WORLD_GROUND_CONSISTENT_SUPPORT",
                "metrics": metrics_c,
                "hypothesis_gates": gates_c,
                "all_gates_pass": all_pass_c,
                "classification": class_c
            }
        },
        "adjudication": {
            "BODY_FRAME_PLANAR_BETTER": b_better,
            "WORLD_GROUND_CONSISTENT_BETTER": c_better,
            "NO_CLEAR_DIFFERENCE": no_diff,
            "rationale": adj_reason
        },
        "qualitative_review_status": "PENDING_MANAGER_REVIEW",
        "video_files": {
            "A_02x": out_a_02x, "A_10x": out_a_10x,
            "B_02x": out_b_02x, "B_10x": out_b_10x,
            "C_02x": out_c_02x, "C_10x": out_c_10x,
            "composite_10x": out_sbs_10x, "composite_02x": out_sbs_02x
        }
    }

    manifest_payload = {
        "milestone": "MW-M25",
        "task": "Support-path geometry falsification test on the exact M8 champion",
        "m8_parameters": m8_params,
        "frozen_heights": height_data,
        "gates": {
            "m8_reproduction_gate": "PASS",
            "condition_b_gates_pass": all_pass_b,
            "condition_c_gates_pass": all_pass_c
        },
        "classifications": {
            "condition_b": class_b,
            "condition_c": class_c
        },
        "adjudication": {
            "BODY_FRAME_PLANAR_BETTER": b_better,
            "WORLD_GROUND_CONSISTENT_BETTER": c_better,
            "NO_CLEAR_DIFFERENCE": no_diff
        },
        "scientific_verdict": (
            "SUPPORTED" if (class_b == "SUPPORTED" or class_c == "SUPPORTED") else (
                "PARTIALLY_SUPPORTED" if (class_b == "PARTIALLY_SUPPORTED" or class_c == "PARTIALLY_SUPPORTED") else
                "SUPPORT_PLANARIZATION_FAILED"
            )
        )
    }

    derived_dir = pathlib.Path("data/derived")
    with open(derived_dir / "m25_support_path_results.json", "w") as f:
        json.dump(results_payload, f, indent=2)
    with open(derived_dir / "m25_support_path_manifest.json", "w") as f:
        json.dump(manifest_payload, f, indent=2)

    print("\n" + "=" * 80)
    print("MW-M25 EXECUTION COMPLETE")
    print(f"Results written to: {derived_dir / 'm25_support_path_results.json'}")
    print(f"Manifest written to: {derived_dir / 'm25_support_path_manifest.json'}")
    print("=" * 80)


if __name__ == "__main__":
    main()
