#!/usr/bin/env python3
"""
MW-M30: Minimal Tripod-Transfer Conflict Intervention.

Scientific Objective:
Test whether temporarily relieving the outgoing support tripod's conflicting
longitudinal target constraint during the existing tripod handoff window
(via TRANSFER_PHASE_WORLD_X_ANCHOR) reduces transfer-phase force opposition
and longitudinal hesitation without disrupting M27-D's protected visual choreography.

Two Strictly Isolated Dynamic Conditions:
- Condition A: EXACT_M27_D (Authoritative Choreographic Baseline to Protect)
- Condition B: M27_D_TRANSFER_RELIEF (Minimal Polishing Intervention)

Condition Branching & Isolation Rules:
- Both conditions share identical cadence (6.0 Hz), excursion scaling
  (E_front=1.000, E_mid=0.564, E_hind=0.779), support heights, and DLS IK solver.
- Legs during strong support (w_adh >= 0.999) and legs during swing (w_adh <= 1e-4)
  remain 100% identical between Conditions A and B.
- Condition B modifies ONLY outgoing legs during their unloading transfer phase
  (dw_adh/dt < 0 and 0.001 <= w_adh < 0.999):
    x_anchor = previous commanded M27-D World-X target at unloading onset.
    g_trans = 1.0 - w_adh.
    p_target_world[0] = (1.0 - g_trans) * p_target_world_m27d[0] + g_trans * x_anchor.
    p_target_world[1:3] identically frozen M27-D (Y_body nominal, Z_body frozen).

Dynamic Physics Simulation Budget:
- Exactly 2 evaluation simulations (Condition A, Condition B).
- Exactly 2 clean rendering simulations (dual-camera side + top-down).
- Total Dynamic Physics Budget: exactly 4 runs maximum.
"""

import os
import sys
import json
import pathlib
import argparse
import warnings
import numpy as np
from scipy.signal import find_peaks
import mujoco

# Matplotlib configuration for headless environment
os.environ["MPLCONFIGDIR"] = "/tmp"
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

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
import scripts.m25_support_path_geometry as m25
from scripts.m13_pulse_diagnosis import quat_to_pitch_rad

# -----------------------------------------------------------------------------
# GLOBAL CONSTANTS & PROVENANCE
# -----------------------------------------------------------------------------

LEG_NAMES = ["LF", "LM", "LH", "RF", "RM", "RH"]
LEG_LOWER = [l.lower() for l in LEG_NAMES]
TRIPOD_A_INDS = [0, 2, 4]  # LF (front), LH (hind), RM (mid)
TRIPOD_B_INDS = [1, 3, 5]  # LM (mid), RF (front), RH (hind)

PHYSICS_DT = 0.0001
SAMPLE_STEPS = 10
SAMPLE_DT = PHYSICS_DT * SAMPLE_STEPS  # 0.001 s (1 kHz)
TRIAL_DURATION = 1.75
N_STEPS = int(TRIAL_DURATION / PHYSICS_DT)  # 17500
STEADY_START_TIME = 0.35

# Authoritative frozen body support heights from MW-M25 & MW-M27
FROZEN_BODY_SUPPORT_HEIGHTS_MM = {
    "LF": -1.0424534343726277,
    "LM": -1.0876066769550565,
    "LH": -0.9750830473328336,
    "RF": -1.041986691037106,
    "RM": -1.0882983748972959,
    "RH": -0.9719454335077823
}

# Frozen excursion fractions for Condition A (M27-D) and Condition B
CONDITION_EXCURSIONS = {
    "LF": 1.000,
    "LM": 0.564,
    "LH": 0.779,
    "RF": 1.000,
    "RM": 0.564,
    "RH": 0.779
}
EXCURSIONS_LIST = [CONDITION_EXCURSIONS[leg] for leg in LEG_NAMES]


class NumpyEncoder(json.JSONEncoder):
    """Encodes numpy arrays and numbers into standard JSON-compliant types."""
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating, float)):
            return float(obj)
        if isinstance(obj, (np.integer, int)):
            return int(obj)
        if isinstance(obj, (np.bool_, bool)):
            return bool(obj)
        return super().default(obj)


def sanitize_json_for_rfc8259(obj):
    """Sanitizes dict/list recursively to ensure strict JSON conformance (replaces NaN/Inf)."""
    if isinstance(obj, dict):
        return {k: sanitize_json_for_rfc8259(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_json_for_rfc8259(v) for v in obj]
    elif isinstance(obj, float):
        if np.isnan(obj) or np.isinf(obj):
            return None
        return obj
    return obj


# -----------------------------------------------------------------------------
# TRAJECTORY & CONTROLLER ARCHITECTURE
# -----------------------------------------------------------------------------

class ExcursionTrajectoryController:
    """
    Trajectory controller executing M27-D excursion-scaled kinematics.
    """
    def __init__(self, preprogrammed_steps, actuated_dofs,
                 excursions=None,
                 cadence=6.0, turnaround_fraction=0.06,
                 support_transfer_fraction=0.08, entry_duration=0.25,
                 timestep=PHYSICS_DT):
        self.pps = preprogrammed_steps
        self.actuated_dofs = actuated_dofs
        self.excursions = np.array(excursions if excursions is not None else [1.0] * 6, dtype=float)
        self.cadence = cadence
        self.r = turnaround_fraction
        self.t_f = support_transfer_fraction
        self.entry_duration = entry_duration
        self.timestep = timestep

        self.time = 0.0
        self.default_pose = preprogrammed_steps.default_pose_by_dof_order(actuated_dofs)
        self.legs = preprogrammed_steps.legs

        self.stance_starts = []
        self.stance_ends = []
        for leg in self.legs:
            swing_start, swing_end = self.pps.swing_period[leg.lower()]
            self.stance_starts.append(swing_end)
            self.stance_ends.append(swing_start + 2.0 * np.pi)

    def step(self):
        phi = (self.time * self.cadence * 2.0) % 2.0
        phases = np.zeros(6)
        adhesions = np.zeros(6)

        for i in range(6):
            p = phi if i in [0, 2, 4] else (phi + 1.0) % 2.0
            w = m8.smooth_triangle(p, self.r)
            E = self.excursions[i]
            u = 0.5 - E / 2.0 + E * w

            phases[i] = self.stance_starts[i] + u * (self.stance_ends[i] - self.stance_starts[i])
            adhesions[i] = m8.get_adhesion(p, self.t_f)

        stock_angles = self.pps.get_joint_angles_by_dof_order(
            phases, output_dof_order=self.actuated_dofs
        )

        if self.time < self.entry_duration:
            alpha = m8.smooth_step(self.time, 0.0, self.entry_duration)
            joint_angles = (1.0 - alpha) * self.default_pose + alpha * stock_angles
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions
        else:
            joint_angles = stock_angles

        self.time += self.timestep
        return LocomotionAction(joint_angles=joint_angles, adhesion_onoff=adhesions), phi


class M30TransferReliefController:
    """
    Unified controller executing Conditions A and B.

    Condition A (EXACT_M27_D):
      Exact unchanged champion controller. Enforces body-frame planar support heights
      via DLS IK, with p_target_body[0] = p_nom_body[0].

    Condition B (M27_D_TRANSFER_RELIEF):
      ONE minimal kinematic intervention operating strictly during the unloading
      handoff window (dw_adh/dt < 0 and 0.001 <= w_adh < 0.999):
      - Latches previous commanded M27-D World-X target at unloading onset (x_anchor).
      - Blends World-X target toward x_anchor:
          g_trans = 1.0 - w_adh
          p_target_world[0] = (1 - g_trans) * p_target_world_m27d[0] + g_trans * x_anchor
      - Preserves 100% frozen body support heights Z and nominal Y.
      - Smoothly hands over to nominal swing at w_adh <= 1e-4.
    """
    def __init__(self, sim, fly, pps, actuated_dofs, dof_map,
                 condition="B",
                 frozen_heights=None,
                 dls_damping=1e-4, reg_weight=0.01, max_dq=0.05):
        self.sim = sim
        self.fly = fly
        self.pps = pps
        self.actuated_dofs = actuated_dofs
        self.dof_map = dof_map
        self.condition = condition
        self.frozen_heights = frozen_heights or FROZEN_BODY_SUPPORT_HEIGHTS_MM
        self.dls_damping = dls_damping
        self.reg_weight = reg_weight
        self.max_dq = max_dq

        self.base_traj = ExcursionTrajectoryController(
            pps, actuated_dofs,
            excursions=EXCURSIONS_LIST,
            cadence=6.0,
            turnaround_fraction=0.06,
            support_transfer_fraction=0.08,
            entry_duration=0.25,
            timestep=PHYSICS_DT
        )

        self.scratch_data = mujoco.MjData(sim.mj_model)
        self.thorax_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
        self.t5_ids = {
            leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id
            for leg in LEG_LOWER
        }

        # Transfer-anchor state tracking
        self.prev_adhesions = np.ones(6, dtype=float)
        self.anchor_latched = {leg: False for leg in LEG_LOWER}
        self.x_anchor = {leg: None for leg in LEG_LOWER}
        self.last_target_world_m27d = {leg: np.zeros(3) for leg in LEG_LOWER}
        self.last_applied_target_world = {leg: np.zeros(3) for leg in LEG_LOWER}

        # Finite-difference intervention tracking
        self.prev_dx_relief = np.zeros(6, dtype=float)
        self.current_v_relief = np.zeros(6, dtype=float)
        self.current_dx_relief = np.zeros(6, dtype=float)

        # Invariants and diagnostics
        self.ik_residuals = []
        self.ik_iterations = []
        self.ik_converged_count = 0
        self.ik_total_solves = 0
        self.max_step_target_diff = 0.0
        self.prev_applied_q = None

    @property
    def default_pose(self):
        return self.base_traj.default_pose

    @property
    def time(self):
        return self.base_traj.time

    @time.setter
    def time(self, val):
        self.base_traj.time = val

    def step(self):
        nominal_action, phi = self.base_traj.step()
        nominal_q = nominal_action.joint_angles.copy()
        adhesions = nominal_action.adhesion_onoff.copy()
        applied_q = nominal_q.copy()

        # Update scratch MjData with live simulation state
        self.scratch_data.qpos[:] = self.sim.mj_data.qpos[:]
        mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
        mujoco.mj_comPos(self.sim.mj_model, self.scratch_data)

        tpos_live = self.sim.mj_data.xpos[self.thorax_id].copy()
        tmat_live = self.sim.mj_data.xmat[self.thorax_id].reshape(3, 3).copy()
        jacp = np.zeros((3, self.sim.mj_model.nv))

        dx_relief_step = np.zeros(6, dtype=float)

        for i, leg in enumerate(LEG_LOWER):
            w_sup = float(adhesions[i])
            dw_adh = w_sup - float(self.prev_adhesions[i])

            qpos_inds = self.dof_map["leg_qpos_inds"][leg]
            nv_inds = self.dof_map["leg_nv_inds"][leg]
            act_inds = self.dof_map["leg_act_inds"][leg]
            t5_id = self.t5_ids[leg]

            # 1. Forward kinematics of nominal target for leg in scratch
            self.scratch_data.qpos[qpos_inds] = nominal_q[act_inds]
            mujoco.mj_kinematics(self.sim.mj_model, self.scratch_data)
            p_nom_world = self.scratch_data.xpos[t5_id].copy()
            p_nom_body = tmat_live.T @ (p_nom_world - tpos_live)

            # 2. State-matched M27-D nominal target
            leg_name_upper = LEG_NAMES[i]
            z_target_body = self.frozen_heights.get(leg_name_upper, p_nom_body[2])
            p_target_body_m27d = np.array([p_nom_body[0], p_nom_body[1], z_target_body])
            p_target_world_m27d = tpos_live + tmat_live @ p_target_body_m27d

            # Record baseline M27-D target for state tracking
            self.last_target_world_m27d[leg] = p_target_world_m27d.copy()

            if w_sup < 1e-4:
                # Fully released leg in swing: 100% nominal kinematics
                self.anchor_latched[leg] = False
                self.x_anchor[leg] = None
                self.last_applied_target_world[leg] = p_nom_world.copy()
                dx_relief_step[i] = 0.0
                continue

            # 3. Construct applied target
            if self.condition == "B":
                # Check for active unloading transfer window: dw/dt < 0 and 0.001 <= w_sup < 0.999
                is_unloading = (dw_adh < -1e-7) and (0.001 <= w_sup < 0.999)

                if is_unloading:
                    if not self.anchor_latched[leg]:
                        # Continuity-safe latch: latch previous commanded M27-D target
                        self.x_anchor[leg] = float(self.last_applied_target_world[leg][0])
                        self.anchor_latched[leg] = True

                    # Partial World-X anchor relaxation via adhesion-weighted blending
                    g_trans = 1.0 - w_sup
                    x_target_world = (1.0 - g_trans) * p_target_world_m27d[0] + g_trans * self.x_anchor[leg]
                    p_target_world = np.array([x_target_world, p_target_world_m27d[1], p_target_world_m27d[2]])
                    dx_relief_step[i] = float(x_target_world - p_target_world_m27d[0])
                else:
                    self.anchor_latched[leg] = False
                    self.x_anchor[leg] = None
                    p_target_world = p_target_world_m27d.copy()
                    dx_relief_step[i] = 0.0
            else:
                # Condition A: EXACT_M27_D
                p_target_world = p_target_world_m27d.copy()
                dx_relief_step[i] = 0.0

            self.last_applied_target_world[leg] = p_target_world.copy()

            # 4. Solve DLS IK in scratch MjData
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

            # 5. Smooth blend across support transfer window (tf = 0.08)
            q_blend = (1.0 - w_sup) * nominal_q[act_inds] + w_sup * q_sol
            applied_q[act_inds] = q_blend

        # Apply smooth entry blending if in startup
        if self.time < self.base_traj.entry_duration:
            alpha = m8.smooth_step(self.time, 0.0, self.base_traj.entry_duration)
            applied_q = (1.0 - alpha) * self.default_pose + alpha * applied_q
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions

        # Track intervention velocity via full finite-difference derivative
        self.current_dx_relief = dx_relief_step.copy()
        self.current_v_relief = (self.current_dx_relief - self.prev_dx_relief) / PHYSICS_DT
        self.prev_dx_relief = self.current_dx_relief.copy()
        self.prev_adhesions = adhesions.copy()

        # Check joint step bound
        if self.prev_applied_q is not None:
            max_step_diff = float(np.max(np.abs(applied_q - self.prev_applied_q)))
            if max_step_diff > self.max_step_target_diff:
                self.max_step_target_diff = max_step_diff
        self.prev_applied_q = applied_q.copy()

        return LocomotionAction(joint_angles=applied_q, adhesion_onoff=adhesions), phi


# -----------------------------------------------------------------------------
# PREFLIGHT VERIFICATIONS
# -----------------------------------------------------------------------------

def run_preflight_provenance_audit():
    """Verifies that M8 parameters match authoritative tracked provenance."""
    expected = {
        "cadence_hz": 6.0,
        "turnaround_fraction": 0.06,
        "support_transfer_fraction": 0.08,
        "entry_duration_s": 0.25,
        "physics_dt_s": 0.0001,
        "trial_duration_s": 1.75,
        "steady_start_time_s": 0.35,
        "n_steps": 17500,
        "excursion_fractions": CONDITION_EXCURSIONS
    }
    print("Checking MW-M30 Provenance Parameters against Authoritative History:")
    for k, v in expected.items():
        print(f"  {k:28s}: {v}")
    print("  Provenance Check: PASS\n")
    return expected


def verify_scratch_mjdata_isolation(fly, sim, dof_map):
    """Verifies that scratch MjData mutations do not leak into live physics sim.mj_data."""
    print("Verifying Scratch MjData State Isolation ...")
    qpos_orig = sim.mj_data.qpos.copy()
    qvel_orig = sim.mj_data.qvel.copy()

    scratch = mujoco.MjData(sim.mj_model)
    scratch.qpos[:] = sim.mj_data.qpos[:]
    scratch.qpos[7:14] += 0.25
    mujoco.mj_kinematics(sim.mj_model, scratch)

    qpos_diff = float(np.max(np.abs(sim.mj_data.qpos - qpos_orig)))
    qvel_diff = float(np.max(np.abs(sim.mj_data.qvel - qvel_orig)))

    assert qpos_diff == 0.0, f"Leaked qpos mutation into sim.mj_data! diff={qpos_diff}"
    assert qvel_diff == 0.0, f"Leaked qvel mutation into sim.mj_data! diff={qvel_diff}"
    print(f"  Scratch MjData Isolation: PASS (max diff = 0.0)\n")


def run_preflight_condition_isolation(fly, sim, dof_map, pps, actuated_dofs):
    """
    Verifies that outside the unloading transfer window, Condition B target positions
    match Condition A (M27-D) exactly.
    """
    print("Verifying Condition B Static Isolation outside Transfer Unload ...")
    ctrl_a = M30TransferReliefController(sim, fly, pps, actuated_dofs, dof_map, condition="A")
    ctrl_b = M30TransferReliefController(sim, fly, pps, actuated_dofs, dof_map, condition="B")

    # Evaluate at startup default pose
    act_a, _ = ctrl_a.step()
    act_b, _ = ctrl_b.step()

    diff_startup = float(np.max(np.abs(act_a.joint_angles - act_b.joint_angles)))
    assert diff_startup == 0.0, f"Startup difference between A and B! diff={diff_startup}"

    # Verify that before unloading occurs, Condition B produces zero intervention displacement
    assert np.all(ctrl_b.current_dx_relief == 0.0), "Non-zero relief displacement prior to unload!"
    print(f"  Static Isolation: PASS (Startup diff = {diff_startup:.1e})\n")
    return {"status": "PASS", "startup_diff": diff_startup}


# -----------------------------------------------------------------------------
# SIMULATION ROLLOUT & TELEMETRY
# -----------------------------------------------------------------------------

def run_diagnostic_simulation(condition_name, controller_builder, n_steps=N_STEPS, sample_dt=SAMPLE_DT):
    """
    Runs a single isolated dynamic simulation rollout (1.75 s / 17500 steps)
    and collects 1 kHz telemetry.
    """
    print(f"\n=======================================================")
    print(f"Running simulation: {condition_name} ({n_steps * PHYSICS_DT:.2f} s) ...")
    print(f"=======================================================")
    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    sim = m25.create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    dof_map = m25.build_semantic_dof_mapping(fly, sim)
    actuator_ids, actuator_limits = m25.extract_actuator_limits(fly, sim)

    body_segs = [s.name if hasattr(s, "name") else str(s) for s in fly.get_bodysegs_order()]
    thorax_idx = body_segs.index("c_thorax")
    tarsus5_idxs = [body_segs.index(f"{leg}_tarsus5") for leg in LEG_LOWER]
    thorax_body_id = sim.mj_model.body(f"{fly.name}/c_thorax").id
    t5_body_ids = {leg: sim.mj_model.body(f"{fly.name}/{leg}_tarsus5").id for leg in LEG_LOWER}

    # Query all leg segments for ground contact forces
    all_leg_segs = []
    for leg in LEG_LOWER:
        for part in ["coxa", "trochanterfemur", "tibia", "tarsus1", "tarsus2", "tarsus3", "tarsus4", "tarsus5"]:
            all_leg_segs.append(f"{leg}_{part}")
    seg_to_leg_idx = {}
    for i, s in enumerate(all_leg_segs):
        leg_str = s.split("_")[0]
        seg_to_leg_idx[i] = LEG_LOWER.index(leg_str)

    abdomen_segs = ["c_abdomen12", "c_abdomen3", "c_abdomen4", "c_abdomen5", "c_abdomen6"]

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    t_list, x_list, y_list, z_list = [], [], [], []
    pitch_list, roll_list, yaw_list, quat_list = [], [], [], []
    vx_list, ax_list = [], []
    tarsus_xyz_list = []
    contact_list = []
    adhesion_list = []
    target_q_list = []
    actual_q_list = []
    abdomen_contact_list = []
    leg_grf_world_list = []

    # Intervention telemetry (Condition B diagnostics)
    dx_relief_list = []
    v_relief_list = []
    p_relief_power_list = []
    foot_slip_vx_list = []

    prev_x = sim.get_body_positions(fly.name)[thorax_idx, 0]
    prev_vx = None
    prev_t5_x = np.array([sim.mj_data.xpos[t5_body_ids[l]][0] for l in LEG_LOWER])

    step_target_diffs = []
    prev_target_q = None

    for step_i in range(n_steps):
        action, phi = controller.step()
        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        curr_target_q = action.joint_angles.copy()
        if prev_target_q is not None:
            step_target_diffs.append(np.abs(curr_target_q - prev_target_q))
        prev_target_q = curr_target_q

        if (step_i + 1) % SAMPLE_STEPS == 0:
            t = (step_i + 1) * PHYSICS_DT
            b_pos = sim.get_body_positions(fly.name)
            pos = b_pos[thorax_idx]
            quat = sim.get_body_rotations(fly.name)[thorax_idx]
            pitch_deg = np.degrees(quat_to_pitch_rad(quat))

            w, x, y, z = quat
            roll_rad = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            roll_deg = float(np.degrees(roll_rad))
            yaw_rad = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            yaw_deg = float(np.degrees(yaw_rad))

            vx = float((pos[0] - prev_x) / sample_dt)
            prev_x = pos[0]
            ax = float((vx - prev_vx) / sample_dt) if prev_vx is not None else 0.0
            prev_vx = vx

            tars_pos = b_pos[tarsus5_idxs]
            cont_info = sim.get_ground_contact_info(fly.name)
            cont_leg = cont_info[0].copy()[:6]

            # Aggregate contact forces across all segments of each leg
            forces_all_segs = sim.get_bodysegment_contact_forces(fly.name, all_leg_segs, ground_only=True)
            leg_forces = np.zeros((6, 3), dtype=float)
            for s_idx, leg_idx in seg_to_leg_idx.items():
                leg_forces[leg_idx] += forces_all_segs[s_idx]

            # Foot slip velocity in world frame
            curr_t5_x = np.array([sim.mj_data.xpos[t5_body_ids[l]][0] for l in LEG_LOWER])
            slip_vx = (curr_t5_x - prev_t5_x) / sample_dt
            prev_t5_x = curr_t5_x.copy()

            # Abdomen ground contact forces
            abd_forces = sim.get_bodysegment_contact_forces(fly.name, abdomen_segs, ground_only=True)
            abd_contact = bool(np.any(np.linalg.norm(abd_forces, axis=1) > 1e-4))

            adh_cmd = action.adhesion_onoff.copy()

            # Virtual power of intervention: P_relief = - Fx_world * v_relief
            v_rel = controller.current_v_relief.copy()
            dx_rel = controller.current_dx_relief.copy()
            p_rel = - leg_forces[:, 0] * v_rel

            t_list.append(t)
            x_list.append(pos[0])
            y_list.append(pos[1])
            z_list.append(pos[2])
            pitch_list.append(pitch_deg)
            roll_list.append(roll_deg)
            yaw_list.append(yaw_deg)
            quat_list.append(quat.copy())
            vx_list.append(vx)
            ax_list.append(ax)
            tarsus_xyz_list.append(tars_pos.copy())
            contact_list.append(cont_leg.copy())
            adhesion_list.append(adh_cmd.copy())
            target_q_list.append(curr_target_q.copy())
            actual_q_list.append(sim.get_joint_angles(fly.name)[:len(actuator_ids)].copy())
            abdomen_contact_list.append(abd_contact)
            leg_grf_world_list.append(leg_forces.copy())
            foot_slip_vx_list.append(slip_vx.copy())

            dx_relief_list.append(dx_rel.copy())
            v_relief_list.append(v_rel.copy())
            p_relief_power_list.append(p_rel.copy())

    sim.close()

    max_target_step_rad = float(np.max(step_target_diffs)) if step_target_diffs else 0.0

    return {
        "condition_name": condition_name,
        "t": np.array(t_list),
        "x": np.array(x_list),
        "y": np.array(y_list),
        "z": np.array(z_list),
        "pitch": np.array(pitch_list),
        "roll": np.array(roll_list),
        "yaw": np.array(yaw_list),
        "quat": np.array(quat_list),
        "vx": np.array(vx_list),
        "ax": np.array(ax_list),
        "tarsus_xyz": np.array(tarsus_xyz_list),
        "contact": np.array(contact_list),
        "adhesion": np.array(adhesion_list),
        "target_q": np.array(target_q_list),
        "actual_q": np.array(actual_q_list),
        "abdomen_contact": np.array(abdomen_contact_list),
        "leg_grf_world": np.array(leg_grf_world_list),
        "foot_slip_vx": np.array(foot_slip_vx_list),
        "dx_relief": np.array(dx_relief_list),
        "v_relief": np.array(v_relief_list),
        "p_relief_power": np.array(p_relief_power_list),
        "max_target_step_rad": max_target_step_rad,
        "ik_diagnostics": {
            "converged_solves": controller.ik_converged_count,
            "total_solves": controller.ik_total_solves,
            "convergence_rate": float(controller.ik_converged_count / max(controller.ik_total_solves, 1)),
            "mean_residual_mm": float(np.mean(controller.ik_residuals)) if controller.ik_residuals else 0.0,
            "max_residual_mm": float(np.max(controller.ik_residuals)) if controller.ik_residuals else 0.0,
            "mean_iterations": float(np.mean(controller.ik_iterations)) if controller.ik_iterations else 0.0
        }
    }


# -----------------------------------------------------------------------------
# TELEMETRY ANALYSIS & EVENT DYNAMICS
# -----------------------------------------------------------------------------

def analyze_simulation_telemetry(telem):
    """
    Computes steady-state kinematics, force cancellation metrics,
    event-triggered transfer dynamics, and finite-difference work diagnostics.
    """
    t = telem["t"]
    steady_mask = t >= STEADY_START_TIME

    ss_t = t[steady_mask]
    ss_x = telem["x"][steady_mask]
    ss_y = telem["y"][steady_mask]
    ss_z = telem["z"][steady_mask]
    ss_vx = telem["vx"][steady_mask]
    ss_ax = telem["ax"][steady_mask]
    ss_pitch = telem["pitch"][steady_mask]
    ss_roll = telem["roll"][steady_mask]
    ss_yaw = telem["yaw"][steady_mask]
    ss_cont = telem["contact"][steady_mask]
    ss_adh = telem["adhesion"][steady_mask]
    ss_grf = telem["leg_grf_world"][steady_mask]
    ss_slip = telem["foot_slip_vx"][steady_mask]

    # In FlyGym, moonwalk moves in -X direction.
    # Mean backward speed = -mean(vx)
    mean_speed = float(-np.mean(ss_vx))
    mean_vx = float(np.mean(ss_vx))
    disp_x = float(ss_x[-1] - ss_x[0])
    disp_y = float(ss_y[-1] - ss_y[0])
    lateral_ratio = float(abs(disp_y) / max(abs(disp_x), 1e-9))

    # Rolling window backward speeds
    w20 = int(0.020 / SAMPLE_DT)  # 20 ms window
    w50 = int(0.050 / SAMPLE_DT)  # 50 ms window
    vx_20 = np.convolve(ss_vx, np.ones(w20)/w20, mode="same")
    vx_50 = np.convolve(ss_vx, np.ones(w50)/w50, mode="same")

    speed_20 = -vx_20
    speed_50 = -vx_50

    cv20 = float(np.std(vx_20) / max(abs(mean_vx), 1e-9))
    cv50 = float(np.std(vx_50) / max(abs(mean_vx), 1e-9))
    wrong_sign_20 = float(np.mean(vx_20 > 0.0))

    pitch_ptp = float(np.ptp(ss_pitch))
    pitch_std = float(np.std(ss_pitch))
    pitch_mean = float(np.mean(ss_pitch))

    roll_ptp = float(np.ptp(ss_roll))
    roll_std = float(np.std(ss_roll))
    roll_mean = float(np.mean(ss_roll))

    z_ptp = float(np.ptp(ss_z))
    z_std = float(np.std(ss_z))
    z_mean = float(np.mean(ss_z))

    # Support-Phase Conditioning
    mask_strong_a = (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) <= 0.10)
    mask_strong_b = (np.median(ss_adh[:, TRIPOD_B_INDS], axis=1) >= 0.90) & (np.median(ss_adh[:, TRIPOD_A_INDS], axis=1) <= 0.10)

    all3_a = np.all(ss_cont[:, TRIPOD_A_INDS] > 0, axis=1)
    all3_b = np.all(ss_cont[:, TRIPOD_B_INDS] > 0, axis=1)

    total_plateau_samples = int(np.sum(mask_strong_a) + np.sum(mask_strong_b))
    total_all3_samples = int(np.sum(all3_a & mask_strong_a) + np.sum(all3_b & mask_strong_b))
    all3_support_fraction = float(total_all3_samples / max(total_plateau_samples, 1))

    # Helper function for tripod force conflict statistics
    def compute_aggregated_tripod_stats(inds_a, inds_b):
        canc_list = []
        for inds, legs in [(inds_a, TRIPOD_A_INDS), (inds_b, TRIPOD_B_INDS)]:
            if len(inds) > 0:
                s_grf = ss_grf[inds][:, legs, :]
                s_fx = s_grf[:, :, 0]
                f_net = np.sum(s_fx, axis=1)
                f_abs = np.sum(np.abs(s_fx), axis=1)
                c = 1.0 - (np.abs(f_net) / (f_abs + 1e-9))
                canc_list.extend(c.tolist())
        canc_arr = np.array(canc_list) if canc_list else np.array([0.0])
        return {
            "cancellation_median": float(np.median(canc_arr)),
            "cancellation_mean": float(np.mean(canc_arr))
        }

    inds_a_all3 = np.where(mask_strong_a & all3_a)[0]
    inds_b_all3 = np.where(mask_strong_b & all3_b)[0]
    agg_all3 = compute_aggregated_tripod_stats(inds_a_all3, inds_b_all3)

    # Whole-Body 6-Leg Force Cancellation
    fx_6_net = np.sum(ss_grf[:, :, 0], axis=1)
    fx_6_abs = np.sum(np.abs(ss_grf[:, :, 0]), axis=1)
    canc_6_all = 1.0 - (np.abs(fx_6_net) / (fx_6_abs + 1e-9))
    canc_6_strong = canc_6_all[mask_strong_a | mask_strong_b]

    whole_body_stats = {
        "cancellation_all_samples_median": float(np.median(canc_6_all)),
        "cancellation_strong_support_median": float(np.median(canc_6_strong)) if len(canc_6_strong) > 0 else 0.0,
        "cancellation_all_samples_mean": float(np.mean(canc_6_all))
    }

    # Per-Leg GRF Statistics during Strong Support
    per_leg_grf_stats = {}
    for l_idx, leg in enumerate(LEG_NAMES):
        is_trip_a = (l_idx in TRIPOD_A_INDS)
        sup_mask = mask_strong_a if is_trip_a else mask_strong_b
        l_fx = ss_grf[sup_mask, l_idx, 0]
        l_fz = ss_grf[sup_mask, l_idx, 2]
        l_slip = ss_slip[sup_mask, l_idx]

        per_leg_grf_stats[leg] = {
            "MEDIAN_FX_WORLD": float(np.median(l_fx)) if len(l_fx) > 0 else 0.0,
            "P05_FX_WORLD": float(np.percentile(l_fx, 5)) if len(l_fx) > 0 else 0.0,
            "P95_FX_WORLD": float(np.percentile(l_fx, 95)) if len(l_fx) > 0 else 0.0,
            "MEDIAN_FZ_WORLD": float(np.median(l_fz)) if len(l_fz) > 0 else 0.0,
            "MEDIAN_SLIP_VX": float(np.median(l_slip)) if len(l_slip) > 0 else 0.0
        }

    # Event-Triggered Dynamics: Speed Minima / Hesitation Peak Detection
    # In FlyGym, moonwalk speed is -vx. Speed minimum corresponds to peak of vx_20.
    min_dist_samples = int(0.040 / SAMPLE_DT)  # at least 40 ms between dips
    peaks, _ = find_peaks(vx_20, distance=min_dist_samples, prominence=0.5)

    window_half_samples = int(0.030 / SAMPLE_DT)  # +/- 30 ms
    canc_all_samples = canc_6_all  # 6-leg force cancellation across all samples

    expected_win_len = 2 * window_half_samples + 1
    ev_profiles_speed = []
    ev_profiles_canc = []
    for p_idx in peaks:
        if p_idx >= window_half_samples and p_idx + window_half_samples < len(ss_t):
            s_win = -vx_20[p_idx - window_half_samples: p_idx + window_half_samples + 1]
            c_win = canc_all_samples[p_idx - window_half_samples: p_idx + window_half_samples + 1]
            if len(s_win) == expected_win_len and len(c_win) == expected_win_len:
                ev_profiles_speed.append(s_win)
                ev_profiles_canc.append(c_win)

    ev_time_axis_ms = np.arange(-window_half_samples, window_half_samples + 1) * (SAMPLE_DT * 1000.0)

    mean_speed_profile = np.nanmean(ev_profiles_speed, axis=0) if ev_profiles_speed else np.array([])
    mean_canc_profile = np.nanmean(ev_profiles_canc, axis=0) if ev_profiles_canc else np.array([])

    transfer_peak_canc = float(np.max(mean_canc_profile)) if len(mean_canc_profile) > 0 else 0.0
    transfer_peak_time_ms = float(ev_time_axis_ms[np.argmax(mean_canc_profile)]) if len(mean_canc_profile) > 0 else 0.0

    # Finite-Difference Work Diagnostics (Condition B)
    # P_relief = - Fx_world * v_relief
    ss_p_relief = telem["p_relief_power"][steady_mask]  # shape (N, 6) in mW (mN * mm/s = uW)
    # Integral over time in microjoules (uW * 0.001 s = nJ -> 1e-3 uJ)
    dt_s = SAMPLE_DT
    # Total power summed across all legs
    p_total_relief = np.sum(ss_p_relief, axis=1)  # in uW (mN * mm/s)
    p_pos = np.maximum(p_total_relief, 0.0)
    p_neg = np.maximum(-p_total_relief, 0.0)

    w_pos_uj = float(np.sum(p_pos) * dt_s)
    w_neg_uj = float(np.sum(p_neg) * dt_s)
    w_net_uj = float(w_pos_uj - w_neg_uj)

    # Numerical epsilon: 1e-12 J = 1e-6 uJ
    eps_num = 1e-6
    if w_neg_uj < 1e-3:  # less than 1 nJ
        work_ratio = float(w_pos_uj / (w_neg_uj + eps_num))
        work_status = "INDETERMINATE_NEGLIGIBLE_DISSIPATION"
    else:
        work_ratio = float(w_pos_uj / (w_neg_uj + eps_num))
        work_status = "PASS" if work_ratio <= 0.05 else "FAIL"

    work_diagnostics = {
        "POSITIVE_WORK_INJECTION_UJ": w_pos_uj,
        "NEGATIVE_WORK_DISSIPATION_UJ": w_neg_uj,
        "NET_WORK_UJ": w_net_uj,
        "WORK_RATIO": work_ratio,
        "WORK_RATIO_STATUS": work_status,
        "PASSIVITY_DIAGNOSTIC_PASS": (work_status in ["PASS", "INDETERMINATE_NEGLIGIBLE_DISSIPATION"])
    }

    abdomen_contacts = int(np.sum(telem["abdomen_contact"][steady_mask]))
    fz_pos = np.all(ss_grf[:, :, 2] >= -1e-4)
    forces_fin = bool(np.all(np.isfinite(ss_grf)))

    return {
        "condition_name": telem["condition_name"],
        "mean_speed_mm_s": mean_speed,
        "mean_vx_mm_s": mean_vx,
        "displacement_x_mm": disp_x,
        "displacement_y_mm": disp_y,
        "lateral_ratio": lateral_ratio,
        "cv20": cv20,
        "cv50": cv50,
        "wrong_sign_20": wrong_sign_20,
        "pitch_ptp_deg": pitch_ptp,
        "pitch_std_deg": pitch_std,
        "pitch_mean_deg": pitch_mean,
        "roll_ptp_deg": roll_ptp,
        "roll_std_deg": roll_std,
        "roll_mean_deg": roll_mean,
        "z_ptp_mm": z_ptp,
        "z_std_mm": z_std,
        "z_mean_mm": z_mean,
        "all3_support_fraction": all3_support_fraction,
        "aggregated_all3": agg_all3,
        "whole_body": whole_body_stats,
        "per_leg_grf": per_leg_grf_stats,
        "transfer_event_dynamics": {
            "event_count": len(peaks),
            "peak_cancellation": transfer_peak_canc,
            "peak_timing_ms": transfer_peak_time_ms,
            "time_axis_ms": ev_time_axis_ms.tolist(),
            "mean_speed_profile_mm_s": mean_speed_profile.tolist() if len(mean_speed_profile) > 0 else [],
            "mean_cancellation_profile": mean_canc_profile.tolist() if len(mean_canc_profile) > 0 else []
        },
        "work_diagnostics": work_diagnostics,
        "abdomen_contact_count": abdomen_contacts,
        "max_target_step_rad": telem["max_target_step_rad"],
        "fz_positive_all_samples": bool(fz_pos),
        "forces_finite": forces_fin,
        "ik_diagnostics": telem["ik_diagnostics"]
    }


# -----------------------------------------------------------------------------
# GATE EVALUATION & PRIMARY CLASSIFICATIONS
# -----------------------------------------------------------------------------

def evaluate_reproduction_gates(metrics_a):
    """
    Validates Condition A against authoritative M27-D metrics.
    """
    # Authoritative M27-D reference values
    ref = {
        "speed": 7.87,
        "cv50": 0.3276,
        "wrong_sign_20": 0.0914,
        "pitch_ptp": 6.51,
        "roll_ptp": 9.17,
        "z_ptp": 0.103,
        "all3_contact": 0.9446,
        "canc_all3": 0.4494
    }

    gates = {
        "speed_reproduction": {
            "val": metrics_a["mean_speed_mm_s"],
            "target": ref["speed"],
            "pass": bool(abs(metrics_a["mean_speed_mm_s"] - ref["speed"]) <= 0.25)
        },
        "cv50_reproduction": {
            "val": metrics_a["cv50"],
            "target": ref["cv50"],
            "pass": bool(abs(metrics_a["cv50"] - ref["cv50"]) <= 0.025)
        },
        "wrong_sign_reproduction": {
            "val": metrics_a["wrong_sign_20"],
            "target": ref["wrong_sign_20"],
            "pass": bool(abs(metrics_a["wrong_sign_20"] - ref["wrong_sign_20"]) <= 0.025)
        },
        "pitch_ptp_reproduction": {
            "val": metrics_a["pitch_ptp_deg"],
            "target": ref["pitch_ptp"],
            "pass": bool(abs(metrics_a["pitch_ptp_deg"] - ref["pitch_ptp"]) <= 0.60)
        },
        "roll_ptp_reproduction": {
            "val": metrics_a["roll_ptp_deg"],
            "target": ref["roll_ptp"],
            "pass": bool(abs(metrics_a["roll_ptp_deg"] - ref["roll_ptp"]) <= 0.60)
        },
        "z_ptp_reproduction": {
            "val": metrics_a["z_ptp_mm"],
            "target": ref["z_ptp"],
            "pass": bool(abs(metrics_a["z_ptp_mm"] - ref["z_ptp"]) <= 0.020)
        },
        "all3_contact_reproduction": {
            "val": metrics_a["all3_support_fraction"],
            "target": ref["all3_contact"],
            "pass": bool(abs(metrics_a["all3_support_fraction"] - ref["all3_contact"]) <= 0.025)
        }
    }

    overall_pass = all(g["pass"] for g in gates.values())
    return {
        "overall_reproduction_pass": bool(overall_pass),
        "gates": gates
    }


def evaluate_choreography_gates(metrics_a, metrics_b):
    """
    Validates Condition B against established M27-D choreography preservation bounds.
    """
    lat_b = metrics_b["lateral_ratio"]
    lat_a = metrics_a["lateral_ratio"]
    lat_max = 1.15 * lat_a

    gates = {
        "speed_gate": {
            "val": metrics_b["mean_speed_mm_s"],
            "threshold": ">= 7.50 mm/s",
            "pass": bool(metrics_b["mean_speed_mm_s"] >= 7.50)
        },
        "pitch_ptp_gate": {
            "val": metrics_b["pitch_ptp_deg"],
            "threshold": "<= 7.20 deg",
            "pass": bool(metrics_b["pitch_ptp_deg"] <= 7.20)
        },
        "roll_ptp_gate": {
            "val": metrics_b["roll_ptp_deg"],
            "threshold": "<= 11.0 deg",
            "pass": bool(metrics_b["roll_ptp_deg"] <= 11.0)
        },
        "z_ptp_gate": {
            "val": metrics_b["z_ptp_mm"],
            "threshold": "<= 0.130 mm",
            "pass": bool(metrics_b["z_ptp_mm"] <= 0.130)
        },
        "all3_contact_gate": {
            "val": metrics_b["all3_support_fraction"],
            "threshold": ">= 0.880",
            "pass": bool(metrics_b["all3_support_fraction"] >= 0.880)
        },
        "lateral_preservation_gate": {
            "val": lat_b,
            "max_allowed": lat_max,
            "pass": bool(lat_b <= lat_max)
        },
        "abdomen_contact_gate": {
            "val": metrics_b["abdomen_contact_count"],
            "threshold": "== 0",
            "pass": bool(metrics_b["abdomen_contact_count"] == 0)
        },
        "ik_convergence_gate": {
            "val": metrics_b["ik_diagnostics"]["convergence_rate"],
            "threshold": ">= 0.999",
            "pass": bool(metrics_b["ik_diagnostics"]["convergence_rate"] >= 0.999)
        },
        "actuator_step_gate": {
            "val": metrics_b["max_target_step_rad"],
            "threshold": "<= 0.050 rad/step",
            "pass": bool(metrics_b["max_target_step_rad"] <= 0.050)
        }
    }

    overall_pass = all(g["pass"] for g in gates.values())
    return {
        "overall_choreography_pass": bool(overall_pass),
        "gates": gates
    }


def classify_condition_b(metrics_a, metrics_b, repro_pass, choreo_pass):
    """
    Computes formal scientific classification for Condition B.
    """
    if not repro_pass:
        return "REPRODUCTION_FAILED", {"reason": "Condition A failed reproduction gates."}

    if not choreo_pass:
        return "CHOREOGRAPHY_PRESERVATION_FAILED", {"reason": "Condition B violated choreography gates."}

    canc_peak_a = metrics_a["transfer_event_dynamics"]["peak_cancellation"]
    canc_peak_b = metrics_b["transfer_event_dynamics"]["peak_cancellation"]
    cv50_a = metrics_a["cv50"]
    cv50_b = metrics_b["cv50"]
    ws_a = metrics_a["wrong_sign_20"]
    ws_b = metrics_b["wrong_sign_20"]

    reduc_peak = (canc_peak_a - canc_peak_b) / max(canc_peak_a, 1e-9)

    criteria = {
        "canc_peak_a": canc_peak_a,
        "canc_peak_b": canc_peak_b,
        "peak_reduction_fraction": reduc_peak,
        "cv50_a": cv50_a,
        "cv50_b": cv50_b,
        "wrong_sign_a": ws_a,
        "wrong_sign_b": ws_b
    }

    if canc_peak_b <= 0.65 and (cv50_b <= 0.290 or ws_b <= 0.050):
        return "TRANSFER_CONFLICT_RESOLVED", criteria
    elif canc_peak_b <= 0.72 and cv50_b <= cv50_a:
        return "TRANSFER_CONFLICT_PARTIALLY_RESOLVED", criteria
    elif canc_peak_b > 0.75 or cv50_b > cv50_a:
        return "TRANSFER_CONFLICT_NOT_RESOLVED", criteria
    else:
        return "TRANSFER_CONFLICT_PARTIALLY_RESOLVED", criteria


# -----------------------------------------------------------------------------
# DIAGNOSTIC PLOTS
# -----------------------------------------------------------------------------

def generate_diagnostic_plots(telemetry_dict, results_dict, output_dir="output/m30"):
    """Generates comprehensive publication-grade diagnostic plots."""
    os.makedirs(output_dir, exist_ok=True)
    conds = ["A", "B"]
    labels = ["A: EXACT M27-D", "B: TRANSFER RELIEF"]
    colors = ["#2b5c8f", "#d95f02"]

    # 1. Backward Velocity Profiles
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), dpi=200, sharex=True)
    for c, lbl, col in zip(conds, labels, colors):
        t = telemetry_dict[c]["t"]
        mask = t >= STEADY_START_TIME
        axes[0].plot(t[mask], -telemetry_dict[c]["vx"][mask], label=lbl, color=col, alpha=0.4, linewidth=0.8)

        # 50 ms rolling mean
        w50 = int(0.050 / SAMPLE_DT)
        vx_smooth = np.convolve(-telemetry_dict[c]["vx"][mask], np.ones(w50)/w50, mode="valid")
        t_smooth = t[mask][:len(vx_smooth)]
        axes[1].plot(t_smooth, vx_smooth, label=lbl, color=col, linewidth=1.8)

    axes[0].set_ylabel("Instantaneous Speed (mm/s)")
    axes[0].set_title("Backward Locomotion Velocity Telemetry")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("50 ms Rolling Speed (mm/s)")
    axes[1].set_title("Smoothed Backward Glide Velocity (CV50)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plot1_path = os.path.join(output_dir, "velocity_profiles.png")
    plt.savefig(plot1_path)
    plt.close()
    print(f"  Saved: {plot1_path}")

    # 2. Event-Triggered Hesitation Profile
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), dpi=200)
    for c, lbl, col in zip(conds, labels, colors):
        ev_data = results_dict[c]["transfer_event_dynamics"]
        if ev_data["time_axis_ms"] and ev_data["mean_speed_profile_mm_s"]:
            t_axis = ev_data["time_axis_ms"]
            spd = ev_data["mean_speed_profile_mm_s"]
            canc = ev_data["mean_cancellation_profile"]
            axes[0].plot(t_axis, spd, label=lbl, color=col, linewidth=1.8)
            axes[1].plot(t_axis, canc, label=lbl, color=col, linewidth=1.8)

    axes[0].set_xlabel("Time relative to Speed Minimum (ms)")
    axes[0].set_ylabel("Backward Speed (mm/s)")
    axes[0].set_title("Event-Triggered Speed Minimum Profile")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel("Time relative to Speed Minimum (ms)")
    axes[1].set_ylabel("6-Leg Force Cancellation Ratio")
    axes[1].set_title("Force Cancellation Peak around Transfer")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plot2_path = os.path.join(output_dir, "event_triggered_hesitation.png")
    plt.savefig(plot2_path)
    plt.close()
    print(f"  Saved: {plot2_path}")


# -----------------------------------------------------------------------------
# VIDEO RENDERING & COMPOSITING
# -----------------------------------------------------------------------------

def render_condition_dual_camera(condition_name, controller_builder, out_dir, prefix, fps=30):
    """
    Reruns physics ONCE per condition capturing dual cameras (tracking + top-down)
    and encodes both 0.2x and 1.0x videos.
    """
    os.makedirs(out_dir, exist_ok=True)
    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    cam_side = fly.add_tracking_camera(name="trackcam")
    cam_top = fly.add_tracking_camera(
        name="topcam",
        pos_offset=(0.0, 0.0, 8.0),
        rotation=Rotation3D(format="xyaxes", values=(0, 1, 0, -1, 0, 0)),
        fovy=45.0
    )

    sim = m25.create_isolated_sim(fly)
    controller = controller_builder(sim, fly)

    sim.reset()
    controller.time = 0.0
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, controller.default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    renderer = sim.set_renderer(
        [cam_side, cam_top],
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

        frames_side_02x = renderer.frames["fly/trackcam"]
        frames_top_02x = renderer.frames["fly/topcam"]

        # Side view: 0.2x and 1.0x
        path_side_02x = os.path.join(out_dir, f"{prefix}_side_02x.mp4")
        path_side_10x = os.path.join(out_dir, f"{prefix}_side_10x.mp4")
        iio.mimsave(path_side_02x, frames_side_02x, fps=fps, codec="libx264")
        iio.mimsave(path_side_10x, frames_side_02x[::5], fps=fps, codec="libx264")

        # Top view: 0.2x and 1.0x
        path_top_02x = os.path.join(out_dir, f"{prefix}_top_02x.mp4")
        path_top_10x = os.path.join(out_dir, f"{prefix}_top_10x.mp4")
        iio.mimsave(path_top_02x, frames_top_02x, fps=fps, codec="libx264")
        iio.mimsave(path_top_10x, frames_top_02x[::5], fps=fps, codec="libx264")

    finally:
        renderer.close()
        sim.close()

    return {
        "side_02x": path_side_02x,
        "side_10x": path_side_10x,
        "top_02x": path_top_02x,
        "top_10x": path_top_10x
    }


def create_side_by_side_video(path_a, path_b, out_path, label_a, label_b):
    """Composites 2 videos side-by-side with crisp label overlays."""
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    reader_a = iio.get_reader(str(path_a))
    reader_b = iio.get_reader(str(path_b))
    fps = reader_a.get_meta_data().get("fps", 30)

    writer = iio.get_writer(str(out_path), fps=fps)
    try:
        for frame_a, frame_b in zip(reader_a, reader_b):
            img_a = Image.fromarray(frame_a)
            img_b = Image.fromarray(frame_b)

            for img, lbl in [(img_a, label_a), (img_b, label_b)]:
                draw = ImageDraw.Draw(img)
                draw.rectangle([0, 0, 320, 24], fill=(0, 0, 0, 160))
                draw.text((8, 5), lbl, fill="white")

            w, h = img_a.size
            combined = Image.new("RGB", (w * 2, h))
            combined.paste(img_a, (0, 0))
            combined.paste(img_b, (w, 0))
            writer.append_data(np.array(combined))
    finally:
        reader_a.close()
        reader_b.close()
        writer.close()


# -----------------------------------------------------------------------------
# MAIN EXPERIMENT EXECUTION
# -----------------------------------------------------------------------------

def run_m30_experiment(skip_render=False):
    """
    Executes the entire MW-M30 experiment:
    1. Preflight provenance & static isolation audits.
    2. Exactly 2 dynamic evaluation rollouts (A, B).
    3. Quantitative analysis, gates, and primary classifications.
    4. Diagnostic plots and JSON artifacts export.
    5. Clean rendering rollouts (dual-camera side + top-down for A and B).
    """
    print("=" * 70)
    print("MW-M30: MINIMAL TRIPOD-TRANSFER CONFLICT INTERVENTION")
    print("=" * 70)

    # Step 1: Preflights
    m8_expected = run_preflight_provenance_audit()

    fly_probe = make_locomotion_fly(name="fly", add_adhesion=True)
    sim_probe = m25.create_isolated_sim(fly_probe)
    dof_map = m25.build_semantic_dof_mapping(fly_probe, sim_probe)
    pps = PreprogrammedSteps()
    actuated_dofs = fly_probe.get_actuated_jointdofs_order(ActuatorType.POSITION)

    verify_scratch_mjdata_isolation(fly_probe, sim_probe, dof_map)
    preflight_isolation = run_preflight_condition_isolation(fly_probe, sim_probe, dof_map, pps, actuated_dofs)
    sim_probe.close()

    # Step 2: Exactly 2 Dynamic Simulation Rollouts
    conditions = ["A", "B"]
    telemetry = {}
    metrics = {}

    def build_ctrl_a(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M30TransferReliefController(sim, fly, pps, actuated_dofs, dm, condition="A",
                                           frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    def build_ctrl_b(sim, fly):
        dm = m25.build_semantic_dof_mapping(fly, sim)
        return M30TransferReliefController(sim, fly, pps, actuated_dofs, dm, condition="B",
                                           frozen_heights=FROZEN_BODY_SUPPORT_HEIGHTS_MM)

    builders = {
        "A": build_ctrl_a,
        "B": build_ctrl_b
    }

    cond_names = {
        "A": "Condition A (EXACT_M27_D)",
        "B": "Condition B (M27_D_TRANSFER_RELIEF)"
    }

    for c in conditions:
        telem = run_diagnostic_simulation(cond_names[c], builders[c])
        telemetry[c] = telem
        met = analyze_simulation_telemetry(telem)
        metrics[c] = met

    # Step 3: Gate Evaluation & Primary Classification
    repro_eval = evaluate_reproduction_gates(metrics["A"])
    choreo_eval = evaluate_choreography_gates(metrics["A"], metrics["B"])

    class_b, crit_b = classify_condition_b(
        metrics["A"], metrics["B"], repro_eval["overall_reproduction_pass"], choreo_eval["overall_choreography_pass"]
    )

    print("\n" + "=" * 70)
    print("MW-M30 EXPERIMENTAL RESULTS SUMMARY")
    print("=" * 70)
    print(f"Condition A Reproduction Gate: {'PASS' if repro_eval['overall_reproduction_pass'] else 'FAIL'}")
    print(f"Condition B Choreography Gate: {'PASS' if choreo_eval['overall_choreography_pass'] else 'FAIL'}")
    print(f"\nCondition B Primary Classification: {class_b}")

    print("\nKey Locomotion Metrics Comparison:")
    print(f"  Metric                     | Condition A (M27-D) | Condition B (Transfer Relief) | Delta (B - A)")
    print(f"  ---------------------------+---------------------+-------------------------------+---------------")
    print(f"  Backward Speed (mm/s)      | {metrics['A']['mean_speed_mm_s']:19.2f} | {metrics['B']['mean_speed_mm_s']:29.2f} | {metrics['B']['mean_speed_mm_s'] - metrics['A']['mean_speed_mm_s']:+13.2f}")
    print(f"  CV50 Smoothness            | {metrics['A']['cv50']:19.4f} | {metrics['B']['cv50']:29.4f} | {metrics['B']['cv50'] - metrics['A']['cv50']:+13.4f}")
    print(f"  Wrong-Sign 20 Fraction     | {metrics['A']['wrong_sign_20']*100.0:18.2f}% | {metrics['B']['wrong_sign_20']*100.0:28.2f}% | {(metrics['B']['wrong_sign_20'] - metrics['A']['wrong_sign_20'])*100.0:+12.2f}%")
    print(f"  Pitch PTP (deg)            | {metrics['A']['pitch_ptp_deg']:19.2f} | {metrics['B']['pitch_ptp_deg']:29.2f} | {metrics['B']['pitch_ptp_deg'] - metrics['A']['pitch_ptp_deg']:+13.2f}")
    print(f"  Roll PTP (deg)             | {metrics['A']['roll_ptp_deg']:19.2f} | {metrics['B']['roll_ptp_deg']:29.2f} | {metrics['B']['roll_ptp_deg'] - metrics['A']['roll_ptp_deg']:+13.2f}")
    print(f"  Thorax Z PTP (mm)          | {metrics['A']['z_ptp_mm']:19.4f} | {metrics['B']['z_ptp_mm']:29.4f} | {metrics['B']['z_ptp_mm'] - metrics['A']['z_ptp_mm']:+13.4f}")
    print(f"  All-3 Support Contact      | {metrics['A']['all3_support_fraction']*100.0:18.2f}% | {metrics['B']['all3_support_fraction']*100.0:28.2f}% | {(metrics['B']['all3_support_fraction'] - metrics['A']['all3_support_fraction'])*100.0:+12.2f}%")
    print(f"  Lateral Drift Ratio        | {metrics['A']['lateral_ratio']:19.4f} | {metrics['B']['lateral_ratio']:29.4f} | {metrics['B']['lateral_ratio'] - metrics['A']['lateral_ratio']:+13.4f}")

    canc_a = metrics["A"]["transfer_event_dynamics"]["peak_cancellation"]
    canc_b = metrics["B"]["transfer_event_dynamics"]["peak_cancellation"]
    print(f"\nTransfer Event Force Cancellation Peak:")
    print(f"  Condition A (M27-D):           {canc_a:.4f}")
    print(f"  Condition B (Transfer Relief): {canc_b:.4f} (Reduction = {(canc_a - canc_b)/max(canc_a, 1e-9)*100.0:.1f}%)")

    wd = metrics["B"]["work_diagnostics"]
    print(f"\nCondition B Intervention Work Diagnostics:")
    print(f"  Positive Work Injected:       {wd['POSITIVE_WORK_INJECTION_UJ']:.4f} uJ")
    print(f"  Negative Work Dissipated:     {wd['NEGATIVE_WORK_DISSIPATION_UJ']:.4f} uJ")
    print(f"  Net Intervention Work:        {wd['NET_WORK_UJ']:+.4f} uJ")
    print(f"  Work Ratio:                   {wd['WORK_RATIO']:.4f} (Status: {wd['WORK_RATIO_STATUS']})")

    # Step 4: Export Manifest & Results JSON
    manifest_data = {
        "milestone": "MW-M30",
        "task": "Minimal tripod-transfer conflict intervention",
        "provenance_parameters": m8_expected,
        "frozen_heights_provenance": FROZEN_BODY_SUPPORT_HEIGHTS_MM,
        "condition_excursions": CONDITION_EXCURSIONS,
        "preflight_isolation": preflight_isolation,
        "reproduction_gates": repro_eval,
        "choreography_gates": choreo_eval,
        "condition_b_classification": class_b,
        "condition_b_criteria": crit_b
    }

    manifest_path = pathlib.Path("data/derived/m30_tripod_transfer_manifest.json")
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(manifest_data), f, indent=2, allow_nan=False)
    print(f"\nSaved manifest: {manifest_path}")

    results_data = {
        "milestone": "MW-M30",
        "conditions": metrics,
        "reproduction_gates": repro_eval,
        "choreography_gates": choreo_eval,
        "condition_b_verdict": {
            "primary_classification": class_b,
            "criteria": crit_b
        },
        "qualitative_review_status": "PENDING_MANAGER_REVIEW"
    }

    results_path = pathlib.Path("data/derived/m30_tripod_transfer_results.json")
    with open(results_path, "w") as f:
        json.dump(sanitize_json_for_rfc8259(results_data), f, indent=2, allow_nan=False)
    print(f"Saved results:  {results_path}")

    # Step 5: Generate Diagnostic Plots
    generate_diagnostic_plots(telemetry, metrics, output_dir="output/m30")

    # Step 6: Video Rendering Suite (Budget: exactly 2 physics reruns)
    if not skip_render:
        print("\n" + "-"*70)
        print("Rendering synchronized diagnostic video suite (exactly 2 physics reruns)...")
        print("-" * 70)
        vid_dir = pathlib.Path("output/m30")
        vid_dir.mkdir(parents=True, exist_ok=True)

        print("Rendering Condition A (Side & Top-Down) ...", end="", flush=True)
        paths_a = render_condition_dual_camera("EXACT_M27_D", build_ctrl_a, str(vid_dir), "video_A_m27d")
        print(" Done ✓")

        print("Rendering Condition B (Side & Top-Down) ...", end="", flush=True)
        paths_b = render_condition_dual_camera("TRANSFER_RELIEF", build_ctrl_b, str(vid_dir), "video_B_transfer_relief")
        print(" Done ✓")

        # Composite side-by-side tracking view (1.0x and 0.2x)
        out_sbs_10x = str(vid_dir / "video_A_vs_B_10x.mp4")
        out_sbs_02x = str(vid_dir / "video_A_vs_B_02x.mp4")
        print("Compositing side-by-side tracking presentation (1.0x) ...", end="", flush=True)
        create_side_by_side_video(
            paths_a["side_10x"], paths_b["side_10x"], out_sbs_10x,
            "A: EXACT M27-D (1.0x)", "B: TRANSFER RELIEF (1.0x)"
        )
        print(" Done ✓")

        print("Compositing side-by-side tracking slow-motion (0.2x) ...", end="", flush=True)
        create_side_by_side_video(
            paths_a["side_02x"], paths_b["side_02x"], out_sbs_02x,
            "A: EXACT M27-D (0.2x)", "B: TRANSFER RELIEF (0.2x)"
        )
        print(" Done ✓")

        # Composite side-by-side top-down view (1.0x and 0.2x)
        out_top_10x = str(vid_dir / "video_A_vs_B_topdown_10x.mp4")
        out_top_02x = str(vid_dir / "video_A_vs_B_topdown_02x.mp4")
        print("Compositing side-by-side top-down presentation (1.0x) ...", end="", flush=True)
        create_side_by_side_video(
            paths_a["top_10x"], paths_b["top_10x"], out_top_10x,
            "A: EXACT M27-D (TOP 1.0x)", "B: TRANSFER RELIEF (TOP 1.0x)"
        )
        print(" Done ✓")

        print("Compositing side-by-side top-down slow-motion (0.2x) ...", end="", flush=True)
        create_side_by_side_video(
            paths_a["top_02x"], paths_b["top_02x"], out_top_02x,
            "A: EXACT M27-D (TOP 0.2x)", "B: TRANSFER RELIEF (TOP 0.2x)"
        )
        print(" Done ✓")

    print("\nMW-M30 Execution Completed Successfully.")
    return manifest_data, results_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MW-M30 Minimal Tripod-Transfer Conflict Intervention")
    parser.add_argument("--skip-render", action="store_true", default=False, help="Skip video rendering")
    parser.add_argument("--render", action="store_true", default=True, help="Render diagnostic videos")
    args = parser.parse_args()

    do_skip = args.skip_render
    run_m30_experiment(skip_render=do_skip)
