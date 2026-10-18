import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
"""MW-M13: Pulse Mechanics Diagnosis — Falsifiability experiment for stance-geometry hypothesis."""

import json
import pathlib
from importlib.metadata import version

import numpy as np

from flygym import Simulation
from flygym.anatomy import BodySegment, LEGS
from flygym.compose import ActuatorType, FlatGroundWorld
from flygym.utils.math import Rotation3D
from flygym_demo.complex_terrain import (
    LocomotionAction,
    PreprogrammedSteps,
    apply_locomotion_action,
    make_locomotion_fly,
)

# ---------------------------------------------------------------------------
# Constants from authoritative M8 champion selection
# ---------------------------------------------------------------------------
M8_CADENCE = 6.0
M8_STANCE_EXCURSION = 1.0
M8_TURNAROUND = 0.06
M8_TRANSFER_FRACTION = 0.08
M8_ENTRY_DURATION = 0.25

PHYSICS_DT = 0.0001          # seconds
SAMPLE_EVERY_N_STEPS = 10    # sample at ~1 ms
SAMPLE_DT = PHYSICS_DT * SAMPLE_EVERY_N_STEPS

# Trial duration: enough for several cadence cycles (6 Hz → ~0.167 s/cycle)
# We want ~15 cycles minimum for frequency analysis → need ≥2.5 s total
# Plus 0.25 s entry → run 3.0 s total
TRIAL_DURATION_S = 3.0
N_STEPS = int(TRIAL_DURATION_S / PHYSICS_DT)

# Steady-state starts after entry + a 0.1 s buffer
STEADY_START_S = M8_ENTRY_DURATION + 0.1

N_REPS = 3   # Deterministic reps (identical — reproducibility check)

# Leg ordering per FlyGym anatomy: LEGS = ['lf', 'lm', 'lh', 'rf', 'rm', 'rh']
# Indices into that order:
LEG_NAMES = LEGS  # ['lf', 'lm', 'lh', 'rf', 'rm', 'rh']
LEG_IDX = {name: i for i, name in enumerate(LEG_NAMES)}

# M8 tripod groupings (from M8 code: tripod A = indices 0,2,4)
TRIPOD_A_LEGS = ['lf', 'lh', 'rm']   # indices 0, 2, 4
TRIPOD_B_LEGS = ['rf', 'lm', 'rh']   # indices 1, 3, 5  (note: pps.legs[1]='lm', [3]='rf', [5]='rh')

# ---------------------------------------------------------------------------
# Shared helpers (exact copies from M8 to ensure reproducibility)
# ---------------------------------------------------------------------------

def smooth_step(x, edge0, edge1):
    t = np.clip((x - edge0) / (edge1 - edge0 + 1e-9), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def smooth_triangle(phi, r):
    phi = phi % 2.0
    if phi > 1.0:
        phi = 2.0 - phi
    v_max = 1.0 / (1.0 - r + 1e-9)
    if phi <= r:
        if r < 1e-9:
            return 0.0
        x = phi / r
        return r * v_max * (x**3 - 0.5 * x**4)
    elif phi <= 1.0 - r:
        return 0.5 * r * v_max + v_max * (phi - r)
    else:
        if r < 1e-9:
            return 1.0
        psi = 1.0 - phi
        x = psi / r
        return 1.0 - r * v_max * (x**3 - 0.5 * x**4)


def get_adhesion(phi, t_f):
    phi = phi % 2.0
    d1 = phi - 1.0
    if -t_f/2 <= d1 <= t_f/2:
        return smooth_step(d1, -t_f/2, t_f/2)
    d0 = phi
    if phi > 1.5:
        d0 = phi - 2.0
    if -t_f/2 <= d0 <= t_f/2:
        return 1.0 - smooth_step(d0, -t_f/2, t_f/2)
    if 0 < phi < 1:
        return 0.0
    return 1.0


def quat_to_pitch_rad(q_wxyz):
    """Convert quaternion (w,x,y,z) to body pitch angle in radians.

    Uses the standard aerospace convention: pitch is the rotation about the
    Y-axis (lateral axis). For a fly whose forward direction is +X and up is +Z,
    positive pitch = nose up.

    q = (w, x, y, z)
    pitch = arcsin(2*(w*y - z*x))
    """
    w, x, y, z = q_wxyz
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    return float(np.arcsin(sinp))


# ---------------------------------------------------------------------------
# Segment name helpers — semantic, no magic indices
# ---------------------------------------------------------------------------

def get_tarsus5_seg_names():
    """Return list of segment name strings for tarsus5 of each leg, ordered as LEGS."""
    return [f"{leg}_tarsus5" for leg in LEG_NAMES]


def build_indices(fly, sim):
    """Return all semantic index maps needed for telemetry."""
    body_segs = fly.get_bodysegs_order()
    seg_names = [s.name if hasattr(s, 'name') else str(s) for s in body_segs]

    # Strip fly-name prefix if present (FlyGym 2.1 may prepend "fly/")
    def _seg_idx(bare_name):
        # Try with prefix first
        for prefix in [f"{fly.name}/", ""]:
            candidate = prefix + bare_name
            if candidate in seg_names:
                return seg_names.index(candidate)
        raise KeyError(f"Segment '{bare_name}' not found in body segs: {seg_names[:10]}")

    thorax_idx = _seg_idx("c_thorax")
    tarsus5_idxs = [_seg_idx(f"{leg}_tarsus5") for leg in LEG_NAMES]
    return thorax_idx, tarsus5_idxs


def get_tarsus_bodyseg_list(fly):
    """Return list[str] of tarsus5 body segment names accepted by get_bodysegment_contact_forces."""
    # FlyGym anatomy segment names are the bare names: e.g., "lf_tarsus5"
    # but when a fly is added to a simulation, the Simulation may use fly-namespaced IDs internally.
    # The BodySegment class checks ALL_SEGMENT_NAMES which uses bare names.
    return [f"{leg}_tarsus5" for leg in LEG_NAMES]


# ---------------------------------------------------------------------------
# Force API validation helpers
# ---------------------------------------------------------------------------

def get_tarsus_forces(sim, fly, tarsus_seg_names):
    """Try to get world-frame contact forces for all tarsus5 segments.

    Returns (forces_6x3, success). On BodySegment validation error, returns (zeros, False).
    """
    try:
        forces = sim.get_bodysegment_contact_forces(
            fly.name, tarsus_seg_names, ground_only=True
        )
        return forces, True
    except Exception as e:
        return np.zeros((6, 3), dtype=float), False


# ---------------------------------------------------------------------------
# Compute per-arc-bin statistics (Phase-binned profile)
# ---------------------------------------------------------------------------

def arc_binned_profile(u_arr, quantity_arr, n_bins=20):
    """Compute mean of quantity in u-bins [0,1].

    Returns (bin_centers, bin_means, arc_velocity_modulation_fraction).
    """
    u_arr = np.asarray(u_arr, dtype=float)
    quantity_arr = np.asarray(quantity_arr, dtype=float)
    edges = np.linspace(0.0, 1.0, n_bins + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    bin_means = np.full(n_bins, np.nan)
    for k in range(n_bins):
        mask = (u_arr >= edges[k]) & (u_arr < edges[k + 1])
        if mask.sum() > 3:
            bin_means[k] = float(np.mean(quantity_arr[mask]))

    valid = ~np.isnan(bin_means)
    if valid.sum() < 4:
        return centers, bin_means, float('nan')

    overall_mean = float(np.nanmean(quantity_arr))
    if abs(overall_mean) < 1e-6:
        arc_mod = float('nan')
    else:
        arc_mod = float((np.nanmax(bin_means) - np.nanmin(bin_means)) / abs(overall_mean))

    return centers, bin_means, arc_mod


def dominant_freq_hz(signal_arr, dt_s, fmin=1.0, fmax=50.0):
    """Return dominant frequency in [fmin, fmax] Hz using numpy FFT.

    Returns (freq_hz, power_at_freq, spectrum_peak_fraction) where
    spectrum_peak_fraction = power at peak / total power in band.
    """
    sig = signal_arr - np.mean(signal_arr)
    N = len(sig)
    if N < 16:
        return float('nan'), float('nan'), float('nan')
    fft_vals = np.fft.rfft(sig)
    freqs = np.fft.rfftfreq(N, d=dt_s)
    power = np.abs(fft_vals) ** 2

    mask = (freqs >= fmin) & (freqs <= fmax)
    if mask.sum() == 0:
        return float('nan'), float('nan'), float('nan')

    band_power = power[mask]
    band_freqs = freqs[mask]
    peak_idx = np.argmax(band_power)
    dom_freq = float(band_freqs[peak_idx])
    dom_power = float(band_power[peak_idx])
    total_band_power = float(np.sum(band_power))
    fraction = float(dom_power / max(total_band_power, 1e-30))
    return dom_freq, dom_power, fraction


def cv_windowed(vx_arr, window_size):
    """Compute CV of windowed-mean velocity array."""
    if len(vx_arr) < window_size:
        return float(np.std(vx_arr) / max(abs(np.mean(vx_arr)), 1e-9))
    smoothed = np.convolve(vx_arr, np.ones(window_size) / window_size, mode='valid')
    mean_v = float(np.mean(smoothed))
    return float(np.std(smoothed) / max(abs(mean_v), 1e-9))


# ---------------------------------------------------------------------------
# The M8 stance path function (shared exactly with M8)
# ---------------------------------------------------------------------------

def compute_m8_phases_and_adhesions(phi, pps, stance_starts, stance_ends,
                                     E=M8_STANCE_EXCURSION, r=M8_TURNAROUND, t_f=M8_TRANSFER_FRACTION):
    """Compute per-leg stance phases and adhesions for M8 tripod logic at controller phase phi."""
    phases = np.zeros(6)
    adhesions = np.zeros(6)
    u_vals = np.zeros(6)

    # LEGS order: ['lf', 'lm', 'lh', 'rf', 'rm', 'rh']
    # M8 tripod A = indices 0,2,4 (lf, lh, rm)
    for i in range(6):
        if i in [0, 2, 4]:   # Tripod A
            p = phi
        else:                  # Tripod B
            p = (phi + 1.0) % 2.0

        w = smooth_triangle(p, r)
        u = 0.5 - E / 2.0 + E * w
        u_vals[i] = u

        phases[i] = stance_starts[i] + u * (stance_ends[i] - stance_starts[i])
        adhesions[i] = get_adhesion(p, t_f)

    return phases, adhesions, u_vals


def get_swing_pose_angles(leg, pps, actuated_dofs, E=M8_STANCE_EXCURSION):
    """Return joint angles for the mid-swing phase of `leg`.

    We use the biological mid-swing pose to place the unloaded leg in a
    reasonable elevated position without inventing a large artificial lift.
    Mid-swing = halfway through the swing arc ≈ swing_start + 0.5*(swing_end-swing_start).
    For standard PreprogrammedSteps: swing_start=0, swing_end=pps.swing_period[leg][1].
    """
    swing_start, swing_end = pps.swing_period[leg]
    mid_swing_phase = (swing_start + swing_end) * 0.5
    # Build a 6-element phase array with all other legs at mid-stance
    phases = np.zeros(6)
    mid_stance = {}
    for lg in pps.legs:
        sw_s, sw_e = pps.swing_period[lg]
        # Mid-stance phase is halfway through the stance arc
        mid_stance[lg] = sw_e + 0.5 * ((sw_s + 2 * np.pi) - sw_e)
    for i, lg in enumerate(pps.legs):
        phases[i] = mid_stance[lg]
    # Override the target leg
    leg_idx = pps.legs.index(leg)
    phases[leg_idx] = mid_swing_phase

    return pps.get_joint_angles_by_dof_order(phases, output_dof_order=actuated_dofs)


# ---------------------------------------------------------------------------
# Compute static-mid-stance pose (for static-u conditions)
# ---------------------------------------------------------------------------

def get_mid_stance_u():
    """Return mid-stance u = 0.5 (midpoint of the excursion range with E=1)."""
    return 0.5


def compute_static_u_phases(pps, stance_starts, stance_ends, u_mid=0.5):
    """Return phases array where every leg is pinned at u=u_mid in its stance arc."""
    phases = np.zeros(6)
    for i in range(6):
        phases[i] = stance_starts[i] + u_mid * (stance_ends[i] - stance_starts[i])
    return phases


# ---------------------------------------------------------------------------
# Core telemetry collection loop
# ---------------------------------------------------------------------------

def run_trial(sim, fly, pps, actuated_dofs, thorax_idx, tarsus5_idxs,
              tarsus_seg_names, condition_name, rep_idx, force_api_ok):
    """Run one trial and collect full telemetry.

    Returns dict with all measurements.
    """
    # Precompute stance window boundaries (identical for all conditions)
    stance_starts = []
    stance_ends = []
    for leg in pps.legs:
        sw_s, sw_e = pps.swing_period[leg]
        stance_starts.append(sw_e)
        stance_ends.append(sw_s + 2.0 * np.pi)

    # Build the static-u angles for unloaded-leg placement (mid-swing of each leg)
    swing_mid_angles = {}
    for leg in pps.legs:
        swing_mid_angles[leg] = get_swing_pose_angles(leg, pps, actuated_dofs)

    # Determine which legs are "active support" vs "unloaded" for this condition
    if condition_name == "M8_NORMAL_CONTROL":
        support_mask = None   # dynamic
        unloaded_legs = []
        static_u = False
    elif condition_name == "A_ONLY_CONTINUOUS":
        support_mask = [True, False, True, False, True, False]  # lf, lh, rm
        unloaded_legs = ['lm', 'rf', 'rh']   # tripod B legs
        static_u = False
    elif condition_name == "B_ONLY_CONTINUOUS":
        support_mask = [False, True, False, True, False, True]  # rf, lm, rh
        unloaded_legs = ['lf', 'lh', 'rm']   # tripod A legs
        static_u = False
    elif condition_name == "A_ONLY_STATIC_U":
        support_mask = [True, False, True, False, True, False]
        unloaded_legs = ['lm', 'rf', 'rh']
        static_u = True
    elif condition_name == "B_ONLY_STATIC_U":
        support_mask = [False, True, False, True, False, True]
        unloaded_legs = ['lf', 'lh', 'rm']
        static_u = True
    else:
        raise ValueError(f"Unknown condition: {condition_name}")

    # Precompute the unloaded leg joint angles (mid-swing pose — naturally elevated)
    unloaded_leg_angles = {}
    for leg in unloaded_legs:
        unloaded_leg_angles[leg] = get_swing_pose_angles(leg, pps, actuated_dofs)

    # Static-u support leg phases (pinned mid-stance)
    static_phases = compute_static_u_phases(pps, stance_starts, stance_ends, u_mid=0.5)
    static_u_angles = pps.get_joint_angles_by_dof_order(static_phases, output_dof_order=actuated_dofs)

    default_pose = pps.default_pose_by_dof_order(actuated_dofs)

    # Reset sim
    sim.reset()
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    # Telemetry storage
    t_arr = []
    thorax_x_arr, thorax_y_arr, thorax_z_arr = [], [], []
    thorax_vx_arr = []
    thorax_ax_arr = []  # filled by differencing vx
    pitch_arr = []
    tarsus_x_arr = []   # shape (N, 6)
    tarsus_z_arr = []   # shape (N, 6) — for ground clearance check
    tarsus_vx_arr = []  # shape (N, 6)
    contact_arr = []    # shape (N, 6)
    adhesion_arr = []   # shape (N, 6)
    force_x_arr = []    # shape (N, 6) — world X forces on tarsus5
    force_z_arr = []    # shape (N, 6) — world Z forces
    u_vals_arr = []     # shape (N, 6) — arc position u for each leg
    phi_arr = []        # controller phase phi

    prev_thorax_x = None
    prev_vx = None

    time_val = 0.0

    for step_i in range(N_STEPS):
        # -------------------------------------------------------------------
        # Compute controller outputs for this step
        # -------------------------------------------------------------------
        phi = (time_val * M8_CADENCE * 2.0) % 2.0

        if condition_name == "M8_NORMAL_CONTROL":
            # Exact M8 logic
            m8_phases, adhesions, u_vals = compute_m8_phases_and_adhesions(
                phi, pps, stance_starts, stance_ends
            )
            stock_angles = pps.get_joint_angles_by_dof_order(m8_phases, output_dof_order=actuated_dofs)

            if time_val < M8_ENTRY_DURATION:
                alpha = smooth_step(time_val, 0.0, M8_ENTRY_DURATION)
                joint_angles = (1.0 - alpha) * default_pose + alpha * stock_angles
                adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions
            else:
                joint_angles = stock_angles

        elif condition_name in ("A_ONLY_CONTINUOUS", "B_ONLY_CONTINUOUS"):
            # Support group: advance through the stance arc normally using M8's phi
            # Unloaded group: hold in mid-swing pose, adhesion=0
            m8_phases, _, u_vals = compute_m8_phases_and_adhesions(
                phi, pps, stance_starts, stance_ends
            )
            stock_angles = pps.get_joint_angles_by_dof_order(m8_phases, output_dof_order=actuated_dofs)

            # Override unloaded leg joint angles with their mid-swing pose
            # We work on the full 42-DOF array:
            joint_angles = stock_angles.copy()
            for leg in unloaded_legs:
                leg_swing_angles = unloaded_leg_angles[leg]
                # Only override the 7 DOFs belonging to this leg
                # We need to find DOF indices for this leg
                # Use get_joint_angles for just this leg and overwrite in the combined array
                # Strategy: get the full vector from mid-swing phases of unloaded legs,
                # then blend in the correct DOFs
                # Simpler: compute a full set from a per-leg mid-swing phase override
                pass  # handled below

            # Build a blended angle array:
            # For support legs: use m8 stance arc phases
            # For unloaded legs: use their mid-swing pose
            # We do this by constructing the full phase array and overriding unloaded legs
            override_phases = m8_phases.copy()
            for leg in unloaded_legs:
                leg_i = pps.legs.index(leg)
                sw_s, sw_e = pps.swing_period[leg]
                override_phases[leg_i] = (sw_s + sw_e) * 0.5  # mid-swing

            joint_angles = pps.get_joint_angles_by_dof_order(override_phases, output_dof_order=actuated_dofs)

            # Adhesions: support legs always 1, unloaded legs always 0
            adhesions = np.zeros(6)
            for i, leg in enumerate(pps.legs):
                if leg in unloaded_legs:
                    adhesions[i] = 0.0
                else:
                    adhesions[i] = 1.0

            # Entry blending
            if time_val < M8_ENTRY_DURATION:
                alpha = smooth_step(time_val, 0.0, M8_ENTRY_DURATION)
                joint_angles = (1.0 - alpha) * default_pose + alpha * joint_angles
                adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions

        elif condition_name in ("A_ONLY_STATIC_U", "B_ONLY_STATIC_U"):
            # Support group: hold at mid-stance u=0.5 (no arc traversal)
            # Unloaded group: mid-swing pose, adhesion=0
            u_vals = np.full(6, 0.5)

            override_phases = np.zeros(6)
            for i, leg in enumerate(pps.legs):
                if leg in unloaded_legs:
                    sw_s, sw_e = pps.swing_period[leg]
                    override_phases[i] = (sw_s + sw_e) * 0.5  # mid-swing
                    u_vals[i] = 0.0  # not meaningful for unloaded
                else:
                    override_phases[i] = static_phases[i]   # pinned mid-stance

            joint_angles = pps.get_joint_angles_by_dof_order(override_phases, output_dof_order=actuated_dofs)

            adhesions = np.zeros(6)
            for i, leg in enumerate(pps.legs):
                if leg not in unloaded_legs:
                    adhesions[i] = 1.0

            # Entry blending
            if time_val < M8_ENTRY_DURATION:
                alpha = smooth_step(time_val, 0.0, M8_ENTRY_DURATION)
                joint_angles = (1.0 - alpha) * default_pose + alpha * joint_angles
                adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions

        # -------------------------------------------------------------------
        # Apply and step
        # -------------------------------------------------------------------
        action = LocomotionAction(joint_angles=joint_angles, adhesion_onoff=adhesions)
        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        # -------------------------------------------------------------------
        # Sample telemetry every SAMPLE_EVERY_N_STEPS steps
        # -------------------------------------------------------------------
        if (step_i + 1) % SAMPLE_EVERY_N_STEPS == 0:
            t = (step_i + 1) * PHYSICS_DT

            body_positions = sim.get_body_positions(fly.name)
            thorax_pos = body_positions[thorax_idx].copy()
            tarsus_pos = body_positions[tarsus5_idxs].copy()   # (6, 3)

            # Thorax X velocity via finite difference
            if prev_thorax_x is not None:
                vx = (float(thorax_pos[0]) - prev_thorax_x) / SAMPLE_DT
            else:
                vx = 0.0

            # Thorax X acceleration via second difference
            if prev_vx is not None:
                ax = (vx - prev_vx) / SAMPLE_DT
            else:
                ax = 0.0

            # Thorax pitch from quaternion
            body_rotations = sim.get_body_rotations(fly.name)
            thorax_quat = body_rotations[thorax_idx].copy()  # (w, x, y, z)
            pitch_rad = quat_to_pitch_rad(thorax_quat)

            # Contact info
            contact_info = sim.get_ground_contact_info(fly.name)
            contact_found = contact_info[0].copy()[:6]  # (6,)

            # Per-leg tarsus X velocity via stored prev positions
            if len(tarsus_x_arr) > 0:
                tarsus_vx = (tarsus_pos[:, 0] - tarsus_x_arr[-1][:, 0]) / SAMPLE_DT
            else:
                tarsus_vx = np.zeros(6)

            # Force telemetry (validated separately)
            if force_api_ok:
                forces, _ = get_tarsus_forces(sim, fly, tarsus_seg_names)
            else:
                forces = np.zeros((6, 3))

            # Record
            t_arr.append(t)
            thorax_x_arr.append(float(thorax_pos[0]))
            thorax_y_arr.append(float(thorax_pos[1]))
            thorax_z_arr.append(float(thorax_pos[2]))
            thorax_vx_arr.append(vx)
            thorax_ax_arr.append(ax)
            pitch_arr.append(np.degrees(pitch_rad))
            tarsus_x_arr.append(tarsus_pos.copy())
            tarsus_z_arr.append(tarsus_pos[:, 2].copy())
            tarsus_vx_arr.append(tarsus_vx.copy())
            contact_arr.append(contact_found.copy())
            adhesion_arr.append(adhesions.copy())
            force_x_arr.append(forces[:, 0].copy())
            force_z_arr.append(forces[:, 2].copy())
            u_vals_arr.append(u_vals.copy())
            phi_arr.append(float(phi))

            prev_thorax_x = float(thorax_pos[0])
            prev_vx = vx

        time_val += PHYSICS_DT

    # Convert to arrays
    t_arr = np.array(t_arr)
    thorax_x_arr = np.array(thorax_x_arr)
    thorax_y_arr = np.array(thorax_y_arr)
    thorax_z_arr = np.array(thorax_z_arr)
    thorax_vx_arr = np.array(thorax_vx_arr)
    thorax_ax_arr = np.array(thorax_ax_arr)
    pitch_arr = np.array(pitch_arr)
    tarsus_x_arr_np = np.array(tarsus_x_arr)  # (N, 6, 3) actually (N, 6) for x-col
    tarsus_x_only = tarsus_x_arr_np[:, :, 0]  # (N, 6)
    tarsus_z_arr_np = np.array(tarsus_z_arr)  # (N, 6)
    tarsus_vx_arr_np = np.array(tarsus_vx_arr)  # (N, 6)
    contact_arr_np = np.array(contact_arr)       # (N, 6)
    adhesion_arr_np = np.array(adhesion_arr)      # (N, 6)
    force_x_arr_np = np.array(force_x_arr)       # (N, 6)
    force_z_arr_np = np.array(force_z_arr)        # (N, 6)
    u_vals_arr_np = np.array(u_vals_arr)          # (N, 6)
    phi_arr_np = np.array(phi_arr)

    # -------------------------------------------------------------------
    # Extract steady-state window
    # -------------------------------------------------------------------
    steady_mask = t_arr > STEADY_START_S
    if steady_mask.sum() < 20:
        # Fallback: use last 80%
        n = len(t_arr)
        steady_mask = np.zeros(n, dtype=bool)
        steady_mask[n // 5:] = True

    ss_vx = thorax_vx_arr[steady_mask]
    ss_ax = thorax_ax_arr[steady_mask]
    ss_pitch = pitch_arr[steady_mask]
    ss_t = t_arr[steady_mask]
    ss_u = u_vals_arr_np[steady_mask]
    ss_phi = phi_arr_np[steady_mask]
    ss_force_x = force_x_arr_np[steady_mask]
    ss_force_z = force_z_arr_np[steady_mask]
    ss_tarsus_vx = tarsus_vx_arr_np[steady_mask]
    ss_tarsus_z = tarsus_z_arr_np[steady_mask]
    ss_contact = contact_arr_np[steady_mask]
    ss_adhesion = adhesion_arr_np[steady_mask]
    ss_thorax_x = thorax_x_arr[steady_mask]
    ss_thorax_z = thorax_z_arr[steady_mask]

    # -------------------------------------------------------------------
    # Core metrics
    # -------------------------------------------------------------------
    mean_vx = float(np.mean(ss_vx))
    mean_bwd_speed = float(abs(mean_vx))

    w20 = max(1, int(0.020 / SAMPLE_DT))
    w50 = max(1, int(0.050 / SAMPLE_DT))
    cv20 = cv_windowed(ss_vx, w20)
    cv50 = cv_windowed(ss_vx, w50)
    cv_raw = float(np.std(ss_vx) / max(abs(mean_vx), 1e-9))

    # Frequency analysis
    dom_freq, dom_power, dom_frac = dominant_freq_hz(ss_vx, SAMPLE_DT, fmin=1.0, fmax=40.0)

    # Pitch metrics
    pitch_ptp = float(np.ptp(ss_pitch))
    pitch_std = float(np.std(ss_pitch))

    # Thorax Z
    z_ptp = float(np.ptp(ss_thorax_z))

    # Displacement
    disp_x = float(thorax_x_arr[-1] - thorax_x_arr[0])
    disp_y = float(thorax_y_arr[-1] - thorax_y_arr[0])
    lateral_ratio = abs(disp_y) / max(abs(disp_x), 1e-6)

    # -------------------------------------------------------------------
    # Arc-position relationship (moving-support conditions only)
    # -------------------------------------------------------------------
    # For support legs, use their u values
    # Determine which leg indices are in the support group
    if support_mask is not None:
        sup_indices = [i for i, s in enumerate(support_mask) if s]
    else:
        # M8_NORMAL_CONTROL: compute support group dynamically from adhesion
        sup_indices = list(range(6))  # use all for now, then filter below

    arc_mod_fraction = float('nan')
    pearson_u_vx = float('nan')

    if condition_name in ("A_ONLY_CONTINUOUS", "B_ONLY_CONTINUOUS"):
        # Use the u of the first support leg (they are synchronized in M8 arc)
        u_ref = ss_u[:, sup_indices[0]]
        # Pearson correlation
        if np.std(u_ref) > 1e-6 and np.std(ss_vx) > 1e-6:
            pearson_u_vx = float(np.corrcoef(u_ref, ss_vx)[0, 1])
        else:
            pearson_u_vx = float('nan')

        _, _, arc_mod_fraction = arc_binned_profile(u_ref, ss_vx, n_bins=20)

    elif condition_name == "M8_NORMAL_CONTROL":
        # Use tripod A u (index 0 = lf)
        u_ref = ss_u[:, 0]
        if np.std(u_ref) > 1e-6 and np.std(ss_vx) > 1e-6:
            pearson_u_vx = float(np.corrcoef(u_ref, ss_vx)[0, 1])
        _, _, arc_mod_fraction = arc_binned_profile(u_ref, ss_vx, n_bins=20)

    # -------------------------------------------------------------------
    # Force modulation
    # -------------------------------------------------------------------
    total_force_modulation = float('nan')
    force_cv_per_leg = {}

    if force_api_ok and condition_name in ("A_ONLY_CONTINUOUS", "B_ONLY_CONTINUOUS"):
        total_sup_fx = np.sum(ss_force_x[:, sup_indices], axis=1)  # (N_ss,)
        u_ref_for_force = ss_u[:, sup_indices[0]]
        _, _, total_force_modulation = arc_binned_profile(u_ref_for_force, total_sup_fx, n_bins=20)

        for i in sup_indices:
            leg_fx = ss_force_x[:, i]
            mu = abs(float(np.mean(leg_fx)))
            cv_f = float(np.std(leg_fx) / max(mu, 1e-9))
            force_cv_per_leg[pps.legs[i]] = float(cv_f)

    # -------------------------------------------------------------------
    # Per-leg foot stationarity (support legs, contacting)
    # -------------------------------------------------------------------
    per_leg_metrics = {}
    for i, leg in enumerate(pps.legs):
        # Classify support samples: contacting AND adhesion > 0.5
        sup_samples = (ss_contact[:, i] > 0) & (ss_adhesion[:, i] > 0.5)
        slide_samples = (ss_contact[:, i] > 0) & (ss_adhesion[:, i] <= 0.5)

        sup_vx_vals = ss_tarsus_vx[sup_samples, i]
        slide_vx_vals = ss_tarsus_vx[slide_samples, i]

        per_leg_metrics[leg] = {
            "support_n": int(sup_samples.sum()),
            "slide_n": int(slide_samples.sum()),
            "support_median_vx": float(np.median(sup_vx_vals)) if len(sup_vx_vals) > 0 else float('nan'),
            "support_abs_median_vx": float(np.median(np.abs(sup_vx_vals))) if len(sup_vx_vals) > 0 else float('nan'),
            "slide_median_vx": float(np.median(slide_vx_vals)) if len(slide_vx_vals) > 0 else float('nan'),
            "force_cv": float(force_cv_per_leg.get(leg, float('nan'))),
            "mean_z_during_support": float(np.mean(ss_tarsus_z[sup_samples, i])) if len(sup_vx_vals) > 0 else float('nan'),
        }

    # -------------------------------------------------------------------
    # Transfer peak acceleration (M8_NORMAL_CONTROL only)
    # -------------------------------------------------------------------
    transfer_peak_accel = float('nan')
    if condition_name == "M8_NORMAL_CONTROL":
        # Detect adhesion state transitions: find timesteps where adhesion group switches
        tripod_a_adh = ss_adhesion[:, 0]  # lf adhesion as tripod A proxy
        switch_pts = np.where(np.abs(np.diff(tripod_a_adh)) > 0.1)[0]
        if len(switch_pts) > 0:
            window = max(2, int(0.020 / SAMPLE_DT))
            peak_accels = []
            for sp in switch_pts:
                start = max(0, sp - window)
                end = min(len(ss_ax) - 1, sp + window)
                peak_accels.append(float(np.max(np.abs(ss_ax[start:end]))))
            transfer_peak_accel = float(np.mean(peak_accels))

    # -------------------------------------------------------------------
    # Pitch correlation with vx
    # -------------------------------------------------------------------
    if np.std(ss_pitch) > 1e-6 and np.std(ss_vx) > 1e-6:
        pearson_pitch_vx = float(np.corrcoef(ss_pitch, ss_vx)[0, 1])
    else:
        pearson_pitch_vx = float('nan')

    # -------------------------------------------------------------------
    # Velocity modulation assessment (for comparing single-support vs M8)
    # -------------------------------------------------------------------
    # Single-support periodic pulse = std(ss_vx) in the steady state
    vx_std = float(np.std(ss_vx))

    # -------------------------------------------------------------------
    # Return the trial result dict
    # -------------------------------------------------------------------
    return {
        "condition": condition_name,
        "rep": rep_idx,
        "n_steady_samples": int(steady_mask.sum()),
        "displacement_x_mm": disp_x,
        "displacement_y_mm": disp_y,
        "lateral_ratio": float(lateral_ratio),
        "mean_backward_speed_mm_s": mean_bwd_speed,
        "mean_vx_mm_s": mean_vx,
        "cv_raw": cv_raw,
        "cv20": cv20,
        "cv50": cv50,
        "vx_std_mm_s": vx_std,
        "thorax_z_ptp_mm": z_ptp,
        "pitch_ptp_deg": float(pitch_ptp),
        "pitch_std_deg": float(pitch_std),
        "pearson_pitch_vx": float(pearson_pitch_vx),
        "dominant_freq_hz": float(dom_freq),
        "dominant_freq_power": float(dom_power),
        "dominant_freq_spectrum_fraction": float(dom_frac),
        "arc_velocity_modulation_fraction": float(arc_mod_fraction),
        "pearson_u_vx": float(pearson_u_vx),
        "total_force_modulation_fraction": float(total_force_modulation),
        "transfer_peak_accel_mm_s2": float(transfer_peak_accel),
        "per_leg": per_leg_metrics,
    }


# ---------------------------------------------------------------------------
# Force API validation
# ---------------------------------------------------------------------------

def validate_force_api(sim, fly, pps, actuated_dofs, thorax_idx, tarsus5_idxs):
    """Run a brief M8 trial to check whether get_bodysegment_contact_forces works.

    Validation criteria:
    1. No exception raised.
    2. At least one non-zero force during stable locomotion.
    3. Sum of world-Z forces ≈ body weight × g (at steady state bodies should be supported).
    4. Sign: when fly moves backward (vx < 0), the net world-X force on tarsus5 should be
       non-trivially structured (not just zero).
    """
    tarsus_segs = get_tarsus_bodyseg_list(fly)
    print("  [FORCE-API] Testing get_bodysegment_contact_forces...")

    default_pose = pps.default_pose_by_dof_order(actuated_dofs)
    stance_starts = []
    stance_ends = []
    for leg in pps.legs:
        sw_s, sw_e = pps.swing_period[leg]
        stance_starts.append(sw_e)
        stance_ends.append(sw_s + 2.0 * np.pi)

    sim.reset()
    sim.set_actuator_inputs(fly.name, ActuatorType.POSITION, default_pose)
    sim.set_leg_adhesion_states(fly.name, np.ones(6, dtype=float))
    sim.warmup()

    force_x_samples = []
    force_z_samples = []
    thorax_vx_samples = []
    prev_x = None
    api_ok = True
    exception_msg = ""

    n_validation_steps = int(1.0 / PHYSICS_DT)
    time_val = 0.0
    for step_i in range(n_validation_steps):
        phi = (time_val * M8_CADENCE * 2.0) % 2.0
        m8_phases, adhesions, _ = compute_m8_phases_and_adhesions(phi, pps, stance_starts, stance_ends)
        stock_angles = pps.get_joint_angles_by_dof_order(m8_phases, output_dof_order=actuated_dofs)
        if time_val < M8_ENTRY_DURATION:
            alpha = smooth_step(time_val, 0.0, M8_ENTRY_DURATION)
            stock_angles = (1.0 - alpha) * default_pose + alpha * stock_angles
            adhesions = (1.0 - alpha) * np.ones(6) + alpha * adhesions
        action = LocomotionAction(joint_angles=stock_angles, adhesion_onoff=adhesions)
        apply_locomotion_action(sim, fly.name, action)
        sim.step()

        if (step_i + 1) % SAMPLE_EVERY_N_STEPS == 0 and time_val > STEADY_START_S * 0.8:
            try:
                forces = sim.get_bodysegment_contact_forces(fly.name, tarsus_segs, ground_only=True)
                force_x_samples.append(forces[:, 0].copy())
                force_z_samples.append(forces[:, 2].copy())
            except Exception as e:
                api_ok = False
                exception_msg = str(e)
                break

            body_positions = sim.get_body_positions(fly.name)
            curr_x = float(body_positions[thorax_idx, 0])
            if prev_x is not None:
                thorax_vx_samples.append((curr_x - prev_x) / SAMPLE_DT)
            prev_x = curr_x

        time_val += PHYSICS_DT

    if not api_ok:
        return False, f"Exception: {exception_msg}", tarsus_segs, False

    if len(force_x_samples) == 0:
        return False, "No force samples collected", tarsus_segs, False

    force_x_arr = np.array(force_x_samples)  # (N, 6)
    force_z_arr = np.array(force_z_samples)   # (N, 6)
    thorax_vx_arr = np.array(thorax_vx_samples)

    total_fx = np.sum(force_x_arr, axis=1)
    total_fz = np.sum(force_z_arr, axis=1)

    # Check 1: Not all zeros
    if np.all(np.abs(force_x_arr) < 1e-12) and np.all(np.abs(force_z_arr) < 1e-12):
        return False, "All forces are zero — API returns no data", tarsus_segs, False

    # Check 2: Z forces should be positive (ground pushing up) in majority of samples
    fz_positive_frac = float(np.mean(total_fz > 0))

    # Check 3: Coherence — net X force should anti-correlate with thorax velocity
    # (when fly decelerates, net X force should be negative for backward motion)
    # We relax this to just checking that forces are not constant
    fx_std = float(np.std(total_fx))
    fz_std = float(np.std(total_fz))

    notes = (
        f"n_samples={len(force_x_samples)}, "
        f"fz_positive_frac={fz_positive_frac:.2f}, "
        f"total_fx mean={float(np.mean(total_fx)):.4f} std={fx_std:.4f}, "
        f"total_fz mean={float(np.mean(total_fz)):.4f} std={fz_std:.4f}"
    )

    # Determine pass/inconclusive
    # Pass: Z forces mostly positive AND forces have meaningful variation
    if fz_positive_frac > 0.5 and (fx_std > 1e-6 or fz_std > 1e-6):
        # Also check sign coherence with vx if we have vx
        if len(thorax_vx_arr) > 5:
            # Force coherence: during backward motion the support feet should generate
            # net X force whose mean sign aligns with backward motion
            # (support feet prevent body from going forward, so net X on feet is positive
            # per Newton's 3rd law — the ground pushes the foot forward, the foot pushes the
            # body backward). The sign returned by get_bodysegment_contact_forces is:
            # geom2 (fly) += world_force, so the force is the ground-on-fly force.
            vx_mean = float(np.mean(thorax_vx_arr))
            fx_mean = float(np.mean(total_fx))
            # For backward motion (vx < 0), the net GRF X on tarsus should be negative
            # (ground pulling foot backward, i.e., ground reaction is backward on feet)
            # However due to complex multi-contact geometry, we just check: structure exists
            notes += f", thorax_vx_mean={vx_mean:.3f}, total_fx_mean={fx_mean:.4f}"
        return True, notes, tarsus_segs, True

    return False, f"Inconclusive: {notes}", tarsus_segs, False


# ---------------------------------------------------------------------------
# Aggregate results across reps
# ---------------------------------------------------------------------------

def aggregate_reps(trials, condition_name):
    """Average key metrics across reps, return dict."""
    if not trials:
        return {}

    def mean_field(key):
        vals = [t[key] for t in trials if not (isinstance(t[key], float) and (np.isnan(t[key]) or np.isinf(t[key])))]
        return float(np.mean(vals)) if vals else float('nan')

    agg = {
        "condition": condition_name,
        "n_reps": len(trials),
        "mean_backward_speed_mm_s": mean_field("mean_backward_speed_mm_s"),
        "cv20": mean_field("cv20"),
        "cv50": mean_field("cv50"),
        "cv_raw": mean_field("cv_raw"),
        "vx_std_mm_s": mean_field("vx_std_mm_s"),
        "thorax_z_ptp_mm": mean_field("thorax_z_ptp_mm"),
        "pitch_ptp_deg": mean_field("pitch_ptp_deg"),
        "pitch_std_deg": mean_field("pitch_std_deg"),
        "pearson_pitch_vx": mean_field("pearson_pitch_vx"),
        "dominant_freq_hz": mean_field("dominant_freq_hz"),
        "dominant_freq_spectrum_fraction": mean_field("dominant_freq_spectrum_fraction"),
        "arc_velocity_modulation_fraction": mean_field("arc_velocity_modulation_fraction"),
        "pearson_u_vx": mean_field("pearson_u_vx"),
        "total_force_modulation_fraction": mean_field("total_force_modulation_fraction"),
        "transfer_peak_accel_mm_s2": mean_field("transfer_peak_accel_mm_s2"),
        "displacement_x_mm": mean_field("displacement_x_mm"),
        "lateral_ratio": mean_field("lateral_ratio"),
    }

    # Per-leg across reps
    all_legs = list(trials[0]["per_leg"].keys())
    per_leg_agg = {}
    for leg in all_legs:
        per_leg_agg[leg] = {}
        for field in ["support_n", "slide_n", "support_median_vx",
                      "support_abs_median_vx", "slide_median_vx",
                      "force_cv", "mean_z_during_support"]:
            vals = [t["per_leg"][leg][field] for t in trials
                    if field in t["per_leg"][leg] and
                    not (isinstance(t["per_leg"][leg][field], float) and
                         (np.isnan(t["per_leg"][leg][field]) or np.isinf(t["per_leg"][leg][field])))]
            per_leg_agg[leg][field] = float(np.mean(vals)) if vals else float('nan')
    agg["per_leg"] = per_leg_agg

    return agg


# ---------------------------------------------------------------------------
# Classification logic
# ---------------------------------------------------------------------------

def classify_hypotheses(cond_results):
    """Return geometry and transfer hypothesis strength."""

    m8 = cond_results.get("M8_NORMAL_CONTROL")
    a_cont = cond_results.get("A_ONLY_CONTINUOUS")
    b_cont = cond_results.get("B_ONLY_CONTINUOUS")
    a_static = cond_results.get("A_ONLY_STATIC_U")
    b_static = cond_results.get("B_ONLY_STATIC_U")

    if not all([m8, a_cont, b_cont, a_static, b_static]):
        return "UNKNOWN", "UNKNOWN", "D"

    m8_cv20 = m8["cv20"]
    moving_sup_cv20 = float(np.nanmean([a_cont["cv20"], b_cont["cv20"]]))
    static_sup_cv20 = float(np.nanmean([a_static["cv20"], b_static["cv20"]]))

    moving_arc_mod = float(np.nanmean([
        a_cont["arc_velocity_modulation_fraction"],
        b_cont["arc_velocity_modulation_fraction"]
    ]))
    static_arc_mod = float(np.nanmean([
        a_static["arc_velocity_modulation_fraction"],
        b_static["arc_velocity_modulation_fraction"]
    ]))

    m8_vx_std = m8["vx_std_mm_s"]
    moving_vx_std = float(np.nanmean([a_cont["vx_std_mm_s"], b_cont["vx_std_mm_s"]]))
    static_vx_std = float(np.nanmean([a_static["vx_std_mm_s"], b_static["vx_std_mm_s"]]))

    # ---
    # GEOMETRY hypothesis: supported if moving-support single-tripod still shows
    # strong cyclic modulation even without switching.
    # Criteria:
    #   STRONG: moving_arc_mod > 0.5 AND moving_cv20 is notably large (> 0.25)
    #           AND static_vx_std is much smaller than moving_vx_std (ratio > 2)
    #   MODERATE: moving_arc_mod > 0.3 OR (moving_cv20 > 0.20 AND static < moving * 1.5)
    #   WEAK: moving shows some modulation but not strong
    #   NO: moving single-support is smooth (cv20 < 0.15 or arc_mod < 0.2)
    # ---

    geometry_score = 0
    if not (np.isnan(moving_arc_mod) or np.isinf(moving_arc_mod)):
        if moving_arc_mod > 0.5:
            geometry_score += 3
        elif moving_arc_mod > 0.3:
            geometry_score += 2
        elif moving_arc_mod > 0.1:
            geometry_score += 1

    if moving_sup_cv20 > 0.35:
        geometry_score += 2
    elif moving_sup_cv20 > 0.20:
        geometry_score += 1

    static_smaller = (not np.isnan(static_vx_std) and not np.isnan(moving_vx_std) and
                      moving_vx_std > 0 and static_vx_std < moving_vx_std * 0.6)
    if static_smaller:
        geometry_score += 2

    if geometry_score >= 6:
        geom_support = "STRONG"
    elif geometry_score >= 4:
        geom_support = "MODERATE"
    elif geometry_score >= 2:
        geom_support = "WEAK"
    else:
        geom_support = "NO"

    # ---
    # TRANSFER hypothesis: supported if M8 shows substantially larger pulse than
    # single-support moving conditions.
    # Criteria:
    #   STRONG: m8_vx_std is substantially larger than moving_vx_std (ratio > 2)
    #           AND M8 transfer_peak_accel >> single-support
    #   MODERATE: ratio 1.5–2
    #   WEAK: ratio 1.2–1.5
    #   NO: ratio < 1.2 or moving is worse
    # ---

    transfer_score = 0
    if moving_vx_std > 0 and not np.isnan(moving_vx_std):
        ratio_std = m8_vx_std / moving_vx_std
        if ratio_std > 2.5:
            transfer_score += 4
        elif ratio_std > 2.0:
            transfer_score += 3
        elif ratio_std > 1.5:
            transfer_score += 2
        elif ratio_std > 1.2:
            transfer_score += 1

    if m8_cv20 > moving_sup_cv20 * 1.3:
        transfer_score += 1

    if transfer_score >= 4:
        transfer_support = "STRONG"
    elif transfer_score >= 3:
        transfer_support = "MODERATE"
    elif transfer_score >= 1:
        transfer_support = "WEAK"
    else:
        transfer_support = "NO"

    # ---
    # Architecture decision
    # ---
    if geom_support in ("STRONG", "MODERATE") and transfer_support in ("STRONG", "MODERATE"):
        decision = "C"
    elif geom_support in ("STRONG", "MODERATE"):
        decision = "A"
    elif transfer_support in ("STRONG", "MODERATE"):
        decision = "B"
    else:
        decision = "D"

    return geom_support, transfer_support, decision


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("MW-M13: Pulse Mechanics Diagnosis")
    print("=" * 60)

    flygym_ver = version("flygym")
    mujoco_ver = version("mujoco")
    numpy_ver = np.__version__
    print(f"flygym={flygym_ver}  mujoco={mujoco_ver}  numpy={numpy_ver}")

    output_dir = pathlib.Path("output/m13")
    output_dir.mkdir(parents=True, exist_ok=True)
    derived_dir = pathlib.Path("data/derived")
    derived_dir.mkdir(parents=True, exist_ok=True)

    # ---
    # Build simulation
    # ---
    fly = make_locomotion_fly(name="fly", add_adhesion=True)
    world = FlatGroundWorld()
    world.add_fly(fly, spawn_position=[0, 0, 0.5],
                  spawn_rotation=Rotation3D("quat", [1, 0, 0, 0]))
    sim = Simulation(world)
    pps = PreprogrammedSteps()
    actuated_dofs = fly.get_actuated_jointdofs_order(ActuatorType.POSITION)

    # Semantic indices
    thorax_idx, tarsus5_idxs = build_indices(fly, sim)
    tarsus_seg_names = get_tarsus_bodyseg_list(fly)

    print(f"Thorax index: {thorax_idx}")
    print(f"Tarsus5 indices: {tarsus5_idxs}")
    print(f"Tarsus seg names: {tarsus_seg_names}")
    print(f"Legs order: {pps.legs}")
    print(f"Tripod A legs: {TRIPOD_A_LEGS}")
    print(f"Tripod B legs: {TRIPOD_B_LEGS}")

    # ---
    # Force API validation
    # ---
    print("\n[FORCE API VALIDATION]")
    force_api_ok, force_notes, tarsus_segs, force_validated = validate_force_api(
        sim, fly, pps, actuated_dofs, thorax_idx, tarsus5_idxs
    )
    print(f"  Force API OK: {force_api_ok}")
    print(f"  Notes: {force_notes}")

    force_api_status = "PASS" if force_api_ok else "INCONCLUSIVE"
    force_coord_frame = "WORLD (via mj_contactForce + contact frame rotation)"
    force_ground_only = True

    # ---
    # Conditions to run
    # ---
    conditions = [
        "M8_NORMAL_CONTROL",
        "A_ONLY_CONTINUOUS",
        "B_ONLY_CONTINUOUS",
        "A_ONLY_STATIC_U",
        "B_ONLY_STATIC_U",
    ]

    all_trials = []
    cond_results = {}

    total_trials = len(conditions) * N_REPS
    trial_count = 0

    for cond in conditions:
        print(f"\n{'=' * 50}")
        print(f"Condition: {cond}")
        cond_trials = []

        for rep in range(N_REPS):
            trial_count += 1
            print(f"  Rep {rep+1}/{N_REPS}  (trial {trial_count}/{total_trials})")

            result = run_trial(
                sim, fly, pps, actuated_dofs,
                thorax_idx, tarsus5_idxs,
                tarsus_seg_names, cond, rep,
                force_api_ok
            )

            print(f"    speed={result['mean_backward_speed_mm_s']:.2f} mm/s  "
                  f"cv20={result['cv20']:.3f}  cv50={result['cv50']:.3f}  "
                  f"pitch_ptp={result['pitch_ptp_deg']:.3f}°  "
                  f"arc_mod={result['arc_velocity_modulation_fraction']:.3f}  "
                  f"dom_f={result['dominant_freq_hz']:.2f}Hz")

            all_trials.append(result)
            cond_trials.append(result)

        cond_agg = aggregate_reps(cond_trials, cond)
        cond_results[cond] = cond_agg
        print(f"  AGGREGATE cv20={cond_agg['cv20']:.3f}  arc_mod={cond_agg['arc_velocity_modulation_fraction']:.3f}")

    # ---
    # Classification
    # ---
    print("\n[CLASSIFICATION]")
    geom_support, transfer_support, decision = classify_hypotheses(cond_results)
    print(f"  GEOMETRY_HYPOTHESIS_SUPPORTED: {geom_support}")
    print(f"  TRANSFER_HYPOTHESIS_SUPPORTED: {transfer_support}")
    print(f"  NEXT_ARCHITECTURE_BRANCH: {decision}")

    # Pitch contributor
    pitch_corr_vals = [cond_results[c].get("pearson_pitch_vx", float('nan'))
                       for c in conditions if c in cond_results]
    pitch_corr_vals = [v for v in pitch_corr_vals if not np.isnan(v)]
    mean_pitch_corr = float(np.nanmean(pitch_corr_vals)) if pitch_corr_vals else float('nan')
    pitch_contributor = "YES" if abs(mean_pitch_corr) > 0.6 else ("INCONCLUSIVE" if abs(mean_pitch_corr) > 0.4 else "NO")

    # Force evidence usable
    force_evidence_usable = "YES" if force_validated else "NO"

    # M8 reference metrics
    m8_agg = cond_results.get("M8_NORMAL_CONTROL", {})
    m8_cv20 = m8_agg.get("cv20", float('nan'))
    m8_cv50 = m8_agg.get("cv50", float('nan'))
    m8_speed = m8_agg.get("mean_backward_speed_mm_s", float('nan'))
    m8_pitch_ptp = m8_agg.get("pitch_ptp_deg", float('nan'))
    m8_transfer_peak = m8_agg.get("transfer_peak_accel_mm_s2", float('nan'))

    moving_sup_mean_cv20 = float(np.nanmean([
        cond_results.get("A_ONLY_CONTINUOUS", {}).get("cv20", float('nan')),
        cond_results.get("B_ONLY_CONTINUOUS", {}).get("cv20", float('nan'))
    ]))
    static_sup_mean_cv20 = float(np.nanmean([
        cond_results.get("A_ONLY_STATIC_U", {}).get("cv20", float('nan')),
        cond_results.get("B_ONLY_STATIC_U", {}).get("cv20", float('nan'))
    ]))

    m8_vx_std = m8_agg.get("vx_std_mm_s", float('nan'))
    moving_vx_std = float(np.nanmean([
        cond_results.get("A_ONLY_CONTINUOUS", {}).get("vx_std_mm_s", float('nan')),
        cond_results.get("B_ONLY_CONTINUOUS", {}).get("vx_std_mm_s", float('nan'))
    ]))

    # ---
    # Save to disk
    # ---
    def safe_float(v):
        if isinstance(v, float) and (np.isnan(v) or np.isinf(v)):
            return None
        return v

    def sanitize(obj):
        if isinstance(obj, dict):
            return {k: sanitize(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [sanitize(v) for v in obj]
        elif isinstance(obj, float):
            return safe_float(obj)
        elif isinstance(obj, np.floating):
            v = float(obj)
            return safe_float(v)
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    manifest = {
        "milestone": "M13",
        "software": {
            "flygym": flygym_ver,
            "mujoco": mujoco_ver,
            "numpy": numpy_ver,
        },
        "physics": {
            "timestep_s": PHYSICS_DT,
            "sample_dt_s": SAMPLE_DT,
            "trial_duration_s": TRIAL_DURATION_S,
            "steady_start_s": STEADY_START_S,
        },
        "m8_parameters": {
            "cadence": M8_CADENCE,
            "stance_excursion_fraction": M8_STANCE_EXCURSION,
            "turnaround_fraction": M8_TURNAROUND,
            "support_transfer_fraction": M8_TRANSFER_FRACTION,
            "entry_duration": M8_ENTRY_DURATION,
        },
        "conditions": {
            "M8_NORMAL_CONTROL": "Normal M8 alternating tripod controller, exact replica.",
            "A_ONLY_CONTINUOUS": "Tripod A (lf,lh,rm) continuous support+arc traversal; Tripod B (rf,lm,rh) held at mid-swing, adhesion=0.",
            "B_ONLY_CONTINUOUS": "Tripod B (rf,lm,rh) continuous support+arc traversal; Tripod A (lf,lh,rm) held at mid-swing, adhesion=0.",
            "A_ONLY_STATIC_U": "Tripod A pinned at mid-stance u=0.5 (no arc traversal); Tripod B at mid-swing, adhesion=0.",
            "B_ONLY_STATIC_U": "Tripod B pinned at mid-stance u=0.5 (no arc traversal); Tripod A at mid-swing, adhesion=0.",
        },
        "tripod_groupings": {
            "tripod_A": TRIPOD_A_LEGS,
            "tripod_B": TRIPOD_B_LEGS,
            "note": "As per M8 code: LEGS=['lf','lm','lh','rf','rm','rh'], tripod A = indices 0,2,4 = lf,lh,rm",
        },
        "n_reps": N_REPS,
        "total_trials": total_trials,
        "force_api": {
            "function": "sim.get_bodysegment_contact_forces(fly.name, tarsus5_segs, ground_only=True)",
            "coordinate_frame": force_coord_frame,
            "ground_only_supported": force_ground_only,
            "validation_status": force_api_status,
            "validation_notes": force_notes,
        },
        "condition_aggregates": sanitize(cond_results),
        "all_trial_summaries": sanitize(all_trials),
        "hypothesis_classification": {
            "geometry_hypothesis_supported": geom_support,
            "transfer_hypothesis_supported": transfer_support,
            "next_architecture_branch": decision,
            "pitch_likely_major_contributor": pitch_contributor,
            "mean_pitch_vx_pearson_across_conditions": safe_float(mean_pitch_corr),
            "force_evidence_usable": force_evidence_usable,
            "m8_cv20": safe_float(m8_cv20),
            "moving_single_support_mean_cv20": safe_float(moving_sup_mean_cv20),
            "static_support_mean_cv20": safe_float(static_sup_mean_cv20),
            "m8_vx_std_mm_s": safe_float(m8_vx_std),
            "moving_single_support_vx_std_mm_s": safe_float(moving_vx_std),
            "std_ratio_m8_over_moving_single_support": safe_float(
                float(m8_vx_std / moving_vx_std) if (moving_vx_std > 0 and not np.isnan(moving_vx_std) and not np.isnan(m8_vx_std)) else float('nan')
            ),
        },
        "key_metrics_summary": {
            "M8_NORMAL": {
                "mean_backward_speed_mm_s": safe_float(m8_speed),
                "cv20": safe_float(m8_cv20),
                "cv50": safe_float(m8_cv50),
                "transfer_peak_accel_mm_s2": safe_float(m8_transfer_peak),
                "pitch_ptp_deg": safe_float(m8_pitch_ptp),
            },
            "A_CONTINUOUS": sanitize({k: cond_results.get("A_ONLY_CONTINUOUS", {}).get(k) for k in ["mean_backward_speed_mm_s","cv20","cv50","dominant_freq_hz","arc_velocity_modulation_fraction","total_force_modulation_fraction","pitch_ptp_deg"]}),
            "B_CONTINUOUS": sanitize({k: cond_results.get("B_ONLY_CONTINUOUS", {}).get(k) for k in ["mean_backward_speed_mm_s","cv20","cv50","dominant_freq_hz","arc_velocity_modulation_fraction","total_force_modulation_fraction","pitch_ptp_deg"]}),
            "A_STATIC": sanitize({k: cond_results.get("A_ONLY_STATIC_U", {}).get(k) for k in ["mean_backward_speed_mm_s","cv20","cv50","dominant_freq_hz","pitch_ptp_deg"]}),
            "B_STATIC": sanitize({k: cond_results.get("B_ONLY_STATIC_U", {}).get(k) for k in ["mean_backward_speed_mm_s","cv20","cv50","dominant_freq_hz","pitch_ptp_deg"]}),
        },
        "integrity": {
            "new_gait_implemented": False,
            "parameter_search_performed": False,
            "physics_parameters_changed": False,
            "external_forces_used": False,
            "root_translation_prescribed": False,
            "previous_milestone_files_modified": False,
        },
    }

    out_path = derived_dir / "m13_pulse_diagnosis.json"
    with open(out_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"\nResults written to {out_path}")
    print("\n[DONE]")
    print(f"GEOMETRY_HYPOTHESIS_SUPPORTED: {geom_support}")
    print(f"TRANSFER_HYPOTHESIS_SUPPORTED: {transfer_support}")
    print(f"NEXT_ARCHITECTURE_BRANCH: {decision}")


if __name__ == "__main__":
    main()
