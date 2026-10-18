# MW-M30: Minimal Tripod-Transfer Conflict Intervention Report

## 1. Executive Summary

| Evaluation Domain | Metric | Condition A (EXACT M27-D) | Condition B (Transfer Relief) | Delta (B − A) | Status / Gate |
|---|---|---|---|---|---|
| **Reproduction (A)** | Backward Speed (mm/s) | 7.87 | — | — | **PASS** (target 7.87) |
| | CV50 Smoothness | 0.3276 | — | — | **PASS** (target 0.3276) |
| | Wrong-Sign 20 Fraction | 9.14% | — | — | **PASS** (target 9.14%) |
| | Pitch PTP (deg) | 6.51° | — | — | **PASS** (target 6.51°) |
| | Roll PTP (deg) | 9.17° | — | — | **PASS** (target 9.17°) |
| | Thorax Z PTP (mm) | 0.1033 | — | — | **PASS** (target 0.103) |
| | All-3 Support Contact | 94.46% | — | — | **PASS** (target 94.46%) |
| | Lateral Drift Ratio | 0.0358 | — | — | **PASS** (target 0.036) |
| **Locomotion Quality (B)** | Backward Speed (mm/s) | 7.87 | 7.91 | +0.04 | **PASS** ($\ge 7.50$) |
| | CV50 Smoothness | 0.3276 | 0.3221 | −0.0055 | Modest change (−1.7%) |
| | Wrong-Sign 20 Fraction | 9.14% | 9.28% | +0.14% | Unchanged |
| | Pitch PTP (deg) | 6.51° | 6.79° | +0.28° | **PASS** ($\le 7.20°$) |
| | Roll PTP (deg) | 9.17° | 9.16° | −0.01° | **PASS** ($\le 11.0°$) |
| | Thorax Z PTP (mm) | 0.1033 | 0.1079 | +0.0047 | **PASS** ($\le 0.130$) |
| | All-3 Support Contact | 94.46% | 94.16% | −0.30% | **PASS** ($\ge 88.0%$) |
| | Lateral Drift Ratio | **0.0358** | **0.1195** | **+0.0837** | **FAIL** ($\le 0.0412$) |
| **Transfer Dynamics** | Transfer Cancellation Peak | **0.9550** | **0.9264** | **−0.0286** | **−3.0% rel** (fails $\ge 10\%$) |
| | Peak Timing relative to Dip | −14.0 ms | −14.0 ms | 0.0 ms | Exactly synchronized |
| | ALL3 Median Cancellation | 0.4494 | 0.4490 | −0.0004 | Preserved |
| **Intervention Work** | Injected Work ($W_{\text{pos}}$) | 0.0 uJ | 2.4475 uJ | +2.4475 uJ | Active energy input |
| | Dissipated Work ($W_{\text{neg}}$)| 0.0 uJ | 0.4278 uJ | +0.4278 uJ | Low dissipation |
| | Net Work ($W_{\text{net}}$) | 0.0 uJ | +2.0197 uJ | +2.0197 uJ | Net positive work |
| | Work Ratio ($W_{\text{pos}}/W_{\text{neg}}$)| — | 5.7210 | — | **FAIL** ($\le 0.05$) |
| **Overall Classification** | Primary Verdict | — | — | — | **`CHOREOGRAPHY_PRESERVATION_FAILED`** |
| | Qualitative Review | — | — | — | **`PENDING_MANAGER_REVIEW`** |

---

## 2. Scientific Findings and Core Conclusions

MW-M30 was designed to test the leading hypothesis that the periodic backward glide hesitation in M27-D (~83.3 ms interval, wrong-sign fraction 9.14%) is causally driven by the force-cancellation spike during the tripod transfer handoff window, and that temporarily relaxing the outgoing support tripod's World-X target toward a stationary floor anchor (`TRANSFER_PHASE_WORLD_X_ANCHOR`) would relieve this conflict.

The experimental outcome provides **three definitive findings**:

1. **Failure to Relieve Transfer Force Cancellation**:
   The transfer force-cancellation peak decreased by only **3.0%** (from 0.9550 to 0.9264), falling far short of the pre-registered threshold for meaningful relief ($\ge 10\%$, target $\le 0.72$). The hesitation event remained essentially identical: CV50 barely moved from 0.3276 to 0.3221 (−1.7% relative), and wrong-sign 20 motion remained at 9.28% (vs 9.14% in A).

2. **Severe Lateral Drift Degradation**:
   Condition B suffered a **+234% relative surge in lateral drift ratio** (0.0358 $\to$ 0.1195), violating the pre-registered choreography preservation ceiling ($\le 1.15 \times \text{fresh A} = 0.0412$) by nearly a factor of three. Modifying the longitudinal kinematics during dual support altered the bilateral yaw moment and lateral force balance, resulting in visible side-to-side snaking. This formally triggers the pre-registered failure classification **`CHOREOGRAPHY_PRESERVATION_FAILED`**.

3. **Active Work Injection**:
   The intervention injected net positive work ($W_{\text{pos}} = 2.45\ \mu\text{J}$, $W_{\text{net}} = +2.02\ \mu\text{J}$, ratio $5.72$). As the thorax retreated backward at ~19 mm/s while the tarsus was commanded toward an anchor forward of the nominal retracting position, the joint position servos pushed forward against the retreating thorax, adding mechanical energy rather than acting passively.

---

## 3. Telemetry & Biomechanical Analysis

### A. Transfer Event Hesitation Profile
Event-triggered dynamics aligned to rolling-window speed minima (17 detected events in steady-state) confirm the exact timing of the conflict:

```
Condition A (EXACT M27-D):
- Body backward speed plunges from +14.5 mm/s to -2.36 mm/s at t = 0 ms.
- Whole-body 6-leg force cancellation peaks at 0.9550 exactly 14 ms prior to the speed dip (t = -14.0 ms).

Condition B (Transfer Relief):
- Body backward speed plunges from +14.6 mm/s to -2.31 mm/s at t = 0 ms.
- Whole-body 6-leg force cancellation peaks at 0.9264 exactly 14 ms prior to the speed dip (t = -14.0 ms).
```

The peak force cancellation remains extremely severe (~0.93) and occurs at the exact same point in the gait cycle.

### B. Per-Leg Ground Reaction Forces (Steady-State Strong Support)
Measured median world forces ($F_x, F_z$) and tarsal slip velocities:

| Leg | Tripod | Median $F_x$ (A, mN) | Median $F_x$ (B, mN) | Median $F_z$ (A, mN) | Median $F_z$ (B, mN) | Median Slip $v_x$ (A, mm/s) | Median Slip $v_x$ (B, mm/s) |
|---|---|---|---|---|---|---|---|
| **LF** | A (Front)  | −4.43 | −4.37 | 42.48 | 42.46 | −0.37 | −0.34 |
| **LM** | B (Middle) | +1.71 | +1.70 | 42.40 | 42.45 | −0.63 | −0.59 |
| **LH** | A (Hind)   | −2.06 | −2.07 | 40.17 | 40.16 | −0.13 | −0.12 |
| **RF** | B (Front)  | −4.21 | −4.28 | 42.46 | 42.39 | −0.34 | −0.33 |
| **RM** | A (Middle) | +1.90 | +1.90 | 42.28 | 42.34 | −0.61 | −0.61 |
| **RH** | B (Hind)   | −2.14 | −2.09 | 40.23 | 40.27 | −0.16 | −0.16 |

Outside the transfer handoff, the steady-state strong-support force structure of Condition B is completely identical to Condition A, confirming that the intervention remained surgically confined to the transfer window as designed.

### C. Why Transfer Anchoring Failed to Relieve Cancellation
1. **Compliance and PD Servo Mechanics**:
   When the outgoing leg's target is anchored in World X during unload, DLS IK commands joint angles that resist the backward retreat of the thorax. However, because the fly's body has substantial backward momentum and the leg joints are actuated by PD position controllers, the leg still exerts substantial contact forces against the ground.
2. **Dual-Support Redundancy**:
   During dual support, both tripods are simultaneously in contact with the ground (4.5 to 5.0 grounded legs on average). Relieving only the outgoing leg's World-X command while the incoming leg touches down and ramps up load does not eliminate the fundamental dynamic opposition between anterior and posterior leg groups.
3. **Lateral Coupling**:
   The fly's legs are mounted at substantial lateral offsets from the centerline. Changing the longitudinal target trajectory on one side during handoff creates an asymmetric longitudinal force differential that translates into a yaw moment, directly exciting lateral oscillation.

---

## 4. Work Accounting & Energetics

Using the exact finite-difference derivative of the intervention relative to state-matched M27-D commands:
$$v_{\text{relief}, i}[k] = \frac{\Delta x_{\text{relief}, i}[k] - \Delta x_{\text{relief}, i}[k-1]}{\Delta t}$$
$$P_{\text{relief}, i}[k] = - F_{x, i}[k] \cdot v_{\text{relief}, i}[k]$$

- **Total Injected Work ($W_{\text{pos}}$)**: $2.4475\ \mu\text{J}$
- **Total Dissipated Work ($W_{\text{neg}}$)**: $0.4278\ \mu\text{J}$
- **Net Work ($W_{\text{net}}$)**: $+2.0197\ \mu\text{J}$
- **Work Ratio**: $5.7210$ (Safety Gate: **FAIL**)

Because $W_{\text{pos}} \gg W_{\text{neg}}$, the intervention acts as an active energy injector rather than a passive compliance element. This provides a direct physical explanation for the lateral destabilization: the added mechanical energy was channeled into parasitic yaw and lateral body motions.

---

## 5. Artifact Provenance & Output Files

- **Simulation Script**: [`scripts/m30_tripod_transfer_relief.py`](../scripts/m30_tripod_transfer_relief.py)
- **Manifest JSON**: [`data/derived/m30_tripod_transfer_manifest.json`](../data/derived/m30_tripod_transfer_manifest.json)
- **Results JSON**: [`data/derived/m30_tripod_transfer_results.json`](../data/derived/m30_tripod_transfer_results.json)
- **Diagnostic Plots**:
  - Velocity Profiles: [`output/m30/velocity_profiles.png`](../output/m30/velocity_profiles.png)
  - Event Hesitation: [`output/m30/event_triggered_hesitation.png`](../output/m30/event_triggered_hesitation.png)
- **Rendered Videos** (`output/m30/`):
  - Side Tracking 1.0x: [`video_A_vs_B_10x.mp4`](../output/m30/video_A_vs_B_10x.mp4)
  - Side Tracking 0.2x: [`video_A_vs_B_02x.mp4`](../output/m30/video_A_vs_B_02x.mp4)
  - Top-Down 1.0x: [`video_A_vs_B_topdown_10x.mp4`](../output/m30/video_A_vs_B_topdown_10x.mp4)
  - Top-Down 0.2x: [`video_A_vs_B_topdown_02x.mp4`](../output/m30/video_A_vs_B_topdown_02x.mp4)
  - Individual condition videos: `video_A_m27d_side_10x.mp4`, `video_A_m27d_side_02x.mp4`, `video_B_transfer_relief_side_10x.mp4`, `video_B_transfer_relief_side_02x.mp4`, `video_A_m27d_top_10x.mp4`, `video_A_m27d_top_02x.mp4`, `video_B_transfer_relief_top_10x.mp4`, `video_B_transfer_relief_top_02x.mp4`.

---

## 6. Strategic Takeaway for Project Moonwalker Fly

MW-M30 provides a clear, rigorous answer to the tripod-transfer conflict hypothesis:
1. **Transfer World-X target relaxation does NOT cure the periodic glide hesitation.** The force-cancellation peak remained at 0.9264, and the wrong-sign motion was completely unaffected (9.28% vs 9.14%).
2. **Kinematic target modifications during transfer degrade lateral stability.** The lateral ratio jumped from 0.0358 to 0.1195 (+234%), introducing snaking drift.
3. **M27-D remains the uncontested visual and dynamic champion.** Condition A reproduced all authoritative metrics with zero deviation. Any further intervention should treat M27-D's lateral stability and bilateral coordination as fragile assets that open-loop transfer interventions readily disrupt.
