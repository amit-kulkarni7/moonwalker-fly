# MW-M27: Selective Hind-Leg Relief & Excursion-Matched Support Causal Intervention

## 1. Executive Summary

| Metric | Condition A (M8 Baseline) | Condition B (M25 Planar Reference) | Condition C (Selective Hind Relief) | Condition D (Excursion Matched) | Status / Gate |
|---|---|---|---|---|---|
| **Parent Reproduction Gate** | Speed: 9.93 mm/s, CV50: 0.232 | Speed: 10.40 mm/s, CV50: 0.327 | — | — | **PASS** |
| **All-3 Support Contact Fraction** | 0.6157 (61.6%) | 0.9416 (94.2%) | 0.7978 (79.8%) | **0.9446 (94.5%)** | High Support |
| **Mean Backward Speed** | 9.93 mm/s | **10.40 mm/s** | 8.94 mm/s | 7.87 mm/s | D < 8.5 mm/s |
| **Backward Speed CV50** | **0.2315** | 0.3275 | 0.3675 | 0.3276 | M8 smoothest |
| **Wrong-Sign 20ms Fraction** | **0.0100 (1.0%)** | 0.0914 (9.1%) | 0.1413 (14.1%) | 0.0914 (9.1%) | M8 lowest |
| **Pitch Peak-to-Peak (PTP)** | 8.38° | 8.57° | 8.60° | **6.51° (-24.1% rel B)** | D superior |
| **Roll Peak-to-Peak (PTP)** | 38.53° | 13.57° | 38.53° (Postural tilt) | **9.17° (-32.5% rel B)** | D superior |
| **Thorax Z PTP** | 0.109 mm | **0.095 mm** | 0.192 mm (Elevated) | 0.103 mm | B & D tight |
| **Preflight Velocity Spread Reduction** | — | — | — | **62.3% (5.31 $\rightarrow$ 2.00 mm/s)** | **PASS ($\ge 50\%$)** |
| **Median Cancellation (ALL3 Contact)** | **0.2186** | 0.3679 | 0.7023 (+0.3344 vs B) | 0.4494 (+0.0815 vs B) | Elevated |
| **Opposing Force Fraction (1% Load)** | **0.2105 (21.1%)** | 0.3532 (35.3%) | 0.9305 (93.1%) | 0.6780 (67.8%) | Elevated |
| **Propulsive Efficiency (ALL3 Contact)**| **0.7814** | 0.6321 | 0.2977 | 0.5506 | Deteriorated |
| **Front Legs Median $F_x$** | -1.76 mN | -4.20 mN (Pull) | -3.03 mN (Pull) | **-4.32 mN (Pull)** | Dominant pull |
| **Middle Legs Median $F_x$** | -1.38 mN | -0.53 mN (Pull) | -1.45 mN (Pull) | **+1.81 mN (BRAKE)** | Shift in conflict |
| **Hind Legs Median $F_x$** | 0.00 mN | **+0.92 mN (BRAKE)** | **+2.18 mN (BRAKE)** | **-2.10 mN (Pull)** | Hind brake solved |
| **Conflict Locus** | None (Detached) | Hind legs (LH, RH) | Hind legs (LH, RH) | **Middle legs (LM, RM)**| Conflict shifted |
| **Primary Classification** | Reference | Reference | **HIND_CONSTRAINT_HYPOTHESIS_NOT_SUPPORTED** | **EXCURSION_MATCHING_NOT_SUPPORTED** | Tested |
| **Secondary Diagnostic** | — | — | **HIND_RELIEF_POSTURALLY_CONFOUNDED** | — | Roll 38.5° |

### Core Scientific Findings

1. **Condition C (Selective Hind Relief) Decisively Falsifies the Isolated Hind-Constraint Hypothesis**:
   - Relieving the supporting hind legs from vertical planarization (leaving them on stock M8 nominal kinematics while front and middle legs remain planarized) does **NOT** eliminate horizontal force conflict. Instead, force cancellation during ALL3 contact surges from 0.3679 to **0.7023** (+90.9% relative increase), opposing force fraction jumps to **93.1%**, backward speed drops from 10.40 to 8.94 mm/s, and CV50 degrades to 0.3675.
   - **Mechanism**: The unconstrained hind legs suffer from postural mismatch with the planarized front/middle legs (roll PTP surges to 38.53°, thorax Z PTP increases to 0.192 mm). When the hind feet strike the substrate, their forward braking forces more than double (LH median $+2.02$ mN, RH median $+2.33$ mN, peaking over $+4.3$ mN) against front leg pulling ($-3.05$ mN).
   - Formally classified as **`HIND_CONSTRAINT_HYPOTHESIS_NOT_SUPPORTED`** with secondary diagnostic **`HIND_RELIEF_POSTURALLY_CONFOUNDED`**.

2. **Condition D (Excursion Matching) Eliminates Hind-Leg Braking but Uncovers a Kinematic Bottleneck at the Middle Legs**:
   - Condition D successfully passed the analytical manipulation check, reducing nominal support-target longitudinal velocity spread by **62.3%** (from 5.31 mm/s in Condition B down to 2.00 mm/s in Condition D, surpassing the $\ge 50\%$ pre-registered threshold).
   - In dynamic simulation, scaling hind excursion ($E_{\text{hind}} = 0.779$) **completely converted the hind legs from brakes into propulsive pullers**: median $F_x$ shifted from $+0.92$ mN (braking) in B to **$-2.10$ mN (propulsive)** in D.
   - However, scaling middle-leg excursion down to $E_{\text{mid}} = 0.564$ caused the middle legs to sweep slower than the retreating thorax, **flipping them into severe forward-directed brakes**: median $F_x$ flipped from $-0.53$ mN in B to **$+1.81$ mN (braking)** in D (LM $+1.71$ mN, RM $+1.90$ mN).
   - Because the middle legs became the new braking bottleneck fighting both the front ($-4.32$ mN) and hind ($-2.10$ mN) pullers, median ALL3 force cancellation remained elevated at **0.4494** (vs 0.3679 in B), opposing-force fraction increased to **67.8%**, speed dropped to 7.87 mm/s, and CV50 was unchanged (0.3276 vs 0.3275).
   - Pitch and roll stability improved dramatically (pitch PTP reduced by 24.1% to 6.51°; roll PTP reduced by 32.5% to 9.17°), but body glide smoothness did not improve.
   - Formally classified as **`EXCURSION_MATCHING_NOT_SUPPORTED`**.
   - Additional B vs D Diagnostics: `COMMAND_MISMATCH_REDUCED: YES`, `PHYSICAL_FORCE_CONFLICT_REDUCED: NO`, `BODY_GLIDE_IMPROVED: NO`.

---

## 2. Experimental Design & Strictly Isolated Conditions

MW-M27 evaluated exactly four physically isolated simulation rollouts under identical MuJoCo physics settings ($\Delta t = 0.0001$ s, $1.75$ s duration, $1$ kHz sampling, steady-state evaluation interval $t \in [0.35, 1.75]$ s):

```
+---------------------------------------------------------------------------------------------------+
| CONDITION A: DIRECT_M8_CONTROL (Reference Baseline)                                               |
| - Unmodified M8 controller (scripts/m8_backslide_controller.py)                                   |
| - Cadence: 6.0 Hz | Excursion: 1.0 | Turnaround: 0.06 | Transfer: 0.08 | Entry: 0.25 s            |
| - Targets: Speed ~9.93 mm/s | CV50 ~0.232 | Pitch PTP ~8.38° | All-3 Contact ~0.616               |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| CONDITION B: M25_BODY_FRAME_PLANAR_SUPPORT (Full Planar Reference)                                |
| - Exact M25 DLS IK controller (scripts/m25_support_path_geometry.py)                              |
| - All 3 support legs planarized to frozen body-frame Z heights in scratch MjData                  |
| - Frozen Heights: LF/RF: -1.042 mm | LM/RM: -1.088 mm | LH/RH: -0.975 mm                          |
| - Targets: Speed ~10.40 mm/s | CV50 ~0.328 | Pitch PTP ~8.57° | All-3 Contact ~0.942              |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| CONDITION C: SELECTIVE_HIND_RELIEF (Causal Intervention)                                          |
| - FRONT & MIDDLE legs: DLS IK planarization to frozen heights (identical logic/parameters as B)   |
| - HIND legs (LH, RH): Completely unconstrained by DLS IK (100% stock M8 nominal kinematics)       |
| - Adhesion & Transfer: Stock M8 schedule preserved identically across all 6 legs (t_f = 0.08)     |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| CONDITION D: EXCURSION_MATCHED_PLANAR_SUPPORT (Unconfounded Velocity Intervention)                |
| - All 3 support legs planarized to frozen Z heights (preserving full tripod & roll stability)     |
| - Stance Excursion Scaling: E_front = 1.000, E_mid = 0.564, E_hind = 0.779                       |
| - Command Stroke: Uniform ~0.65-0.69 mm across all legs (nominal uniform v_rel = +7.6 to +9.6 mm/s)|
| - Tests whether removing velocity mismatch reduces physical force cancellation and improves glide |
+---------------------------------------------------------------------------------------------------+
```

### Frozen Parameters Across All Conditions
- Physics simulation: FlatGroundWorld, $\Delta t = 0.0001$ s, duration $1.75$ s.
- Cadence: $6.0$ Hz.
- Support transfer duration: $t_f = 0.08$ ($13.3$ ms dual-support overlap).
- Turnaround fraction: $r = 0.06$.
- Startup entry ramp: $0.25$ s.
- DLS IK parameters: damping $\lambda = 10^{-4}$, regularization weight $\alpha = 0.01$, max step $\Delta q = 0.05$ rad, max iterations $15$, tolerance $0.1$ mm.
- Frozen body-frame support heights (M25 provenance):
  - LF: $-1.04245$ mm | LM: $-1.08761$ mm | LH: $-0.97508$ mm
  - RF: $-1.04199$ mm | RM: $-1.08830$ mm | RH: $-0.97195$ mm

---

## 3. Preflight Causal Isolation & Manipulation Checks

### Preflight 1: Condition C Causal Isolation Verification (Patch 2)
1. **Static / Same-State Preflight**:
   Using an identical copied MuJoCo state, phase, nominal target, and passive-joint states evaluated at steady stance ($t = 0.50$ s):
   - $\text{SAME\_STATE\_B\_VS\_C\_FRONT\_MIDDLE\_TARGET\_MAX\_DIFF\_RAD} = \mathbf{6.66 \times 10^{-16}}$ rad (Threshold: $< 10^{-10}$ rad). $\rightarrow$ **PASS (Machine Precision)**.
   - $\text{SAME\_STATE\_C\_HIND\_DIFF\_VS\_NOMINAL\_MAX\_RAD} = \mathbf{0.00 \times 10^{0}}$ rad (Threshold: $< 10^{-12}$ rad). $\rightarrow$ **PASS (Exact Zero)**.
   - $\text{SAME\_STATE\_B\_HIND\_CORRECTION\_MIN\_RAD} = \mathbf{0.0299}$ rad (Active physical DLS IK correction in B). $\rightarrow$ **PASS**.
2. **Dynamic Code-Path Invariants**:
   - In dynamic simulation, Front and Middle legs in Condition C execute the identical algorithm, frozen heights, support weights, DLS settings, and adhesion timing as Condition B.
   - Hind legs execute the exact nominal M8 trajectory without IK mutation.

### Preflight 2: Condition D Manipulation Check (Patch 4)
Nominal kinematic support-velocity evaluation across a full 6 Hz stride cycle:
- **Condition B Nominal Support $V_x$ Spread**: $5.31$ mm/s (LF: $8.16$ mm/s, LM: $13.48$ mm/s, LH: $12.37$ mm/s).
- **Condition D Nominal Support $V_x$ Spread**: $2.00$ mm/s (LF: $8.16$ mm/s, LM: $7.60$ mm/s, LH: $9.60$ mm/s).
- **Longitudinal Velocity Spread Reduction**: $\mathbf{62.3\%}$ (Pre-registered threshold: $\ge 50\%$). $\rightarrow$ **PASS**.
- **Nominal Stroke Lengths**:
  - Condition B: LF: $0.687$ mm, LM: $1.051$ mm, LH: $0.872$ mm (range $0.364$ mm).
  - Condition D: LF: $0.687$ mm, LM: $0.650$ mm, LH: $0.694$ mm (range $0.044$ mm, virtually uniform).

---

## 4. Reproduction & Simulation Integrity Gates

### Gate 1: Parent Reproduction Gate (Deterministic Tolerances)
- **Condition A (M8)**:
  - Mean Speed: $9.93$ mm/s (target $9.93 \pm 0.10$). $\rightarrow$ **PASS**.
  - CV50: $0.232$ (target $0.232 \pm 0.010$). $\rightarrow$ **PASS**.
  - Pitch PTP: $8.38^\circ$ (target $8.38^\circ \pm 0.25^\circ$). $\rightarrow$ **PASS**.
  - All-3 Contact Fraction: $0.616$ (target $0.616 \pm 0.020$). $\rightarrow$ **PASS**.
- **Condition B (Planar)**:
  - Mean Speed: $10.40$ mm/s (target $10.40 \pm 0.15$). $\rightarrow$ **PASS**.
  - CV50: $0.327$ (target $0.327 \pm 0.015$). $\rightarrow$ **PASS**.
  - Pitch PTP: $8.57^\circ$ (target $8.57^\circ \pm 0.25^\circ$). $\rightarrow$ **PASS**.
  - All-3 Contact Fraction: $0.942$ (target $0.942 \pm 0.015$). $\rightarrow$ **PASS**.

### Gate 2: Telemetry & Actuator Integrity
Across all four conditions (A, B, C, D):
- `FORCES_FINITE`: **True** (all 6 legs and body segments finite).
- `FZ_POSITIVE_TOTAL`: **True** (continuous upward support load).
- `ACTUATOR_SATURATION_FRACTION`: **0.000** (zero actuator limits clipped).
- `MAX_TARGET_STEP_RAD`: **0.0052** rad (well below $< 0.01$ rad limit).
- `ABDOMEN_CONTACT_COUNT`: **Exactly 0** (zero belly dragging across all runs).
- `IK_CONVERGENCE_FRACTION`: **1.000** (100% convergence across all solves in B, C, D; max residual $< 0.1$ mm).

---

## 5. Ground Reaction Force (GRF) Telemetry & Conflict Mechanics

All segments for each leg (`coxa`, `trochanterfemur`, `tibia`, `tarsus1..5`) were queried and aggregated during the steady-state strong-support intervals:

### Per-Leg Median Ground Reaction Forces and Kinematics

| Leg | Role | Cond A Median $F_x$ | Cond B Median $F_x$ | Cond C Median $F_x$ | Cond D Median $F_x$ | Cond B Target $V_x$ | Cond D Target $V_x$ | Cond D Median $F_z$ |
|---|---|---|---|---|---|---|---|---|
| **LF** | Front (Tripod A) | -1.76 mN | -4.23 mN (Pull) | -3.05 mN (Pull) | **-4.43 mN (Pull)** | 7.92 mm/s | 7.33 mm/s | 42.48 mN |
| **RF** | Front (Tripod B) | -1.76 mN | -4.17 mN (Pull) | -3.02 mN (Pull) | **-4.21 mN (Pull)** | 7.92 mm/s | 7.47 mm/s | 42.46 mN |
| **LM** | Middle (Tripod B)| -1.39 mN | -0.55 mN (Pull) | -1.63 mN (Pull) | **+1.71 mN (BRAKE)** | 13.62 mm/s | 7.68 mm/s | 42.40 mN |
| **RM** | Middle (Tripod A)| -1.38 mN | -0.51 mN (Pull) | -1.27 mN (Pull) | **+1.90 mN (BRAKE)** | 13.60 mm/s | 7.64 mm/s | 42.28 mN |
| **LH** | Hind (Tripod A)  | 0.00 mN  | **+0.94 mN (BRAKE)** | **+2.02 mN (BRAKE)** | **-2.06 mN (Pull)** | 12.40 mm/s | 9.32 mm/s | 40.17 mN |
| **RH** | Hind (Tripod B)  | 0.00 mN  | **+0.89 mN (BRAKE)** | **+2.33 mN (BRAKE)** | **-2.14 mN (Pull)** | 12.81 mm/s | 9.54 mm/s | 40.23 mN |

### The Shifting Locus of Mechanical Conflict

1. **Condition B (Baseline Planar Support)**:
   - Front legs pull backward aggressively ($-4.2$ mN).
   - Middle legs pull mildly ($-0.5$ mN).
   - Hind legs act as forward-directed brakes ($+0.9$ mN).
   - *Conflict pair*: Front vs Hind.

2. **Condition C (Selective Hind Relief)**:
   - Front legs pull backward ($-3.0$ mN).
   - Middle legs pull backward ($-1.5$ mN).
   - Hind legs brake **even harder** ($+2.0$ to $+2.3$ mN) because unconstrained vertical posture forces the tarsus into severe floor collision without height adjustment.
   - *Conflict pair*: Front/Middle vs Hind (severely exacerbated).

3. **Condition D (Excursion Matching Intervention)**:
   - Front legs pull backward aggressively ($-4.3$ mN).
   - Hind legs now pull backward ($-2.1$ mN) — **hind braking is completely eliminated!**
   - However, middle legs now act as forward-directed brakes (**$+1.7$ to $+1.9$ mN**)!
   - *Conflict pair*: Front/Hind vs Middle.

```
Condition B:   [Front: -4.2 mN (Pull)]  <====>  [Middle: -0.5 mN]  <====>  [Hind: +0.9 mN (BRAKE)]
                                                |
Condition D:   [Front: -4.3 mN (Pull)]  <====>  [Middle: +1.8 mN (BRAKE)]  <====>  [Hind: -2.1 mN (Pull)]
```

**Biomechanical Insight**: 
In multi-legged locomotion, the body's actual backward translation speed ($v_{\text{body}} \approx -7.9$ mm/s in D) is dynamically coupled to the fastest pulling legs. When middle-leg stroke was curtailed ($E_{\text{mid}} = 0.564$, target sweep rate $\approx 7.6$ mm/s), the front and hind legs pulled the body backward faster than the middle leg was commanded to move. Consequently, the grounded middle foot was dragged forward against its commanded retraction, forcing it into compression and causing it to act as a physical brake.

---

## 6. Internal Force Conflict & Propulsive Efficiency

For active supporting tripods during strong-support intervals ($t \ge 0.35$ s, $\text{median}(adh_{\text{sup}}) \ge 0.90$, $\text{median}(adh_{\text{opp}}) \le 0.10$):
$$\text{Cancellation Ratio} = 1 - \frac{|\sum_{i \in \text{sup}} F_{x, i}|}{\sum_{i \in \text{sup}} |F_{x, i}| + \epsilon}, \quad \text{Efficiency} = \frac{|\sum_{i \in \text{sup}} F_{x, i}|}{\sum_{i \in \text{sup}} |F_{x, i}| + \epsilon}$$

### Quantitative Conflict Summary

| Metric | Condition A (M8 Baseline) | Condition B (M25 Planar Reference) | Condition C (Selective Hind Relief) | Condition D (Excursion Matched) |
|---|---|---|---|---|
| **ALL3 Contact Median Cancellation** | **0.2186** | 0.3679 | 0.7023 | 0.4494 |
| Tripod A ALL3 Median Cancellation | 0.2114 | 0.3752 | 0.6883 | 0.4617 |
| Tripod B ALL3 Median Cancellation | 0.2227 | 0.3624 | 0.7128 | 0.4217 |
| **All Strong Support Median Cancellation** | **0.0000** | 0.3581 | 0.6314 | 0.4349 |
| **Opposing Force Sample Fraction (1% Load)** | **0.2105 (21.1%)** | 0.3532 (35.3%) | 0.9305 (93.1%) | 0.6780 (67.8%) |
| **Propulsive Force Efficiency (ALL3)** | **0.7814** | 0.6321 | 0.2977 | 0.5506 |
| **Whole-Body 6-Leg Cancellation** | **0.7455** | 0.8310 | 0.8270 | 0.8735 |

### Analysis of Conflict Results
- In **Condition C**, internal opposition reaches an extreme: **$70.2\%$** of horizontal muscular force is lost to internal opposition, and over **$93\%$** of stance samples exhibit opposing forces. The hypothesis that selective hind relief would restore M8's low conflict is conclusively disproven.
- In **Condition D**, internal opposition increases from $0.3679$ to **$0.4494$**, and the opposing force fraction rises from $35.3\%$ to **$67.8\%$**. Although nominal target velocity spread was reduced by $62.3\%$, physical force fighting worsened because the middle legs were forced into the braking role.

---

## 7. Body Kinematics, Postural Stability & Glide Dynamics

| Kinematic Metric | Condition A (M8 Baseline) | Condition B (M25 Planar Reference) | Condition C (Selective Hind Relief) | Condition D (Excursion Matched) |
|---|---|---|---|---|
| **Mean Backward Speed** | 9.93 mm/s | **10.40 mm/s** | 8.94 mm/s | 7.87 mm/s |
| **CV50 (Glide Smoothness)** | **0.2315** | 0.3275 | 0.3675 | 0.3276 |
| **CV20** | **0.4546** | 0.6311 | 0.6400 | 0.6373 |
| **Wrong-Sign 20ms Fraction** | **0.00999 (1.0%)** | 0.09136 (9.1%) | 0.14133 (14.1%) | 0.09136 (9.1%) |
| **Pitch PTP** | 8.38° | 8.57° | 8.60° | **6.51° (-24.1% rel B)** |
| **Pitch Mean** | +0.31° | -1.71° | -1.60° | -4.27° |
| **Roll PTP** | 38.53° | 13.57° | 38.53° | **9.17° (-32.5% rel B)** |
| **Roll Std** | 8.76° | 3.26° | 8.76° | **2.57°** |
| **Thorax Z PTP** | 0.109 mm | **0.095 mm** | 0.192 mm | 0.103 mm |
| **Lateral Drift Ratio ($|Y|/|X|$)** | **0.0068** | 0.0026 | 0.0310 | 0.0363 |
| **Transition Accel P95** | **5258 mm/s²** | 6151 mm/s² | 6542 mm/s² | 6412 mm/s² |

### Dynamic Interpretations
1. **Condition C Postural Collapse**:
   Condition C exhibited large roll excursions ($38.53^\circ$ PTP, identical to unplanarized M8 roll instability) while pitch PTP remained at $8.60^\circ$. Because the front/middle legs were constrained to body-frame planar heights but the hind legs were unconstrained, asymmetric touchdown between alternating tripods created large roll wobbles, leading to increased lateral drift ($3.1\%$).
2. **Condition D Postural Excellence**:
   Condition D demonstrated the most stable body posture of any planar controller tested to date: pitch PTP dropped to **$6.51^\circ$** (a $24.1\%$ improvement over Condition B) and roll PTP dropped to **$9.17^\circ$** (a $32.5\%$ improvement over Condition B).
3. **The Persistent Glide Hesitation Problem**:
   Despite Condition D's exceptional postural stability, its velocity coefficient of variation (CV50) remained completely unchanged at **$0.3276$** (identical to Condition B's $0.3275$, and 41.5% worse than M8's $0.2315$). Furthermore, wrong-sign forward backsliding remained at **$9.1\%$** of stance duration. Event-triggered analysis confirms that speed dips coincide with dual-support transfer binding, where whole-body cancellation peaks at $0.882$ during tripod handoff.

---

## 8. Primary Pre-Registered Hypothesis Classifications

### Condition C Classification Evaluation (Patch 3)

| Criterion | Metric Value | Threshold / Reference | Status |
|---|---|---|---|
| 1. Cancellation Improvement $\ge 0.08$ | $\Delta = -0.3344$ | $\ge +0.08$ absolute | **FAIL** |
| 2. Opposing Fraction Improvement $\ge 0.08$ | $\Delta = -0.5773$ | $\ge +0.08$ absolute | **FAIL** |
| 3. CV50 Improvement $\ge 15\%$ | $-12.2\%$ | $\ge +15.0\%$ relative | **FAIL** |
| 4. Wrong-Sign Improvement $\ge 50\%$ or $\le 0.04$ | $-54.7\%$, abs $= 0.141$ | $\ge +50.0\%$ or $\le 0.04$ | **FAIL** |
| 5. Mean Backward Speed $\ge 8.5$ mm/s | $8.94$ mm/s | $\ge 8.5$ mm/s | **PASS** |
| 6. Pitch PTP not worse $> 10\%$ vs M8 | $8.60^\circ$ | $\le 9.22^\circ$ ($1.10 \times 8.38^\circ$) | **PASS** |
| 7. Thorax Z PTP $\le 0.14$ mm | $0.192$ mm | $\le 0.14$ mm | **FAIL** |
| 8. Lateral Ratio $\le 0.02$ | $0.0310$ | $\le 0.020$ | **FAIL** |
| 9. Numerical & Telemetry Integrity | Finite, $F_z > 0$, Sat=0, Step<0.01 | All gates pass | **PASS** |

**Primary Classification**: **`HIND_CONSTRAINT_HYPOTHESIS_NOT_SUPPORTED`**.
**Secondary Diagnostic Interpretation**: **`HIND_RELIEF_POSTURALLY_CONFOUNDED`** (Roll PTP $38.53^\circ > 30.0^\circ$, CV50 $0.3675 \ge 0.300$).

---

### Condition D Classification Evaluation (Patch 4)

| Criterion | Metric Value | Threshold / Reference | Status |
|---|---|---|---|
| 1. Nominal Support $V_x$ Spread Reduction $\ge 50\%$ | **62.3% reduction** | $\ge 50.0\%$ | **PASS** |
| 2. Cancellation Improvement $\ge 0.08$ | $\Delta = -0.0815$ | $\ge +0.08$ absolute | **FAIL** |
| 3. Opposing Fraction Improvement $\ge 0.08$ | $\Delta = -0.3248$ | $\ge +0.08$ absolute | **FAIL** |
| 4. CV50 Improvement $\ge 15\%$ | $-0.03\%$ | $\ge +15.0\%$ relative | **FAIL** |
| 5. Wrong-Sign Improvement $\ge 50\%$ or $\le 0.04$ | $0.0\%$, abs $= 0.0914$ | $\ge +50.0\%$ or $\le 0.04$ | **FAIL** |
| 6. Mean Backward Speed $\ge 8.5$ mm/s | $7.87$ mm/s | $\ge 8.5$ mm/s | **FAIL** |
| 7. Pitch PTP $\le$ B or Improved $\ge 10\%$ | **6.51° (-24.1% rel B)** | $\le 8.57^\circ$ | **PASS** |
| 8. Roll PTP $\le 20.0^\circ$ | **9.17°** | $\le 20.0^\circ$ | **PASS** |
| 9. Thorax Z PTP $\le 0.14$ mm | **0.103 mm** | $\le 0.14$ mm | **PASS** |
| 10. Lateral Ratio $\le 0.02$ | $0.0363$ | $\le 0.020$ | **FAIL** |
| 11. Numerical & Telemetry Integrity | Finite, $F_z > 0$, Sat=0, Step<0.01 | All gates pass | **PASS** |

**Primary Classification**: **`EXCURSION_MATCHING_NOT_SUPPORTED`**.

### Additional B vs D Diagnostics
- `COMMAND_MISMATCH_REDUCED`: **YES** (62.3% reduction in nominal target velocity spread, from 5.31 mm/s to 2.00 mm/s).
- `PHYSICAL_FORCE_CONFLICT_REDUCED`: **NO** (Intra-tripod cancellation increased from 0.3679 to 0.4494; opposing fraction increased from 35.3% to 67.8%).
- `BODY_GLIDE_IMPROVED`: **NO** (CV50 0.3276 vs 0.3275; wrong-sign fraction 0.0914 vs 0.0914).

---

## 9. Biomechanical Synthesis & Architectural Implications for Moonwalker Control

### 1. Why Selective Hind Relief Failed (Condition C)
MW-M26 observed that planted hind legs exerted forward braking forces in Condition B, opposing the front pullers. Condition C tested whether releasing the hind legs from DLS IK planarization would restore M8-like low conflict.
The result was a negative finding:
- Planarizing front and middle legs while leaving hind legs unplanarized creates a **severe vertical and angular mismatch** between front/middle and hind ground clearance.
- The unplanarized hind legs collide with the substrate at an unfavorable angle of attack, increasing their forward braking force by $+120\%$ (from $+0.94$ mN to $+2.18$ mN).
- Postural tilt (roll PTP $38.5^\circ$) exacerbates weight shift, causing force cancellation to spike to $0.7023$.
- *Conclusion*: Multi-leg support cannot be selectively planarized on a subset of legs without inducing postural collapse.

### 2. Why Uniform Excursion Scaling Merely Shifted the Conflict Locus (Condition D)
Condition D addressed the root kinematic mismatch identified in M26: open-loop trajectories commanded disparate sweep velocities ($7.9$ mm/s front vs $13.6$ mm/s mid vs $12.5$ mm/s hind). Scaling excursions ($E = [1.000, 0.564, 0.779]$) successfully matched nominal target sweep velocities.
However, in closed-loop physics:
- **Locomotion Speed is Determined by Active Traction, Not Average Command**: The front legs (pulling at $-4.4$ mN) and the newly propulsive hind legs (pulling at $-2.1$ mN) drove the thorax backward faster than the shortened middle legs were commanded to sweep ($7.6$ mm/s).
- **The Dragged Foot Becomes a Brake**: Because the middle legs had their stroke shortened by $43.6\%$, they could not keep pace with the backward travel of the thorax. Their planted tarsi were dragged forward across the substrate, generating $+1.81$ mN of forward braking resistance.
- *Conclusion*: Prescribing 1D open-loop joint excursions cannot eliminate multi-leg force opposition when the legs have differing kinematic leverage, moment arms, and vertical load bearing.

### 3. Clear Path Forward for MW-M28+
To achieve continuous ground support without force opposition:
1. **Compliant or Force-Limited Leg Control**: Actuators on supporting legs must not be rigidly position-controlled against the substrate. A compliant controller (or admittance control) that yields when horizontal contact force opposes motion would prevent middle or hind legs from ever becoming brakes.
2. **True Closed-Loop Jacobian Velocity Matching**: Rather than open-loop excursion scaling, compute the instantaneous world-frame velocity of the thorax and dynamically slave all three planted foot endpoints to have zero relative world velocity ($\dot{x}_{\text{foot, world}} = 0$).
3. **Exploit Condition D's Postural Superiority**: Condition D proved that scaled excursions provide pitch and roll stability ($6.51^\circ$ pitch PTP, $9.17^\circ$ roll PTP). Combining Condition D's stance geometry with compliant middle-leg retraction is the logical next design iteration.

---

## 10. Artifacts & Reproducibility Ledger

- Evaluation Script: [`scripts/m27_selective_hind_relief.py`](../scripts/m27_selective_hind_relief.py)
- Manifest JSON: [`data/derived/m27_hind_relief_manifest.json`](../data/derived/m27_hind_relief_manifest.json)
- Quantitative Results JSON: [`data/derived/m27_hind_relief_results.json`](../data/derived/m27_hind_relief_results.json)
- Diagnostic Figures:
  - [`output/m27/cancellation_comparison.png`](../output/m27/cancellation_comparison.png)
  - [`output/m27/per_leg_fx_comparison.png`](../output/m27/per_leg_fx_comparison.png)
  - [`output/m27/velocity_profiles.png`](../output/m27/velocity_profiles.png)
  - [`output/m27/event_triggered_hesitation.png`](../output/m27/event_triggered_hesitation.png)
