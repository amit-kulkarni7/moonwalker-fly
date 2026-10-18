# MW-M28: Conservative Middle-Leg Brake-Relief Polish of M27-D

## 1. Executive Summary

| Metric | Condition A (M8 Baseline) | Condition B (Exact M27-D) | Condition C (Middle Brake Relief) | Status / Gate |
|---|---|---|---|---|
| **Parent Reproduction Gate** | Speed: 9.93 mm/s, CV50: 0.232 | Speed: 7.87 mm/s, CV50: 0.328 | — | **PASS** |
| **All-3 Support Contact Fraction** | 0.6157 (61.6%) | 0.9446 (94.5%) | **0.9476 (94.8%)** | Preserved ($\ge 0.88$) |
| **Mean Backward Speed** | 9.93 mm/s | 7.87 mm/s | **7.90 mm/s** | Preserved ($\ge 7.5$ mm/s) |
| **Backward Speed CV50** | **0.2315** | 0.3276 | 0.3210 (-2.0% rel B) | Gate: $\ge 10\%$ (FAIL) |
| **Wrong-Sign 20ms Fraction** | **0.0100 (1.0%)** | 0.0914 (9.1%) | 0.0857 (-6.2% rel B) | Gate: $\ge 30\%$ or $\le 0.06$ (FAIL) |
| **Pitch Peak-to-Peak (PTP)** | 8.38° | 6.51° | **6.14° (-5.6% rel B)** | Preserved ($\le 7.2^\circ$) |
| **Roll Peak-to-Peak (PTP)** | 38.53° | **9.17°** | 9.29° (+1.3% rel B) | Preserved ($\le 11.0^\circ$) |
| **Thorax Z PTP** | 0.109 mm | 0.103 mm | **0.101 mm (-1.9% rel B)** | Preserved ($\le 0.13$ mm) |
| **Primary Lateral Ratio** | **0.0068** | 0.0363 | 0.0448 (+23.5% rel B) | Gate: $\le 1.15\times B$ (FAIL: $0.0448 > 0.0417$) |
| **Literal Lateral Gate (Diagnostic)** | **0.0068** | 0.0363 | 0.0448 | Diagnostic: $\le 0.020$ (FAIL) |
| **Median Cancellation (ALL3 Contact)** | **0.2186** | 0.4494 | 0.4062 (+0.0431 rel B) | Gate: $\ge 0.06$ (FAIL) |
| **Opposing Force Fraction (1% Load)** | **0.2105 (21.0%)** | 0.6442 (64.4%) | 0.6158 (+0.0284 rel B) | Gate: $\ge 0.08$ (FAIL) |
| **Front Legs Median $F_x$** | -1.76 mN | **-4.32 mN (Pull)** | -4.07 mN (Pull) | Dominant pull preserved |
| **Middle Legs Median $F_x$** | **-1.39 mN (Pull)** | +1.81 mN (Brake) | +1.54 mN (Brake, 14.8% relief) | Gate: $\ge 30\%$ relief (FAIL) |
| **Hind Legs Median $F_x$** | 0.00 mN | **-2.10 mN (Pull)** | -2.09 mN (Pull) | Hind pulling preserved |
| **Dissipated Compliance Work ($W_{\text{neg}}$)** | — | — | **0.900 $\mu$J** | Yielding damping |
| **Injected Compliance Work ($W_{\text{pos}}$)** | — | — | **0.000 $\mu$J** | Strict passive reset |
| **Positive / Dissipated Work Ratio** | — | — | **0.0000** | Gate: $\le 0.05$ (**PASS**) |
| **Compliance Activation Duty (Support)** | — | — | **99.8%** | Bounded displacement |
| **Primary Classification** | Reference | Baseline | **CHOREOGRAPHY_PRESERVATION_FAILED** | Decisive Outcome |

---

### Core Scientific Findings

1. **Conservative Middle-Leg Brake Relief Produces Modest Mechanical Relief (14.8%) but Fails Pre-Registered Significance Gates**:
   - Applying a one-sided, low-magnitude ($80\ \mu\text{m}$, $5.0$ mm/s rate-clamped) compliance law in World X to active supporting middle legs (`LM`, `RM`) successfully reduced forward-directed middle braking forces from median $+1.81$ mN in Condition B down to **$+1.54$ mN** in Condition C (a **$14.8\%$ reduction**; `LM` $-19.3\%$, `RM` $-10.8\%$).
   - This force relief propagated into a modest improvement in active support ALL3 force cancellation (median ratio dropped from **$0.4494$ down to $0.4062$**, a $+0.0431$ absolute improvement) and opposing-force fraction (reduced from $64.4\%$ to $61.6\%$).
   - However, this $14.8\%$ relief fell well short of the pre-registered $\ge 30\%$ primary relief threshold (and below the $\ge 20\%$ partial relief threshold). Velocity hesitation CV50 improved by only $2.0\%$ (from $0.3276$ to $0.3210$), and wrong-sign backward motion dropped slightly from $9.14\%$ to $8.57\%$.

2. **Thermodynamic Passivity Fully Verified with Strict Low-Load Reset ($W_{\text{pos}} = 0.000\ \mu\text{J}$, Ratio = $0.0000 \le 0.05$)**:
   - By eliminating backward displacement driving while forward braking force persists ($F_x > 0$), the refined low-load reset policy completely prevents pushback under load. Return toward zero occurs only when contact forces subside or during swing.
   - Continuous 10 kHz work integration confirms **$0.000000\ \mu\text{J}$** of positive work injection against **$0.900\ \mu\text{J}$** of dissipated work, satisfying the passivity safety gate with a work ratio of **$0.0000 \le 0.05$**.

3. **Pitch and Roll Postural Calmness are Preserved, but Primary Lateral Preservation Fails**:
   - The compliance law strictly satisfied direct vertical command invariance ($\Delta Z_{\text{world}} \equiv 0$). Consequently, Condition C preserved M27-D's calm posture with exceptional fidelity: pitch PTP improved to **$6.14^\circ$** (vs $6.51^\circ$ in B), roll PTP remained stable at **$9.29^\circ$** (vs $9.17^\circ$ in B), thorax vertical oscillation was unchanged at **$0.101$ mm** (vs $0.103$ mm in B), and continuous three-leg ground contact fraction remained pristine at **$94.8\%$** (vs $94.5\%$ in B).
   - However, because middle-leg braking relief was naturally asymmetric between `LM` ($-19.3\%$) and `RM` ($-10.8\%$) under the fly's inherent tripod contact asymmetry, uncompensated yaw torque altered lateral tracking, causing steady-state lateral drift ratio to rise from $0.0363$ to **$0.0448$** (a **$+23.5\%$ increase**). Because $0.0448 > 1.15 \times 0.03626 = 0.0417$, Condition C **failed the primary relative lateral preservation gate**.

4. **Formal Primary Classification: `CHOREOGRAPHY_PRESERVATION_FAILED`**:
   - While Condition C demonstrated that mechanical braking relief is achievable via longitudinal compliance with zero work injection, it breached the primary lateral preservation constraint ($\text{lateral\_ratio}_C \le 1.15 \times \text{lateral\_ratio}_B$). Under the pre-registered rubric, failure of this primary gate assigns the classification `CHOREOGRAPHY_PRESERVATION_FAILED`.

---

## 2. Experimental Design & Strictly Isolated Conditions

The experiment evaluated exactly three dynamic conditions under identical MuJoCo physics settings ($\Delta t = 0.0001$ s, $1.75$ s duration, $1$ kHz sampling, steady-state evaluation interval $t \in [0.35, 1.75]$ s):

```
+---------------------------------------------------------------------------------------------------+
| CONDITION A: DIRECT_M8_CONTROL (Reference Macro-Glide Baseline)                                   |
| - Unmodified M8 controller (scripts/m8_backslide_controller.py)                                   |
| - Cadence: 6.0 Hz | Excursion: 1.0 | Turnaround: 0.06 | Transfer: 0.08 | Entry: 0.25 s            |
| - Targets: Speed ~9.93 mm/s | CV50 ~0.232 | Pitch PTP ~8.38° | Roll PTP ~38.53°                   |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| CONDITION B: EXACT_M27_D (Authoritative Choreographic Baseline to Protect)                        |
| - Exact committed M27-D controller (scripts/m27_selective_hind_relief.py)                         |
| - All 3 support legs planarized to frozen body-frame Z heights in scratch MjData                  |
| - Excursions: E_front = 1.000 | E_middle = 0.564 | E_hind = 0.779                                 |
| - Targets: Speed ~7.87 mm/s | CV50 ~0.328 | Pitch PTP ~6.51° | Roll PTP ~9.17° | Z PTP ~0.103 mm  |
+---------------------------------------------------------------------------------------------------+
                                                  |
                                                  v
+---------------------------------------------------------------------------------------------------+
| CONDITION C: M27_D_PLUS_CONSERVATIVE_MIDDLE_BRAKE_RELIEF (Minimal Polishing Intervention)         |
| - FRONT & HIND legs: Identical to Condition B (100% frozen M27-D kinematics & DLS IK)             |
| - MIDDLE legs (LM, RM) during swing: Identical to Condition B (dx_adm == 0.0)                     |
| - MIDDLE legs during active support: Yields in World X under forward braking (rho_excess > 0)     |
| - Dimensionless brake law: rho_brake = max(Fx_filt, 0) / (Fz_filt + 1.0 mN)                       |
| - Causal 1st-order IIR filter: tau = 8.0 ms (alpha_f = 0.01235 at 10 kHz)                        |
| - Single frozen gain: K_rho = 1.25 mm | Hard clamps: dx_adm in [0, 80 um], |v_adm| <= 5.0 mm/s   |
| - Low-load reset policy: hold dx_adm during strong loaded support; reset during unloading         |
| - Direct vertical command invariance: Delta Z_world == 0.0, Delta Y_world == 0.0                  |
+---------------------------------------------------------------------------------------------------+
```

### Frozen Parameters Across All Conditions
- Physics simulation: FlatGroundWorld, $\Delta t = 0.0001$ s, duration $1.75$ s.
- Cadence: $6.0$ Hz.
- Support transfer duration: $t_f = 0.08$ ($13.3$ ms dual-support overlap).
- Turnaround fraction: $r = 0.06$.
- Startup entry ramp: $0.25$ s.
- DLS IK parameters: damping $\lambda = 10^{-4}$, regularization weight $\alpha = 0.01$, max step $\Delta q = 0.05$ rad, max iterations $15$, tolerance $0.1$ mm.
- Frozen body-frame support heights (M25/M27 provenance):
  - LF: $-1.04245$ mm | LM: $-1.08761$ mm | LH: $-0.97508$ mm
  - RF: $-1.04199$ mm | RM: $-1.08830$ mm | RH: $-0.97195$ mm

---

## 3. Preflight Causal Isolation & Passivity Checks

### Preflight 0: Provenance & Scratch Isolation
- Tracked M8 parameter provenance verified: cadence $6.0$ Hz, excursion $1.0$, turnaround $0.06$, transfer $0.08$, entry $0.25$ s (**PASS**).
- Scratch `MjData` isolation verified: kinematic solves in scratch data did not mutate live simulation physics state (**PASS**).

### Preflight 1: Condition C Causal Isolation Verification
1. **Static Same-State Check at Steady Stance ($t = 0.54$ s, LM in Strong Stance)**:
   - Front and Hind joint angle targets matched Condition B to machine precision: **$0.00\times 10^{-10}$ rad** (threshold $< 10^{-10}$ rad, **PASS**).
   - Middle leg joint angle targets at zero force matched Condition B to machine precision: **$0.00\times 10^{-10}$ rad** (threshold $< 10^{-10}$ rad, **PASS**).
2. **Synthetic Forward Yielding Test**:
   - Injecting $+2.0$ mN forward braking force under $+40.0$ mN vertical load resulted in forward compliance yielding: $\Delta x_{\text{adm}} = 24.5\ \mu\text{m} > 0$, with joint angle change of $0.0109$ rad (**PASS**).

### Preflight 2: Passivity & Low-Load Reset Verification
- Synthetic load cycle test:
  - Braking onset ($F_x = 2.0$ mN, $F_z = 40.0$ mN, $g=1.0$): yielded forward to peak displacement of $42.2\ \mu\text{m}$.
  - Loaded hold ($F_x \to 0$, $F_z = 40.0$ mN, $g=1.0$): held accumulated displacement at exactly $42.2\ \mu\text{m}$ (zero pushback under load, **PASS**).
  - Unloading reset ($g \to 0$, $F_z \to 0$): reset smoothly toward zero ($0.0\ \mu\text{m}$, **PASS**).
  - Synthetic work ratio: $0.0000 \le 0.01$ (**PASS**).

---

## 4. Reproduction & Simulation Integrity Gates

### Gate 1: Parent Reproduction Gate (Deterministic Tolerances)
Both historical parents reproduced with exact determinism:
- **Condition A (M8 Champion)**:
  - Speed: $9.93$ mm/s (target $9.93 \pm 0.10$) -> **PASS**
  - CV50: $0.232$ (target $0.232 \pm 0.010$) -> **PASS**
  - Pitch PTP: $8.38^\circ$ (target $8.38^\circ \pm 0.25^\circ$) -> **PASS**
  - Roll PTP: $38.53^\circ$ (target $38.53^\circ \pm 1.00^\circ$) -> **PASS**
  - All-3 Support: $0.616$ (target $0.616 \pm 0.020$) -> **PASS**
- **Condition B (Exact M27-D)**:
  - Speed: $7.87$ mm/s (target $7.87 \pm 0.15$) -> **PASS**
  - CV50: $0.328$ (target $0.328 \pm 0.015$) -> **PASS**
  - Wrong-Sign 20ms: $0.0914$ (target $0.0914 \pm 0.015$) -> **PASS**
  - Pitch PTP: $6.51^\circ$ (target $6.51^\circ \pm 0.25^\circ$) -> **PASS**
  - Roll PTP: $9.17^\circ$ (target $9.17^\circ \pm 0.35^\circ$) -> **PASS**
  - Thorax Z PTP: $0.103$ mm (target $0.103 \pm 0.015$ mm) -> **PASS**
  - All-3 Support: $0.945$ (target $0.945 \pm 0.015$) -> **PASS**
  - ALL3 Force Cancellation: $0.449$ (target $0.449 \pm 0.025$) -> **PASS**

### Gate 2: Telemetry & Actuator Integrity
- Continuous positive vertical load: $F_z > 0$ across $100\%$ of simulation samples (**PASS**).
- Actuator torque saturation: $0.000\%$ across all 42 joints (**PASS**).
- Maximum joint target step at $10$ kHz: $0.0044$ rad (limit $< 0.010$ rad, **PASS**).
- Abdomen ground contact: exactly $0$ events (**PASS**).
- DLS IK convergence fraction: $1.000$ ($100\%$ converged, max residual $< 0.1$ mm, **PASS**).

---

## 5. Ground Reaction Force (GRF) Telemetry & Conflict Mechanics

### Per-Leg Median Ground Reaction Forces and Kinematics (Strong Support Interval)

| Leg | Class | Condition A (M8) $F_x$ | Condition B (M27-D) $F_x$ | Condition C (Relief) $F_x$ | Delta (C vs B) | Condition C $F_z$ | Condition C Contact |
|---|---|---|---|---|---|---|---|
| **LF** | Front | -1.76 mN | -4.43 mN | **-4.21 mN** | +0.22 mN (Pull) | 42.42 mN | 94.5% |
| **RF** | Front | -1.54 mN | -4.21 mN | **-3.92 mN** | +0.29 mN (Pull) | 42.38 mN | 95.0% |
| **LM** | Middle| -1.39 mN | +1.71 mN | **+1.38 mN** | **-0.33 mN (-19.3%)** | 42.40 mN | 100.0% |
| **RM** | Middle| -1.51 mN | +1.90 mN | **+1.69 mN** | **-0.21 mN (-10.8%)**  | 42.23 mN | 100.0% |
| **LH** | Hind  | 0.00 mN  | -2.06 mN | **-2.02 mN** | +0.04 mN (Pull) | 40.21 mN | 100.0% |
| **RH** | Hind  | 0.00 mN  | -2.15 mN | **-2.16 mN** | -0.01 mN (Pull) | 40.23 mN | 100.0% |

### Analysis of Force Opposition Mechanics
1. **Middle-Leg Braking Relief Magnitude**:
   - The compliance law succeeded in reducing forward-directed braking in both middle legs:
     - `LM` braking dropped from $+1.71$ mN to $+1.38$ mN ($-19.3\%$ reduction).
     - `RM` braking dropped from $+1.90$ mN to $+1.69$ mN ($-10.8\%$ reduction).
     - Combined median middle braking fell from $+1.81$ mN to **$+1.54$ mN** ($-14.8\%$ overall relief).
2. **Front-Hind Pulling Coordination**:
   - Front and hind pulling remained robust and dominant:
     - Front legs pulled at median $-4.07$ mN ($-4.21$ mN LF, $-3.92$ mN RF).
     - Hind legs pulled at median $-2.09$ mN ($-2.02$ mN LH, $-2.16$ mN RH).
   - Because the middle legs still exerted $+1.54$ mN of forward braking, the total mechanical push-pull opposition was attenuated but not resolved.

---

## 6. Internal Force Conflict & Propulsive Efficiency

### Quantitative Conflict Summary

| Metric | Condition A (M8) | Condition B (M27-D) | Condition C (Relief) | C vs B Delta |
|---|---|---|---|---|
| **Active ALL3 Force Cancellation (Median)** | **0.2186** | 0.4494 | **0.4062** | **-0.0431 (Improved)** |
| **Active ALL3 Cancellation (P05 / P95)** | 0.00 / 0.72 | 0.20 / 0.65 | 0.17 / 0.62 | Narrower band |
| **Opposing-Force Fraction (1% Load)** | **0.2105** | 0.6442 | **0.6158** | **-0.0284 (Improved)** |
| **Active Propulsive Efficiency (Median)** | **0.7814** | 0.5506 | **0.5938** | **+0.0431 (Improved)** |
| **Whole-Body Cancellation (Median)** | **0.7455** | 0.8735 | 0.8568 | -0.0167 |

- **Conflict Attenuation**: Force cancellation improved across active tripod stances from $0.4494$ to $0.4062$, and propulsive efficiency rose from $55.1\%$ to $59.4\%$.
- **Sub-Threshold Relief**: The absolute improvement of $+0.0431$ fell short of the pre-registered threshold of $+0.0600$, confirming that $80\ \mu\text{m}$ of compliance is insufficient to fully decouple the middle legs from the substrate during strong stance.

---

## 7. Compliance Telemetry, Work Diagnostics & Passivity Analysis

### Detailed Work & Power Telemetry

| Telemetry Channel | Measured Value | Pre-Registered Limit | Status |
|---|---|---|---|
| **Dissipated Compliance Work ($W_{\text{neg}}$)** | **0.900 $\mu$J** | — | Energy removed |
| **Injected Compliance Work ($W_{\text{pos}}$)** | **0.000 $\mu$J** | — | Zero work injection |
| **Net Compliance Work ($W_{\text{net}}$)** | **-0.900 $\mu$J** | $\le 0.0\ \mu$J | Strictly dissipative |
| **Positive / Dissipated Work Ratio** | **0.0000** | $\le 0.0500$ | **PASS** |
| **Contact-Bearing Work Ratio** | **0.0000** | $\le 0.0500$ | **PASS** |
| **Total Trial Work Ratio** | **0.0000** | — | Strictly passive |
| **Middle Strong Support Active Duty** | **99.8%** | — | Continuous activation |
| **Total Steady Gait Active Duty** | **100.0%** | — | Continuous activation |

### Biomechanical Passivity & Strict Low-Load Reset
1. **Elimination of Pushback Under Load**:
   - When forward braking loads subside during stance or transfer, the controller holds accumulated displacement $\Delta x_{\text{adm}}$ rather than pulling backward against ground resistance.
   - Return toward zero occurs smoothly only when contact forces subside ($F_x \le 0$) or when the leg is in swing ($w_{\text{sup}} < 10^{-4}$).
   - This completely eliminates positive work injection under load, resulting in **$0.000000\ \mu\text{J}$** of positive work injection and establishing strict thermodynamic passivity ($W_{\text{pos}} / W_{\text{neg}} = 0.0000 \le 0.05$).

---

## 8. Body Kinematics, Postural Stability & Glide Dynamics

| Kinematic Metric | Condition A (M8 Baseline) | Condition B (Exact M27-D) | Condition C (Relief) | Preservation Gate | Verdict |
|---|---|---|---|---|---|
| Kinematic Metric | Condition A (M8 Baseline) | Condition B (Exact M27-D) | Condition C (Relief) | Preservation Gate | Verdict |
|---|---|---|---|---|---|
| **Mean Backward Speed** | 9.93 mm/s | 7.87 mm/s | **7.90 mm/s** | $\ge 7.5$ mm/s | **PASS** |
| **Backward Speed CV50** | **0.2315** | 0.3276 | 0.3210 | $\ge 10\%$ improvement | **FAIL (-2.0%)** |
| **Wrong-Sign 20ms Fraction** | **0.0100** | 0.0914 | 0.0857 | $\ge 30\%$ or $\le 0.06$ | **FAIL (-6.2%)** |
| **Pitch Peak-to-Peak (PTP)** | 8.38° | 6.51° | **6.14°** | $\le 7.2^\circ$ | **PASS** |
| **Pitch Standard Deviation** | 2.07° | 1.76° | **1.72°** | — | Calm |
| **Roll Peak-to-Peak (PTP)** | 38.53° | **9.17°** | 9.29° | $\le 11.0^\circ$ | **PASS** |
| **Roll Standard Deviation** | 8.76° | **2.57°** | 2.57° | — | Calm |
| **Thorax Z PTP** | 0.109 mm | 0.103 mm | **0.101 mm** | $\le 0.13$ mm | **PASS** |
| **Thorax Z Standard Deviation**| 0.020 mm | **0.016 mm** | 0.016 mm | — | Flat |
| **Lateral Drift Ratio** | **0.0068** | 0.0363 | 0.0448 | $\le 1.15\times B$ ($0.0417$) | **FAIL (+23.5%)** |

### Posture and Lateral Drift Analysis
- **Postural Stability Fully Preserved**: Pitch PTP ($6.14^\circ$), Roll PTP ($9.29^\circ$), and Z oscillation ($0.101$ mm) remained in the calm regime discovered in M27-D, proving that modulating World X introduces zero catastrophic pitch/roll crosstalk.
- **Lateral Asymmetry**: The slight difference in braking relief between LM ($-19.3\%$) and RM ($-10.8\%$) introduced an asymmetric yaw torque across the gait cycle, driving lateral displacement up from $0.43$ mm in B to $0.53$ mm in C, which breached the $1.15\times$ relative preservation threshold ($0.0448 > 0.0417$).

---

## 9. Primary Pre-Registered Hypothesis Classifications

### Condition C Classification Evaluation

| Criterion | Metric | Condition C Value | Threshold | Result |
|---|---|---|---|---|
| 1 | Middle-Leg Braking Relief | $14.8\%$ | $\ge 30\%$ relative to B | **FAIL** |
| 2 | ALL3 Force Cancellation Delta | $+0.0431$ | $\ge +0.0600$ absolute | **FAIL** |
| 3 | Opposing-Force Fraction Delta | $+0.0284$ | $\ge +0.0800$ absolute | **FAIL** |
| 4 | Wrong-Sign20 Improvement | $-6.2\%$ ($0.0857$) | $\ge 30\%$ or $\le 0.0600$ | **FAIL** |
| 5 | CV50 Improvement | $-2.0\%$ | $\ge 10\%$ relative to B | **FAIL** |
| 6 | Backward Speed | $7.90$ mm/s | $\ge 7.5$ mm/s | **PASS** |
| 7 | Postural Preservation (Pitch/Roll/Z) | $6.14^\circ$ / $9.29^\circ$ / $0.101$ mm | $\le 7.2^\circ$ / $\le 11.0^\circ$ / $\le 0.13$ mm | **PASS** |
| 8 | All-3 Support Contact Fraction | $0.9476$ | $\ge 0.8800$ | **PASS** |
| 9 | Primary Lateral Preservation | $0.0448$ | $\le 1.15\times B$ ($0.0417$) | **FAIL** |
| 10| Passivity Work Ratio | $0.0000$ | $\le 0.0500$ | **PASS** |
| 11| Numerical & Actuator Integrity | Saturation $0.0\%$, step $0.0043$ | Sat $0.0\%$, step $< 0.01$ | **PASS** |

### Formal Classification Verdict
**`CHOREOGRAPHY_PRESERVATION_FAILED`**

- **Adjudication Summary**:
  Condition C achieved partial mechanical relief ($14.8\%$ middle braking reduction, $+0.0431$ cancellation improvement) and strictly preserved M27-D's calm vertical posture, continuous ground support, and thermodynamic passivity ($W_{\text{pos}} = 0.000\ \mu\text{J}$). However, it failed the primary lateral preservation gate ($0.0448 > 0.0417$, $+23.5\%$ drift increase due to bilateral relief asymmetry). Under the pre-registered rubric, breach of the primary lateral preservation constraint formally assigns the classification `CHOREOGRAPHY_PRESERVATION_FAILED`. Furthermore, the sub-threshold mechanical relief confirms that this conservative parameterization is `MIDDLE_BRAKE_RELIEF_NOT_SUPPORTED`.

---

## 10. Biomechanical Synthesis & Strategic Path Forward

1. **Why Conservative Compliance Yields Insufficient Relief ($14.8\%$)**:
   - The frozen compliance gain ($K_{\rho} = 1.25$ mm) and tight displacement bound ($80\ \mu\text{m}$) were chosen deliberately to protect M27-D's visual choreography.
   - However, the middle-leg stance stroke in M27-D is $\approx 0.65$ mm. Yielding forward by at most $35$–$45\ \mu\text{m}$ relieves only $\approx 6$–$7\%$ of the required longitudinal kinematic adjustment. To achieve $\ge 30\%$ relief, compliance displacement would need to exceed $150\ \mu\text{m}$ ($23\%$ of stroke), which risks introducing visible foot slippage and pitch/roll instability.

2. **Thermodynamic Passivity and Low-Load Reset Invariants**:
   - Ensuring that backward reset occurs only when contact forces have genuinely subsided ($F_x \le 0$) or the leg has lifted off completely eliminates positive work injection under load ($0.000000\ \mu\text{J}$).
   - Any compliant controller must maintain this strict force-gated reset invariant to ensure unconditional multi-body physical stability.

3. **Status of the Moonwalker Fly Control Architecture**:
   - **M27-D remains the authoritative choreographic baseline**: calm, continuous three-leg support, pitch PTP $6.51^\circ$, roll PTP $9.17^\circ$.
   - Middle-leg brake relief via feedforward or compliance remains a viable polishing candidate, but requires:
     1. Bilaterally coupled lateral constraint enforcement to prevent yaw asymmetry.
     2. Strict force-gated low-load reset logic to maintain zero work injection.

---

## 11. Artifacts & Reproducibility Ledger

### Tracked Code and Artifacts
- Evaluation & Rendering Script: [`scripts/m28_middle_brake_relief.py`](../scripts/m28_middle_brake_relief.py)
- Manifest Payload: [`data/derived/m28_middle_brake_relief_manifest.json`](../data/derived/m28_middle_brake_relief_manifest.json)
- Results Payload: [`data/derived/m28_middle_brake_relief_results.json`](../data/derived/m28_middle_brake_relief_results.json)
- Final Milestone Report: [`reports/m28_middle_brake_relief.md`](../reports/m28_middle_brake_relief.md)

### Generated Diagnostic Plots
- Cancellation Comparison: [`output/m28/cancellation_comparison.png`](../output/m28/cancellation_comparison.png)
- Per-Leg Fx Comparison: [`output/m28/per_leg_fx_comparison.png`](../output/m28/per_leg_fx_comparison.png)
- Velocity Profiles: [`output/m28/velocity_profiles.png`](../output/m28/velocity_profiles.png)
- Event-Triggered Speed Hesitation: [`output/m28/event_triggered_hesitation.png`](../output/m28/event_triggered_hesitation.png)
- Compliance Telemetry & Passivity Deep Dive: [`output/m28/compliance_telemetry.png`](../output/m28/compliance_telemetry.png)

### Synchronized Diagnostic & Presentation Videos
- Condition A (M8 Baseline, 1.0x): [`output/m28/video_A_m8_10x.mp4`](../output/m28/video_A_m8_10x.mp4)
- Condition A (M8 Baseline, 0.2x): [`output/m28/video_A_m8_02x.mp4`](../output/m28/video_A_m8_02x.mp4)
- Condition B (Exact M27-D, 1.0x): [`output/m28/video_B_planar_10x.mp4`](../output/m28/video_B_planar_10x.mp4)
- Condition B (Exact M27-D, 0.2x): [`output/m28/video_B_planar_02x.mp4`](../output/m28/video_B_planar_02x.mp4)
- Condition C (Middle Brake Relief, 1.0x): [`output/m28/video_C_brake_relief_10x.mp4`](../output/m28/video_C_brake_relief_10x.mp4)
- Condition C (Middle Brake Relief, 0.2x): [`output/m28/video_C_brake_relief_02x.mp4`](../output/m28/video_C_brake_relief_02x.mp4)
- 3-Way Composite Comparison (1.0x): [`output/m28/video_A_vs_B_vs_C_10x.mp4`](../output/m28/video_A_vs_B_vs_C_10x.mp4)
- 3-Way Composite Comparison (0.2x): [`output/m28/video_A_vs_B_vs_C_02x.mp4`](../output/m28/video_A_vs_B_vs_C_02x.mp4)
