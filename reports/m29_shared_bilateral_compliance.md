# MW-M29: Support-Weighted Shared Bilateral Middle-Leg Compliance Polish of M27-D

## 1. Executive Summary

| Metric | Condition A (M8 Baseline) | Condition B (Exact M27-D) | Condition C (Shared Compliance) | Status / Gate Threshold | Gate Outcome |
|---|---|---|---|---|---|
| **Parent Reproduction Gate** | Speed: 9.93 mm/s, CV50: 0.232 | Speed: 7.87 mm/s, CV50: 0.328 | — | Deterministic tolerances | **PASS** |
| **All-3 Support Contact Fraction** | 0.6157 (61.6%) | 0.9446 (94.5%) | **0.9506 (95.1%)** | Preserved ($\ge 0.880$) | **PASS** |
| **Mean Backward Speed** | 9.93 mm/s | 7.87 mm/s | **7.92 mm/s** | Preserved ($\ge 7.50$ mm/s) | **PASS** |
| **Backward Speed CV50** | **0.2315** | 0.3276 | **0.3163 (-3.4% rel B)** | Preservation ($\le 1.03\times B = 0.3374$) | **PASS** |
| **Wrong-Sign 20ms Fraction** | **0.0100 (1.0%)** | 0.0914 (9.1%) | **0.0849 (-7.1% rel B)** | Preservation ($\le B + 0.010 = 0.1014$) | **PASS** |
| **Pitch Peak-to-Peak (PTP)** | 8.38° | 6.51° | **6.21° (-4.5% rel B)** | Preserved ($\le 7.20^\circ$) | **PASS** |
| **Roll Peak-to-Peak (PTP)** | 38.53° | **9.17°** | 9.44° (+3.0% rel B) | Preserved ($\le 11.00^\circ$) | **PASS** |
| **Thorax Z PTP** | 0.109 mm | 0.103 mm | **0.101 mm (-1.9% rel B)** | Preserved ($\le 0.130$ mm) | **PASS** |
| **Primary Relative Lateral Ratio** | **0.0068** | 0.0363 | 0.0516 (+42.3% rel B) | Gate: $\le 1.15\times B$ ($0.0417$) | **FAIL ($0.0516 > 0.0417$)** |
| **Literal Lateral Gate (Diagnostic)** | **0.0068** | 0.0363 | 0.0516 | Diagnostic: $\le 0.020$ | **FAIL** |
| **Median Cancellation (ALL3 Contact)** | **0.2186** | 0.4494 | **0.4102 (-0.0391 rel B)** | Target: $\ge 0.020$ improvement | **PASS (Improved by 0.0391)** |
| **Opposing Force Fraction (1% Load)** | **0.2105 (21.0%)** | 0.6780 (67.8%) | **0.6454 (64.5%)** | Attenuated (-3.3% abs) | **PASS** |
| **Middle Leg Combined Median $F_x$** | **-1.38 mN (Pull)** | +1.81 mN (Brake) | +1.64 mN (Brake) | Gate: $\ge 10.0\%$ relief | **FAIL (8.99% relief)** |
| **Middle Braking Relief Asymmetry** | — | — | LM: 15.89%, RM: 2.77% | Gate: $|LM - RM| \le 5.95\%$ | **FAIL (13.11% asymmetry)** |
| **Dissipated Compliance Work ($W_{\text{neg}}$)** | — | — | **0.0179 $\mu$J** | Passive damping | **PASS** |
| **Injected Compliance Work ($W_{\text{pos}}$)** | — | — | **0.0000 $\mu$J** | Strict passive reset | **PASS** |
| **Positive / Dissipated Work Ratio** | — | — | **0.0000** | Safety Gate: $\le 0.0500$ | **PASS** |
| **Compliance Activation Duty (Support)** | — | — | **100.0%** | Bounded state displacement | **PASS** |
| **Primary Classification** | Reference | Champion | **CHOREOGRAPHY_PRESERVATION_FAILED** | Pre-Registered Protocol | **DECISIVE OUTCOME** |

---

### Core Scientific Findings

1. **Shared Latent State Provides Modest Braking Relief (8.99%) and Smooths Velocity (CV50 0.3163), but Fails the Middle-Relief Target**:
   - Pooling bilateral middle-leg forces into a single causal state $dx_{\text{shared}}$ drove combined median middle braking from $+1.806$ mN down to **$+1.644$ mN** (an **$8.99\%$ reduction**). While positive and directionally correct, this fell slightly short of the pre-registered $\ge 10.0\%$ relief gate.
   - Active tripod ALL3 force cancellation improved by **$0.0391$ absolute** (dropping from $0.4494$ to **$0.4102$**, surpassing the $\ge 0.020$ improvement gate).
   - This force decoupling translated into smoother macro-kinematics: body velocity hesitation CV50 improved from $0.3276$ to **$0.3163$** ($-3.4\%$), and wrong-sign 20ms backward deceleration fell from $0.0914$ to **$0.0849$** ($-7.1\%$).

2. **Thermodynamic Passivity Strictly Verified with Zero Work Injection ($W_{\text{pos}} = 0.0000\ \mu\text{J}$, Ratio = $0.0000$)**:
   - The low-load reset logic—holding $dx_{\text{shared}}$ when braking force is positive and resetting toward zero only when $F_x \le 0$ or during unloading ($w_{\text{sup}} < 0.01$)—completely eliminated pushback under load.
   - Continuous 10 kHz power integration confirmed **$0.000000\ \mu\text{J}$** of positive work injection against **$0.017931\ \mu\text{J}$** of negative dissipated work. The passivity safety gate ($W_{\text{pos}} / W_{\text{neg}} \le 0.05$) passed with machine precision.

3. **Pitch, Roll, and Grounding are Preserved, but Primary Relative Lateral Preservation Breaches ($+42.3\%$)**:
   - Condition C maintained M27-D's calm postural profile: pitch PTP improved to **$6.21^\circ$** (vs $6.51^\circ$ in B), roll PTP remained tightly bounded at **$9.44^\circ$** (vs $9.17^\circ$ in B), vertical oscillation was unchanged at **$0.101$ mm** (vs $0.103$ mm in B), and continuous three-leg support contact rose to **$95.1\%$** (vs $94.5\%$ in B). Direct vertical command invariance ($\Delta Z \equiv 0$) guaranteed zero posture destabilization.
   - However, steady-state lateral displacement increased from $+0.431$ mm in B to **$+0.619$ mm** in C, raising the lateral drift ratio from $0.0363$ to **$0.0516$** (a **$+42.3\%$ degradation**). Because $0.0516 > 1.15 \times 0.03626 = 0.0417$, Condition C **breached the primary relative lateral preservation gate**.

4. **Mechanistic Root Cause of Lateral Drift: Physical Stance Asymmetry Defeats Latent Symmetry**:
   - Despite mathematically enforcing a single shared latent compliance state $dx_{\text{shared}}$ and a single shared force filter, the physical response remained strongly asymmetric: `LM` experienced **$15.89\%$ braking relief** ($+1.713 \to +1.441$ mN), whereas `RM` experienced only **$2.77\%$ braking relief** ($+1.900 \to +1.847$ mN). The resulting asymmetry $|15.89\% - 2.77\%| = 13.11\%$ far exceeded the pre-registered $\le 5.95\%$ threshold.
   - In alternating tripod locomotion, Tripod A (LF, RM, LH) and Tripod B (RF, LM, RH) interact with the substrate with inherent anatomical differences (contact angle, yaw moment arm, body roll bias). Even when commanded with identical latent compliance $dx_{\text{shared}}$, the physical yielding under load produced differential net longitudinal traction, generating an uncompensated yaw moment that drove the lateral drift.

5. **Formal Milestone Classification: `CHOREOGRAPHY_PRESERVATION_FAILED`**:
   - Under the pre-registered protocol, failure of the primary relative lateral preservation gate ($\text{lateral\_ratio}_C \le 1.15 \times \text{lateral\_ratio}_B$) assigns the formal classification **`CHOREOGRAPHY_PRESERVATION_FAILED`**.
   - **Conclusion**: Modulating middle-leg longitudinal compliance—whether independently (M28) or via a shared bilateral latent state (M29)—consistently triggers lateral yaw steering. Therefore, **middle-leg compliance is NOT a viable path for polishing the fly-moonwalk**. Condition B (M27-D) remains the authoritative protected choreographic champion.

---

## 2. Experimental Design & Strictly Isolated Conditions

The experiment evaluated exactly three dynamic conditions under identical MuJoCo physics settings ($\Delta t = 0.0001$ s, duration $1.75$ s, $10$ kHz physics stepping, $1$ kHz data sampling, steady-state evaluation interval $t \in [0.35, 1.75]$ s):

```
+---------------------------------------------------------------------------------------------------+
| CONDITION A: DIRECT_M8_CONTROL (Reference Macro-Glide Baseline)                                   |
| - Unmodified M8 controller (scripts/m8_backslide_controller.py provenance)                        |
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
| CONDITION C: M29_SHARED_BILATERAL_COMPLIANCE (Support-Weighted Shared State)                       |
| - FRONT & HIND legs: Identical to Condition B (100% frozen M27-D kinematics & DLS IK)             |
| - MIDDLE legs (LM, RM) during swing: Identical to Condition B (g_sup == 0 -> dx_applied == 0.0)    |
| - ONE shared latent state dx_shared and ONE shared force filter state (Fx_pool_filt, Fz_pool_filt) |
| - Support-weighted pooling: Fx_pool = (w_LM*Fx_LM + w_RM*Fx_RM) / (w_LM + w_RM)                   |
| - Dimensionless braking ratio: rho_brake = max(Fx_pool_filt, 0) / (Fz_pool_filt + 1.0 mN)        |
| - Single frozen gain: K_rho = 1.25 mm | Clamps: dx_shared in [0, 80 um], |v| <= 5.0 mm/s           |
| - Low-load reset policy: hold dx_shared during positive braking; decay only when Fx<=0 or swing   |
| - Direct vertical/lateral invariance: Delta Z_world == 0.0, Delta Y_world == 0.0                  |
+---------------------------------------------------------------------------------------------------+
```

### Frozen Parameters Across All Conditions
- Physics environment: FlatGroundWorld, $\Delta t = 0.0001$ s, duration $1.75$ s.
- Gait Cadence: $6.0$ Hz.
- Support transfer duration: $t_f = 0.08$ ($13.3$ ms dual-support overlap).
- Turnaround fraction: $r = 0.06$.
- Startup entry ramp: $0.25$ s.
- DLS IK solver parameters: damping $\lambda = 10^{-4}$, regularization weight $\alpha = 0.01$, max step $\Delta q = 0.05$ rad, max iterations $15$, tolerance $0.1$ mm.
- Frozen body-frame support heights (M25/M27 provenance):
  - LF: $-1.04245$ mm | LM: $-1.08761$ mm | LH: $-0.97508$ mm
  - RF: $-1.04199$ mm | RM: $-1.08830$ mm | RH: $-0.97195$ mm
- Frozen compliance hyperparameters:
  - $K_\rho = 1.25$ mm (frozen from M28; no hyperparameter re-tuning).
  - $dx_{\max} = 0.080$ mm ($80\ \mu\text{m}$).
  - $v_{\max} = 5.0$ mm/s.
  - $\rho_{\text{thresh}} = 0.015$.
  - $\tau = 8.0$ ms ($\alpha_f = 0.01235$ at 10 kHz).
  - $F_{z, \text{offset}} = 1.0$ mN.

---

## 3. Preflight Causal Isolation & Passivity Checks

### Preflight 0: Provenance & Scratch Isolation
- Tracked M8 parameter provenance verified: cadence $6.0$ Hz, excursion $1.0$, turnaround $0.06$, transfer $0.08$, entry $0.25$ s (**PASS**).
- Scratch `MjData` isolation verified: kinematic solves in scratch data did not mutate live simulation physics state (**PASS**).

### Preflight 1: Condition C Causal Isolation & Bilateral Symmetry Verification
1. **Static Same-State Check at Steady Stance ($t = 0.54$ s, LM in Strong Stance)**:
   - Front and Hind joint angle targets matched Condition B to machine precision: **$0.00\times 10^{-10}$ rad** (threshold $< 10^{-10}$ rad, **PASS**).
   - Middle leg joint angle targets at zero force matched Condition B to machine precision: **$0.00\times 10^{-10}$ rad** (threshold $< 10^{-10}$ rad, **PASS**).
2. **Synthetic Forward Yielding Test**:
   - Injecting $+2.0$ mN forward braking force under $+40.0$ mN vertical load resulted in forward compliance yielding: $\Delta x_{\text{shared}} = 24.5\ \mu\text{m} > 0$, with joint angle change of $0.0109$ rad (**PASS**).
3. **Bilateral Mathematical Symmetry Invariant**:
   - Single shared latent state object verified: `ONE_SHARED_STATE_OBJECT == True` (**PASS**).
   - Single shared filter state verified: `ONE_SHARED_FILTER_STATE == True` (**PASS**).
   - Equal commanded yield verified: whenever $w_{\text{sup}, i} > 0.01$, $\Delta x_{\text{applied}, i} / w_{\text{sup}, i} == dx_{\text{shared}}$ within numerical tolerance (**PASS**).

### Preflight 2: Passivity & Low-Load Reset Verification
- Synthetic load cycle test:
  - Braking onset ($F_x = 2.0$ mN, $F_z = 40.0$ mN, $g=1.0$): yielded forward to peak displacement of $42.2\ \mu\text{m}$.
  - Loaded hold ($F_x \to 0$, $F_z = 40.0$ mN, $g=1.0$): held accumulated displacement at exactly $42.2\ \mu\text{m}$ (zero pushback under load, **PASS**).
  - Unloading reset ($g \to 0$, $F_z \to 0$): reset smoothly toward zero ($0.0\ \mu\text{m}$, **PASS**).
  - Synthetic work accounting: $W_{\text{diss}} = 0.0844\ \mu\text{J}$, $W_{\text{inj}} = 0.0000\ \mu\text{J}$, work ratio $= 0.0000 \le 0.01$ (**PASS**).

---

## 4. Reproduction & Simulation Integrity Gates

### Gate 1: Parent Reproduction Gate (Deterministic Tolerances)
Both historical parents reproduced with exact determinism:
- **Condition A (M8 Champion)**:
  - Speed: $9.93$ mm/s (target $9.93 \pm 0.10$) -> **PASS** ($9.9257$)
  - CV50: $0.232$ (target $0.232 \pm 0.010$) -> **PASS** ($0.2315$)
  - Pitch PTP: $8.38^\circ$ (target $8.38^\circ \pm 0.25^\circ$) -> **PASS** ($8.3810^\circ$)
  - Roll PTP: $38.53^\circ$ (target $38.53^\circ \pm 1.00^\circ$) -> **PASS** ($38.5308^\circ$)
  - All-3 Support: $0.616$ (target $0.616 \pm 0.020$) -> **PASS** ($0.6157$)
- **Condition B (Exact M27-D)**:
  - Speed: $7.87$ mm/s (target $7.87 \pm 0.15$) -> **PASS** ($7.8697$)
  - CV50: $0.328$ (target $0.328 \pm 0.015$) -> **PASS** ($0.3276$)
  - Wrong-Sign 20ms: $0.0914$ (target $0.0914 \pm 0.015$) -> **PASS** ($0.0914$)
  - Pitch PTP: $6.51^\circ$ (target $6.51^\circ \pm 0.25^\circ$) -> **PASS** ($6.5053^\circ$)
  - Roll PTP: $9.17^\circ$ (target $9.17^\circ \pm 0.35^\circ$) -> **PASS** ($9.1680^\circ$)
  - Thorax Z PTP: $0.103$ mm (target $0.103 \pm 0.015$ mm) -> **PASS** ($0.1033$)
  - All-3 Support: $0.945$ (target $0.945 \pm 0.015$) -> **PASS** ($0.9446$)
  - ALL3 Force Cancellation: $0.449$ (target $0.449 \pm 0.025$) -> **PASS** ($0.4494$)

### Gate 2: Telemetry & Actuator Integrity
- Continuous positive vertical load: $F_z > 0$ across $100\%$ of simulation samples (**PASS**).
- Actuator torque saturation: $0.000\%$ across all 42 joints (**PASS**).
- Maximum joint target step at $10$ kHz: $0.0043$ rad (limit $< 0.010$ rad, **PASS**).
- Abdomen ground contact: exactly $0$ events (**PASS**).
- DLS IK convergence fraction: $1.000$ ($100\%$ converged, max residual $< 0.1$ mm, **PASS**).

---

## 5. Ground Reaction Force (GRF) Telemetry & Opposition Breakdown

### Per-Leg Median Ground Reaction Forces and Kinematics (Steady-State Interval)

| Leg | Class | Condition A (M8) $F_x$ | Condition B (M27-D) $F_x$ | Condition C (Shared) $F_x$ | Delta (C vs B) | Relief % | Condition C $F_z$ | Condition C Contact |
|---|---|---|---|---|---|---|---|---|
| **LF** | Front | -1.764 mN | -4.429 mN | **-4.292 mN** | +0.137 mN (Pull) | -3.1% | 42.44 mN | 94.8% |
| **RF** | Front | -1.756 mN | -4.208 mN | **-3.912 mN** | +0.296 mN (Pull) | -7.0% | 42.41 mN | 95.3% |
| **LM** | Middle| -1.388 mN | +1.713 mN | **+1.441 mN** | **-0.272 mN** | **-15.89%** | 42.45 mN | 100.0% |
| **RM** | Middle| -1.381 mN | +1.900 mN | **+1.847 mN** | **-0.053 mN** | **-2.77%** | 42.27 mN | 100.0% |
| **LH** | Hind  | 0.000 mN  | -2.055 mN | **-2.158 mN** | -0.103 mN (Pull) | +5.0% | 40.23 mN | 100.0% |
| **RH** | Hind  | 0.000 mN  | -2.141 mN | **-2.140 mN** | +0.001 mN (Pull) | -0.0% | 40.24 mN | 100.0% |

### Analysis of Force Opposition Mechanics
1. **Middle-Leg Braking Relief Magnitude**:
   - The compliance law succeeded in reducing forward-directed braking in both middle legs:
     - `LM` braking dropped from $+1.713$ mN to $+1.441$ mN (**$-15.89\%$ reduction**).
     - `RM` braking dropped from $+1.900$ mN to $+1.847$ mN (**$-2.77\%$ reduction**).
     - Combined median middle braking fell from $+1.806$ mN to **$+1.644$ mN** (**$8.99\%$ overall relief**).
   - Although relief was achieved, the $8.99\%$ overall reduction fell short of the pre-registered $\ge 10.0\%$ threshold.
2. **Persistent Bilateral Relief Asymmetry**:
   - Despite commanding identical latent compliance through the single state $dx_{\text{shared}}$, the physical relief asymmetry was severe: $|15.89\% - 2.77\%| = \mathbf{13.11\%}$.
   - This far exceeded the pre-registered threshold of $\le 5.95\%$, proving that equalizing the controller state does not equalize physical ground interaction under asymmetric body dynamics.
3. **Front-Hind Pulling Coordination**:
   - Front pulling remained dominant: LF pulled at $-4.292$ mN, RF pulled at $-3.912$ mN (combined median $-4.102$ mN).
   - Hind pulling remained rock-solid: LH pulled at $-2.158$ mN, RH pulled at $-2.140$ mN (combined median $-2.149$ mN).
   - Because the middle legs still exerted $+1.644$ mN of forward braking, the three-leg push-pull conflict was dampened but remained fundamentally active.

---

## 6. Internal Force Conflict & Propulsive Efficiency

### Quantitative Conflict Summary

| Metric | Condition A (M8) | Condition B (M27-D) | Condition C (Shared) | C vs B Delta | Gate / Interpretation |
|---|---|---|---|---|---|
| **Active ALL3 Force Cancellation (Median)** | **0.2186** | 0.4494 | **0.4102** | **-0.0391** | Gate $\ge 0.020$ (**PASS**) |
| **Active ALL3 Cancellation (P05 / P95)** | 0.00 / 0.72 | 0.21 / 0.65 | 0.18 / 0.64 | Narrower band | Conflict dampened |
| **Opposing-Force Fraction (1% Load)** | **0.2105** | 0.6780 | **0.6454** | **-0.0326** | Reduced opposition |
| **Active Propulsive Efficiency (Median)** | **0.7814** | 0.5506 | **0.5898** | **+0.0391** | +7.1% relative gain |
| **Whole-Body Cancellation (Median)** | **0.7455** | 0.8735 | 0.8727 | -0.0007 | Unchanged (transfer bound) |

- **Tripod Stance Efficiency**: Force cancellation across active three-leg support stances improved from $0.4494$ down to $0.4102$, representing an absolute reduction of $-0.0391$ and passing the pre-registered $\ge 0.020$ gate.
- **Transfer Window Invariance**: Whole-body cancellation across all samples remained virtually identical ($0.8735 \to 0.8727$). This confirms that the severe cancellation spikes ($>0.80$) occurring during the six-leg dual-tripod transfer window are unaffected by stance compliance.

---

## 7. Compliance Telemetry, Work Diagnostics & Passivity Analysis

### Detailed Work & Power Telemetry

| Telemetry Channel | Measured Value | Pre-Registered Limit | Status |
|---|---|---|---|
| **Dissipated Compliance Work ($W_{\text{neg}}$)** | **0.0179 $\mu$J** | — | Energy removed |
| **Injected Compliance Work ($W_{\text{pos}}$)** | **0.0000 $\mu$J** | — | Strict zero injection |
| **Net Compliance Work ($W_{\text{net}}$)** | **-0.0179 $\mu$J** | $\le 0.0\ \mu$J | Strictly dissipative |
| **Positive / Dissipated Work Ratio** | **0.0000** | $\le 0.0500$ | **PASS** |
| **Contact-Bearing Work Ratio** | **0.0000** | $\le 0.0500$ | **PASS** |
| **Total Trial Work Ratio** | **0.0025** | — | Negligible |
| **Middle Strong Support Active Duty** | **100.0%** | — | Continuous activation |
| **Total Steady Gait Active Duty** | **100.0%** | — | Continuous activation |

### Biomechanical Passivity & Low-Load Reset Performance
1. **Zero Pushback Under Load**:
   - The low-load reset rule holds accumulated displacement $dx_{\text{shared}}$ while forward braking force is positive ($F_x > 0$), even if the braking ratio falls below the threshold.
   - Return toward zero occurs only when contact forces subside ($F_x \le 0$) or when support weight drops below $0.01$.
   - This prevents the controller from ever pushing backward into a resisting ground contact, ensuring **$0.000000\ \mu\text{J}$** of positive work injection and establishing strict thermodynamic passivity.

---

## 8. Body Kinematics, Postural Stability & Glide Dynamics

| Kinematic Metric | Condition A (M8 Baseline) | Condition B (Exact M27-D) | Condition C (Shared) | Preservation Gate | Verdict |
|---|---|---|---|---|---|
| **Mean Backward Speed** | 9.93 mm/s | 7.87 mm/s | **7.92 mm/s** | $\ge 7.50$ mm/s | **PASS** |
| **Backward Speed CV50** | **0.2315** | 0.3276 | **0.3163** | $\le 1.03\times B = 0.3374$ | **PASS (-3.4%)** |
| **Backward Speed CV20** | **0.4546** | 0.6373 | **0.6141** | — | Improved (-3.6%) |
| **Wrong-Sign 20ms Fraction** | **0.0100** | 0.0914 | **0.0849** | $\le B + 0.010 = 0.1014$ | **PASS (-7.1%)** |
| **Pitch Peak-to-Peak (PTP)** | 8.38° | 6.51° | **6.21°** | $\le 7.20^\circ$ | **PASS (-4.5%)** |
| **Pitch Standard Deviation** | 2.07° | 1.76° | **1.73°** | — | Calm |
| **Pitch Mean** | +0.31° | -4.27° | **-4.18°** | — | Stable nose-down |
| **Roll Peak-to-Peak (PTP)** | 38.53° | **9.17°** | 9.44° | $\le 11.00^\circ$ | **PASS (+3.0%)** |
| **Roll Standard Deviation** | 8.76° | **2.57°** | 2.77° | — | Calm |
| **Thorax Z PTP** | 0.109 mm | 0.103 mm | **0.101 mm** | $\le 0.130$ mm | **PASS (-1.9%)** |
| **Thorax Z Standard Deviation**| 0.020 mm | **0.016 mm** | 0.016 mm | — | Flat |
| **Lateral Drift Ratio** | **0.0068** | 0.0363 | 0.0516 | $\le 1.15\times B = 0.0417$ | **FAIL (+42.3%)** |
| **Displacement X (Longitudinal)** | -14.93 mm | -11.89 mm | -11.99 mm | — | Matched backward glide |
| **Displacement Y (Lateral)** | -0.10 mm | +0.43 mm | +0.62 mm | — | Increased rightward drift |

### Posture and Lateral Drift Analysis
- **Postural Calmness Preserved**: Pitch PTP ($6.21^\circ$), Roll PTP ($9.44^\circ$), and Z oscillation ($0.101$ mm) remained in the calm regime discovered in M27-D. Modulating World X with zero vertical command displacement ($\Delta Z \equiv 0$) completely avoided pitch/roll destabilization.
- **Lateral Drift Breakdown**: The fly's lateral ratio rose from $0.0363$ to **$0.0516$** ($+42.3\%$). This breached the primary relative preservation gate ($0.0516 > 0.0417$), proving that bilateral middle-leg compliance destabilizes heading.

---

## 9. Asymmetry & Lateral Drift Root-Cause Analysis

### Why Did a Mathematically Shared Latent State Fail to Equalize Physical Relief?

A central hypothesis of MW-M29 was that pooling forces into a single latent compliance state $dx_{\text{shared}}$ would enforce bilateral symmetry and eliminate the lateral drift seen in M28. The empirical results definitively falsified this hypothesis:

```
Command Space (Symmetric):
  w_LM, w_RM -> Fx_pool -> Filter -> dx_shared (IDENTICAL FOR BOTH LEGS)
                                 |
                                 v
Physical Execution Space (Asymmetric):
  Tripod A (LF, RM, LH) Stance:   RM Braking = +1.847 mN  (Relief:  2.77%)
  Tripod B (RF, LM, RH) Stance:   LM Braking = +1.441 mN  (Relief: 15.89%)
                                 -----------------------------------------
                                 Relief Difference: 13.11% (Severe Asymmetry)
                                 Net Yaw Torque Imbalance: Rightward Drift
```

1. **Alternating Tripod Contact Dynamics**:
   - Because insect locomotion is alternating, LM and RM are never in strong stance simultaneously. They plant during opposite half-cycles.
   - During Tripod B stance (LM supporting), the body pitch and roll angles are slightly different than during Tripod A stance (RM supporting).
   - In M27-D, LM already experienced slightly different kinematic loading than RM due to the natural roll offset ($+0.19^\circ$).
2. **Coupling with Ground Friction**:
   - When LM yielded by $dx_{\text{shared}}$, the ground reaction force dropped substantially from $+1.713$ mN to $+1.441$ mN ($-15.89\%$).
   - When RM yielded by the same $dx_{\text{shared}}$, the ground reaction force barely budged (from $+1.900$ mN to $+1.847$ mN, $-2.77\%$).
   - This differential compliance response created an unbalanced longitudinal traction couple between the left and right tripods, producing a sustained net yaw torque that steered the fly rightward ($Disp_Y$ increased from $+0.431$ mm to $+0.619$ mm).

---

## 10. Event-Triggered Transfer Telemetry & Velocity Hesitation Analysis

The 61-point event-triggered analysis (centered at velocity local minima $t = 0.0$ ms) reveals the temporal structure of the hesitation:

| Event Metric | Condition B (M27-D) | Condition C (Shared Compliance) | Interpretation |
|---|---|---|---|
| **Detected Hesitation Events** | 17 events | 17 events | Exact cycle count |
| **Minimum Body Velocity Profile** | -2.42 mm/s | -2.03 mm/s | Hesitation slightly smoothed |
| **Velocity Profile Minimum Time** | $t = 0.0$ ms | $t = 0.0$ ms | Benchmark alignment |
| **Peak Force Cancellation Profile** | 0.8070 | 0.7411 | -8.2% reduction at transfer peak |
| **Peak Cancellation Time** | $t = -13.0$ ms | $t = -13.0$ ms | Precedes velocity minimum |

1. **Causal Sequence of the Hesitation Pulse**:
   - At $t = -13.0$ ms: Transfer begins. All 6 legs make ground contact as adhesion transfers between tripods ($t_f = 13.3$ ms). The opposing tripods fight each other mechanically, driving force cancellation to its cycle peak ($0.807$ in B, $0.741$ in C).
   - At $t = -13.0$ to $0.0$ ms: The opposing forces decelerate the body.
   - At $t = 0.0$ ms: Body longitudinal speed drops to its local minimum ($-2.42$ mm/s in B, $-2.03$ mm/s in C).
   - At $t = +10.0$ ms: The outgoing tripod fully unloads and lifts off. The single supporting tripod accelerates the body backward, restoring glide speed to $>11$ mm/s.
2. **Transfer Dominance**:
   - This temporal sequence confirms that **the six-leg transfer overlap is the primary driver of body-speed hesitation**, not middle-leg stance braking. Middle-leg compliance slightly dampens the hesitation peak ($-2.42 \to -2.03$ mm/s), but cannot eliminate the transfer-induced braking pulse.

---

## 11. Pre-Registered Hypothesis & Gate Verdicts

### Complete Criterion-by-Criterion Evaluation

| # | Criterion | Pre-Registered Rule | Measured Value | Result |
|---|---|---|---|---|
| 1 | **Middle-Leg Braking Relief** | Combined relief $\ge 10.0\%$ rel B | $8.99\%$ ($1.806 \to 1.644$ mN) | **FAIL** |
| 2 | **Relief Asymmetry Reduction** | $|LM_{\text{relief}} - RM_{\text{relief}}| \le 5.95\%$ | $13.11\%$ ($15.89\%$ vs $2.77\%$) | **FAIL** |
| 3 | **ALL3 Force Cancellation** | Cancellation improvement $\ge 0.020$ | $+0.0391$ ($0.4494 \to 0.4102$) | **PASS** |
| 4 | **CV50 Preservation** | $\text{CV50}_C \le 1.03\times \text{CV50}_B$ ($0.3374$) | $0.3163$ (-3.4% rel B) | **PASS** |
| 5 | **Wrong-Sign Preservation** | $\text{WS20}_C \le \text{WS20}_B + 0.010$ ($0.1014$) | $0.0849$ (-7.1% rel B) | **PASS** |
| 6 | **Backward Speed Preservation** | $\text{Speed}_C \ge 7.50$ mm/s | $7.92$ mm/s | **PASS** |
| 7 | **Postural Preservation** | Pitch $\le 7.20^\circ$, Roll $\le 11.00^\circ$, Z $\le 0.130$ mm | Pitch $6.21^\circ$, Roll $9.44^\circ$, Z $0.101$ mm | **PASS** |
| 8 | **All-3 Support Contact** | All-3 fraction $\ge 0.880$ | $0.9506$ ($95.1\%$) | **PASS** |
| 9 | **Primary Relative Lateral Gate** | $\text{lateral\_ratio}_C \le 1.15\times \text{lateral\_ratio}_B$ ($0.0417$) | **$0.0516$ (+42.3% rel B)** | **FAIL** |
| 10 | **Literal Lateral Diagnostic** | Lateral ratio $\le 0.020$ | $0.0516$ | **FAIL** |
| 11 | **Thermodynamic Passivity** | $W_{\text{pos}} / W_{\text{neg}} \le 0.0500$ | $0.0000$ ($W_{\text{pos}} = 0.000\ \mu\text{J}$) | **PASS** |
| — | **Primary Formal Classification** | Pre-Registered Rubric | **CHOREOGRAPHY_PRESERVATION_FAILED** | **DECISIVE** |

---

## 12. Visual Review & Qualitative Moonwalk Assessment

### Multi-Camera Video Deliverables in `output/m29/`
- **Side/Tracking Synchronized Comparisons (240x320 per pane)**:
  - `video_A_m8_10x.mp4` / `video_A_m8_02x.mp4` (Reference macro-glide)
  - `video_B_m27d_10x.mp4` / `video_B_m27d_02x.mp4` (Authoritative baseline champion)
  - `video_C_shared_10x.mp4` / `video_C_shared_02x.mp4` (Shared compliance)
  - `video_A_vs_B_vs_C_10x.mp4` / `video_A_vs_B_vs_C_02x.mp4` (3-pane synchronized side-by-side)
- **Top-Down Synchronized Comparisons (240x320 per pane)**:
  - `video_B_topdown_10x.mp4` / `video_B_topdown_02x.mp4` (Condition B overhead view)
  - `video_C_topdown_10x.mp4` / `video_C_topdown_02x.mp4` (Condition C overhead view)
  - `video_B_vs_C_topdown_10x.mp4` / `video_B_vs_C_topdown_02x.mp4` (2-pane synchronized top-down comparison)

### Perceptual Review Findings
1. **Side View (0.2x and 1.0x)**:
   - Condition C retains M27-D's calm posture, continuous three-leg sliding appearance, and lack of robotic stepping twitches. To the casual eye from the side camera, Condition C looks almost indistinguishable from Condition B.
2. **Top-Down View (0.2x and 1.0x)**:
   - The top-down synchronized video (`video_B_vs_C_topdown_02x.mp4`) clearly reveals the drift mechanism: Condition B travels backward with a very slight, stable trajectory, whereas Condition C visibly veers to the right over the course of the run, confirming the numerical lateral ratio breach ($0.0516 > 0.0417$).

---

## 13. Strategic Recommendation & Next Steps

1. **Retain Condition B (M27-D) as the Authoritative Moonwalk Baseline**:
   - Both M28 (independent compliance) and M29 (shared bilateral compliance) demonstrated that modifying middle-leg longitudinal compliance introduces uncompensated yaw torques and lateral steering drift.
   - M27-D remains the superior overall backward moonwalk controller, balancing high contact fraction ($94.5\%$), calm posture (pitch $6.51^\circ$, roll $9.17^\circ$), and acceptable lateral tracking ($0.0363$).
2. **No Further Middle-Leg Compliance Tuning**:
   - Do NOT attempt further parameter tuning of $K_\rho$, filter time constants, or compliance gains. The failure is structural (alternating tripod contact mechanics), not parametric.
3. **Future Focus: Transfer Dynamics**:
   - The event-triggered dynamics conclusively show that the remaining velocity hesitation originates at the six-leg dual-support transfer window ($t_f = 0.08$), where opposing tripods overlap. Future milestones should investigate smooth adhesion unloading and transfer geometry rather than stance compliance.
