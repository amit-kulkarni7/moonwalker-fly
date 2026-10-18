# MW-M26: Diagnosis of Internal Mechanical Conflict within the Supporting Tripod

## 1. Executive Summary

| Diagnostic Metric | Condition A (DIRECT M8) | Condition B (M25 Planar Support) | Difference (B - A) | Status / Gate |
|---|---|---|---|---|
| **M8 Reproduction Gate** | Speed: 9.93 mm/s, CV50: 0.232 | — | — | **PASS** |
| **Condition B Reproduction Gate** | — | Speed: 10.40 mm/s, CV50: 0.327 | — | **PASS** |
| **All-3 Support Contact Fraction** | 0.6157 (61.6%) | 0.9416 (94.2%) | +0.3259 (+52.9% rel) | Replicated |
| **Force Telemetry Integrity** | Finite, $F_z > 0$ continuous | Finite, $F_z > 0$ continuous | — | **YES** |
| **Median Force Cancellation (ALL3)** | **0.2186** | **0.3679** | **+0.1493 (+68.3% rel)** | **Elevated ($\ge 0.10$)** |
| **Median Force Cancellation (Tripod A)** | 0.2114 | 0.3752 | +0.1638 | Symmetrical |
| **Median Force Cancellation (Tripod B)** | 0.2227 | 0.3624 | +0.1397 | Symmetrical |
| **Opposing Force Fraction (1% Load)** | 0.2105 (21.1%) | 0.3532 (35.3%) | +0.1427 (+67.8% rel) | Elevated ($\ge 0.10$) |
| **Propulsive Efficiency (ALL3)** | 0.7814 | 0.6321 | -0.1493 (-19.1% rel) | Deteriorated |
| **Target Relative Vx Spread** | Range: 13.55 mm/s | Range: 13.84 mm/s | — | Severe mismatch |
| **Peak Conflict Timing** | — | Peaks at **0.880** at **$t = -14$ ms** | Precedes speed dip | Precedes minima |
| **Wrong-Sign Motion Cancellation** | 0.0000 (rare) | 0.1920 (vs 0.3797 correct) | -0.1877 | Decoupled in rebound |
| **Dominant Conflict Class** | Hind legs (LH, RH) | Hind legs (LH, RH) | — | Hind brakes (+1.0 mN) |
| **Final Classification** | — | — | — | **SUPPORT_CONFLICT_PARTIALLY_SUPPORTED** |

**Scientific Conclusion**:
MW-M25 established that enforcing three-leg planar ground contact eliminated contact loss (all-3 contact increased from 61.6% to 94.2%) but failed to improve velocity smoothness (CV50 worsened from 0.230 to 0.328) and caused a ninefold increase in forward wrong-sign backsliding (0.010 to 0.091). 

MW-M26 answers the core scientific question:
> *"When all three M25-B supporting legs remain planted, do they generate mutually conflicting horizontal ground reaction forces / kinematics that mechanically oppose one another through the thorax?"*

**The experimental answer is YES, with specific dynamic qualifications**:
1. **Clear Internal Force Cancellation**: Enforcing all-three support legs to remain planted causes a **+68.3% relative increase** in median horizontal force cancellation ratio during all-three contact (0.2186 $\rightarrow$ 0.3679, exceeding the pre-registered meaningful effect threshold of $\ge 0.10$).
2. **Anatomical Source of Conflict**: The internal opposition is driven primarily by the **hind legs (LH, RH)**. While the front legs pull backward with large propulsive forces (median $F_x \approx -4.2$ mN), the planted hind legs act as **forward-directed brakes** (median $F_x \approx +0.9$ to $+1.0$ mN, peaking up to $+7.06$ mN during transfer).
3. **Kinematic Root Cause**: In backward locomotion ($v_{x, \text{body}} \approx -10.4$ mm/s), a planted foot anchored to the floor must move forward relative to the retreating thorax at $+10.4$ mm/s. However, open-loop trajectories command incompatible forward sweep velocities across the tripod: front legs sweep at only $+7.92$ mm/s (thus pulling backward on the thorax at $-4.2$ mN), while hind legs sweep forward at $+12.40$ to $+12.81$ mm/s, exceeding thorax retreat velocity and forcing the planted hind feet to push forward against the substrate (generating braking forces).
4. **Dynamic Relation to Body Pulsing**: Intra-tripod force cancellation surges to an extreme peak of **0.880** exactly **14 ms prior to the body backward-speed minimum**, precisely matching the onset of the dual-support adhesion transfer window ($\phi = 0.96 / 1.96$). During the subsequent wrong-sign hesitation interval, severe cross-tripod transfer binding occurs (with an average of **4.79 legs simultaneously grounded** and full 6-leg force cancellation of **0.7470**). Under the strict pre-registered 3-leg support tripod metric, cancellation evaluates to 0.1920 because the dragging opposite hind leg is excluded by the 3-leg mask.
5. **Formal Classification**: Because the internal force-conflict increase is clear, elevated in both tripods, and temporally precedes the speed dips, but 3-leg cancellation drops during the rebound interval itself, the milestone formally classifies as **`SUPPORT_CONFLICT_PARTIALLY_SUPPORTED`** per Section 11 of the specification.

---

## 2. Experimental Conditions & Provenance

The experiment evaluated exactly two dynamic conditions in fresh, physically isolated simulations using the exact parameters established in M8 and M25:

- **Condition A (`DIRECT_M8_CONTROL`)**: Fresh baseline rollout of the validated M8 champion controller (`scripts/m8_backslide_controller.py`).
- **Condition B (`M25_BODY_FRAME_PLANAR_SUPPORT`)**: Exact implementation of M25's damped least-squares (DLS) IK body-frame planarization controller (`scripts/m25_support_path_geometry.py`). DLS IK solves were conducted in an isolated scratch `MjData` instance without live physics state mutation.
- **Frozen Heights**: Derived from Condition A steady-state strong-support plateaus and frozen identically to M25:
  - LF: $-1.04245$ mm | LM: $-1.08761$ mm | LH: $-0.97508$ mm
  - RF: $-1.04199$ mm | RM: $-1.08830$ mm | RH: $-0.97195$ mm

### Reproduction Verification
Both conditions satisfied all deterministic reproduction tolerances:
- **Condition A (M8)**: Speed $9.93$ mm/s (target $\approx 9.93$), CV50 $0.232$ (target $\approx 0.230$), Pitch PTP $8.38^\circ$ (target $\approx 8.38^\circ$), All-3 Contact $0.616$ (target $\approx 0.616$). $\rightarrow$ **PASS**.
- **Condition B (Planar)**: Speed $10.40$ mm/s (target $\approx 10.40$), CV50 $0.327$ (target $\approx 0.328$), Pitch PTP $8.57^\circ$ (target $\approx 8.57^\circ$), All-3 Contact $0.942$ (target $\approx 0.942$). $\rightarrow$ **PASS**.

---

## 3. World-Frame Per-Leg Ground Reaction Force (GRF) Telemetry

### Coordinate Frame & Sign Convention Audit
In FlyGym/MuJoCo world coordinates:
- $+X$ points forward (towards the head).
- $-X$ points backward (towards the abdomen).
- $+Z$ points upward against gravity.
- Contact forces returned by `sim.get_bodysegment_contact_forces` represent the net world-frame force acting **on the fly's body segments** from the ground.
- Therefore:
  - $F_z > 0$: Upward vertical support countering gravity.
  - $F_x < 0$: Backward force accelerating/propelling the fly in the moonwalk direction.
  - $F_x > 0$: Forward force resisting or braking the backward moonwalk.

### Per-Leg Ground Reaction Forces during Strong Support
All segments for each leg (`coxa`, `trochanterfemur`, `tibia`, `tarsus1..5`) were queried and aggregated.

| Leg | Anatomical Class | Contact Fraction (A) | Contact Fraction (B) | Median $F_x$ (A, mN) | Median $F_x$ (B, mN) | P05 / P95 $F_x$ (B, mN) | Median $F_z$ (B, mN) |
|---|---|---|---|---|---|---|---|
| **LF** | FRONT (Tripod A) | 0.906 | **0.939** | -1.76 | **-4.23** | -5.24 / 0.00 | 43.02 |
| **LM** | MIDDLE (Tripod B)| 1.000 | **1.000** | -1.39 | **-0.55** | -1.95 / +0.75 | 41.89 |
| **LH** | HIND (Tripod A)  | 0.603 | **1.000** | 0.00 | **+0.94** | -2.87 / **+1.96** | 40.83 |
| **RF** | FRONT (Tripod B) | 0.914 | **0.944** | -1.76 | **-4.17** | -5.19 / 0.00 | 43.05 |
| **RM** | MIDDLE (Tripod A)| 1.000 | **1.000** | -1.38 | **-0.51** | -2.09 / +0.84 | 41.81 |
| **RH** | HIND (Tripod B)  | 0.627 | **1.000** | 0.00 | **+0.89** | -2.26 / **+1.94** | 40.79 |

### Key Biomechanical Finding: The Hind-Leg Braking Effect
In Condition A (M8), the hind legs frequently break physical contact during strong support (contact fraction $\approx 60\%-63\%$). Across all strong support, their median horizontal force is $0.00$ mN (due to non-contact intervals), and even during ALL3 contact intervals, their opposition is minimal (LH median $+0.17$ mN, RH median $+0.06$ mN).
In Condition B (Planar Support), the hind legs are held in continuous ground contact ($1.000$ contact fraction). Crucially, their median horizontal force flips strongly to **positive braking values** ($+0.94$ mN for LH, $+0.89$ mN for RH, with 95th percentiles approaching $+2.0$ mN, and instantaneous spikes reaching $+7.06$ mN during transfer). 
Meanwhile, the front legs pull backward with massive compensatory propulsive force ($-4.23$ mN for LF, $-4.17$ mN for RF). 
**Thus, the front legs and hind legs are in continuous direct mechanical opposition**: the front legs must pull significantly harder to drag the fly backward against the braking resistance of the planted hind legs.

---

## 4. Internal Horizontal Force Conflict & Efficiency

For each support tripod at each valid strong-support sample ($t \ge 0.35$ s, $\text{median}(adh_{\text{sup}}) \ge 0.90$, $\text{median}(adh_{\text{opp}}) \le 0.10$):
$$F_{x, \text{net}} = \sum_{i \in \text{support}} F_{x, i}, \quad F_{x, \text{abs\_sum}} = \sum_{i \in \text{support}} |F_{x, i}|$$
$$\text{FORCE\_CANCELLATION\_RATIO} = 1 - \frac{|F_{x, \text{net}}|}{F_{x, \text{abs\_sum}} + \epsilon}$$
$$\text{PROPULSIVE\_FORCE\_EFFICIENCY} = \frac{|F_{x, \text{net}}|}{F_{x, \text{abs\_sum}} + \epsilon}$$

### Tripod Breakdown & Contact Conditioning

| Evaluation Subset | Condition A (M8) Median Cancellation | Condition B (Planar) Median Cancellation | Absolute Increase ($\Delta$) | Condition A Efficiency | Condition B Efficiency |
|---|---|---|---|---|---|
| **ALL3_CONTACT (Aggregated)** | **0.2186** | **0.3679** | **+0.1493** | **0.7814** | **0.6321** |
| Tripod A (ALL3 Contact) | 0.2114 | 0.3752 | +0.1638 | 0.7886 | 0.6248 |
| Tripod B (ALL3 Contact) | 0.2227 | 0.3624 | +0.1397 | 0.7773 | 0.6376 |
| **ALL STRONG SUPPORT** | **0.0000** | **0.3581** | **+0.3581** | **1.0000** | **0.6419** |
| Tripod A (All Strong) | 0.0000 | 0.3642 | +0.3642 | 1.0000 | 0.6358 |
| Tripod B (All Strong) | 0.0000 | 0.3531 | +0.3531 | 1.0000 | 0.6469 |
| **LT3_CONTACT (Fewer than 3 legs)** | $0.0000$ | $0.0000$ | $0.0000$ | $1.0000$ | $1.0000$ |

### Interpretation
1. In Condition A, whenever only 2 legs touch the ground (`LT3_CONTACT`), cancellation is virtually 0 (efficiency $\approx 1.0$), because both contacting legs push in the same direction. Across all strong support, the median cancellation is $0.0000$ because the hind leg is detached more than a third of the time. When all 3 touch, cancellation rises moderately to $0.219$.
2. In Condition B, because all 3 legs remain grounded 94.2% of the time, the median cancellation ratio surges to **$0.368$** in ALL3 contact and **$0.358$** across the entire strong-support phase (up from $0.000$ in Condition A).
3. Propulsive force efficiency drops from **$78.1\%$** in M8 down to **$63.2\%$** in Condition B. More than one-third of the muscular effort exerted by the supporting legs is wasted fighting other legs of the same tripod.

---

## 5. Support-Foot Kinematic Conflict

To identify why the legs generate opposing forces, we evaluated the commanded endpoint velocities relative to the thorax using exact forward kinematics (FK) in `scratch_data` on the applied joint angles, as well as the actual foot velocities in the world frame:

| Metric | Condition A (M8) | Condition B (Planar Support) | Interpretation |
|---|---|---|---|
| **LF Median Target Rel $V_x$** | 7.92 mm/s | 7.92 mm/s | Commanded slow retraction |
| **LM Median Target Rel $V_x$** | 13.62 mm/s | 13.62 mm/s | Commanded fast retraction |
| **LH Median Target Rel $V_x$** | 12.40 mm/s | 12.40 mm/s | Commanded fast retraction |
| **RF Median Target Rel $V_x$** | 7.92 mm/s | 7.92 mm/s | Commanded slow retraction |
| **RM Median Target Rel $V_x$** | 13.60 mm/s | 13.60 mm/s | Commanded fast retraction |
| **RH Median Target Rel $V_x$** | 12.81 mm/s | 12.81 mm/s | Commanded fast retraction |
| **Target Rel $V_x$ Range (Tripod Spread)** | **13.55 mm/s** | **13.84 mm/s** | **Severe velocity mismatch** |
| **Target Rel $V_x$ Std (Tripod Spread)** | **5.91 mm/s** | **5.94 mm/s** | **Severe velocity mismatch** |
| **Actual Foot World $V_x$ Range** | 0.85 mm/s (ALL3) | 1.33 mm/s (ALL3) | Grounded anchor spread |
| **Actual Foot World $V_x$ Std** | 0.37 mm/s (ALL3) | 0.57 mm/s (ALL3) | Grounded anchor spread |

### Kinematic Conflict Mechanism
In backward locomotion with the thorax translating backward at $v_{x, \text{body}} \approx -10.40$ mm/s, a stationary foot anchored to the substrate must translate forward relative to the thorax coordinate system at exactly $+10.40$ mm/s ($v_{\text{target\_rel}} = \dot{x}_{\text{foot}} - \dot{x}_{\text{body}} = 0 - (-10.40) = +10.40$ mm/s).
However, the open-loop joint angle trajectories command widely disparate longitudinal sweep rates:
- **Front Legs**: Command a slower forward relative sweep ($v_{\text{target\_rel}} \approx +7.92$ mm/s). Because the retreating thorax moves backward faster than the commanded leg stroke, the front legs are pulled into tension, pulling backward on the thorax ($F_x \approx -4.2$ mN) and providing the primary propulsive drive.
- **Middle & Hind Legs**: Command a substantially faster forward relative sweep ($v_{\text{target\_rel}} \approx +13.6$ mm/s for middle, $+12.4$ to $+12.8$ mm/s for hind). The commanded sweep rate exceeds the backward travel speed of the thorax.
- **Why M8 Tolerated the Mismatch**: In M8, the hind legs passively detach from the substrate early in stance (contact fraction $\approx 60\%-63\%$), allowing them to swing forward freely through air without fighting the front legs.
- **Why Condition B Fails**: When Condition B uses DLS IK to enforce continuous ground contact ($1.000$ hind contact), the planted hind feet are physically anchored to the substrate. Because the hind leg is commanded to sweep forward faster than the fly is retreating, the tarsus pushes forward against the substrate, and the substrate reacts with a forward braking force ($F_x \approx +0.9$ to $+2.0$ mN) directly opposing the backward moonwalk.

---

## 6. Pulse & Hesitation Dynamics

Using automated peak detection on the 20 ms filtered body longitudinal velocity signal ($v_{x, 20}$), exactly 16 backward-speed minima / hesitation events were identified during steady-state locomotion.

Around each event, a $\pm 30$ ms window was analyzed:
- **Event-Triggered Alignment**: Time $t = 0$ corresponds to the local minimum of backward speed (peak of $v_{x, 20} \approx +2.33$ mm/s, i.e., instantaneous forward slip).
- **Temporal Profile of Force Cancellation**:
  - $t = -30$ ms to $-22$ ms: Cancellation rises from $0.08$ to $0.29$.
  - $t = -20$ ms to $-16$ ms: Cancellation surges from $0.40$ to $0.69$.
  - **$t = -14$ ms: PEAK CANCELLATION REACHES 0.880** (precisely matching the onset of the dual-support adhesion transfer window at $\phi = 0.96 / 1.96$, where the transfer duration is $0.08 \times 166.7\text{ ms} \approx 13.3$ ms).
  - $t = -10$ ms to $0$ ms: Body speed rapidly collapses, crossing zero into wrong-sign forward motion at $t \approx -3$ ms.
  - $t = 0$ ms: Body reaches maximum forward rebound velocity ($+2.33$ mm/s).
  - $t = +10$ ms to $+20$ ms: Cancellation stabilizes back around $0.35$ as the incoming tripod engages.

```
       Force Cancellation Ratio vs Time to Speed Dip
0.90 |                         * (t = -14 ms, Peak = 0.880)
0.75 |                       *   *
0.60 |                     *       *
0.45 |                   *           *
0.30 |                 *               *
0.15 |               *                   *
0.00 |___*_______*_________________________*_______*_______
      -30ms    -20ms   -14ms     0ms     +10ms   +20ms   +30ms
                         ^
                  [Speed Minimum]
```

### Interpretation of Temporal Sequence
The surge in internal force cancellation **systematically precedes** the body-velocity dip by **14 ms**, exactly aligned with the beginning of the dual-support tripod exchange. The mechanical conflict peaks as the outgoing tripod reaches full extension while the incoming tripod attaches. The resulting horizontal drag on the thorax halts backward momentum and precipitates the velocity collapse.

### Wrong-Sign vs Correct-Sign Motion & Cross-Tripod Transfer Binding
- **Correct-Sign Motion ($v_{x, 20} \le 0$)**: Median cancellation is **$0.3797$**, opposing fraction is $35.3\%$, pitch is $-2.11^\circ$. The front and hind legs actively fight each other throughout the steady glide.
- **Wrong-Sign Motion ($v_{x, 20} > 0$)**:
  - In Condition B, 128 strong-support samples occur during wrong-sign motion ($9.1\%$ of the session).
  - During this hesitation/rebound phase, the fly is in the midst of tripod handoff: an average of **4.79 legs are grounded simultaneously**.
  - The outgoing hind leg (which belongs to the opposing tripod and is therefore excluded by the nominal 3-leg support tripod mask) remains anchored to the floor and exerts severe forward braking forces (peaking up to $+7.06$ mN), while the incoming front legs pull backward at $-1.2$ to $-1.6$ mN.
  - Across all 6 legs on the body, the true whole-body horizontal force cancellation during wrong-sign motion is **$0.7470$**.
  - However, because the pre-registered protocol strictly defined `FORCE_CANCELLATION_RATIO` on the nominal 3-leg support tripod ($F_{x, \text{net}} = \sum_{i \in \text{support}} F_{x, i}$), the dragging opposite hind leg was excluded from that metric, yielding $0.1920$ within the support tripod alone.
  - This rigorously explains why Criterion 3 strictly evaluates to false under the pre-registered 3-leg formula, while the underlying physical conflict across the full 6-leg contact topology is severe.

---

## 7. Load Distribution Analysis

Per-leg positive vertical load fraction was calculated as:
$$\text{load\_fraction}_i = \frac{\max(F_{z, i}, 0)}{\sum_{j=1}^6 \max(F_{z, j}, 0) + \epsilon}$$

| Anatomical Class | Median Load Fraction (Condition A) | Median Load Fraction (Condition B) | Dominant Class (Normal Motion) | Dominant Class (Speed Minima) | Dominant Class (Wrong-Sign) |
|---|---|---|---|---|---|
| **FRONT** | 0.329 | **0.332** | FRONT ($33.1\%$) | FRONT ($33.4\%$) | FRONT ($32.8\%$) |
| **MIDDLE**| 0.334 | **0.323** | MIDDLE ($32.5\%$) | MIDDLE ($31.3\%$) | MIDDLE ($32.2\%$) |
| **HIND**  | 0.317 | **0.318** | HIND ($31.8\%$) | HIND ($30.8\%$) | HIND ($32.0\%$) |

In Condition B, vertical load is partitioned almost equally ($\approx 33\%$ each) among front, middle, and hind support legs. Vertical load distribution remains remarkably constant across normal motion, speed dips, and wrong-sign intervals. 
**Crucial Insight**: The velocity pulsation is NOT caused by sudden vertical load shifting or loss of vertical support; it is purely a **horizontal shear force conflict** between front and hind legs.

---

## 8. Primary Diagnostic Hypothesis Classification

### Pre-Registered Hypothesis Criteria Evaluation
1. **Meaningfully higher median cancellation ratio during ALL3_CONTACT**:
   - Condition A: $0.2186$ $\rightarrow$ Condition B: $0.3679$ ($\Delta = +0.1493 \ge 0.10$). $\rightarrow$ **PASS**.
2. **Meaningfully higher opposing force sample fraction**:
   - At $1\%$ vertical load threshold ($\approx 1.3$ mN): Condition A: $0.2105$ $\rightarrow$ Condition B: $0.3532$ ($\Delta = +0.1427 \ge 0.10$). $\rightarrow$ **PASS**.
3. **Elevated cancellation during wrong-sign motion**:
   - Condition B Correct-Sign: $0.3797$ vs Wrong-Sign: $0.1920$ ($\Delta = -0.1877 < 0$). Under the pre-registered 3-leg support tripod metric, cancellation drops during wrong-sign motion because the dragging opposite hind leg (exerting up to $+7.06$ mN of braking) is excluded by the 3-leg mask (full 6-leg cancellation is $0.7470$). Additionally, peak intra-tripod cancellation ($0.880$) systematically precedes the speed minimum by 14 ms rather than coinciding with the rebound peak. $\rightarrow$ **FAIL (Strict 3-leg pre-registered definition)**.
4. **Bilateral presence across both alternating tripods**:
   - Tripod A: $0.2114 \rightarrow 0.3752$ ($\Delta = +0.1638$).
   - Tripod B: $0.2227 \rightarrow 0.3624$ ($\Delta = +0.1397$).
   - Symmetrical across both tripods; not an isolated pathological leg. $\rightarrow$ **PASS**.
5. **Reproduction and telemetry integrity**:
   - Both M8 and Condition B passed reproduction gates; forces finite, positive vertical support verified. $\rightarrow$ **PASS**.

### Formal Classification
Per Section 11 of the specification:
> *"Classify `SUPPORT_CONFLICT_PARTIALLY_SUPPORTED` if the force-conflict increase is clear but not consistently associated with pulse/wrong-sign timing or both tripods."*

**Final Classification: `SUPPORT_CONFLICT_PARTIALLY_SUPPORTED`**.

The increase in internal force cancellation is indisputable (+68.3% relative increase, bilateral across both tripods, driven by $+0.9$ to $+1.0$ mN hind-leg braking against $-4.2$ mN front-leg pulling). However, because intra-tripod cancellation peaks **14 ms before** the speed dips and evaluates to 0.1920 during the forward rebound phase under the strict 3-leg support tripod formula (despite 6-leg transfer binding reaching 0.7470), the hypothesis is formally classified as partially rather than fully supported under the pre-registered criteria.

---

## 9. Architectural Implications for M27+

1. **Why Planarization Failed**: M25 attempted to fix the pitch/velocity pulse by forcing all three support legs to stay on the ground. However, the open-loop joint angle trajectories command a **$\sim 6$ mm/s velocity spread** between front and hind legs. In M8, the hind legs passively detached, masking this mismatch. Forcing them to stay on the ground turned the hind legs into anchors that fight the front legs.
2. **Implication for Future Controllers**: Eliminating contact loss alone is insufficient and counterproductive if the planted legs have incompatible kinematics. Any future high-support controller must:
   - **Enforce Kinematic Velocity Compatibility**: Use Jacobian-based inverse kinematics or stance rate shaping so that all three planted legs command identical world-frame longitudinal velocities ($\dot{x}_{\text{LF}} = \dot{x}_{\text{RM}} = \dot{x}_{\text{LH}}$).
   - **Allow Compliant Hind-Leg Compliance**: Permit the hind legs to slip or compliantly yield when front-leg propulsion dominates, preventing them from acting as passive brakes.
   - **Re-evaluate Dual-Drive vs Stabilizer Roles**: Recognize that in backward walking, the front legs are the primary tractive pullers, and the hind legs should serve as low-drag stabilizers rather than rigidly coupled drivers.

---

## 10. Artifacts Generated
- Script: `scripts/m26_support_conflict_diagnosis.py`
- Manifest: `data/derived/m26_support_conflict_manifest.json`
- Results JSON: `data/derived/m26_support_conflict_results.json`
- Diagnostic Plots:
  - `output/m26/cancellation_distributions.png`
  - `output/m26/cancellation_vs_phase.png`
  - `output/m26/event_triggered_minima.png`
  - `output/m26/vx_vs_cancellation.png`
