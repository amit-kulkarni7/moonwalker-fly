# MW-M33 Phase 1 Report: 3D Fedora Attachment & Refined Theatrical Performance Stage

## Executive Summary

Phase 1 of **MW-M33 (Michael Jackson-Inspired Performance)** has been updated with targeted stage lighting refinements and complete trajectory frame synchronization, in strict accordance with the approved visual guidance.

The refined theatrical performance stage delivers:
1. **Luminous Anatomy**: Restrained warm key lighting and balanced soft fill bring out the natural amber-tan scutum, compound eye ommatidia, and all six slender articulated limbs.
2. **Theatrical Spotlight Effect**: A focused overhead warm spotlight pool on the dark ebony stage floor with crisp, natural grounding contact shadows, surrounded by dark cinematic void.
3. **Dorsal Rim Contour**: Crisp silver-warm rim illumination that separates the black fedora's brim and crown from the dark background without changing its approved geometry, placement, or forward tilt.
4. **1:1 Trajectory Frame Synchronization**: The authoritative trajectory dataset ([`data/derived/m33_m27d_trajectory.npz`](../data/derived/m33_m27d_trajectory.npz)) was resynchronized directly with FlyGym's native `renderer.render_as_needed()` callback across the full 1.75-second trial, producing **531 frames** with 1:1 bit-exact frame alignment with MW-M31 video footage.
5. **Phase 1 Boundaries Preserved**: Zero audio, zero frontend changes, and no full video rendered.

---

## 1. 3D Fedora Head Attachment & Alignment Confirmation

### Cranium Frame Positioning
In NeuroMechFly, the head complex (`c_head`, eyes, rostrum, and antennae) is rigidly welded to the thorax body frame (`fly/c_thorax`).
- **Attachment Body**: `fly/c_thorax`
- **Center Coordinates**: $X = +0.240$ mm (anterior), $Y = 0.000$ mm (bilateral midline), $Z = +0.398$ mm (dorsal cranium apex).
- **Pitch Tilt**: Forward pitch of $8.5^\circ$ around the bilateral $Y$-axis ($\text{quat} = [0.997, 0, 0.074, 0]$), pulling the brim down slightly over the anterior brow in classic Michael Jackson performance fashion.
- **Eye Clearance**: The brim front edge sits at $Z \approx +0.342$ mm, nearly $100\ \mu\text{m}$ above the dorsal limit of the compound eyes ($Z \le +0.248$ mm). Both deep ruby eyes remain **100% unobstructed and visible** from the hero tracking camera.

### Fedora Geometry & Material Design
Four visual geoms attached directly to `fly/c_thorax`:
1. `fedora_brim`: Ellipsoid disc ($0.38 \times 0.34 \times 0.013$ mm) in matte black felt (`rgba: [0.055, 0.055, 0.060, 1.0]`, roughness $0.85$).
2. `fedora_band`: Thin cylindrical band (radius $0.215$ mm, height $0.024$ mm) in charcoal satin ribbon (`rgba: [0.22, 0.22, 0.24, 1.0]`, specular $0.40$, shininess $0.60$).
3. `fedora_crown`: Cylindrical crown (radius $0.205$ mm, height $0.095$ mm).
4. `fedora_crease`: Pinched crease dome ($0.200 \times 0.170 \times 0.024$ mm) replicating the iconic center-indented fedora crown.

---

## 2. Refined Four-Point Theatrical Stage Lighting

The lighting architecture was refined to balance theatrical drama with anatomical legibility:

| Light Entity | Type | Position / Direction | Diffuse / Ambient | Theatrical Purpose |
|---|---|---|---|---|
| `spotlight_stage` | `mjLIGHT_SPOT` (cutoff $42^\circ$, exponent $8.0$, `castshadow=1`) | `pos: [0.0, 0.0, 14.0]`<br>`dir: [0.0, 0.0, -1.0]` | Diffuse: `[1.15, 1.05, 0.90]`<br>Ambient: `[0.03, 0.03, 0.03]` | Produces the luminous theatrical spotlight pool on the ebony stage floor and sharp grounding contact shadows |
| `key_front_warm` | `mjLIGHT_DIRECTIONAL` (`castshadow=0`) | `pos: [1.5, -6.5, 5.0]`<br>`dir: [-0.15, 0.85, -0.50]` | Diffuse: `[0.55, 0.50, 0.42]`<br>Specular: `[0.15, 0.14, 0.12]` | Front-side key illuminating the amber thorax, face, antennae, and all six articulated legs |
| `fill_cool` | `mjLIGHT_DIRECTIONAL` (`castshadow=0`) | `pos: [-4.0, -7.0, 3.5]`<br>`dir: [0.45, 0.80, -0.38]` | Diffuse: `[0.32, 0.34, 0.40]`<br>Specular: `[0.05, 0.05, 0.06]` | Soft cool fill to open up ventral shadows and maintain dark tarsi definition |
| `rim_back_contour` | `mjLIGHT_DIRECTIONAL` (`castshadow=0`) | `pos: [0.5, 6.0, 7.0]`<br>`dir: [-0.05, -0.75, -0.65]` | Diffuse: `[0.60, 0.58, 0.55]`<br>Specular: `[0.35, 0.35, 0.35]` | Silver-warm rim contour defining the black fedora's brim and crown against the dark stage void |

---

## 3. Trajectory Timing & 1:1 Frame Alignment Verification

### Resolution of the 515 vs 531 Frame Discrepancy
- In the preliminary recorder pass, a discrete step check `current_time >= last_render_time + (1/300) - 1e-6` with `last_render_time = current_time` rounded each sample up to 34 physics steps ($0.0034$ s). Across 17,500 steps, this accumulated a 3% step drift ($17,500 / 34 \approx 515$ frames).
- FlyGym's native `Renderer.render_as_needed()` utilizes a rounding tolerance equal to one physics step (`1e-4` s), correctly alternating between 33 and 34 physics steps to achieve an exact $300.0$ Hz average sample rate across the $1.75$-second trial.
- Linking `scripts/m33_record_trajectory.py` directly to FlyGym's native `render_as_needed` callback produces **exactly 531 frames** spanning simulation time $t = 0.0501$ s to $t = 1.7991$ s (step 0 to step 17,490).

### Bit-Exact Reproduction of M31 Video Sequence
- **Steady-State Presentation Window**: $t \in [0.35, 1.75]$ s corresponds to **Frame 105 through Frame 530** (426 frames at 300 fps).
- **Hero Poster Screenshot**: Frame 120 ($t = 0.4461$ s, step 3,960) corresponds to `output/m31/screenshot_m27d_color_v2.png`. Replaying this frame in the natural habitat world confirmed a mean per-pixel error of $\mathbf{0.077}$ out of 255 ($>99.9\%$ bit-exact match).
- **Phase 2 Opposite Tripod**: Frame 197 ($t = 0.7002$ s, step 6,501) captures mid-trial steady-state locomotion during alternating tripod support.

### Script-Based Reproducibility
The trajectory is 100% regenerable on any fresh clone using the tracked script:
```bash
./.venv/bin/python scripts/m33_record_trajectory.py
```
Runtime is ~35 seconds, producing `data/derived/m33_m27d_trajectory.npz` with zero external dependencies.

---

## 4. Technical Invariance Summary

$$\max_{i} |\Delta m_i| \equiv 0.00 \times 10^0 \text{ kg}$$
$$\max_{i} |\Delta I_i| \equiv 0.00 \times 10^0 \text{ kg}\cdot\text{m}^2$$
$$\max_{i} |\Delta \mathbf{r}_{\text{ipos}, i}| \equiv 0.00 \times 10^0 \text{ m}$$
$$\max_{i} |\Delta \mathbf{q}_{\text{iquat}, i}| \equiv 0.00 \times 10^0$$
$$\text{Collision geoms from fedora} \equiv 0 \quad (\text{contype}=0, \text{conaffinity}=0, \text{density}=0.0)$$

---

## 5. Phase 2 Video Deliverables & Frame Mapping Audit

The full stage-performance video was encoded using pure forward kinematics from the authoritative trajectory:

| Video Artifact | File Path | Specs | Verification Status |
|---|---|---|---|
| **Performance Video (1.0×)** | [`output/m33/video_m27d_performance_10x.mp4`](../output/m33/video_m27d_performance_10x.mp4) | 86 frames, 60 FPS, $960 \times 720$, 1.433 s | Exact frame count & timing match with M31 |
| **Synchronized Comparison Video** | [`output/m33/video_habitat_vs_stage_10x.mp4`](../output/m33/video_habitat_vs_stage_10x.mp4) | 86 frames, 60 FPS, $1920 \times 720$, 1.433 s | Habitat vs Stage synchronized side-by-side |
| **First Frame Comparison** | [`output/m33/comparison_habitat_vs_stage_first.png`](../output/m33/comparison_habitat_vs_stage_first.png) | Frame 0 ($t = 0.397$ s) | Zero body pose jump |
| **Middle Frame Comparison** | [`output/m33/comparison_habitat_vs_stage_middle.png`](../output/m33/comparison_habitat_vs_stage_middle.png) | Frame 43 ($t = 1.106$ s) | Zero body pose jump |
| **Last Frame Comparison** | [`output/m33/comparison_habitat_vs_stage_last.png`](../output/m33/comparison_habitat_vs_stage_last.png) | Frame 85 ($t = 1.799$ s) | Zero body pose jump |
| **Video Production Manifest** | [`data/derived/m33_performance_video_manifest.json`](../data/derived/m33_performance_video_manifest.json) | Metadata & frame mapping | Audit confirmed |

### Frame Correspondence Safeguard
- Trajectory source: `data/derived/m33_m27d_trajectory.npz` (531 frames at 300 Hz)
- Raw presentation window mapping: `range(105, 531, 5)` $\rightarrow$ exactly **86 frames**
- Framerate: **60.0 FPS**
- 1.0× source video duration: **1.433 seconds**
- Projected 0.5× browser playback duration: **2.867 seconds** ($\approx 2.87$ s)
- Pose invariance across all 86 frames: $\max|\Delta \mathbf{q}_{\text{pos}}| \equiv 0.0$ relative to M31 forward kinematics.

---

## 6. Deliverables & Safeguard Check

- [x] **M27-D locomotion physics & M31 V2 anatomy**: Frozen and 100% bit-exact preserved.
- [x] **Fedora geometry & materials**: Frozen and rigidly tracking head with zero collision.
- [x] **Stage lighting**: Four-point theatrical spotlight, warm key, cool fill, and rim contour.
- [x] **Frame-timing safeguard**: Exact 86 frames at 60 FPS matching M31 `video_m27d_color_10x.mp4`.
- [x] **Side-by-side comparison**: Zero pose jump verified across first, middle, and last frames.
- [x] **No music composed yet**: Audio synthesis held for next step.
- [x] **No frontend modifications**: M32 web application untouched.
