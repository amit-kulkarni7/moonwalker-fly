# MW-M31 Milestone Report: Full-Colour, Realistic Visual Foundation for Moonwalker Fly

## Executive Summary

Milestone **MW-M31** successfully establishes the full-colour, realistic visual foundation for Moonwalker Fly, transitioning the project from biomechanics locomotion research to visual presentation and user experience. 

The authoritative biomechanical moonwalk controller—**M27-D (`EXCURSION_MATCHED_PLANAR_SUPPORT`)**—remains **100% frozen and bit-exact**. Through a step-by-step mathematical comparison across the entire 17,500-step (1.75-second) dynamic simulation rollout, the new visual configuration was proven to produce:
$$\max_{t \in [0, 1.75\text{s}]} |q_{\text{pos, color}}(t) - q_{\text{pos, mono}}(t)| \equiv 0.00 \times 10^0 \text{ rad/mm}$$
$$\max_{t \in [0, 1.75\text{s}]} |q_{\text{vel, color}}(t) - q_{\text{vel, mono}}(t)| \equiv 0.00 \times 10^0 \text{ rad/mm/s}$$

The sterile monochrome fly on checkered grey tiles has been replaced by a natural *Drosophila melanogaster* fruit fly traversing an organic earthy forest floor in warm natural daylight, with all six legs and contact points remaining clearly visible throughout the moonwalk.

---

## 1. Provenance & Frozen Locomotion Architecture

The underlying controller is the exact protected champion from **MW-M27-D** (via MW-M30 Condition A):
- **Gait Cadence**: 6.0 Hz (period $T = 166.7$ ms)
- **Kinematic Excursion Scaling**:
  $$E_{\text{front}} = 1.000, \quad E_{\text{mid}} = 0.564, \quad E_{\text{hind}} = 0.779$$
- **Turnaround Fraction ($r$)**: 0.06
- **Support Transfer Fraction ($t_f$)**: 0.08
- **Entry Duration**: 0.25 s
- **Support Heights**: Frozen body-frame support heights ($Z_{\text{body}}$) per leg derived from M25/M27
- **Inverse Kinematics**: Dedicated scratch `MjData` DLS IK solver ($\lambda = 10^{-4}$, $w_{\text{reg}} = 0.01$, $|\Delta q| \le 0.05$ rad)
- **Actuators & Adhesion**: 42 active position servos ($k_p = 45.0$), active tarsus adhesion ($k_{\text{adh}} = 40.0$)

---

## 2. Anatomical Drosophila Melanogaster Coloration (V2)

The biological coloration was configured using FlyGym's native `colorize()` system via [`data/visuals/visuals_drosophila_melanogaster_v2.yaml`](../data/visuals/visuals_drosophila_melanogaster_v2.yaml), mapped onto the segmented NeuroMechFly meshes:

| Anatomical Structure | Model Geoms | Color & Material Tuning | Visual Effect |
|---|---|---|---|
| **Thorax (Scutum)** | `fly/c_thorax` | `rgb: [0.42, 0.28, 0.15]`, `mark: none`, spec: 0.12, shin: 0.16 | Natural muted amber-brown chitin; no high-frequency facet noise |
| **Head & Frons** | `fly/c_head` | `rgb: [0.44, 0.30, 0.16]`, `mark: none` | Amber-tan epicranium matching natural Drosophila head |
| **Sensory Aristae** | `fly/*_arista` | `rgba: [0.15, 0.10, 0.06, 1.0]` | Fine dark melanic sensory bristles |
| **Compound Eyes** | `fly/*_eye` | `rgba: [0.44, 0.07, 0.07, 1.0]`, spec: 0.40, shin: 0.50 | Deep authentic burgundy/ruby ommatidia with glossy cornea glint |
| **Abdomen (A1–A4)** | `c_abdomen12..4` | Gradient: tan `[0.54, 0.38, 0.22]` to dark `[0.10, 0.06, 0.03]` | Distinct segmented tiger-striping visible through wings |
| **Abdomen (A5–A6)** | `c_abdomen5..6` | Gradient/Flat: solid dark melanic tip `[0.08, 0.05, 0.03]` | Authentic melanized dorsal posterior cap |
| **Articulated Legs** | `*_coxa`, `femur`, `tibia`, `tarsus` | Amber femur `[0.46, 0.32, 0.17]`, straw tibia `[0.48, 0.34, 0.18]` | Slender warm limbs ending in dark pretarsal claws |
| **Membranous Wings** | `fly/*_wing` | `rgba: [0.68, 0.72, 0.75, 0.13]`, spec: 0.25, shin: 0.20 | Thin glassy transparent membrane with subtle specular edge sheen |
| **Halteres** | `fly/*_haltere` | `rgba: [0.65, 0.52, 0.34, 1.0]` | Pale amber balancing organs |

---

## 3. Natural Habitat & Three-Point Lighting Architecture

1. **Substrate (Flat Plane Physics Invariant)**:
   - Ground geometry remains the exact flat mathematical plane ($z = 0$) with identical contact physics.
   - Textured with a custom organic forest-floor map ([`data/visuals/forest_floor_soil_v2.png`](../data/visuals/forest_floor_soil_v2.png)) featuring deep peat/humus loam and restrained natural moss flecks (roughness $0.92$, reflectance $0.0$).
2. **Natural Daylight Lighting**:
   - **Key Light (Sun)**: Warm diffused directional sunlight (`diffuse: [0.72, 0.68, 0.60]`, ambient $0.32$) with `castshadow=1`, casting soft directional shadows beneath the body and feet.
   - **Fill Light (Sky)**: Cool sky ambient fill (`diffuse: [0.38, 0.42, 0.46]`) from upper-rear-left to illuminate the underbelly and slender legs without blowing out highlights.
   - **Rim Light**: Soft directional contour fill (`diffuse: [0.22, 0.22, 0.20]`) separating limb margins from the dark substrate.
3. **Camera Framing**:
   - Hero side/three-quarter tracking camera (`pos_offset=(-0.65, -6.6, 3.4)`, $29^\circ$ FOV).
   - Entire fly framed comfortably with breathing room on all sides. All six legs remain visible simultaneously.

---

## 4. Deliverables Produced

All deliverables are available in `output/m31/`:

| Deliverable | File Path | Format & Frame Rate | Description |
|---|---|---|---|
| **Hero Screenshot (V2)** | `output/m31/screenshot_m27d_color_v2.png` | $960 \times 720$ PNG | High-resolution hero image showing full fly coloration and substrate |
| **Real-Time Video (1.0×)** | `output/m31/video_m27d_color_10x.mp4` | 60 FPS, H.264 ($960 \times 720$) | Real-time moonwalk playback across steady-state window ($t \in [0.35, 1.75]$ s) |
| **Slow-Motion Video (0.2×)**| `output/m31/video_m27d_color_02x.mp4` | 60 FPS, H.264 ($960 \times 720$) | 0.2× slow-motion playback (7.0 s duration) for choreographic inspection |
| **Synchronized Comparison** | `output/m31/video_monochrome_vs_color_10x.mp4` | 60 FPS, H.264 ($1920 \times 720$) | Side-by-side synchronized comparison of Monochrome Baseline vs Full-Colour V2 |
| **Slow-Motion Comparison** | `output/m31/video_monochrome_vs_color_02x.mp4` | 60 FPS, H.264 ($1920 \times 720$) | 0.2× slow-motion side-by-side comparison |

---

## 5. Technical Limitations & Future Steps

- **Mesh Resolution**: FlyGym's underlying NeuroMechFly meshes are triangular approximations; subtle faceting is inherent to the physics model geometry.
- **Rendering Shaders**: Offscreen MuJoCo rasterization uses standard OpenGL Blinn-Phong shading without ray tracing or dynamic wing venation deformation.
- **Next Milestone**: Interactive user interface, audio synchronization, and choreographic transition controls.
