# OV²SLAM Parameter Reference

This document explains every setting in the OV²SLAM YAML parameter files.
It is written for someone new to visual SLAM who wants to understand what
each knob does and why it matters.

---

## How the parameter files are organised

```
parameters_files/
├── accurate/          ← High-accuracy preset (more computation, lower speed)
│   ├── euroc/         ← EuRoC MAV benchmark dataset
│   ├── kitti/         ← KITTI autonomous-driving dataset
│   ├── tartanair/     ← TartanAir simulation dataset
│   └── rpi-cam/       ← This project's IMX708 stereo rig on Raspberry Pi 5
├── average/           ← Balanced preset
└── fast/              ← Low-latency preset (less computation, higher speed)
```

Each preset folder contains YAML files tuned for that speed/accuracy tradeoff.
The only file you need for this project is:

**`accurate/rpi-cam/rpi_cam_stereo.yaml`**

YAML changes (calibration values, SLAM tuning) take effect immediately on the next launch — no recompile needed. Only `.cpp` / `.hpp` source changes require
a rebuild.

---

## Section 1 — Camera Parameters

These tell OV²SLAM the physical properties of your cameras. If any value is wrong, the entire SLAM estimate will be incorrect regardless of how well the
SLAM parameters are tuned.

### ROS topic names

```yaml
Camera.topic_left:  /cam1/image_raw
Camera.topic_right: /cam0/image_raw
```

The ROS topic that each camera image is published on.

> **Note for this rig:** the physical left camera is `cam1` and the physical
> right camera is `cam0`. The topic names are swapped from what you might
> expect — this is intentional and matches the calibration.

### Camera model

```yaml
Camera.model_left:  pinhole
Camera.model_right: pinhole
```

The mathematical model used to describe how the camera projects 3D points onto the image plane. `pinhole` is the standard model for almost all cameras. The only alternative OV²SLAM supports is a fisheye model, which is not used here.

### Image resolution

```yaml
Camera.left_nwidth:  800
Camera.left_nheight: 600
Camera.right_nwidth: 800
Camera.right_nheight: 600
```

The width and height of the images in pixels. Must match the actual resolution being streamed. If the camera is streaming at a different resolution, SLAM will
produce wrong scale estimates.

---

### Intrinsic calibration — focal lengths and principal point

```yaml
Camera.fxl: 1240.693312   # Left camera, focal length in X (pixels)
Camera.fyl: 1242.935090   # Left camera, focal length in Y (pixels)
Camera.cxl:  502.875929   # Left camera, principal point X (pixels)
Camera.cyl:  218.045056   # Left camera, principal point Y (pixels)

Camera.fxr: 1242.482172   # Right camera, focal length in X (pixels)
Camera.fyr: 1244.109997   # Right camera, focal length in Y (pixels)
Camera.cxr:  513.156169   # Right camera, principal point X (pixels)
Camera.cyr:  222.264871   # Right camera, principal point Y (pixels)
```

**Focal length (fx, fy):** How strongly the lens magnifies the scene onto the sensor. Expressed in pixels (physical focal length ÷ pixel size). A higher value means the camera has a narrower field of view (zoomed in). fx and fy are usually very close to each other for a well-manufactured lens.

**Principal point (cx, cy):** The pixel coordinate where the optical axis (centre of the lens) hits the image sensor. Ideally this is the exact centre of the image (400, 300 for an 800×600 image), but in practice it is slightly off.

> **Why cy ≈ 218 and not ~300?** The `cy` value being much lower than half the
> image height (300) means the calibration images did not cover the top portion
> of the frame well. If you recalibrate, ensure the checkerboard reaches the
> top of the image.

These four values per camera are determined by the stereo calibration procedure.

Do not edit them by hand — run `calibration/calibrate_stereo.py` and copy the
output.

---

### Lens distortion coefficients

```yaml
Camera.k1l: 0.0639980828    # Left radial distortion, 1st order
Camera.k2l: -0.1030374162   # Left radial distortion, 2nd order
Camera.p1l: -0.0229625505   # Left tangential distortion
Camera.p2l: -0.0059209159   # Left tangential distortion

Camera.k1r: 0.0182392817    # Right radial distortion, 1st order
Camera.k2r: 0.4558669783    # Right radial distortion, 2nd order
Camera.p1r: -0.0230022984   # Right tangential distortion
Camera.p2r: -0.0046476242   # Right tangential distortion
```

Real lenses are not perfect. Light passing through the edges of a lens is bent more than light through the centre, creating a "barrel" or "pincushion" distortion in the image. These coefficients describe that distortion so the software can correct for it.

**k1, k2 (radial distortion):** Describe how much straight lines bow outward (barrel, positive k1) or inward (pincushion, negative k1). k1 is the dominant term; k2 corrects for higher-order effects farther from the image centre.

**p1, p2 (tangential distortion):** Caused by the lens not being perfectly parallel to the sensor. Usually small (< 0.01). Large values suggest physical misalignment of the camera modules.

Do not edit these by hand — they come from calibration.

---

### Extrinsic calibration — camera poses

```yaml
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0.9999..., -0.0068..., 0.0069..., 0.02957...,
          ...]
```

A 4×4 transformation matrix (rotation + translation) that describes where each camera is positioned relative to a common reference frame called the "body" frame. Written as `T_body_camera`, meaning: "apply this transform to a point in camera coordinates to get the point in body coordinates."

**cam0 is the identity matrix** — it defines the origin. The entire coordinate system is anchored to cam0.

**cam1's matrix** encodes:
- The top-left 3×3 block is a rotation matrix — it describes how cam1 is
  rotated relative to cam0 (very close to identity, meaning the cameras are
  nearly parallel).
- The rightmost column (the first three values of the last column) is the
  translation vector in metres: `[0.02957, 0.000081, 0.00111]`. This says
  cam1 is ~2.96 cm to the left of cam0 along the X axis. This is the stereo
  **baseline**.

The baseline is the most important extrinsic value — it is the ruler that gives
the SLAM system its absolute scale. With a 2.96 cm baseline, OV²SLAM reports
positions in real metres.

Do not edit these by hand — they come from calibration.

---

## Section 2 — SLAM Parameters

Unlike the camera parameters, these are algorithmic tuning knobs. They can be changed without recalibrating the cameras, and most take effect on the next launch (no rebuild).

---

### Debug and logging

```yaml
debug: 1
log_timings: 0
```

**`debug`:** When `1`, prints verbose per-frame status to the terminal: KLT track counts, stereo match counts, 3D point counts, pose, loop closure activity, and timing summaries. Set to `0` for cleaner output in production.

**`log_timings`:** When `1`, logs detailed timing breakdowns for every pipeline stage to a file. Useful for profiling performance bottlenecks. Leave at `0` unless you are benchmarking.

---

### Sensor mode

```yaml
mono: 0
stereo: 1
```

Whether to run in monocular or stereo mode. Exactly one must be `1`.

**Monocular (`mono: 1`):** Only the left camera is used. The system can estimate direction of motion but **cannot determine absolute scale** — all distances are relative. Works well for smooth, forward motion with sufficient parallax.

**Stereo (`stereo: 1`):** Both cameras are used. The fixed baseline provides **absolute metric scale** from the very first frame. More robust to initialization and static scenes. This is the mode used for this project.

---

### Real-time enforcement

```yaml
force_realtime: 0
```

When `1`, the system drops frames if the back-end processing falls behind the camera frame rate, ensuring the front-end always tracks the latest image. When `0`, the system processes every frame even if it means falling behind real time.

For live use on the Raspberry Pi, `0` (off) is safer — the Pi is slow enough that forcing real-time can cause the tracker to lose frames during heavy computation (e.g. loop closure). Set to `1` only if you are replaying a recorded bag file and want to test at full speed.

---

### Estimator mode

```yaml
slam_mode: 1
```

Controls whether the system runs as pure odometry or full SLAM.

- `slam_mode: 1` — Full SLAM mode. Maintains a map of 3D landmarks, runs bundle adjustment (optimises both camera poses and map point positions together), and supports loop closure. Better accuracy over long runs.
- `slam_mode: 0` — Visual odometry mode. Lighter computation, no global map maintenance. Useful for very constrained hardware.

---

### Loop closing

```yaml
buse_loop_closer: 1
```

When `1`, enables the loop closure module. Loop closure detects when the camera revisits a previously seen place and "closes" the accumulated drift by re-aligning the current pose with the stored map of that location.

Without loop closure, odometry drift accumulates linearly over time — after travelling 10 m you might be off by 20–50 cm. With loop closure, returning to a known location resets that drift.

Confirmed active in the debug output as:
```
[LoopCloser] >>> Adding KF #XXX ... to insert into Vocabulary Tree
```
A detection would print `[LoopCloser] Loop detected!`.

**Requires:** the camera must revisit a previously seen place with enough recognisable visual features. Works best in textured indoor environments.

---

### Stereo rectification

```yaml
bdo_stereo_rect: 0
alpha: 0.
```

**`bdo_stereo_rect`:** When `1`, assumes the stereo pair has been rectified — i.e., the two image planes are perfectly co-planar and the epipolar lines are exactly horizontal. This allows a simpler, faster stereo matching algorithm (just compare pixels on the same row).

When `0` (our setting), the system handles non-rectified stereo using the full fundamental matrix for epipolar geometry. This is necessary because the IMX708 cameras have slight physical misalignment.

**`alpha`:** Only used when `bdo_stereo_rect: 1`. Controls how much of the image border is kept after rectification warping (0 = crop to valid pixels, 1 = keep full border with black fill). Ignored when rectification is off.

---

### Image undistortion

```yaml
bdo_undist: 0
```

When `1`, the system warps each image to remove lens distortion before processing. This costs CPU time but can improve feature tracking in images with
strong barrel distortion.

When `0`, the system uses the distortion coefficients mathematically (applies them when computing bearing vectors and projections) without warping the pixels. This is faster and is the correct setting for this rig. Note: CLAHE contrast enhancement (`use_clahe`) is unrelated — it changes pixel intensities but not pixel positions, so it does not conflict with leaving geometric undistortion off.

---

### Keyframe parallax threshold

```yaml
finit_parallax: 5.
```

The minimum average pixel movement (parallax) between the current frame and the last keyframe before a new keyframe is created. A keyframe is a frame saved into the map for future reference.

**Lower value (e.g. 5 px):** Creates keyframes more frequently. Better for slow-moving cameras; builds the map faster. More CPU load.

**Higher value (e.g. 20 px):** Only creates keyframes when there has been substantial camera motion. Better for fast-moving cameras to avoid redundant keyframes.

For a hand-held or drone-mounted camera at typical indoor speeds, 5 px is appropriate. The EuRoC/KITTI presets use 20 px because those datasets involve faster platform motion.

---

### Feature detector

```yaml
use_shi_tomasi: 0
use_fast:        0
use_brief:       1
use_singlescale_detector: 1
```

Which algorithm to use to find and describe image features (corners and distinctive patches that can be tracked across frames).

**`use_shi_tomasi`:** Shi-Tomasi corner detector (also called Good Features To Track). High quality, moderate speed.

**`use_fast`:** FAST corner detector. Very fast but less precise. Used in the `fast/` presets.

**`use_brief`:** BRIEF binary descriptor. Describes a small patch around each keypoint as a compact binary string. Used for loop closure — matching descriptors between keyframes to detect revisited places. (AKA if loop closure is on, this mustbe set to `1`)

**`use_singlescale_detector`:** When `1`, detects features at a single image scale only (faster). When `0`, uses a multi-scale pyramid (slower but finds features at different distances).

> For this project: single-scale BRIEF detection is the right trade-off for
> the Raspberry Pi's limited CPU.

---

### Feature density

```yaml
nmaxdist: 35
```

Minimum pixel distance between any two detected keypoints. Controls how densely features are spread across the image.

**Lower value (e.g. 20):** More features, denser coverage. Better tracking through textureless regions, but more CPU load.

**Higher value (e.g. 50):** Fewer features, sparser coverage. Faster but may lose tracking in sparse-texture scenes.

35 px on an 800×600 image gives roughly 400–500 keypoints per frame, which is a good balance for the Pi 5.

---

### Feature quality thresholds

```yaml
nfast_th: 10
dmaxquality: 0.001
```

**`nfast_th`:** Minimum response score for the FAST detector. Higher = only the sharpest corners are kept. Ignored when `use_fast: 0` (which it usually is for loop closure).

**`dmaxquality`:** Minimum quality ratio for the single-scale detector (and Shi-Tomasi if used). Both use GFTT-style quality scoring internally, so this threshold applies to either. Features with quality below `dmaxquality × best_feature_quality` are discarded. Smaller = more features accepted. Only ignored when both `use_shi_tomasi: 0` and `use_singlescale_detector: 0` (i.e. FAST-only mode).

---

### Image pre-processing — CLAHE

```yaml
use_clahe: 1
fclahe_val: 3
```

**`use_clahe`:** When `1`, applies CLAHE (Contrast Limited Adaptive Histogram Equalisation) to each image before feature detection. CLAHE boosts local contrast, making features more detectable in dark or over-exposed regions. Highly recommended for indoor scenes with uneven lighting (e.g. a bright window next to a dark wall).

**`fclahe_val`:** The "clip limit" — how aggressively contrast is boosted. Range is typically 1–10. Higher values boost contrast more but can amplify noise. 3 is a good default for indoor scenes.

---

### KLT optical flow tracking

KLT (Kanade-Lucas-Tomasi) is the algorithm used to track feature points from one frame to the next. It works by searching a small window around each point in the new frame to find where it moved.

```yaml
do_klt: 1
```
Enable KLT tracking. Should always be `1` — the alternatives are not implemented for this use case.

```yaml
klt_use_prior: 1
```
When `1`, uses the previous frame's velocity to predict where each point will be in the new frame (constant-velocity prior). This "warm-starts" KLT closer to the true location, reducing the search needed. Strongly recommended — disabling it causes KLT to start from the exact previous position, which may miss fast motions.

```yaml
btrack_keyframetoframe: 0
```
When `1`, tracks features from the last keyframe to the current frame (skipping intermediate frames). When `0`, tracks frame-to-frame. Frame-to-frame is smoother and more robust for live use.

```yaml
nklt_win_size: 9
```
The half-width of the KLT search window in pixels. The actual window is `(2×nklt_win_size + 1) × (2×nklt_win_size + 1)` = 19×19 pixels. Larger windows track through more blur and illumination change but are slower and can "slip" past sharp edges.

```yaml
nklt_pyr_lvl: 4
```
Number of pyramid levels used for KLT. KLT builds a stack of downsampled images (pyramid) and tracks from coarse to fine. Each extra level doubles the maximum displacement that can be tracked:

| nklt_pyr_lvl | Max trackable motion |
|---|---|
| 1 | ~9 px |
| 2 | ~18 px |
| 3 | ~36 px |
| 4 | ~72 px |

With a 2.96 cm baseline, a feature at 1 m distance has ~37 px disparity between left and right cameras. `pyr_lvl: 4` is required to track stereo pairs at close range. Using `pyr_lvl: 3` causes stereo matching to fail for anything closer than ~2 m.

```yaml
nmax_iter: 30
fmax_px_precision: 0.01
```
KLT iteration limit and convergence criterion. Stops iterating once the feature moves less than `fmax_px_precision` pixels between iterations, or after `nmax_iter` iterations. These OpenCV defaults are fine for all normal uses (nmax_iter: 30 and fmax_px_precision: 0.01).

```yaml
fmax_fbklt_dist: 2.0
```
Forward-backward KLT consistency check threshold. KLT tracks each point forward (frame N → N+1), then backward (N+1 → N), and rejects the point if it doesn't return within `fmax_fbklt_dist` pixels of its starting position. This catches points that drifted to the wrong location.

**Lower value (e.g. 0.5):** Stricter — only the most reliable tracks survive. Fewer tracks, higher quality.

**Higher value (e.g. 2.0):** More permissive — keeps marginally-reliable tracks. More tracks, slightly lower quality. Useful for slow motion or near-stationary scenes where genuine movement is small.

```yaml
nklt_err: 100.
```
Maximum allowed KLT tracking error, as reported by OpenCV's `calcOpticalFlowPyrLK` when run with the `OPTFLOW_LK_GET_MIN_EIGENVALS` flag. This is a normalised internal metric — it is **not in pixels**. Lower values indicate a better-conditioned, more reliable track; higher values indicate an ambiguous or low-texture patch.

Concrete scale reference:
| Value | Meaning |
|---|---|
| < 10 | Very reliable track — strong corner, sharp texture |
| 10–30 | Good track — typical well-lit indoor corner |
| 30–100 | Marginal track — low-texture surface (painted wall, floor) |
| > 100 | Poor track — nearly featureless region, likely to drift |

The original OV²SLAM default is `30`, which rejects any track in the "marginal" band. This rig uses `100` to retain keypoints on the plain indoor surfaces common in the test environment — at the cost of slightly noisier tracks. If you see a lot of feature drift or jumpy pose estimates, tightening this back toward `30` may help.

---

### Local map matching

```yaml
bdo_track_localmap: 1
```
When `1`, after tracking features frame-to-frame, the system also projects nearby 3D map points into the current frame and matches them to keypoints. This is one of the most important accuracy-boosting steps — it re-establishes connections to the map even when KLT temporarily loses a feature, and increases the number of 3D constraints used for pose estimation.

```yaml
fmax_desc_dist: 0.2
```
Maximum allowed descriptor distance when matching a map point to a keypoint. This is a **fraction of the total descriptor length in bits** — the code converts it as `threshold_bits = fmax_desc_dist × 256` (for 256-bit BRIEF). Two descriptors are compared using Hamming distance (count of bits that differ).

Concrete scale:

| fmax_desc_dist | Threshold (bits out of 256) | Meaning |
|---|---|---|
| 0.1 | 26 bits | Very strict — only near-identical patches match |
| 0.2 | 51 bits | Default — good balance for well-calibrated systems |
| 0.35 | 90 bits | Permissive — accepts similar but not identical patches |
| 0.5 | 128 bits | Useless — random descriptors already differ by ~128 bits on average |

The loop closer uses `fmax_desc_dist × 1.5` (= 0.30, ~77 bits) internally for its own matching pass, giving it slightly more tolerance than the local map matcher.

```yaml
fmax_proj_pxdist: 2.
```
When projecting a 3D map point into the current frame, only consider keypoints within `fmax_proj_pxdist` pixels of the projected location as potential matches. This is a spatial pre-filter before descriptor comparison.

At 800×600 with ~400 keypoints, a 2 px radius covers roughly 1–2 candidate keypoints on average — tight enough to prevent false matches without missing the correct one when pose prediction is accurate.

**Adaptive behaviour:** when the frame has fewer than 30 tracked 3D points, the system automatically doubles this to 4 px to compensate for a less certain pose prediction.

---

### RANSAC and epipolar filtering

```yaml
doepipolar: 1
```
Enables epipolar filtering after KLT tracking. Uses the 5-point Essential Matrix algorithm to find a consistent set of feature motions and reject outliers (mismatched features). Essential for robust pose estimation. Always leave enabled.

```yaml
dop3p: 0
```
When `1`, uses P3P (Perspective-3-Point) RANSAC to estimate pose from 3D–2D correspondences (3D map points projected into the current frame). When `0`, uses the 5-point Essential Matrix method (2D–2D correspondences only). P3P requires at least 5 3D points; if your map is sparse, leave this `0`.

```yaml
bdo_random: 1
```
When `1`, randomises RANSAC sampling (non-deterministic). When `0`, uses a fixed seed (reproducible but potentially biased). Always `1` for live use.

```yaml
nransac_iter: 100
```
Number of RANSAC hypothesis iterations. More iterations = higher probability of finding the true inlier set, at the cost of more CPU time. 100 is a good default.

```yaml
fransac_err: 10.
```
The pixel reprojection error threshold used inside the epipolar / RANSAC filtering to classify a feature as an inlier or outlier. In practical terms on an 800×600 image:

| fransac_err | Meaning |
|---|---|
| 1–2 px | Extremely tight — only valid for near-perfect calibration |
| 3 px | Standard — original OV²SLAM default, suitable for well-calibrated rigs |
| 5–10 px | Relaxed — for slightly imperfect calibration or rough hand-held motion |
| > 15 px | Too permissive — wrong matches will be accepted as inliers |

This rig uses `10` because the motion is hand-held with occasional fast rotation, and the calibration has ~1 px stereo reprojection error.

> **Important:** this value does NOT control stereo matching. It only affects the temporal (frame-to-frame) epipolar filter and P3P RANSAC. The stereo matching Sampson distance is a separate hardcoded threshold in the C++ source (`src/map_manager.cpp`).

---

### Triangulation and map point quality

```yaml
fmax_reproj_err: 5.
```
After a new 3D map point is triangulated (stereo or temporal), it is projected back into the image and the pixel distance to the original keypoint is measured. If this reprojection error exceeds `fmax_reproj_err` pixels, the point is discarded.

**Lower value (e.g. 3 px):** Only keeps well-triangulated points. Fewer map points but higher quality.

**Higher value (e.g. 5–7 px):** Accepts points with slightly noisy triangulation. More map points, helpful when the stereo calibration is imperfect or the baseline is small.

> For this rig, 5 px is used because the 2.96 cm baseline gives relatively
> small disparities, and the reprojection errors after triangulation are
> naturally a bit larger than they would be for a wider baseline.

```yaml
buse_inv_depth: 1
```
When `1`, map points are parameterised by their inverse depth (1/distance) rather than their 3D coordinates. Inverse depth is numerically better-conditioned for points near the horizon (very large depth), where small angular errors translate to huge position uncertainties. Almost always `1`.

---

### Bundle adjustment — Ceres solver

Bundle adjustment (BA) jointly optimises all camera poses and 3D map point
positions to minimise reprojection errors. It is what makes SLAM accurate over
long runs.

```yaml
robust_mono_th: 5.9915
# (20% : 3.2189 / 10% : 4.6052 / 5% : 5.9915 / 2%: 7.8240 / 1%: 9.2103)
```
Threshold for the Cauchy robust loss function used in bundle adjustment. Any residual above this threshold is down-weighted (treated as an outlier). The comment shows chi-squared thresholds at different confidence levels for a 2-DOF residual:

The value translates to approximate pixel error as follows — at 5% significance (default), map points with more than ~1.8 px reprojection error after BA convergence get down-weighted:

| robust_mono_th | Significance | Approx. pixel error |
|---|---|---|
| 3.2189 | 20% | ~1.3 px |
| 4.6052 | 10% | ~1.6 px |
| 5.9915 | 5% (default) | ~1.8 px |
| 7.8240 | 2% | ~2.1 px |
| 9.2103 | 1% | ~2.3 px |

- Lower value = more aggressive outlier rejection (may discard valid points in noisy conditions).
- Higher value = more tolerant of noisy map points.

```yaml
use_sparse_schur: 1
```
Uses the Schur complement trick to exploit the sparse block structure of the bundle adjustment problem. Dramatically faster than a dense solve. Always `1`.

```yaml
use_dogleg: 0
use_subspace_dogleg: 0
```
Alternative optimisation strategies within Ceres. Dogleg can converge faster on some problems. In practice, the default Levenberg-Marquardt (both `0`) is more robust for SLAM.

```yaml
use_nonmonotic_step: 0
```
Allows the optimiser to temporarily accept steps that increase the cost, hoping to escape local minima. Generally leave `0`.

```yaml
apply_l2_after_robust: 1
```
After the robust (outlier-resistant) BA pass, runs a second refinement pass with a standard L2 (least squares) cost on the inlier set. This polishes the solution to a tighter optimum. Always `1`.

---

### Co-visibility graph

```yaml
nmin_covscore: 25
```
Minimum number of shared map points (co-observations) for two keyframes to be considered "co-visible" and included in each other's local bundle adjustment window. Higher = only tightly connected keyframes are optimised together (faster but less global). 25 is a good default for indoor scenes.

---

### Keyframe culling

```yaml
fkf_filtering_ratio: 0.95
```
After each bundle adjustment, redundant keyframes are removed to keep the map manageable. A keyframe is considered redundant if at least `fkf_filtering_ratio` (95%) of its map points are also observed by at least 3 other keyframes.

**Higher value (e.g. 0.95):** Aggressively removes keyframes — keeps the map small and fast, but may remove keyframes needed for future loop closure.

**Lower value (e.g. 0.8):** Keeps more keyframes — larger map, more memory, but better loop closure recall.

---

### Final bundle adjustment

```yaml
do_full_ba: 0
```
When `1`, runs a full global bundle adjustment over all keyframes and map points when the system shuts down. This produces the most accurate final trajectory but can take a long time (minutes for a long run). For live real-time use, leave `0`.

---

## Quick reference — values changed from OV²SLAM defaults for this rig

| Parameter | Default | This rig | Reason |
|---|---|---|---|
| `nklt_pyr_lvl` | 3 | 4 | 2.96 cm baseline needs ~37 px stereo search range |
| `fmax_fbklt_dist` | 0.5 | 2.0 | Slow/hover motion keeps FB error slightly above 0.5 |
| `nklt_err` | 30 | 100 | Indoor walls are low-texture; keeps more keypoints |
| `fransac_err` | 3 | 10 | Temporal epipolar tolerance for slightly rough motion |
| `fmax_reproj_err` | 3 | 5 | Small baseline means looser reprojection after triangulation |
| `finit_parallax` | 20 | 5 | Hand-held motion is slower; need lower threshold for KF creation |

---

## Preset comparison

The `accurate/`, `average/`, and `fast/` folders differ mainly in:

| Setting | fast | average | accurate |
|---|---|---|---|
| `use_fast` detector | 1 | varies | 0 |
| `use_singlescale_detector` | 0 | varies | 1 |
| `nmaxdist` | 50 | 40 | 35 |
| `use_clahe` | 0 | varies | 1 |
| `buse_loop_closer` | 0 | varies | 1 |
| `fkf_filtering_ratio` | 0.9 | 0.9 | 0.95 |

The `fast` preset trades accuracy for speed by using a faster detector, fewer features, no contrast enhancement, and no loop closure. The `accurate` preset does the opposite.
