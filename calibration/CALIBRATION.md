# Stereo Camera Calibration Guide

This guide covers the full calibration process for the IMX708 stereo camera pair,
from capturing images through to updating the OV2SLAM config.

Target stereo reprojection error: **< 1px**

---

## Requirements

- A rigid, flat checkerboard (12x8 square grid = 11x7 inner corners, 25mm squares)
  - Use foam board or aluminium backing — paper warps and will ruin the calibration
- RealVNC Viewer connected to the Raspberry Pi (display `:99`)
- All three terminals open on the Pi (via SSH or VNC)

---

## Step 1 — Start the cameras

The two camera nodes must be started in **separate terminals** because the libcamera
pipeline handler cannot be shared within a single script.

**Terminal 1:**
```bash
./calibration/start_cameras.sh 0
```

**Terminal 2:**
```bash
./calibration/start_cameras.sh 1
```

Wait until both terminals show the camera streaming (no errors).

---

## Step 2 — Start the calibration tool

**Terminal 3:**
```bash
./calibration/stereo_calibration.sh
```

This opens the `camera_calibration` GUI in the VNC viewer showing both camera feeds
side by side, with four progress bars: **X, Y, Size, Skew**.

---

## Step 3 — Capture calibration images

Move the checkerboard around in front of the cameras. The tool auto-captures a pair
when both cameras detect the board simultaneously within 50ms of each other.

Tips for getting all four bars to green (aim for 60-100 captures):
- **X / Y** — move the board to all corners and edges of the frame, including top/bottom
- **Size** — vary the distance: both close (board fills the frame) and far away
- **Skew** — tilt the board at steep angles (~45°) in X and Y
- Hold the board **still** for each capture — motion blur corrupts corner detection
- Cover the **full image area**, especially corners and the top of the frame
- Use good even lighting with no glare on the checkerboard surface

Do not click **Calibrate** until all four bars are fully green.

---

## Step 4 — Calibrate and save

Once all bars are green, click **Calibrate**. The tool will compute the calibration
and display the reprojection error in the terminal.

- If the error is **< 1px**: click **Save** — images are written to `/tmp/` as a `.tar.gz`
- If the error is **> 1px**: add more image pairs (especially at angles and corners)
  and click Calibrate again without restarting

**Copy the archive out of `/tmp/` immediately** — it will be lost on reboot:
```bash
cp /tmp/calibrationdata.tar.gz calibration/
```

Extract and place the `left-XXXX.png` / `right-XXXX.png` pairs into a subdirectory
under `calibration/` (e.g. `calibration/second_calibration/`):
```bash
mkdir calibration/second_calibration
tar -xzf calibration/calibrationdata.tar.gz -C calibration/second_calibration
```

---

## Step 5 — Run the calibration script

```bash
python3 calibration/calibrate_stereo.py --images-dir calibration/second_calibration
```

This processes the image pairs, runs stereo calibration, and prints the results.
Output is also saved to `calibration/calibration_result.txt`.

Check the printed stereo reprojection error. If it is above 1px, go back to Step 3
and recapture with better board coverage (particularly the top of the frame and
steep tilt angles).

---

## Step 6 — Update the OV2SLAM config

Open `parameters_files/accurate/rpi-cam/rpi_cam_stereo.yaml` and replace the
camera parameter block with the values from `calibration/calibration_result.txt`.

**Important:** the calibration script outputs generic topic names (`/left/image_raw`,
`/right/image_raw`) — keep the existing topic names in the YAML:
```yaml
Camera.topic_left: /cam0/image_raw
Camera.topic_right: /cam1/image_raw
```

Replace everything else in the camera block (fx, fy, cx, cy, k1, k2, p1, p2,
body_T_cam0, body_T_cam1) with the new values, and update the baseline and
reprojection error comments at the bottom of the block.

---

## Notes

- **Do not move the cameras relative to each other** between calibration and use —
  even slight flex in the mount invalidates the extrinsics.
- The `cy` values (vertical principal point) should be close to half the image height
  (~300 for 600px). If they are significantly lower, the top of the frame was not
  covered during calibration — recapture with more images in the upper region.
- The `p1`/`p2` tangential distortion values should be small (< 0.01). Large values
  suggest physical misalignment between the two camera modules.
