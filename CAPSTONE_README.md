# Stereo Visual Odometry on Raspberry Pi 5 — Capstone Project

Real-time 6-DOF pose estimation using two IMX708 cameras and the OV2SLAM stereo SLAM
system running on a Raspberry Pi 5 under ROS 2 Humble.

---

## Hardware

| Component | Details |
|---|---|
| Single Board Computer | Raspberry Pi 5 (8 GB) |
| Cameras | 2× IMX708 (Raspberry Pi Camera Module 3), fixed focus |
| Baseline | ~2.96 cm horizontal separation |
| Sync | Hardware frame sync — cam0 master (SyncMode=1), cam1 slave (SyncMode=2) |
| Resolution | 800×600 @ 30 fps (ideally, realistically less) |

---

## Software Stack

| Layer | Package |
|---|---|
| OS | Ubuntu 22.04 (aarch64) |
| Middleware | ROS 2 Humble |
| Camera driver | `camera_ros` with libcamera backend |
| SLAM | OV2SLAM (stereo mode, KLT optical flow) |

---

## Running the System

Open five terminals (SSH or VNC to display `:99`).

**Terminal 1 — Left camera (master):**
```bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
/lib/systemd/systemd-udevd --daemon && udevadm trigger && udevadm settle
ros2 run camera_ros camera_node --ros-args \
  -r /camera/image_raw:=/cam0/image_raw \
  -p camera:=/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a \
  -p SyncMode:=1
```

**Terminal 2 — Right camera (slave):**
```bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
ros2 run camera_ros camera_node --ros-args \
  -r /camera/image_raw:=/cam1/image_raw \
  -p camera:=/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a \
  -p SyncMode:=2
```

**Terminal 3 — OV2SLAM:**
```bash
source /opt/ros/humble/setup.bash
source /root/ov2slam-rpi/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
ros2 run ov2slam ov2slam_node \
  /root/ov2slam-rpi/parameters_files/accurate/rpi-cam/rpi_cam_stereo.yaml
```

**Terminal 4 — Visualisation (optional):**
```bash
source /opt/ros/humble/setup.bash
rviz2 -d /root/ov2slam-rpi/ov2slam.rviz
```

**Terminal 5 — Position output:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /vo_pose --field pose.position
```

The `x`, `y`, `z` values are in **meters** relative to the starting pose.

---

## Camera Startup Warnings — What They Mean

The camera nodes print several warnings on startup. The following are harmless.

**`Unsupported V4L2 pixel format RPBP`**
The ISP's internal raw Bayer packed format is not exposed to V4L2 userspace. The
camera still streams correctly in the XRGB8888 colour format selected automatically.

**`no pixel format selected, auto-selecting XRGB8888`**
No `format` parameter was passed to camera_ros, so it chooses the first supported
colour format. XRGB8888 (`bgra8`) is correct — OV2SLAM converts it to grayscale
internally using cv_bridge.

**`no dimensions selected, auto-selecting 800x600`**
No explicit width/height was passed, so the node defaults to 800×600. This matches
the resolution used during stereo calibration.

**`Camera calibration file ... not found`**
camera_ros looks for a ROS-format `camera_info` YAML under `~/.ros/camera_info/`.
This file does not exist because OV2SLAM reads calibration directly from
`parameters_files/accurate/rpi-cam/rpi_cam_stereo.yaml` — the ROS camera_info
mechanism is not used at all.

**`AfWindows / AF_PAUSE / AF_TRIGGER` warnings**
The IMX708 driver exposes autofocus controls, but these camera modules have
fixed-focus lenses. camera_ros tries to configure autofocus and fails silently.
Fixed focus is what SLAM requires — moving focus would change the effective focal
length and invalidate the calibration.

**`Sync mode set to server`**
The libcamera IPA is confirming that hardware frame synchronisation is active.
cam0 acts as the sync master (server) and cam1 as the slave (client). Both cameras
expose frames within ~30–80 µs of each other, which is well within the requirement
for stereo matching.

---

## Calibration

See [calibration/CALIBRATION.md](calibration/CALIBRATION.md) for the full
calibration workflow.

Key results from the most recent calibration:
- Stereo reprojection error: **1.01 px**
- Baseline: **2.96 cm**
- Image pairs used: 91

---

## Output Topics

| Topic | Type | Description |
|---|---|---|
| `/vo_pose` | `geometry_msgs/PoseStamped` | Camera pose in world frame (metres) |
| `/map_points` | `sensor_msgs/PointCloud2` | 3D map points |
| `/image_track` | `sensor_msgs/Image` | Feature tracking visualisation |

---

## Rebuilding

After any changes to `.cpp` or `.hpp` source files:
```bash
cd ~/ov2slam-rpi
colcon build --packages-select ov2slam --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

YAML parameter changes (calibration, SLAM tuning) take effect on the next launch
without rebuilding.
