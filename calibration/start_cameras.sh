#!/bin/bash
# Start a single camera node for stereo calibration.
# Run this script TWICE in two separate terminals:
#
#   Terminal 1: ./calibration/start_cameras.sh 0
#   Terminal 2: ./calibration/start_cameras.sh 1
#
# Then run ./calibration/stereo_calibration.sh in a third terminal.

CAM=${1:-0}

# Activate udev rules for cameras
/lib/systemd/systemd-udevd --daemon && udevadm trigger && udevadm settle

source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/

if [ "$CAM" = "0" ]; then
    echo "Starting cam0 (master, SyncMode=1) on /i2c@88000/imx708@1a"
    ros2 run camera_ros camera_node --ros-args \
      -r /camera/image_raw:=/cam0/image_raw \
      -r /camera/camera_info:=/cam0/camera_info \
      -p camera:=/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a \
      -p SyncMode:=1
elif [ "$CAM" = "1" ]; then
    echo "Starting cam1 (slave, SyncMode=2) on /i2c@80000/imx708@1a"
    ros2 run camera_ros camera_node --ros-args \
      -r /camera/image_raw:=/cam1/image_raw \
      -r /camera/camera_info:=/cam1/camera_info \
      -p camera:=/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a \
      -p SyncMode:=2
else
    echo "Usage: $0 [0|1]"
    exit 1
fi
