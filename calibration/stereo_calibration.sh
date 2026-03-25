#!/bin/bash
# Stereo camera calibration tool.
#
# BEFORE running this script, start both cameras in separate terminals:
#   Terminal 1: ./calibration/start_cameras.sh 0
#   Terminal 2: ./calibration/start_cameras.sh 1
#
# Board: 11x7 inner corners (12x8 square grid), 25mm squares
#
# Tips for a good calibration (target <1px reprojection error):
#   - Use a rigid, flat board (foam board or aluminium, not paper)
#   - Collect 60-100 image pairs
#   - Fill all four progress bars (X, Y, Size, Skew) to green before clicking Calibrate
#   - Cover all corners and edges of the frame
#   - Tilt the board significantly in X and Y (~45 degrees)
#   - Vary distance: both close (board fills frame) and far
#   - Hold the board still for each capture — do not move it during the shot

source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/

# --approximate 0.05: 50ms sync tolerance to reduce mismatched stereo pairs
# --k-coefficients 3: fit k1,k2,k3 for better distortion modelling on the IMX708
ros2 run camera_calibration cameracalibrator \
  --size 11x7 \
  --square 0.025 \
  --approximate 0.05 \
  --k-coefficients 3 \
  --ros-args \
  -r left:=/cam0/image_raw \
  -r right:=/cam1/image_raw \
  -r left_camera:=/cam0 \
  -r right_camera:=/cam1
