# This script allows for stereo camera calibration

# Activate udev rules for cameras
/lib/systemd/systemd-udevd --daemon && udevadm trigger && udevadm settle

# Source ros and workspace setup files, and set the library path for the camera IPA module
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/

# Run the two camera nodes seperately then run camera calibrator
# Currently, this is configured to work with VNC, 
#   but it can be modified to work with a monitor and keyboard/mouse, 
#   or previously it has worked with X11 forwarding, but it has proven unrelibale in comparison
ros2 run camera_ros camera_node --ros-args   -r /camera/image_raw:=/cam0/image_raw   -r /camera/camera_info:=/cam0/camera_info   -p camera:=/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a &
ros2 run camera_ros camera_node --ros-args   -r /camera/image_raw:=/cam1/image_raw   -r /camera/camera_info:=/cam1/camera_info   -p camera:=/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a &
ros2 run camera_calibration cameracalibrator   --size 11x7   --square 0.025   --approximate 0.1   --ros-args   -r left:=/cam0/image_raw   -r right:=/cam1/image_raw   -r left_camera:=/cam0   -r right_camera:=/cam1 &
wait