# ============================================================
# OV²SLAM — Raspberry Pi 5 (arm64) fork
# ============================================================
#
# Changes from the upstream Dockerfile (documented here):
#
# 1. ARCHITECTURE defaults to arm64v8 (was amd64).
#
# 2. libcamera 0.7.0 is built from source.
#    Ubuntu 22.04 apt does not carry the Raspberry Pi IPA modules
#    needed for IMX708 cameras. The IPA modules are installed to
#    /usr/local/lib/aarch64-linux-gnu/libcamera/ipa/ and must be
#    pointed at via LIBCAMERA_IPA_MODULE_PATH at runtime.
#
# 3. camera_ros is built from source in a separate /ros2_ws.
#    This is the ROS 2 driver that wraps libcamera and publishes
#    /cam0/image_raw and /cam1/image_raw for OV²SLAM.
#
# 4. message_filters is added as a ROS dependency.
#    ov2slam_node.cpp uses message_filters::TimeSynchronizer to
#    sync left/right image streams for stereo. This was required
#    to handle the bgra8 (XRGB8888) format the IMX708 outputs.
#
# 5. Build parallelism is capped at -j2.
#    The Raspberry Pi 5 has 8 GB RAM but compiling ov2slam.cpp
#    at -O3 with debug symbols causes cc1plus OOM at -j4.
#    The source CMakeLists.txt already sets -O2 (no -g) for the
#    main library and -O1 -g0 for ov2slam_node.cpp specifically.
#
# 6. udev and v4l2 tools are added.
#    udevd must run inside the container for camera device
#    enumeration. Start it before launching camera nodes:
#      /lib/systemd/systemd-udevd --daemon && udevadm trigger && udevadm settle
#
# Runtime env var required (set in your run command or compose file):
#   LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
# ============================================================

# System architecture: arm64v8 for Raspberry Pi 5, amd64 for x86
ARG ARCHITECTURE=arm64v8
ARG ROS_DISTRO=humble
ARG BASE_IMAGE=${ARCHITECTURE}/ros:${ROS_DISTRO}-ros-core
FROM $BASE_IMAGE

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y curl \
    && sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-colcon-common-extensions build-essential cmake pkg-config git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/ov2slam/Thirdparty
RUN git clone https://gitlab.com/libeigen/eigen.git
RUN git clone https://github.com/opencv/opencv.git
RUN git clone https://github.com/opencv/opencv_contrib.git
RUN git clone https://github.com/laurentkneip/opengv.git

##############
### Eigen3 ###
##############
WORKDIR /ws/src/ov2slam/Thirdparty/eigen
RUN git checkout 3.3.7
RUN mkdir build && cd build/ \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
    && make -j2 install \
    && ln -s /usr/local/include/eigen3/Eigen /usr/local/include/Eigen \
    && rm -rf /usr/include/eigen3

##############
### OpenCV ###
##############
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libgtk2.0-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        python3-dev \
        python3-numpy \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/ov2slam/Thirdparty/opencv_contrib
RUN git checkout 4.5.0

WORKDIR /ws/src/ov2slam/Thirdparty/opencv
RUN git checkout 4.5.0
RUN mkdir build install && cd build \
    && cmake .. -D CMAKE_BUILD_TYPE=Release \
        -D INSTALL_C_EXAMPLES=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_EXTRA_MODULES_PATH="/ws/src/ov2slam/Thirdparty/opencv_contrib/modules" \
        -D BUILD_EXAMPLES=OFF \
        -D OPENCV_ENABLE_NONFREE=ON \
        -D WITH_IPP=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_opencv_adas=OFF \
        -D BUILD_opencv_bgsegm=OFF \
        -D BUILD_opencv_bioinspired=OFF \
        -D BUILD_opencv_ccalib=OFF \
        -D BUILD_opencv_datasets=ON \
        -D BUILD_opencv_datasettools=OFF \
        -D BUILD_opencv_face=OFF \
        -D BUILD_opencv_latentsvm=OFF \
        -D BUILD_opencv_line_descriptor=OFF \
        -D BUILD_opencv_matlab=OFF \
        -D BUILD_opencv_optflow=ON \
        -D BUILD_opencv_reg=OFF \
        -D BUILD_opencv_saliency=OFF \
        -D BUILD_opencv_surface_matching=OFF \
        -D BUILD_opencv_text=OFF \
        -D BUILD_opencv_tracking=ON \
        -D BUILD_opencv_xobjdetect=OFF \
        -D BUILD_opencv_xphoto=OFF \
        -D BUILD_opencv_stereo=OFF \
        -D BUILD_opencv_hdf=OFF \
        -D BUILD_opencv_cvv=OFF \
        -D BUILD_opencv_fuzzy=OFF \
        -D BUILD_opencv_dnn=OFF \
        -D BUILD_opencv_dnn_objdetect=OFF \
        -D BUILD_opencv_dnn_superres=OFF \
        -D BUILD_opencv_dpm=OFF \
        -D BUILD_opencv_quality=OFF \
        -D BUILD_opencv_rapid=OFF \
        -D BUILD_opencv_rgbd=OFF \
        -D BUILD_opencv_sfm=OFF \
        -D BUILD_opencv_shape=ON \
        -D BUILD_opencv_stitching=OFF \
        -D BUILD_opencv_structured_light=OFF \
        -D BUILD_opencv_alphamat=OFF \
        -D BUILD_opencv_aruco=OFF \
        -D BUILD_opencv_phase_unwrapping=OFF \
        -D BUILD_opencv_photo=OFF \
        -D BUILD_opencv_gapi=OFF \
        -D BUILD_opencv_video=ON \
        -D BUILD_opencv_ml=ONN \
        -D BUILD_opencv_python2=OFF \
        -D WITH_GSTREAMER=OFF \
        -D ENABLE_PRECOMPILED_HEADERS=OFF \
    && make -j2 install

##############
### OpenGV ###
##############
WORKDIR /ws/src/ov2slam/Thirdparty/opengv
RUN mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
    && make -j2 install

ADD ./Thirdparty /ws/src/ov2slam/Thirdparty

##############
### Sophus ###
##############
WORKDIR /ws/src/ov2slam/Thirdparty/Sophus
RUN mkdir build install && cd build/ \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DBUILD_SOPHUS_EXAMPLES=OFF \
    && make -j2 install

####################
### Ceres Solver ###
####################
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/ov2slam/Thirdparty/ceres-solver
RUN mkdir build install && cd build/ \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_CXX_STANDARD=14 -DBUILD_EXAMPLES=OFF \
    && make -j2 install

################
### OBIndex2 ###
################
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libboost-system-dev libboost-filesystem-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/ov2slam/Thirdparty/obindex2
RUN mkdir build && cd build/ \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
    && make -j2 install

################
### iBoW-LCD ###
################
WORKDIR /ws/src/ov2slam/Thirdparty/ibow_lcd
RUN mkdir build && cd build/ \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
    && make -j2 install

#################
### libcamera ###
#################
# Must be built from source — Ubuntu 22.04 apt packages do not include
# the Raspberry Pi IPA modules required for IMX708 cameras.
# Version 0.7.0 matches what is tested on this rig.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        meson \
        ninja-build \
        python3-pip \
        python3-dev \
        python3-yaml \
        python3-ply \
        python3-jinja2 \
        libyaml-dev \
        libudev-dev \
        libevent-dev \
        libssl-dev \
        libgnutls28-dev \
        libtiff-dev \
        udev \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyyaml ply jinja2

WORKDIR /opt
RUN git clone https://git.libcamera.org/libcamera/libcamera.git \
    && cd libcamera \
    && git checkout v0.7.0 \
    && meson setup build \
        -Dpipelines=rpi/vc4,rpi/pisp \
        -Dipas=rpi/vc4,rpi/pisp \
        -Dcam=disabled \
        -Dtest=false \
        -Dprefix=/usr/local \
    && ninja -C build install \
    && ldconfig

##################
### camera_ros ###
##################
# ROS 2 driver wrapping libcamera. Publishes image_raw topics consumed
# by OV²SLAM. Built in a separate workspace (/ros2_ws) so it can source
# /opt/ros/humble independently.
WORKDIR /ros2_ws
RUN git clone https://github.com/christianrauch/camera_ros.git src/camera_ros

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-image-transport \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH \
       colcon build --packages-select camera_ros \
       --cmake-args -DCMAKE_PREFIX_PATH=/usr/local

#################
### cv_bridge ###
#################
WORKDIR /ws
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libboost-python-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/ros-perception/vision_opencv.git -b ${ROS_DISTRO} src/vision_opencv
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --packages-select cv_bridge

###############
### OV²SLAM ###
###############
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-message-filters \
        libpcl-dev \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pcl-msgs \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-tf2 \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-ros2bag \
        ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-backward-ros \
    && rm -rf /var/lib/apt/lists/*

ADD . /ws/src/ov2slam

# Note: CMakeLists.txt in this fork already sets:
#   -O2 (not -O3) for the SLAM library  — avoids cc1plus OOM on RPi
#   -O1 -g0 for ov2slam_node.cpp        — ROS glue, no need for heavy opt
#   WITH_OPENCV_CONTRIB OFF             — not needed at runtime
#   message_filters as a find_package   — required for stereo sync

WORKDIR /ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH \
       colcon build --cmake-args -DCMAKE_PREFIX_PATH=/usr/local

# Runtime environment
ENV LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/
