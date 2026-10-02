FROM osrf/ros:jazzy-desktop-full

ENV GZ_VERSION=harmonic
ARG DISTRO=jazzy

RUN apt update -y \
    && apt install -y pipx \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt update -y \
    && apt install -y ros-${DISTRO}-plotjuggler \
    ros-${DISTRO}-joint-state-publisher \
    ros-${DISTRO}-joint-state-publisher-gui \
    ros-${DISTRO}-gz-ros2-control \
    ros-${DISTRO}-teleop-twist-joy \
    ros-${DISTRO}-joy \
    ros-${DISTRO}-pinocchio \
    ros-${DISTRO}-ros2-control \
    ros-${DISTRO}-ros2-controllers \
    ros-${DISTRO}-xacro \
    ros-${DISTRO}-rosbag2-storage-mcap \
    ros-${DISTRO}-rmw-cyclonedds-cpp \
    ros-${DISTRO}-nmea-msgs \
    ros-${DISTRO}-mavros-msgs \
    ros-${DISTRO}-realsense2-camera \
    ros-${DISTRO}-realsense2-description \
    ros-${DISTRO}-nav2-common \
    ros-${DISTRO}-nav2-controller \
    ros-${DISTRO}-nav2-smoother \
    ros-${DISTRO}-nav2-planner \
    ros-${DISTRO}-nav2-behaviors \
    ros-${DISTRO}-nav2-bt-navigator \
    ros-${DISTRO}-nav2-waypoint-follower \
    ros-${DISTRO}-nav2-velocity-smoother \
    ros-${DISTRO}-nav2-lifecycle-manager \
    ros-${DISTRO}-nav2-dwb-controller \
    ros-${DISTRO}-nav2-navfn-planner \
    ros-${DISTRO}-nav2-rviz-plugins \
    ros-${DISTRO}-nav2-graceful-controller \
    ros-${DISTRO}-robot-localization \
    ros-${DISTRO}-vision-msgs \
    ros-${DISTRO}-pcl-ros \
    ros-${DISTRO}-pcl-conversions \
    ros-${DISTRO}-cv-bridge \
    ros-${DISTRO}-message-filters \
    ros-${DISTRO}-image-transport \
    ros-${DISTRO}-image-transport-plugins \
    ros-${DISTRO}-compressed-image-transport \
    ros-${DISTRO}-zstd-image-transport \
    zenity \
    python3-rosdep \
    software-properties-common \
    ca-certificates \
    ffmpeg \
    gnupg \
    wget \
    libmetis-dev \
    libasio-dev \
    libpcap0.8-dev \
    iproute2 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi
RUN groupadd --gid ${USER_GID} ${USERNAME} 
RUN useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt update \
    && apt install -y sudo \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}



# --- CUDA repo (Ubuntu 24.04) ---
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
 && dpkg -i cuda-keyring_1.1-1_all.deb \
 && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin \
 && mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600 

ENV CUDAToolkit_ROOT=/usr/local/cuda

RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
RUN sudo add-apt-repository -y ppa:koide3/iridescence
RUN apt update -y \
    && apt install -y libiridescence-dev \
    libgtsam-points-cuda13.1-dev \
    cuda-cudart-13-1 \
    cuda-nvcc-13-1 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


# ============================================================
# PYTHON ML DEPENDENCIES (required by onboard_detector_v2's YOLO nodes)
# ============================================================
# Same stack as jo-sim_lv-dot. The apt sympy/mpmath conflict with the pip
# ones pulled in by torch, so they are removed first.
RUN apt-get update \
    && apt-get remove -y python3-sympy python3-mpmath \
    && apt-get install -y python3-opencv python3-scipy python3-sklearn python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# PyTorch wheels bundle their own CUDA runtime; cu128 only needs a driver
# new enough for it (one supporting CUDA 13.1 is).
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    --retries 10 --timeout 300 \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cu128

# lap is needed by ultralytics' trackers (model.track()); ultralytics would
# otherwise try to pip-install it at runtime, which fails (PEP 668).
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
    "numpy>=2.0" \
    ultralytics \
    "lap>=0.5.12"

ENV PATH=/usr/local/cuda-13.1/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-13.1/lib64:${LD_LIBRARY_PATH}
ENV CUDAToolkit_ROOT=/usr/local/cuda-13.1

USER ros
RUN sudo usermod -aG dialout ros
WORKDIR /home/$USERNAME/
RUN mkdir src

RUN rosdep update

# VELODYNE
RUN cd src/ && git clone https://github.com/ros-drivers/velodyne.git

# IMU
RUN cd src && git clone --branch ros2 https://github.com/mlisi1/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git

# Agilex Bunker
RUN cd src && git clone https://github.com/mlisi1/ugv_sdk.git
RUN cd src && git clone -b jazzy https://github.com/agilexrobotics/bunker_ros2.git

RUN cd src && git clone https://github.com/sutharsan-311/nav2_config.git


RUN cd src && git clone https://github.com/introlab/rtabmap.git
RUN cd src && git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git

SHELL ["/bin/bash", "-c"]

# Build everything except the rtabmap* family with full parallelism.
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-skip-regex '^rtabmap.*' --cmake-args -DCMAKE_BUILD_TYPE=Release

# The rtabmap/rtabmap_ros repos expand into ~14 rtabmap_* packages, several of
# which (rtabmap, rtabmap_slam, rtabmap_sync, rtabmap_util, ...) are heavy per
# compile unit. Building them all at once (as the full-workspace build above
# would) OOMs on 32GB. Build them one package at a time, capped to 8 make jobs each.
RUN source /opt/ros/jazzy/setup.bash \
 && MAKEFLAGS="-j6" colcon build --packages-select-regex '^rtabmap.*' --executor sequential --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

# GLIM (giacomotambe fork, dynamic object rejection), jo_msgs and
# onboard_detector_v2 come from this repo (submodules / folder) and are also
# mounted as volumes at runtime (see compose.yaml). They are copied here only
# so the heavy build is cached in the image; the mounted sources override them.
# Kept after rtabmap so editing them doesn't invalidate that slow build.
COPY --chown=ros:ros jo_msgs             src/jo_msgs
COPY --chown=ros:ros glim                src/glim
COPY --chown=ros:ros glim_ros2           src/glim_ros2
COPY --chown=ros:ros onboard_detector_v2 src/onboard_detector_v2

# jo_msgs first, with --symlink-install like the entrypoint rebuilds it
# (glim_ros2 depends on it).
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select jo_msgs --symlink-install

RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash \
 && colcon build --packages-select glim glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

# Prebuild onboard_detector_v2 with --symlink-install so the entrypoint's
# rebuild at startup is incremental. -DCUDAToolkit_ROOT must be passed
# explicitly: its cmake_minimum_required(3.8) predates CMP0074, so the
# CUDAToolkit_ROOT env var (needed via gtsam_points) is otherwise ignored.
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash \
 && colcon build --packages-select onboard_detector_v2 --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDAToolkit_ROOT=${CUDAToolkit_ROOT}

COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

RUN git clone https://github.com/mlisi1/DendROS && cd DendROS && bash install.sh -y

RUN echo '[ -f /home/ros/utils/utilities.sh ] && source /home/ros/utils/utilities.sh' >> ~/.bashrc


ENTRYPOINT ["/entrypoint.sh"]
