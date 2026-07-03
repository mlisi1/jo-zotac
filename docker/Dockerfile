FROM osrf/ros:jazzy-desktop-full

ENV GZ_VERSION=harmonic
ARG DISTRO=jazzy

RUN apt update -y \
    && apt install -y pipx \
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
    zenity \
    python3-rosdep \
    software-properties-common \
    ca-certificates \
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

# GLIM
RUN cd src && git clone https://github.com/mlisi1/glim_ros2.git


# Pinned past v1.2.1 to commit a9be365, which fixes ImuFactor-related build
# errors (koide3/glim#306) caused by GTSAM 4.3's templated ImuFactor/
# PreintegratedImuMeasurements no longer matching glim's old forward declarations.
RUN mkdir src/glim && cd src/glim \
    && git init \
    && git remote add origin https://github.com/koide3/glim.git \
    && git fetch --depth 1 origin a9be365001e571ef4b16cfbd74bf1da82672480b \
    && git checkout FETCH_HEAD

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

COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh


RUN git clone https://github.com/mlisi1/DendROS && cd DendROS && bash install.sh -y

COPY docker/save_map.bash /save_map.bash
COPY docker/record_all.bash /record_all.bash
COPY docker/utilities.sh /etc/utilities.sh
RUN echo 'source /etc/utilities.sh' >> ~/.bashrc


ENTRYPOINT ["/entrypoint.sh"]
