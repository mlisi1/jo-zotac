FROM osrf/ros:jazzy-desktop-full

ARG DISTRO=jazzy

RUN apt update -y \
    && apt install -y pipx \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt update -y \
    && apt install -y ros-${DISTRO}-plotjuggler \
    ros-${DISTRO}-joint-state-publisher \
    ros-${DISTRO}-teleop-twist-joy \
    ros-${DISTRO}-joy \
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
    ros-${DISTRO}-std-srvs \
    ros-${DISTRO}-diagnostic-updater \
    ros-${DISTRO}-diagnostic-msgs \
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

# GTSAM 4.3 and iridescence from the koide3 PPA; gtsam_points from source (see GTSAM_POINTS_REF).
# Keep CUDA / GTSAM / gtsam_points / GLIM / glim_ros2 in sync with jo_sim's Dockerfile.
ARG CUDA_VERSION=13-1
ARG CUDA_DIR=13.1
RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash \
 && add-apt-repository -y ppa:koide3/iridescence \
 && apt update -y \
 && apt install -y libiridescence-dev \
    libgtsam-notbb-dev \
    cuda-cudart-dev-${CUDA_VERSION} \
    cuda-nvcc-${CUDA_VERSION} \
    cuda-cccl-${CUDA_VERSION} \
    cuda-driver-dev-${CUDA_VERSION} \
    libcurand-dev-${CUDA_VERSION} \
    libcusparse-dev-${CUDA_VERSION} \
    libcusolver-dev-${CUDA_VERSION} \
 && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV PATH=/usr/local/cuda-${CUDA_DIR}/bin:${PATH}
# ${VAR:+:$VAR}: no trailing ':' when unset (an empty entry means the current directory)
ENV LD_LIBRARY_PATH=/usr/local/cuda-${CUDA_DIR}/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CUDAToolkit_ROOT=/usr/local/cuda-${CUDA_DIR}

# koide3/gtsam_points from source: GLIM master needs IndexedSlidingWindow::has_index, which is
# newer than the last release (1.2.2, the newest in the koide3 PPA).
ARG GTSAM_POINTS_REF=8e94654558dfbe7ddb0341de300daaf17e25ee02
# CUDA architectures ("native" can't work: no GPU during docker build). 89 = RTX 3500 Ada
# (the Zotac). Use all-major for any GPU (slower build). Overridable from .env (compose.yaml).
ARG CUDA_ARCHS=89
ARG BUILD_JOBS=8
RUN mkdir -p /tmp/gtsam_points && cd /tmp/gtsam_points && git init -q \
 && git remote add origin https://github.com/koide3/gtsam_points.git \
 && git fetch -q --depth 1 origin ${GTSAM_POINTS_REF} && git checkout -q FETCH_HEAD \
 && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=ON \
      -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHS}" \
 && cmake --build build -j${BUILD_JOBS} && cmake --install build \
 && ldconfig && rm -rf /tmp/gtsam_points

USER ros
RUN sudo usermod -aG dialout ros
WORKDIR /home/$USERNAME/
RUN mkdir src

RUN rosdep update

# VELODYNE
RUN cd src/ && git clone https://github.com/ros-drivers/velodyne.git

# IMU
RUN cd src && git clone --branch ros2 https://github.com/mlisi1/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git

# GLIM: koide3/glim >= 1a197c9 is needed by glim_ros2@dev (contains the a9be365 GTSAM 4.3
# ImuFactor fix, koide3/glim#306). glim_ros2: mlisi1 fork, branch dev (pinned commit).
ARG GLIM_REF=1a197c999e4865f5cab7c9cd25474128413ebbca
ARG GLIM_ROS_REF=f762545
RUN mkdir src/glim && cd src/glim && git init -q \
    && git remote add origin https://github.com/koide3/glim.git \
    && git fetch -q --depth 1 origin ${GLIM_REF} && git checkout -q FETCH_HEAD
RUN cd src && git clone -q --branch dev https://github.com/mlisi1/glim_ros2.git \
    && cd glim_ros2 && git checkout -q ${GLIM_ROS_REF}

# Agilex Bunker
RUN cd src && git clone https://github.com/mlisi1/ugv_sdk.git
RUN cd src && git clone -b jazzy https://github.com/agilexrobotics/bunker_ros2.git

# RTAB-Map pinned to jo_sim's commits (same visual odometry / SLAM code in the sim and on the robot)
ARG RTABMAP_REF=7d8b585217033e8d25d01b380a36744effe816df
ARG RTABMAP_ROS_REF=3e1cdcb072ad21fa2b129412332760bd3e456279
RUN for r in "rtabmap ${RTABMAP_REF}" "rtabmap_ros ${RTABMAP_ROS_REF}"; do set -- $r; \
      mkdir -p src/$1 && git -C src/$1 init -q \
   && git -C src/$1 remote add origin https://github.com/introlab/$1.git \
   && git -C src/$1 fetch -q --depth 1 origin $2 && git -C src/$1 checkout -q FETCH_HEAD || exit 1; done

SHELL ["/bin/bash", "-c"]

# Build everything except the rtabmap* family with full parallelism.
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-skip-regex '^rtabmap.*' --cmake-args -DCMAKE_BUILD_TYPE=Release

# The rtabmap/rtabmap_ros repos expand into ~14 rtabmap_* packages, several of
# which (rtabmap, rtabmap_slam, rtabmap_sync, rtabmap_util, ...) are heavy per
# compile unit. Building them all at once (as the full-workspace build above
# would) OOMs on 32GB. Build them one package at a time, capped to 6 make jobs each.
RUN source /opt/ros/jazzy/setup.bash \
 && MAKEFLAGS="-j6" colcon build --packages-select-regex '^rtabmap.*' --executor sequential --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

# DendROS (ros2 wrapper / TUI), pinned to a release: bump DENDROS_VERSION for a new one
ARG DENDROS_VERSION=v1.1.0
RUN git clone -q --depth 1 --branch ${DENDROS_VERSION} https://github.com/mlisi1/DendROS.git /tmp/DendROS \
 && bash /tmp/DendROS/install.sh -y \
 && sudo rm -rf /tmp/DendROS

# Testudo (topic health). TESTUDO_VERSION: a tag, branch or commit. For now a commit on dev (the fixes
# from the robot bags after 1.0.0; >= ef154f0 reads node_profiling.nodes, which
# jo_bringup/config/testudo.yaml uses); back to a release tag once dev is merged into main and tagged.
# A commit rather than "dev": the layer is cached by this value, so a branch name never picks up new commits.
ARG TESTUDO_VERSION=50b272e
RUN pip install --user --break-system-packages "textual>=6" \
 && git clone -q https://github.com/mlisi1/testudo.git src/testudo \
 && git -C src/testudo checkout -q ${TESTUDO_VERSION} \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select testudo


# Optional glim_ros2 commit on top of the cached workspace: setting GLIM_ROS_COMMIT (in .env)
# checks it out and rebuilds only glim_ros (minutes), not the whole workspace and RTAB-Map.
# Declared here, not at the top, so changing it doesn't invalidate the earlier layers.
ARG GLIM_ROS_COMMIT=
RUN if [ -n "${GLIM_ROS_COMMIT}" ]; then \
      cd src/glim_ros2 && git fetch -q origin && git checkout -q "${GLIM_ROS_COMMIT}" \
   && cd ~ && source /opt/ros/jazzy/setup.bash \
   && colcon build --packages-select glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release ; \
    else echo "glim_ros2 at GLIM_ROS_REF=${GLIM_ROS_REF}" ; fi

# ~/.config/dendROS is a volume (compose.yaml): create it as ros, or Docker makes ~/.config root-owned
RUN mkdir -p ~/.config/dendROS \
 && echo '[ -f /home/ros/utils/utilities.sh ] && source /home/ros/utils/utilities.sh' >> ~/.bashrc

# Clipboard for the DendROS TUI's copy: VTE terminals (Terminator) ignore OSC 52, so it needs xclip,
# which copies to the container's X display ($DISPLAY, the laptop's with ssh -X). Last layer on
# purpose: adding it doesn't rebuild the layers above.
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends xclip \
 && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
