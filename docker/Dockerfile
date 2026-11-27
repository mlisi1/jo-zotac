FROM osrf/ros:jazzy-desktop-full

ARG DISTRO=jazzy

# Layers go from the most stable/expensive to the most often bumped/cheap: system packages, CUDA,
# gtsam_points, RTAB-Map, GLIM, then the drivers, glim_ros2 and the tools. Each ARG is declared right
# before the layer that uses it: BuildKit hands every declared ARG to the later RUNs, so changing one
# rebuilds everything after its declaration, and nothing before it.

# --- System packages (one layer). xclip: the DendROS TUI's copy (VTE terminals such as Terminator ignore
# OSC 52), it copies to the container's X display ($DISPLAY, the laptop's with ssh -X).
RUN apt update -y \
    && apt install -y pipx \
    python3-pip \
    nano \
    sudo \
    xclip \
    ros-${DISTRO}-plotjuggler \
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

# --- Non-root user (uid 1000, passwordless sudo)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if id -u ${USER_UID} ; then userdel `id -un ${USER_UID}` ; fi \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- CUDA repo (Ubuntu 24.04), GTSAM 4.3 and iridescence from the koide3 PPA.
# Keep CUDA / GTSAM / gtsam_points / GLIM / glim_ros2 in sync with jo_sim's Dockerfile.
ARG CUDA_VERSION=13-1
ARG CUDA_DIR=13.1
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
 && dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb \
 && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin \
 && mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
 && curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash \
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

# --- koide3/gtsam_points from source: GLIM master needs IndexedSlidingWindow::has_index, which is
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

# --- Workspace /home/ros/src, built layer by layer below
USER ros
WORKDIR /home/${USERNAME}/
SHELL ["/bin/bash", "-c"]
RUN sudo usermod -aG dialout ros && mkdir src && rosdep update

# --- RTAB-Map pinned to jo_sim's commits (same visual odometry / SLAM code in the sim and on the robot).
# The two repos expand into ~14 rtabmap_* packages, several heavy per compile unit: all at once OOMs on
# 32 GB, so one package at a time, 6 make jobs each.
ARG RTABMAP_REF=7d8b585217033e8d25d01b380a36744effe816df
ARG RTABMAP_ROS_REF=3e1cdcb072ad21fa2b129412332760bd3e456279
RUN for r in "rtabmap ${RTABMAP_REF}" "rtabmap_ros ${RTABMAP_ROS_REF}"; do set -- $r; \
      mkdir -p src/$1 && git -C src/$1 init -q \
   && git -C src/$1 remote add origin https://github.com/introlab/$1.git \
   && git -C src/$1 fetch -q --depth 1 origin $2 && git -C src/$1 checkout -q FETCH_HEAD || exit 1; done \
 && source /opt/ros/jazzy/setup.bash \
 && MAKEFLAGS="-j6" colcon build --packages-select-regex '^rtabmap.*' --executor sequential \
      --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

# --- GLIM core: koide3/glim >= 1a197c9 is needed by glim_ros2@dev (contains the a9be365 GTSAM 4.3
# ImuFactor fix, koide3/glim#306).
ARG GLIM_REF=1a197c999e4865f5cab7c9cd25474128413ebbca
RUN mkdir src/glim && cd src/glim && git init -q \
 && git remote add origin https://github.com/koide3/glim.git \
 && git fetch -q --depth 1 origin ${GLIM_REF} && git checkout -q FETCH_HEAD \
 && cd ~ && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select glim --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Sensor drivers: Velodyne VLP-16, Xsens MTi + NTRIP (the maintainer's fork). Unpinned on purpose.
RUN cd src && git clone -q https://github.com/ros-drivers/velodyne.git \
 && git clone -q --branch ros2 https://github.com/mlisi1/Xsens_MTi_ROS_Driver_and_Ntrip_Client.git \
 && cd ~ && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-skip-build-finished --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- Agilex Bunker: the mlisi1 forks of ugv_sdk and bunker_ros2, branch dev (pinned commits): both drivers
# in /bunker_status, publish_tf (off by default), odometry covariance/stamps, /bunker_rc_state.
ARG UGV_SDK_REF=71149ee
ARG BUNKER_ROS2_REF=07db0df
RUN cd src && git clone -q --branch dev https://github.com/mlisi1/ugv_sdk.git \
 && git -C ugv_sdk checkout -q ${UGV_SDK_REF} \
 && git clone -q --branch dev https://github.com/mlisi1/bunker_ros2.git \
 && git -C bunker_ros2 checkout -q ${BUNKER_ROS2_REF} \
 && cd ~ && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select ugv_sdk bunker_msgs bunker_base --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- glim_ros2: mlisi1 fork, branch dev, pinned commit. Late on purpose: bumping GLIM_ROS_REF rebuilds only
# glim_ros and the tool layers below (minutes), not RTAB-Map, GLIM or the drivers.
ARG GLIM_ROS_REF=f762545
RUN cd src && git clone -q --branch dev https://github.com/mlisi1/glim_ros2.git \
 && git -C glim_ros2 checkout -q ${GLIM_ROS_REF} \
 && cd ~ && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select glim_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- DendROS (ros2 wrapper / TUI), pinned to a release: bump DENDROS_VERSION for a new one
ARG DENDROS_VERSION=v1.1.0
RUN git clone -q --depth 1 --branch ${DENDROS_VERSION} https://github.com/mlisi1/DendROS.git /tmp/DendROS \
 && bash /tmp/DendROS/install.sh -y \
 && sudo rm -rf /tmp/DendROS

# --- Testudo (topic health). TESTUDO_VERSION: a tag, branch or commit. For now a commit on dev (the fixes
# from the robot bags after 1.0.0; >= ef154f0 reads node_profiling.nodes, which
# jo_bringup/config/testudo.yaml uses); back to a release tag once dev is merged into main and tagged.
# A commit rather than "dev": the layer is cached by this value, so a branch name never picks up new commits.
ARG TESTUDO_VERSION=50b272e
RUN pip install --user --break-system-packages "textual>=6" \
 && git clone -q https://github.com/mlisi1/testudo.git src/testudo \
 && git -C src/testudo checkout -q ${TESTUDO_VERSION} \
 && source /opt/ros/jazzy/setup.bash \
 && colcon build --packages-select testudo

# --- Shell setup and entrypoint (last: they change without touching anything above)
# ~/.config/dendROS is a volume (compose.yaml): create it as ros, or Docker makes ~/.config root-owned
RUN mkdir -p ~/.config/dendROS \
 && echo '[ -f /home/ros/utils/utilities.sh ] && source /home/ros/utils/utilities.sh' >> ~/.bashrc
COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
