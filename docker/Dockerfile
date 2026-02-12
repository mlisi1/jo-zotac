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
    cuda-cudart-13-0 \
    cuda-nvcc-13-0 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


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
RUN cd src && git clone https://github.com/koide3/glim_ros2.git
RUN cd src && git clone https://github.com/koide3/glim.git

# Agilex Bunker
RUN cd src && git clone https://github.com/mlisi1/ugv_sdk.git
RUN cd src && git clone -b jazzy https://github.com/agilexrobotics/bunker_ros2.git


RUN rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build

COPY docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
RUN echo "source /opt/ros/${DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros/install/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc 


ENTRYPOINT ["/entrypoint.sh"]
