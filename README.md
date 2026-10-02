# Jo-zotac
Jo-zotac is a dockerfile image containing all the software needed to interface with AgileX Bunker Pro and its sensors (Jo). 

> **Branch `jo-zotac-dynamic-slam`:** adds GLIM with dynamic object rejection, `onboard_detector_v2` and the nav2 `dynamic_obstacle_layer`. Its compose project, service, image and container are all named `jo-zotac-dynamic-slam`, so it can coexist with the original `jo-zotac` image/container.

## Installation
### Docker configuration
To correctly install the docker with GPU access, these steps need to be followed:
1. Install Docker Engine with their [guide](https://docs.docker.com/engine/install/ubuntu/)
2. Allow Docker usage as non-root user ([guide](https://docs.docker.com/engine/install/linux-postinstall/))
3. Correctly install [nVidia drivers](https://github.com/oddmario/NVIDIA-Ubuntu-Driver-Guide)
4. Install the [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

After docker is correctly installed, run
```bash
docker compose up --build
```
After the docker is built, run:
```bash
docker compose up -d && docker compose exec jo-zotac-dynamic-slam bash -c "nvidia-smi"
```
to confirm that everything works fine. 

## Usage
The dockerfile already clones and built all the packages needed to interface with the sensors. The `jo_bringup` package is instead shared as a volume and built when the docker is started up as a symlink. This allows to edit the launch or config files without the need to rebuild the docker or the packages.

The following packages are also shared as volumes (git submodules, run `git submodule update --init` after cloning):
- `glim` / `glim_ros2` - [giacomotambe/glim](https://github.com/giacomotambe/glim) and [giacomotambe/glim_ros2](https://github.com/giacomotambe/glim_ros2), GLIM with dynamic object rejection. They are built inside the image and **not** rebuilt at startup: after editing them run `build_glim` inside the container.
- `onboard_detector_v2` - [ducciopet/onboard_detector_v2](https://github.com/ducciopet/onboard_detector_v2), lidar/camera dynamic obstacle detection and tracking. Rebuilt incrementally (symlink) at startup.
- `jo_msgs` - obstacle messages shared by `onboard_detector_v2`, `glim_ros2` and the `jo_navigation` dynamic obstacle costmap layer. Rebuilt at startup.

To run the bringup package, open a terminal and run 
```bash
docker compose up
```
Then, in another terminal run
```bash
docker compose exec jo-zotac-dynamic-slam ros2 launch jo_bringup jo_bringup.launch.py
```
By default this will launch the IMU interface and the lidar interface. It is possible to customize what is launched using the provided parameters. These are booleans that decide wether that module is launched or not.
### Module launch file parameters
- `imu:=<bool>` - Xsense IMU module.
- `gnss:=<bool>` - Xsense GNSS module. Needs the IMU module to be loaded and a working internet access.
- `lidar:=<bool>` - Velodyne VLP16 Lidar module. 
- `front_cam:=<bool>` - Realsense D455 module. It is tied to the camera serial number.
- `back_cam:=<bool>` - Realsense D455 module. It is tied to the camera serial number.
- `bunker:=<bool>` - Agilex Bunker Pro CAN interface module. It brings up the CAN interface when launched and brings it down when closed.
- `glim:=<bool>` - GLIM SLAM module. 
- `rviz:=<bool>` - rViz visualization

**Note** that, as of now launching with ```bunker:=true``` may not work. The node seems to fail a couple of times before correctly launchin, so it's suggested to launch it in a different console.

### Handy aliases
Given that the launch commands can be quite long, some aliases to quickly run some combinations of the perception stack have been added to the bashrc.
+ ```light_stack``` - Launches the perception stack with imu and lidar
+ ```full_stack``` - Launches the perception stack with imu, lidar, front and back camera, GNSS and glim
+ ```glim_only``` - Only launches glim SLAM
+ ```glim_sim``` - Launches glim SLAM with ```use_sim_time:=true```
+ ```offline_viewer``` - Opens glim's offline viewer to analyze created maps
+ ```build_glim``` - Rebuilds the volume-mounted `glim` and `glim_ros2` packages
+ ```bunker_only``` - Launches Bunker command interface
+ ```save_map <map_name>``` - Saves the last map created by glim in the folder ```saved_maps/YYYYMMDD_HHMMSS_<map_name>```
+ ```localization``` - Launches local odometry module
+ ```localization_gps``` - Launches global odometry with dueal EKF configuration with GPS
+ ```navigation``` - Launches nav2 stack configured to work with local odometry
+ ```navigation_gps``` - Launches nav2 stack configured to work with global odometry and GPS
+ ```record_all <bag_name>``` - Records a rosbag of **all** topics and saves it in the folder ```bags/YYYYMMDD_HHMMSS_<bag_name>```
+ ```record_compressed <bag_name>``` - Records a rosbag of **all** topics (only subscribing to compressed topics for image transport) and saves it in the folder ```bags/YYYYMMDD_HHMMSS_<bag_name>```

> **Note:** Both ```record_all``` and ```record_compressed``` do not actually record all available topics. The first command avoids topics that contain "scanend" (this creates problems with GLIM if subscribed to) and several topics that create problems with image transports (theora, compressedDepth and some compressed version of some topics). The second command is identical to the first, with the exception that it does not record the raw version of topics (so all topics ending in "image_raw" and "image_rect_raw" are excluded).

### Other launch file parameters
- `imu_param:=<path/to/yaml>` - Provides a path for the IMU config file.
- `gnss_param:=<path/to/yaml>` - Provides a path for the IMU GNSS config file.
- `glim_param:=<path/to/config/folder>` - Provides a path for GLIM config folder.

If none of these parameters are provided, they default to the config files present in the `config` folder. 

#### Minimal working SLAM example
Run inside the docker
```bash
ros2 launch jo_bringup jo_bringup.launch.py glim:=true
```
or simply
```bash
light_stack glim:=true
```


