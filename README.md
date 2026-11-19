# jo-zotac

Docker image that runs on **Jo**, an AgileX Bunker Pro (tracked) with a Zotac PC and an NVIDIA GPU.
It contains the drivers for every sensor and the perception, localization and navigation stack:

| Part | What |
|---|---|
| Sensors | Velodyne VLP-16, Xsens MTi (IMU + GNSS, NTRIP client for RTK), two RealSense D455 (front / back) |
| Base | AgileX Bunker Pro over CAN (`bunker_base`) |
| SLAM / localization | GLIM (LiDAR-inertial, GPU), robot_localization EKFs, navsat_transform, RTAB-Map visual odometry / SLAM |
| Navigation | Nav2 (local and GPS) |
| Tools | DendROS (`ros2` wrapper with grouped, coloured logs), Testudo (topic health), recording, calibration and map tools |

The simulator twin is [jo_sim](https://github.com/mlisi1/jo_sim) (Gazebo Harmonic). It exposes the
same ROS interface as this image, which is the reference for it.

## Repository

| Path | Content |
|---|---|
| `docker/Dockerfile`, `docker/entrypoint.sh` | the image (ROS 2 Jazzy, CUDA 13.1, everything built from source) |
| `compose.yaml` | how the container runs on the robot |
| `jo_bringup/` | top-level launch, sensor configs, RViz views, DendROS groups |
| `jo_description/` | submodule: URDF and `robot_state_publisher` |
| `jo_navigation/` | submodule: GLIM configs, EKFs, Nav2, visual odometry, GPS missions |
| `docker/utils/` | aliases, recording profiles, save_map, calibration scripts, CycloneDDS config |
| `host/` | settings for the robot PC itself (USB, CPU idle) |
| `bags/`, `saved_maps/`, `calibration/` | recordings, maps and calibration results (not in git) |

`jo_bringup`, `jo_description`, `jo_navigation` and `docker/utils` are mounted into the container
and the three packages are rebuilt (`--symlink-install`) at every start: launch files, configs, URDF
and aliases change without rebuilding the image. Only `docker/Dockerfile` and `docker/entrypoint.sh`
need `docker compose build`.

## Installation (robot PC)

1. Docker Engine ([guide](https://docs.docker.com/engine/install/ubuntu/)), usable without root
   ([guide](https://docs.docker.com/engine/install/linux-postinstall/)), Docker Compose >= 2.24.
2. NVIDIA driver and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
3. Host settings, once, **on the host**:
   ```bash
   sudo bash host/install_host_rules.sh
   ```
   Xsens FTDI latency 1 ms (otherwise IMU samples arrive in bursts), USB autosuspend and USB3 LPM off
   for the cameras, larger usbfs buffer, no deep CPU idle states. Without them the cameras drop
   frames ("Incomplete video frame detected").
4. Clone with the submodules:
   ```bash
   git clone --recursive https://github.com/mlisi1/jo-zotac.git
   ```
5. NTRIP credentials (only for RTK GNSS) in `.env`, next to `compose.yaml`. It is optional and must
   never be committed:
   ```bash
   NTRIP_USERNAME=...
   NTRIP_PASSWORD=...
   ```
6. Build and start:
   ```bash
   docker compose up -d --build
   docker compose exec jo-zotac bash      # every shell has the aliases below
   ```
   The GPU target is `CUDA_ARCHS=89` (RTX 3500 Ada); another GPU needs `CUDA_ARCHS` in `.env`.
   `GLIM_ROS_COMMIT` in `.env` rebuilds only GLIM's ROS node at another glim_ros2 commit.

## Running

Everything starts from `jo_bringup.launch.py`; the aliases cover the usual combinations:

| Alias | Launches |
|---|---|
| `sensors` | IMU + LiDAR (the launch defaults) |
| `full_sensors` | IMU, LiDAR, both cameras, GNSS with NTRIP |
| `full_navigation` | sensors + cameras + local localization (GLIM + EKF) + Nav2 + RViz |
| `full_gps_navigation` | the GPS variant: dual EKF + navsat + GPS Nav2 + RViz |
| `bunker_only` | the base only: CAN up, `bunker_base` (CAN down when it stops) |

Start the base in its own terminal (`bunker_only`), separately from the stack, so it can be stopped
on its own.

Launch arguments (booleans unless noted), all appendable to the aliases:

| Argument | Default | |
|---|---|---|
| `imu`, `lidar` | true | Xsens MTi, Velodyne |
| `gnss` | false | NTRIP client (needs the IMU, internet and `.env`) |
| `front_cam`, `back_cam` | false | RealSense cameras, bound to their serial numbers |
| `bunker` | false | Bunker base (prefer `bunker_only` in another terminal) |
| `localization` / `localization_gps` | false | GLIM + EKF / dual EKF + navsat |
| `navigation` / `navigation_gps` | false | Nav2 for local / GPS localization |
| `rviz` | false | RViz, with the view `rviz_config` |
| `rviz_config` | (auto) | `sensors`, `navigation`, `gps_navigation` or a `.rviz` path |
| `glim_view` | false | GLIM's own 3D viewer (needs a display) |
| `visodom`, `vslam` | | RTAB-Map visual odometry / SLAM, see `jo_navigation` |
| `imu_param`, `gnss_param` | `jo_bringup/config/imu/` | yaml paths |

### RViz

There is one RViz, started by `jo_bringup` with `rviz:=true`. Every alias selects its view
(`rviz_config`), so `sensors rviz:=true` opens the sensor view, and `full_gps_navigation` opens the GPS
one. Without `rviz_config` the view follows the stacks launched. The views are in `jo_bringup/rviz/`:

- `sensors`: LiDAR, camera clouds and images, TF (fixed frame `base_footprint`)
- `navigation`: Nav2 (costmaps, plan, footprint), GLIM map, `/odometry/local` (fixed frame `map`)
- `gps_navigation`: Nav2, `/odometry/global` and `/odometry/gps` (fixed frame `map`)

## Maps

`save_map <name>` saves to `saved_maps/<YYYYmmdd_HHMMSS>_<name>/`: GLIM's dump (opens with
`offline_viewer`), `map.pcd`, `map.pgm` + `map.yaml` (Nav2 map server) and `map_anchor.txt`.

- GLIM running: it saves through GLIM's `save_map` service, while SLAM keeps running.
- GLIM stopped: it copies the dump GLIM wrote at shutdown, `saved_maps/glim_last_run/`. That folder
  holds the last run only and is cleared when GLIM starts again.

`export_vslam` / `open_vslam` export or open the RTAB-Map database of a `vslam:=true` run
(`saved_maps/*_rtabmap/rtabmap.db`).

## Recording

```bash
record_all <name>          # every topic of the stack, raw images
record_compressed <name>   # the same with compressed images
record_sensors <name>      # sensors only (offline reprocessing), compressed images
record_calib <profile> <name>   # calibration recordings: imu_static, mag, floor, motion, camera
```

Bags go to `bags/<YYYYmmdd_HHMMSS>_<name>` (MCAP). The topic lists, QoS overrides and storage
settings are in `docker/utils/recording/*.yaml`: a new topic has to be added there to be recorded.
`extract_videos <bag>` writes one mp4 per camera stream.

## Tools

- **DendROS**: `ros2` in the container is DendROS's wrapper. Launch output is grouped (SENS, BASE,
  GLIM, LOC, NAV, ...) by `jo_bringup/config/dendROS.yaml`; global settings (`dendros config`) are
  kept in `docker/utils/dendROS/`.
- **Testudo**: `health` (TUI) or `health_headless` (results on `/testudo/*`) check rate, latency and
  content of every topic.
- **Calibration**: `docker/utils/calib/` (Allan variance, magnetometer check, floor plane, LiDAR-IMU
  rotation, straight-drive yaw, extrinsics consistency). Results go to `calibration/runs/`.
- `no_gpu <cmd>` runs a GUI without the NVIDIA offload variables.

## Notes

- DDS is CycloneDDS with multicast for discovery only (`docker/utils/cyclonedds.xml`): multicast data
  saturated the LAN and delayed `/velodyne_points`.
- The container runs with `pid: host`: `pkill -f` inside it also matches host processes.
- The submodules follow the `dev` branch; `main` is the release branch.
