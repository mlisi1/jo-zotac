# Jo-zotac
Jo-zotac is a dockerfile image containing all the software needed to interface with AgileX Bunker Pro and its sensors (Jo). 

## Installation
### Docker configuration
To correctly install the docker with GPU access, these steps need to be followed:
1. Install Docker Engine with their [guide](https://docs.docker.com/engine/install/ubuntu/)
2. Allow Docker usage as non-root user ([guide](https://docs.docker.com/engine/install/linux-postinstall/))
3. Correctly install [nVidia dirvers](https://github.com/oddmario/NVIDIA-Ubuntu-Driver-Guide)
4. Install the [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

After docker is correctly installed, run
```bash
docker compose up --build
```
After the docker is built, run:
```bash
docker compose up -d && docker compose exec jo-zotac bash nvidia-smi
```
to confirm that everything works fine. If your output looks like this
```
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 570.195.03             Driver Version: 570.195.03     CUDA Version: 12.8     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 4060 ...    Off |   00000000:01:00.0  On |                  N/A |
| N/A   53C    P8              5W /   60W |      70MiB /   8188MiB |     10%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                         
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI              PID   Type   Process name                        GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A            1497      G   /usr/lib/xorg/Xorg                       55MiB |
+-----------------------------------------------------------------------------------------+
```
everything should work fine.

## Usage
The dockerfile already clones and built all the packages needed to interface with the sensors. The `jo_bringup` package is instead shared as a volume and built when the docker is started up as a symlink. This allows to edit the launch or config files without the need to rebuild the docker or the packages.

To run the bringup package, open a terminal and run 
```bash
docker compose up
```
Then, in another terminal run
```bash
docker compose exec jo-zotac ros2 launch jo_bringup jo_bringup.launch.py
```
By default this will launch the IMU interface and the lidar interface. It is possible to customize what is launched using the provided parameters. These are booleans that decide wether that module is launched or not.
### Module launch file parameters
- `imu:=<bool>` - Xsense IMU module.
- `gnss:=<bool>` - Xsense GNSS module. Needs the IMU module to be loaded and a working internet access.
- `lidar:=<bool>` - Velodyne VLP16 Lidar module. 
- `cam1:=<bool>` - Realsense D455 module. It is tied to the camera serial number.
- `cam2:=<bool>` - Realsense D455 module. It is tied to the camera serial number.
- `bunker:=<bool>` - Agilex Bunker Pro CAN interface module. It brings up the CAN interface when launched and brings it down when closed.
- `glim:=<bool>` - GLIM SLAM module. 
- `rviz:=<bool>` - rViz visualization

### Other launch file parameters
- `imu_param:=<path/to/yaml>` - Provides a path for the IMU config file.
- `gnss_param:=<path/to/yaml>` - Provides a path for the IMU GNSS config file.
- `glim_param:=<path/to/config/folder>` - Provides a path for GLIM config folder.

If none of these parameters are provided, they default to the config files present in the `config` folder. 

#### Minimal working SLAM example
```bash
docker compose exec jo-zotac ros2 launch jo_bringup jo_bringup.launch.py glim:=true
```


