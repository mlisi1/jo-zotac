source /opt/ros/jazzy/setup.bash
source /home/ros/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
alias sensors='ros2 launch jo_bringup jo_bringup.launch.py'
alias full_sensors='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true back_cam:=true gnss:=true'
alias bunker_only='ros2 launch jo_bringup jo_bringup.launch.py imu:=false lidar:=false bunker:=true'
alias offline_viewer='ros2 run glim_ros offline_viewer'
source /save_map.bash
source /record_all.bash
alias no_gpu='__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME='

alias full_navigation='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true localization:=true navigation:=true rviz:=true'
alias full_gps_navigation='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true gnss:=true localization_gps:=true navigation_gps:=true rviz:=true'

