source /opt/ros/jazzy/setup.bash
source /home/ros/install/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# rviz_config: the RViz view of each alias; append rviz:=true to open it (one RViz, launched by jo_bringup)
alias sensors='ros2 launch jo_bringup jo_bringup.launch.py rviz_config:=sensors'
alias full_sensors='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true back_cam:=true gnss:=true rviz_config:=sensors'
alias bunker_only='ros2 launch jo_bringup jo_bringup.launch.py imu:=false lidar:=false bunker:=true'
alias offline_viewer='ros2 run glim_ros offline_viewer'
source /home/ros/utils/save_map.bash
source /home/ros/utils/recordings.bash
alias no_gpu='__NV_PRIME_RENDER_OFFLOAD=0 __GLX_VENDOR_LIBRARY_NAME='

alias full_navigation='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true localization:=true navigation:=true rviz:=true rviz_config:=navigation back_cam:=true'
alias full_gps_navigation='ros2 launch jo_bringup jo_bringup.launch.py front_cam:=true gnss:=true localization_gps:=true navigation_gps:=true rviz:=true rviz_config:=gps_navigation back_cam:=true'

alias extract_videos='bash /home/ros/utils/extract_camera_videos.sh'
alias export_vslam='bash /home/ros/utils/export_vslam_map.sh'
alias open_vslam='bash /home/ros/utils/open_vslam_map.sh'

# Topic health (Testudo), default config: every topic is monitored, nothing is declared. A robot
# config (thresholds, declared topics) comes after an on-board run. health_headless: no TUI,
# results on /testudo/overall and /testudo/<plugin> (scripts, remote checks).
alias health='ros2 run testudo testudo watch'
alias health_headless='ros2 run testudo testudo watch --headless'


# Prompt: magenta and tagged [jo-zotac], so a container shell can't be mistaken for the host's
# (network_mode: host gives the container the host's hostname, and Ubuntu's default prompt is green there too)
if [[ $- == *i* ]]; then
    PS1='\[\e]0;[jo-zotac] \u@\h: \w\a\]\[\033[01;35m\][jo-zotac] \u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
fi
