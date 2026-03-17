from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown, OnProcessExit
import os
from launch.actions import TimerAction
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction


def generate_launch_description():

    # ENV VAR
    tty1 = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
    tty2 = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')
    tty3 = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')


    # NODE LAUNCH PARAMS
    launch_imu_arg = DeclareLaunchArgument(
        'imu',
        default_value='true',
        description='Whether to launch the IMU stack'
    )

    launch_gnss_arg = DeclareLaunchArgument(
        'gnss',
        default_value='false',
        description='Whether to launch the GNSS stack (needs imu:=true)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 if true'
    )

    launch_lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value='true',
        description='Whether to launch the Velodyne stack'
    )

    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='false',
        description='Whether to launch the GLIM stack'
    )

    launch_front_cam_arg = DeclareLaunchArgument(
        'front_cam',
        default_value='false',
        description='Whether to launch the RealSense front camera stack'
    )

    launch_back_cam_arg = DeclareLaunchArgument(
        'back_cam',
        default_value='false',
        description='Whether to launch the RealSense back camera stack'
    )

    launch_bunker_arg = DeclareLaunchArgument(
        'bunker',
        default_value='false',
        description='Whether to launch the Agilex Bunker interface'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
   


    # PKGS
    imu_pkg = get_package_share_directory('xsens_mti_ros2_driver')
    camera_pkg = get_package_share_directory('realsense2_camera')
    velodyne_pkg = get_package_share_directory('velodyne')
    gnss_pkg = get_package_share_directory('ntrip')
    self_pkg = get_package_share_directory('jo_bringup')
    bunker_pkg = get_package_share_directory('bunker_base')

    #SCRIPTS
    can_up = os.path.join(self_pkg, 'utils', 'can_up.sh')
    can_down  = os.path.join(self_pkg, 'utils', 'can_down.sh')


    # LAUNCH FILES
    imu_launch = os.path.join(imu_pkg, 'launch', 'xsens_mti_node.launch.py')
    velodyne_launch = os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
    camera_launch = os.path.join(camera_pkg, 'launch', 'rs_launch.py')
    gnss_launch = os.path.join(gnss_pkg, 'launch', 'ntrip_launch.py')
    bunker_launch = os.path.join(bunker_pkg, 'launch', 'bunker_base.launch.py')
    



    # CONFIG PATHS
    rviz_config = os.path.join(self_pkg, 'config', 'jo.rviz')
    imu_param = os.path.join(self_pkg, 'config', 'imu', 'xsens_mti_node.yaml')
    gnss_param = os.path.join(self_pkg, 'config', 'imu', 'ntrip-param.yaml')
    glim_config = os.path.join(self_pkg, 'config', 'glim', 'glim_config_bunker_sim')



    # CONFIG PATH PARAMS
    imu_param_file_arg = DeclareLaunchArgument(
        'imu_param',
        default_value=imu_param,
        description='IMU param file passed to imu launch'
    )

    gnss_param_file_arg = DeclareLaunchArgument(
        'gnss_param',
        default_value=gnss_param,
        description='GNSS param file passed to ntrip launch'
    )

    glim_param_folder_arg = DeclareLaunchArgument(
        'glim_param',
        default_value=glim_config,
        description='GLIM param folder passed to its node'
    )


    


    # INCLUDED LAUNCH FILES

    ## IMU
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch),          
        launch_arguments={
            'param_file': LaunchConfiguration('imu_param'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('imu'))
    )

    ## GNSS
    gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gnss_launch),          
        launch_arguments={
            'param_file': LaunchConfiguration('gnss_param'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('gnss'))
    )

    ## Velodyne
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch),          
        condition=IfCondition(LaunchConfiguration('lidar'))
    )   

    ## Bunker interface
    bunker_condition = IfCondition(LaunchConfiguration('bunker'))

    bunker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bunker_launch),          
        condition=bunker_condition
    )

    can_up_action = ExecuteProcess(
        cmd=['bash', can_up],
        output='screen',
        condition=bunker_condition,
    )

    def _run_can_down(context, *args, **kwargs):
        # Run command directly via subprocess; avoids ExecuteProcess re-execution issues
        import subprocess
        subprocess.run(['bash', can_down], check=False)
        return []

    can_down_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(function=_run_can_down),
        ]),
        condition=bunker_condition,
    )




    # NODES 

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', rviz_config],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('rviz')),
    )


    glim = Node(
        package='glim_ros',
        executable='glim_rosnode',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('glim')),
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '0',
        },
        parameters=[
            {'config_path': LaunchConfiguration('glim_param')},
            {'use_sim_time' : LaunchConfiguration('use_sim_time')}
            ],
    )
    
    front_cam = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='front_camera',
        output='screen',
        parameters=[{
            'serial_no': '239222303721',
            'base_frame_id': 'rs1_link',
            'publish_tf': True,
            'pointcloud.enable': True,
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 6,
            'align_depth.enable': True,
            'pointcloud.ordered_pc': True,
            'pointcloud.stream_filter': 2, # COLOR
        }],
        condition=IfCondition(LaunchConfiguration('front_cam')),
    )

    back_cam = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='back_camera',
        output='screen',
        parameters=[{
            'serial_no': '242422305079',
            'base_frame_id': 'rs2_link',
            'publish_tf': True,
            'pointcloud.enable': True,
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 6,
            'align_depth.enable': True,
            'pointcloud.ordered_pc': True,
            'pointcloud.stream_filter': 2, # COLOR
        }],
        condition=IfCondition(LaunchConfiguration('back_cam')),
    )

    
    return LaunchDescription([
        tty1,
        tty2,
        tty3,
        launch_imu_arg,
        launch_lidar_arg,
        launch_gnss_arg,
        launch_rviz_arg,    
        launch_glim_arg,
        launch_front_cam_arg,
        launch_back_cam_arg,
        launch_bunker_arg,
        imu_param_file_arg,  
        gnss_param_file_arg,  
        glim_param_folder_arg,
        use_sim_time_arg,
        imu,
        gnss,
        bunker,
        velodyne,
        rviz,
        glim,
        front_cam,
        back_cam,
        can_up_action,
        can_down_on_shutdown
    ])
