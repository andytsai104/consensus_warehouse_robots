import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # === Paths ===
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('warehouse_robot_description')
    pkg_gazebo_sim = get_package_share_directory('gazebo_sim')
    pkg_bringup_sim = get_package_share_directory('bringup_sim')
    install_dir = os.path.dirname(pkg_description)                                          # Gets 'install/share' parent
    rviz_config_file = os.path.join(pkg_bringup_sim, 'config', 'rviz_scan_map.rviz')        # RViz config
    world_file = os.path.join(pkg_gazebo_sim, 'worlds', 'warehouse_world.sdf')              # World files
    xacro_file = os.path.join(pkg_description, 'urdf', '2wd_warehouse_robots.urdf.xacro')   # Robot description
    bridge_config_file = os.path.join(pkg_bringup_sim, 'config', 'ros_gz_bridges.yaml')     # Bridge config
    slam_config_file = os.path.join(pkg_bringup_sim, 'config', 'slam_mapper_params.yaml')   # SLAM config
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')                          # SLAM Toolbox package 
    

    # === Arguments ===
    # Mesh Resources    
    resource_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_dir
    )

    # Use NVIDIA GPU for rendering if available
    set_gpu_env = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_env = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')


    # SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_file,
            'use_sim_time': 'true'}.items(),
    )

    # Launch Teleop in a NEW Terminal Window
    teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )


    # === Nodes ===
    # Gazebo Fortress Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn the Robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'warehouse_bot_1',
            '-topic', 'robot_description', # Read from the topic robot_state_publisher publishes
            '-x', '0.0',
            '-y', '2.5',
            '-z', '1.5', # Drop it from height to prevent clipping
        ],
        output='screen'
    )

    # Bridge (ROS 2 <-> Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Robot State Publisher (Required for TF tree)
    robot_description = Command(['xacro ', xacro_file])         # Robot Description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_description, value_type=str),
        }]
    )


    return LaunchDescription([
        resource_env,
        set_gpu_env,
        set_glx_env,
        gazebo,        # 先啟動 Gazette
        bridge,        # 再啟動 TF + /clock + scan bridge
        robot_state_publisher,  # 最後啟動 RSP
        spawn,
        slam,
        rviz,
        teleop_node,
    ])