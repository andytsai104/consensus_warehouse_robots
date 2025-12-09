import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # === Paths ===
    pkg_bringup_sim = get_package_share_directory('bringup_sim')
    rviz_config_file = os.path.join(pkg_bringup_sim, 'config', 'rviz_navigation.rviz')        # RViz config


    # === Activate gazebo_navigation.launch.py ===
    gz_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup_sim, 'launch', 'gazebo_slam.launch.py'),
        ),
        launch_arguments = {'teleop': 'False'}.items()
    )


    # === Nodes ===
    # Nav2
    nav2_params_file = os.path.join(pkg_bringup_sim, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_bringup_sim, 'maps', 'warehouse_map.yaml')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'map': map_file,
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
    )

    # Auctioneer
    auctioneer = Node(
        package='fleet_manager',
        executable='auctioneer_node',
        name='auctioneer_node',
        output='screen',
        parameters=[{'bid_timeout_sec': 3.0}],
    )
    
    # Bidder
    bidder_robot1 = Node(
        package='fleet_manager',
        executable='bidder_node',
        name='bidder_robot1',
        output='screen',
        parameters=[{'robot_id': 'robot1'}],
    )    

    return LaunchDescription([
        gz_simulation,
        nav2,
        rviz,
        auctioneer,
        bidder_robot1,
    ])
