import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- 1. Define Package Names (SPLIT STRUCTURE) ---
    description_pkg_name = 'warehouse_robot_description' 
    bringup_pkg_name = 'bringup_sim'

    # Ensure this matches your actual filename in the 'urdf' folder!
    xacro_file_name = 'warehouse_robots.urdf.xacro' 
    rviz_file_name = 'rviz_display.rviz'

    # --- 2. Build Paths ---
    # Find the URDF inside the description package
    xacro_path = os.path.join(
        get_package_share_directory(description_pkg_name),
        'urdf',
        xacro_file_name
    )

    # Find the Rviz config inside the bringup package
    rviz_path = os.path.join(
        get_package_share_directory(bringup_pkg_name),
        'config',
        rviz_file_name
    )

    # Process the URDF file
    robot_description = Command(['xacro ', xacro_path])
    
    # --- 3. Nodes ---

    # Node A: Robot State Publisher (Publishes TF tree)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # Node C: RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])