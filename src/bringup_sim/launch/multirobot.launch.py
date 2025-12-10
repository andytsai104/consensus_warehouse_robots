import os
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # === Packages & paths ===
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_description = get_package_share_directory("warehouse_robot_description")
    pkg_gazebo_sim = get_package_share_directory("gazebo_sim")
    pkg_bringup_sim = get_package_share_directory("bringup_sim")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    world_file = os.path.join(pkg_gazebo_sim, "worlds", "warehouse_world.sdf")
    xacro_file = os.path.join(pkg_description, "urdf", "2wd_multirobot.urdf.xacro")
    nav2_params_file = os.path.join(pkg_bringup_sim, "config", "nav2_params.yaml")
    map_file = os.path.join(pkg_bringup_sim, "maps", "warehouse_map.yaml")
    rviz_config_file = os.path.join(pkg_bringup_sim, "config", "rviz_gazebo_sim.rviz")
    install_dir = os.path.dirname(pkg_description)
    
    # === Gazebo world ===
    # Mesh Resources    
    resource_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_dir
    )

        # Use NVIDIA GPU for rendering if available
    set_gpu_env = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_glx_env = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # === RViz (全域一個) ===
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # === Global Auctioneer (no namespace) ===
    auctioneer = Node(
        package="fleet_manager",
        executable="auctioneer_node",
        name="auctioneer_node",
        output="screen",
        parameters=[{"bid_timeout_sec": 3.0}],
    )


    # Bridge for the simulation clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )


    # === Per-robot group (namespace + Nav2 + bridge + bidder) ===
    robot_groups = []
    initial_positions = [
        (0, 2.5),
        (-2, 7.5),
        (-7.5, -7.5),
        (2.5, -5),
        (8, -3),
    ]

    robot_num = len(initial_positions)
    for idx in range(robot_num):
        robot_index = idx + 1
        ns = f"robot{robot_index}"
        x_init, y_init = initial_positions[idx]

        # 1) Robot description
        # robot_description = Command(["xacro ", xacro_file])
        robot_description = Command([
            "xacro ", xacro_file,
            " prefix:=", ns, "/",
            " robot_name:=", f"warehouse_bot_{robot_index}",
        ])

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=ns,
            output="both",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": ParameterValue(
                        robot_description, value_type=str
                    ),
                    # "frame_prefix": f"{ns}/",
                }
            ],
            # remappings=[
            #     ("/tf", "/tf"),
            #     ("/tf_static", "/tf_static")
            # ]
        )

        # 2) Spawn robot in Gazebo
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            namespace=ns,
            arguments=[
                "-name", f"warehouse_bot_{robot_index}",
                "-topic", f"robot_description",
                "-x", str(x_init),
                "-y", str(y_init),
                "-z", "1.5",
            ],
            output="screen",
        )

        # 3) ROS <-> Gazebo bridge for this robot
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns,
            name=f"bridge_{ns}", # Good practice to name nodes uniquely
            arguments=[
                # Cmd Vel: ROS -> Gazebo
                f"/model/warehouse_bot_{robot_index}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                
                # Odometry: Gazebo -> ROS
                f"/model/warehouse_bot_{robot_index}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                
                # Scan: Gazebo -> ROS
                f"/model/warehouse_bot_{robot_index}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                
                # TF: Gazebo -> ROS
                f"/model/warehouse_bot_{robot_index}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                
                # Joint State: Gazebo <-> ROS
                f"/model/warehouse_bot_{robot_index}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
            # Remap the specific Gazebo model topics to standard ROS names inside the namespace
            remappings=[
                (f"/model/warehouse_bot_{robot_index}/cmd_vel", "cmd_vel"),
                (f"/model/warehouse_bot_{robot_index}/odometry", "odom"),
                (f"/model/warehouse_bot_{robot_index}/scan", "scan"),
                (f"/model/warehouse_bot_{robot_index}/tf", "tf"),
                (f"/model/warehouse_bot_{robot_index}/joint_states", "joint_states"),
            ]
        )


        # Nav2
        param_substitutions = {
            'use_sim_time': 'True',
            'base_frame_id': f'{ns}/base_footprint',  # robot1/base_footprint
            'odom_frame_id': f'{ns}/odom',            # robot1/odom
            'scan_topic': 'scan',
            'global_frame_id': 'map',
            'robot_base_frame': f'{ns}/base_footprint' # Critical for costmaps
        }

        configured_params = RewrittenYaml(
            source_file=nav2_params_file,
            root_key=ns,
            param_rewrites=param_substitutions,
            convert_types=True
        )
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "namespace": ns,
                "use_sim_time": "true",
                "params_file": configured_params,
                "map": map_file,
                "robot_base_frame": f"{ns}/base_footprint",
                "global_frame": "map",
                "odom_topic": f"{ns}/odom"
            }.items(),
        )

        # 5) Bidder node for this robot
        bidder = Node(
            package="fleet_manager",
            executable="bidder_node",
            name=f"bidder_{ns}",
            namespace=ns,
            output="screen",
            parameters=[{"robot_id": ns}],
        )

        robot_group = GroupAction(
            actions=[
                robot_state_publisher,
                spawn,
                bridge,
                nav2,
                # bidder,
            ]
        )
        robot_groups.append(robot_group)

    return LaunchDescription(
        [   
            set_glx_env,
            set_gpu_env,
            resource_env,
            clock_bridge,
            gazebo,
            rviz,
            *robot_groups,
        ]
    )

