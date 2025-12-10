import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
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
    xacro_file = os.path.join(
        pkg_description, "urdf", "2wd_warehouse_robots.urdf.xacro"
    )
    bridge_config_file = os.path.join(
        pkg_bringup_sim, "config", "ros_gz_bridges.yaml"
    )
    nav2_params_file = os.path.join(
        pkg_bringup_sim, "config", "nav2_params.yaml"
    )
    map_file = os.path.join(pkg_bringup_sim, "maps", "warehouse_map.yaml")
    rviz_config_file = os.path.join(
        pkg_bringup_sim, "config", "rviz_navigation.rviz"
    )

    # === Gazebo world ===
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

    data_logger = Node(
    	package='fleet_manager',
    	executable='data_logger_node',
    	name='data_logger',
    	output='screen',
        parameters=[{'bid_timeout_sec': 3.0}],
    )
    
    # === Per-robot group (namespace + Nav2 + bridge + bidder) ===
    robot_groups = []

    # 初始位置可以依需求調整
    initial_positions = [
        (0.0, 2.5),
        (1.0, 2.5),
        (2.0, 2.5),
        (0.0, 1.5),
        (1.0, 1.5),
    ]

    for idx in range(5):
        robot_index = idx + 1
        ns = f"robot{robot_index}"
        x_init, y_init = initial_positions[idx]

        # 1) Robot description (正確展開 xacro)
        robot_description = Command(["xacro ", xacro_file])

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
                    # 為 TF 加前綴，避免多機器人 frame 名稱衝突
                    "frame_prefix": f"{ns}/",
                }
            ],
        )

        # 2) Spawn robot in Gazebo
        # 注意：這裡放在同一個 namespace，這樣
        #   - robot_state_publisher 會在 /<ns>/robot_description 發 topic
        #   - create -topic robot_description 也會從同一個 namespace 訂閱
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            namespace=ns,
            arguments=[
                "-name",
                f"warehouse_bot_{robot_index}",
                "-topic",
                "robot_description",
                "-x",
                str(x_init),
                "-y",
                str(y_init),
                "-z",
                "1.5",
            ],
            output="screen",
        )

        # 3) ROS <-> Gazebo bridge for this robot
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns,
            parameters=[
                {
                    "config_file": bridge_config_file,
                    "expand_gz_topic_names": True,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        )

        # 4) Nav2 bringup for this robot
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "namespace": ns,
                "use_sim_time": "true",
                "params_file": nav2_params_file,
                "map": map_file,
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
                bidder,
            ]
        )
        robot_groups.append(robot_group)

    return LaunchDescription(
        [
            gazebo,
            rviz,
            auctioneer,
            *robot_groups,
            data_logger,
        ]
    )

