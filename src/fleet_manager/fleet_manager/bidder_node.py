#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose


def yaw_to_quaternion_z(yaw: float):
    """Convert yaw angle to quaternion (z, w components only for planar robots)."""
    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return qz, qw


class BidderNode(Node):
    """Bidder node for Phase 3.

    Current version:
    - Subscribes to /task_auction (JSON encoded tasks).
    - Computes a real cost using Nav2 ComputePathToPose (path length).
    - Publishes bids on /task_bids.
    - Subscribes to /task_assignment and reacts when this robot is the winner.
    - Executes assigned tasks by sending a NavigateToPose goal to Nav2.
    """

    def __init__(self) -> None:
        super().__init__("bidder_node")

        # Parameter to identify this robot in the auction
        self.declare_parameter("robot_id", "robot1")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        # Simple availability flag so the robot only handles one task at a time
        self.available: bool = True

        # Publisher to send bids to the auctioneer
        self.bid_pub = self.create_publisher(String, "/task_bids", 10)

        # Subscriber to receive task auctions
        self.task_sub = self.create_subscription(
            String, "/task_auction", self.task_callback, 10
        )

        # Subscriber to receive final task assignments
        self.assignment_sub = self.create_subscription(
            String, "/task_assignment", self.assignment_callback, 10
        )

        # Action client to compute path cost with Nav2
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )

        # Action client to send navigation goals to Nav2
        # Note: with namespace="robot1", this becomes /robot1/navigate_to_pose.
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.get_logger().info(
            f"BidderNode started for robot_id={self.robot_id}"
        )

    # ---------------- Auction handling ----------------

    def task_callback(self, msg: String) -> None:
        """Handle incoming task and publish a bid based on path length."""
        if not self.available:
            # Robot is busy executing another task; ignore new auctions
            return

        try:
            task = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON task.")
            return

        task_id = task.get("task_id")
        target = task.get("target")

        if task_id is None or target is None:
            self.get_logger().warn("Task is missing task_id or target.")
            return

        # Compute cost using Nav2 ComputePathToPose
        cost = self.compute_cost_to_target(target)
        if cost is None:
            self.get_logger().warn(
                f"Failed to compute path cost for task {task_id}; no bid sent."
            )
            return

        bid = {
            "task_id": task_id,
            "robot_id": self.robot_id,
            "cost": cost,
        }

        bid_msg = String()
        bid_msg.data = json.dumps(bid)
        self.bid_pub.publish(bid_msg)

        self.get_logger().info(
            f"Sent bid for task {task_id} with cost {cost:.3f} "
            f"from robot {self.robot_id}"
        )

    def compute_cost_to_target(self, target: dict):
        """Use Nav2 ComputePathToPose action to estimate path cost.

        Returns the path length as the cost, or None on failure.
        """
        # Wait for ComputePathToPose action server
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "ComputePathToPose action server not available. Cannot compute cost."
            )
            return None

        pose = PoseStamped()
        pose.header.frame_id = target.get("frame_id", "map")
        pose.pose.position.x = float(target["x"])
        pose.pose.position.y = float(target["y"])
        yaw = float(target.get("yaw", 0.0))
        qz, qw = yaw_to_quaternion_z(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose

        self.get_logger().info(
            f"Requesting path to "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw={yaw:.2f})"
        )

        send_future = self.compute_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("ComputePathToPose goal was rejected.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        path = result.result.path
        if len(path.poses) < 2:
            return 0.0

        length = 0.0
        last = path.poses[0].pose.position
        for pose_stamped in path.poses[1:]:
            p = pose_stamped.pose.position
            dx = p.x - last.x
            dy = p.y - last.y
            length += math.hypot(dx, dy)
            last = p

        self.get_logger().info(f"Computed path length: {length:.3f}")
        return length

    # ---------------- Assignment handling ----------------

    def assignment_callback(self, msg: String) -> None:
        """React to task assignments from the auctioneer."""
        try:
            assignment = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON assignment.")
            return

        task_id = assignment.get("task_id")
        robot_id = assignment.get("robot_id")
        target = assignment.get("target")

        # Ignore assignments meant for other robots
        if robot_id != self.robot_id:
            return

        if task_id is None or target is None:
            self.get_logger().warn("Assignment is missing task_id or target.")
            return

        if not self.available:
            self.get_logger().warn(
                f"Received assignment for task {task_id} while busy. Ignoring."
            )
            return

        # Mark as busy and execute task via Nav2
        self.available = False
        self.get_logger().info(
            f"Task {task_id} assigned to me ({self.robot_id}). "
            f"Target: {target}"
        )

        self.execute_task(task_id, target)

        # Mark as available again after execution
        self.available = True
        self.get_logger().info(
            f"Task {task_id} completed by {self.robot_id}."
        )

    # ---------------- Nav2 execution ----------------

    def execute_task(self, task_id: int, target: dict) -> None:
        """Execute the task by sending a NavigateToPose goal to Nav2."""
        # Wait for Nav2 action server to be available
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                "NavigateToPose action server not available. Cannot execute task."
            )
            return

        pose = PoseStamped()
        pose.header.frame_id = target.get("frame_id", "map")
        pose.pose.position.x = float(target["x"])
        pose.pose.position.y = float(target["y"])
        yaw = float(target.get("yaw", 0.0))
        qz, qw = yaw_to_quaternion_z(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending NavigateToPose goal for task {task_id}: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw={yaw:.2f})"
        )

        # Send goal and wait for result (simple blocking pattern)
        send_future = self.navigate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"NavigateToPose goal for task {task_id} was rejected."
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        self.get_logger().info(
            f"NavigateToPose result for task {task_id}: status={result.status}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BidderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

