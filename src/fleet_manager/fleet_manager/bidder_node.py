#!/usr/bin/env python3

import json
import math
from typing import Dict, List, Optional

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
    """Bidder node for consensus-based Phase 3.

    In the consensus version:
    - Subscribes to /task_auction (JSON encoded tasks).
      * Computes a real cost using Nav2 ComputePathToPose (path length).
      * Publishes its bid on /task_bids.
    - Subscribes to /task_bids and collects all bids for the current task.
    - After a local timeout, selects the lowest-cost bid LOCALLY.
      * If this robot is the winner, it executes the task via NavigateToPose.
    - No central node decides the winner; every bidder runs the same rule,
      so they reach consensus on the winner.
    """

    def __init__(self) -> None:
        super().__init__("bidder_node")

        # Parameter to identify this robot in the auction
        self.declare_parameter("robot_id", "robot1")
        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        # Parameter controlling how long we wait for bids (seconds)
        self.declare_parameter("bid_timeout_sec", 3.0)
        self.bid_timeout_sec = (
            self.get_parameter("bid_timeout_sec").get_parameter_value().double_value
        )

        # Simple availability flag so the robot only handles one task at a time
        self.available: bool = True

        # Current task under auction (for this robot's local consensus)
        self.current_task: Optional[Dict] = None
        self.current_task_start_time: Optional[float] = None
        self.collected_bids: List[Dict] = []

        # Publisher to send bids
        self.bid_pub = self.create_publisher(String, "/task_bids", 10)

        # Subscriber to receive task auctions
        self.task_sub = self.create_subscription(
            String, "/task_auction", self.task_callback, 10
        )

        # Subscriber to receive all bids (from all robots)
        self.bids_sub = self.create_subscription(
            String, "/task_bids", self.bid_callback, 10
        )

        # Action client to compute path cost with Nav2
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )

        # Action client to send navigation goals to Nav2
        # With namespace="robot1", this becomes /robot1/navigate_to_pose.
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        # Timer to periodically check whether the bidding window has closed
        self.consensus_timer = self.create_timer(
            0.5, self.consensus_timer_callback
        )

        self.get_logger().info(
            f"BidderNode (consensus mode) started for robot_id={self.robot_id}, "
            f"bid_timeout_sec={self.bid_timeout_sec:.1f}"
        )

    # ---------------- Auction handling ----------------

    def task_callback(self, msg: String) -> None:
        """Handle incoming task and publish a bid based on path length."""
        # If we are already processing a task (waiting for bids or executing),
        # ignore new tasks for now. Policy can be changed if needed.
        if self.current_task is not None:
            return

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

        # Store this as the current task under local auction
        self.current_task = task
        self.current_task_start_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )
        self.collected_bids = []

        # Compute cost using Nav2 ComputePathToPose
        cost = self.compute_cost_to_target(target)
        if cost is None:
            self.get_logger().warn(
                f"Failed to compute path cost for task {task_id}; no bid sent."
            )
            # We still keep current_task so we can see others' bids and
            # participate in consensus on the winner, even if we do not bid.
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
            f"[BID] Sent bid for task {task_id} with cost {cost:.3f} "
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
            f"[COST] Requesting path to "
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

        self.get_logger().info(f"[COST] Computed path length: {length:.3f}")
        return length

    # ---------------- Bid collection (for consensus) ----------------

    def bid_callback(self, msg: String) -> None:
        """Collect all bids for the current task from /task_bids."""
        if self.current_task is None:
            # No active task under local auction
            return

        try:
            bid = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON bid.")
            return

        task_id = bid.get("task_id")
        robot_id = bid.get("robot_id")
        cost = bid.get("cost")

        if task_id is None or robot_id is None or cost is None:
            self.get_logger().warn("Bid is missing task_id, robot_id or cost.")
            return

        # Only collect bids for the current task
        if task_id != self.current_task.get("task_id"):
            return

        # Append to local list; we rely on the same set of bids being seen by
        # all robots to achieve consensus on the winner.
        self.collected_bids.append(bid)
        self.get_logger().info(
            f"[BID-RECV] Saw bid for task {task_id} from {robot_id} "
            f"with cost {cost:.3f}"
        )

    # ---------------- Local consensus on the winner ----------------

    def consensus_timer_callback(self) -> None:
        """Periodically check if the bidding window has closed for the current task."""
        if self.current_task is None:
            return

        if self.current_task_start_time is None:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now_sec - self.current_task_start_time

        if elapsed < self.bid_timeout_sec:
            # Still waiting for more bids
            return

        task_id = self.current_task.get("task_id")

        if not self.collected_bids:
            self.get_logger().warn(
                f"[CONSENSUS] No bids received for task {task_id} "
                f"(from local perspective). Dropping task."
            )
            # Clear state and wait for the next task
            self.current_task = None
            self.current_task_start_time = None
            self.collected_bids = []
            return

        # Select the bid with the smallest cost (local decision).
        best_bid = min(self.collected_bids, key=lambda b: b["cost"])
        winner_robot = best_bid["robot_id"]

        self.get_logger().info(
            f"[CONSENSUS] Local winner for task {task_id}: {winner_robot} "
            f"(cost={best_bid['cost']:.3f})"
        )

        # If this robot is the winner, execute the task
        if winner_robot == self.robot_id:
            if not self.available:
                self.get_logger().warn(
                    f"[CONSENSUS] I am winner for task {task_id}, "
                    f"but I am marked as busy. Skipping execution."
                )
            else:
                target = self.current_task.get("target")
                if target is None:
                    self.get_logger().warn(
                        f"[CONSENSUS] No target found in task {task_id}."
                    )
                else:
                    self.available = False
                    self.get_logger().info(
                        f"[EXEC] Task {task_id} assigned to me ({self.robot_id}). "
                        f"Target: {target}"
                    )
                    self.execute_task(task_id, target)
                    self.available = True
                    self.get_logger().info(
                        f"[EXEC] Task {task_id} completed by {self.robot_id}."
                    )

        # Clear state so the next task can be processed
        self.current_task = None
        self.current_task_start_time = None
        self.collected_bids = []

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
            f"[EXEC] Sending NavigateToPose goal for task {task_id}: "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, yaw={yaw:.2f})"
        )

        # Send goal and wait for result (simple blocking pattern)
        send_future = self.navigate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f"[EXEC] NavigateToPose goal for task {task_id} was rejected."
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        self.get_logger().info(
            f"[EXEC] NavigateToPose result for task {task_id}: status={result.status}"
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

