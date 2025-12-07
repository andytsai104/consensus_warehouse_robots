#!/usr/bin/env python3

import json
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AuctioneerNode(Node):
    """Auctioneer node for Phase 3.

    Current version:
    - Manages a list of tasks.
    - Starts an auction by broadcasting a task on /task_auction.
    - Collects bids from /task_bids.
    - After a timeout, selects the lowest-cost bid.
    - Publishes the assignment on /task_assignment.
    """

    def __init__(self) -> None:
        super().__init__("auctioneer_node")

        # Publisher for broadcasting tasks to all robots
        self.task_pub = self.create_publisher(String, "/task_auction", 10)

        # Publisher for announcing the winner of each task
        self.assignment_pub = self.create_publisher(String, "/task_assignment", 10)

        # Subscriber to receive bids from bidders
        self.bid_sub = self.create_subscription(
            String, "/task_bids", self.bid_callback, 10
        )

        # Simple static task list for testing
        self.tasks: List[Dict] = [
            {
                "task_id": 1,
                "target": {"frame_id": "map", "x": 1.0, "y": 1.0, "yaw": 0.0},
            },
            {
                "task_id": 2,
                "target": {"frame_id": "map", "x": -1.0, "y": 2.0, "yaw": 0.0},
            },
        ]

        # State for the current auction
        self.current_task: Optional[Dict] = None
        self.collected_bids: List[Dict] = []

        # Parameters
        self.declare_parameter("bid_timeout_sec", 3.0)
        self.bid_timeout_sec = (
            self.get_parameter("bid_timeout_sec").get_parameter_value().double_value
        )

        # Timer that periodically checks whether to start a new auction
        self.auction_timer = self.create_timer(1.0, self.auction_timer_callback)

        # Timer for bid collection timeout (created dynamically)
        self.bid_timeout_timer = None

        self.get_logger().info("AuctioneerNode with bidding logic started.")

    # ---------------- Auction lifecycle ----------------

    def auction_timer_callback(self) -> None:
        """Start a new auction if there is no active task and tasks are available."""
        # If an auction is currently running, do nothing.
        if self.current_task is not None:
            return

        # No more tasks to schedule
        if not self.tasks:
            return

        # Pop next task and start an auction
        self.current_task = self.tasks.pop(0)
        self.collected_bids = []

        msg = String()
        msg.data = json.dumps(self.current_task)
        self.task_pub.publish(msg)

        self.get_logger().info(
            f"Started auction for task {self.current_task['task_id']}: {msg.data}"
        )

        # Start or reset the bid timeout timer
        if self.bid_timeout_timer is not None:
            self.bid_timeout_timer.cancel()
        self.bid_timeout_timer = self.create_timer(
            self.bid_timeout_sec, self.bid_timeout_callback
        )

    # ---------------- Bid handling ----------------

    def bid_callback(self, msg: String) -> None:
        """Handle incoming bids from bidders on /task_bids."""
        if self.current_task is None:
            # No active auction; ignore stray bids
            return

        try:
            bid = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received invalid JSON bid.")
            return

        # Check that the bid belongs to the current task
        if bid.get("task_id") != self.current_task["task_id"]:
            return

        # Basic validation
        if "robot_id" not in bid or "cost" not in bid:
            self.get_logger().warn("Bid is missing robot_id or cost.")
            return

        self.collected_bids.append(bid)
        self.get_logger().info(
            f"Received bid for task {bid['task_id']} from {bid['robot_id']} "
            f"with cost {bid['cost']:.3f}"
        )

    # ---------------- Bid timeout / assignment ----------------

    def bid_timeout_callback(self) -> None:
        """Called when the bidding window closes for the current task."""
        # Stop this timer; we will create a new one for the next auction
        if self.bid_timeout_timer is not None:
            self.bid_timeout_timer.cancel()
            self.bid_timeout_timer = None

        if self.current_task is None:
            return

        task_id = self.current_task["task_id"]

        if not self.collected_bids:
            self.get_logger().warn(
                f"No bids received for task {task_id}. Dropping this task."
            )
            # Drop the task for now; policy can be changed (e.g. requeue).
            self.current_task = None
            return

        # Select the bid with the smallest cost
        best_bid = min(self.collected_bids, key=lambda b: b["cost"])
        winner_robot = best_bid["robot_id"]

        assignment = {
            "task_id": task_id,
            "robot_id": winner_robot,
            "target": self.current_task["target"],
        }

        msg = String()
        msg.data = json.dumps(assignment)
        self.assignment_pub.publish(msg)

        self.get_logger().info(
            f"Task {task_id} assigned to {winner_robot} "
            f"(cost={best_bid['cost']:.3f}). Assignment: {msg.data}"
        )

        # Clear current task so the next auction can start
        self.current_task = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AuctioneerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


