#!/usr/bin/env python3

import json
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AuctioneerNode(Node):
    """Task broadcaster for consensus-based Phase 3.

    In the consensus version:
    - This node ONLY broadcasts tasks on /task_auction.
    - It DOES NOT collect bids or decide the winner.
    - Each bidder node will subscribe to /task_bids and
      run the "winner selection" logic locally (consensus).
    """

    def __init__(self) -> None:
        super().__init__("auctioneer_node")

        # Publisher for broadcasting tasks to all robots
        self.task_pub = self.create_publisher(String, "/task_auction", 10)

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
        self.current_index = 0

        # Timer: broadcast one task every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info(
            "AuctioneerNode (task broadcaster only) started for consensus mode."
        )

    def timer_callback(self) -> None:
        """Publish the next task in the list on /task_auction."""
        if self.current_index >= len(self.tasks):
            self.get_logger().info("No more tasks to broadcast.")
            return

        task = self.tasks[self.current_index]
        self.current_index += 1

        msg = String()
        msg.data = json.dumps(task)
        self.task_pub.publish(msg)

        self.get_logger().info(
            f"[BROADCAST] Task {task['task_id']} on /task_auction: {msg.data}"
        )


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

