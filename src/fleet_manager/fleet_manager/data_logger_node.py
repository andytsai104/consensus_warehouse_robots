#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from nav_msgs.msg import Odometry


class RobotState(Enum):
    """Simple robot state for idle/working accounting."""
    IDLE = auto()
    WORKING = auto()


@dataclass
class RobotMetrics:
    """Per-robot metrics and state."""
    robot_id: str

    # Distance metrics
    total_distance: float = 0.0
    last_odom_x: Optional[float] = None
    last_odom_y: Optional[float] = None

    # Time tracking
    state: RobotState = RobotState.IDLE
    last_state_change_time: Optional[Time] = None
    total_idle_time: float = 0.0
    total_work_time: float = 0.0

    # Current task context
    current_task_id: Optional[int] = None
    current_task_start_time: Optional[Time] = None
    distance_at_task_start: float = 0.0


@dataclass
class TaskMetrics:
    """Per-task metrics."""
    task_id: int
    robot_id: str
    start_time: Time
    end_time: Optional[Time] = None
    completion_time: Optional[float] = None
    distance_traveled: Optional[float] = None


class DataLoggerNode(Node):
    """Data logger for multi-robot metrics.

    Metrics recorded:
      - Total distance traveled per robot (from /<robot_id>/odom).
      - Task completion time per task (from /task_events).
      - Idle time per robot (from /task_events and robot state).
    """

    def __init__(self) -> None:
        super().__init__("data_logger")

        # List of robot ids/namespaces. Default: 5 robots.
        # Example: ["robot1", "robot2", "robot3", "robot4", "robot5"]
        self.declare_parameter(
            "robot_ids",
            ["robot1", "robot2", "robot3", "robot4", "robot5"],
        )
        robot_ids_param = self.get_parameter("robot_ids").get_parameter_value()
        self.robot_ids = list(robot_ids_param.string_array_value)

        # Per-robot metrics
        self.robots: Dict[str, RobotMetrics] = {
            robot_id: RobotMetrics(robot_id=robot_id)
            for robot_id in self.robot_ids
        }

        # Per-task metrics
        self.tasks: Dict[int, TaskMetrics] = {}

        # Subscriptions for odometry of each robot
        for robot_id in self.robot_ids:
            topic_name = f"/{robot_id}/odom"
            self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                20,
            )
            self.get_logger().info(
                f"[LOGGER] Subscribing to odometry: {topic_name}"
            )

        # Subscription for task events (JSON)
        # Expected schema:
        #   {"event": "task_started", "task_id": 1, "robot_id": "robot3"}
        #   {"event": "task_completed", "task_id": 1, "robot_id": "robot3"}
        self.task_event_sub = self.create_subscription(
            String, "/task_events", self.task_event_callback, 50
        )

        # Periodic timer to optionally print periodic summaries (optional)
        self.summary_timer = self.create_timer(30.0, self.periodic_summary)

        now = self.get_clock().now()
        for metrics in self.robots.values():
            metrics.last_state_change_time = now

        self.get_logger().info(
            f"DataLoggerNode started for robots: {', '.join(self.robot_ids)}"
        )

    # ------------------------------------------------------------------
    # Odometry handling: total distance per robot
    # ------------------------------------------------------------------

    def odom_callback(self, msg: Odometry, robot_id: str) -> None:
        """Update total distance traveled for a robot from odometry."""
        metrics = self.robots.get(robot_id)
        if metrics is None:
            # Unknown robot id; ignore
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if metrics.last_odom_x is not None and metrics.last_odom_y is not None:
            dx = x - metrics.last_odom_x
            dy = y - metrics.last_odom_y
            dist = math.hypot(dx, dy)
            metrics.total_distance += dist

        metrics.last_odom_x = x
        metrics.last_odom_y = y

    # ------------------------------------------------------------------
    # Task event handling: task completion time + idle time
    # ------------------------------------------------------------------

    def task_event_callback(self, msg: String) -> None:
        """Handle task events to update idle time and task metrics."""
        try:
            evt = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"[LOGGER] Received invalid JSON on /task_events: {msg.data}"
            )
            return

        event_type = evt.get("event")
        task_id = evt.get("task_id")
        robot_id = evt.get("robot_id")

        if event_type is None or task_id is None or robot_id is None:
            self.get_logger().warn(
                "[LOGGER] /task_events missing event, task_id or robot_id."
            )
            return

        if robot_id not in self.robots:
            self.get_logger().warn(
                f"[LOGGER] /task_events refers to unknown robot_id={robot_id}."
            )
            return

        metrics = self.robots[robot_id]
        now = self.get_clock().now()

        if event_type == "task_started":
            self._handle_task_started(metrics, int(task_id), now)
        elif event_type == "task_completed":
            self._handle_task_completed(metrics, int(task_id), now)
        else:
            self.get_logger().warn(
                f"[LOGGER] Unknown event type on /task_events: {event_type}"
            )

    def _handle_task_started(
        self,
        metrics: RobotMetrics,
        task_id: int,
        now: Time,
    ) -> None:
        """Update robot and task state on task start."""
        # Close previous idle period if currently idle
        if metrics.state == RobotState.IDLE and metrics.last_state_change_time:
            idle_sec = (
                now - metrics.last_state_change_time
            ).nanoseconds * 1e-9
            metrics.total_idle_time += idle_sec

        metrics.state = RobotState.WORKING
        metrics.last_state_change_time = now
        metrics.current_task_id = task_id
        metrics.current_task_start_time = now
        metrics.distance_at_task_start = metrics.total_distance

        # Create or overwrite task metrics
        self.tasks[task_id] = TaskMetrics(
            task_id=task_id,
            robot_id=metrics.robot_id,
            start_time=now,
        )

        self.get_logger().info(
            f"[LOGGER] Task {task_id} started by {metrics.robot_id} "
            f"at t={self._format_time(now)}"
        )

    def _handle_task_completed(
        self,
        metrics: RobotMetrics,
        task_id: int,
        now: Time,
    ) -> None:
        """Update robot and task state on task completion."""
        # Close work period if currently working
        if metrics.state == RobotState.WORKING and metrics.last_state_change_time:
            work_sec = (
                now - metrics.last_state_change_time
            ).nanoseconds * 1e-9
            metrics.total_work_time += work_sec

        metrics.state = RobotState.IDLE
        metrics.last_state_change_time = now

        # Compute task metrics if we have a record
        task = self.tasks.get(task_id)
        if task is None:
            # Create one if it does not exist
            task = TaskMetrics(
                task_id=task_id,
                robot_id=metrics.robot_id,
                start_time=now,
            )
            self.tasks[task_id] = task

        task.end_time = now
        task.completion_time = (
            (task.end_time - task.start_time).nanoseconds * 1e-9
            if task.start_time and task.end_time
            else None
        )
        task.distance_traveled = (
            metrics.total_distance - metrics.distance_at_task_start
        )

        # Reset current task context on robot
        metrics.current_task_id = None
        metrics.current_task_start_time = None
        metrics.distance_at_task_start = metrics.total_distance

        # Print a clear per-task summary
        self._log_task_summary(task, metrics)

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log_task_summary(
        self,
        task: TaskMetrics,
        metrics: RobotMetrics,
    ) -> None:
        """Print a summary block for a completed task and robot metrics."""
        start_str = self._format_time(task.start_time)
        end_str = self._format_time(task.end_time)
        completion = task.completion_time if task.completion_time else 0.0
        dist = task.distance_traveled if task.distance_traveled else 0.0

        lines = [
            "================ TASK METRICS ================",
            f"Task ID             : {task.task_id}",
            f"Robot               : {task.robot_id}",
            f"Start time          : {start_str}",
            f"End time            : {end_str}",
            f"Completion time     : {completion:.2f} s",
            f"Distance in task    : {dist:.3f} m",
            "---------------- ROBOT METRICS ---------------",
            f"Total distance      : {metrics.total_distance:.3f} m",
            f"Total idle time     : {metrics.total_idle_time:.2f} s",
            f"Total work time     : {metrics.total_work_time:.2f} s",
            "==============================================",
        ]
        self.get_logger().info("\n" + "\n".join(lines))

    def periodic_summary(self) -> None:
        """Optional: periodically print overall metrics per robot."""
        self.get_logger().info("[LOGGER] Periodic robot metrics summary:")
        for robot_id, metrics in self.robots.items():
            self.get_logger().info(
                f"  {robot_id}: "
                f"distance={metrics.total_distance:.3f} m, "
                f"idle={metrics.total_idle_time:.2f} s, "
                f"work={metrics.total_work_time:.2f} s"
            )

    @staticmethod
    def _format_time(t: Optional[Time]) -> str:
        """Format ROS Time as seconds with 3 decimals."""
        if t is None:
            return "N/A"
        sec = t.nanoseconds * 1e-9
        return f"{sec:.3f}s (sim time)"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

