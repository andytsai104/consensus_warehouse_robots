#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

class WarehouseRobot(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Get the robot name (namespace) from launch parameters
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value

        # Setup Nav2 Navigator
        self.navigator = BasicNavigator(namespace=self.robot_name)

        # State Variables (For your Consensus Algorithm later)
        self.current_status = "IDLE" # IDLE, BUSY, BIDDING
        self.current_pose = None

        self.get_logger().info(f"Robot Controller started for: {self.robot_name}")

        # # Wait for Nav2
        # self.navigator.waitUntilNav2Active()

    def go_to_task(self, x, y):
        """
        This function will be called when your Consensus Algorithm 
        decides this robot wins the bid.
        """
        self.current_status = "BUSY"
        self.get_logger().info(f"Starting task: Go to ({x}, {y})")

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0 # Facing forward

        self.navigator.goToPose(goal)
        
        # In a real node, you wouldn't block here with a while loop.
        # You would use a timer or callback to check progress so 
        # the node can still listen to other messages (like "Cancel Task").

    def bid_on_task(self, task_x, task_y):
        """
        Your Phase 3 Logic: Calculate cost to see if we want this job.
        """
        # Example: Calculate distance from self to task
        # cost = distance(self.current_pose, task_pose)
        # return cost
        pass

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = WarehouseRobot()
    
    # For testing right now:
    print("Waiting for Nav2...")
    node.navigator.waitUntilNav2Active()
    print("Nav2 is ready!")

    node.go_to_task(0.0, -5.0)

    print("Navigation is completed!")
    # Keep the node alive to listen for tasks
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()