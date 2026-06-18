#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class TestDriver(Node):
    def __init__(self, goal_x, goal_y, obstacle_edge, delay_seconds):
        super().__init__('test_driver')
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle_detected', 10)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.obstacle_edge = obstacle_edge
        self.delay_seconds = delay_seconds

    def run(self):
        time.sleep(1.0)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = self.goal_x
        goal_msg.pose.position.y = self.goal_y
        goal_msg.pose.orientation.w = 1.0
        self.get_logger().info(f"Publishing goal toward ({self.goal_x}, {self.goal_y})...")
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Waiting {self.delay_seconds}s before publishing obstacle on edge '{self.obstacle_edge}'...")
        time.sleep(self.delay_seconds)
        obstacle_msg = String()
        obstacle_msg.data = self.obstacle_edge
        self.obstacle_pub.publish(obstacle_msg)
        self.get_logger().info(f"Published obstacle on edge '{self.obstacle_edge}'.")
        time.sleep(1.0)


def main():
    if len(sys.argv) != 5:
        print("Usage: python3 test_driver.py <goal_x> <goal_y> <obstacle_edge> <delay_seconds>")
        sys.exit(1)
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    obstacle_edge = sys.argv[3]
    delay_seconds = float(sys.argv[4])
    rclpy.init()
    node = TestDriver(goal_x, goal_y, obstacle_edge, delay_seconds)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
