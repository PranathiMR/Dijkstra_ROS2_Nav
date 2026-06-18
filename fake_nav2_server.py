import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import FollowWaypoints
import time


class FakeNav2Server(Node):
    def __init__(self):
        super().__init__('fake_nav2_server')

        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            'follow_waypoints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._cancel_requested = False

        self.get_logger().info(
            "Fake Nav2 FollowWaypoints server is up. Waiting for goals..."
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal with {len(goal_request.poses)} waypoints. Accepting."
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received. Accepting.")
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._cancel_requested = False
        waypoints = goal_handle.request.poses
        feedback_msg = FollowWaypoints.Feedback()

        self.get_logger().info(
            f"Executing goal: driving through {len(waypoints)} waypoints "
            f"(simulated, ~2s per waypoint)..."
        )

        for i, pose in enumerate(waypoints):
            if self._cancel_requested:
                self.get_logger().info(
                    f"Cancellation honored at waypoint index {i}. Stopping."
                )
                goal_handle.canceled()
                result = FollowWaypoints.Result()
                result.missed_waypoints = list(range(i, len(waypoints)))
                return result

            elapsed = 0.0
            while elapsed < 2.0:
                if self._cancel_requested:
                    self.get_logger().info(
                        f"Cancellation honored mid-travel toward waypoint "
                        f"index {i}. Stopping before reaching it."
                    )
                    goal_handle.canceled()
                    result = FollowWaypoints.Result()
                    result.missed_waypoints = list(range(i, len(waypoints)))
                    return result
                time.sleep(0.1)
                elapsed += 0.1

            feedback_msg.current_waypoint = i + 1
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"Reached waypoint index {i} "
                f"(x={pose.pose.position.x}, y={pose.pose.position.y}). "
                f"Feedback published: current_waypoint={i + 1}"
            )

        goal_handle.succeed()
        result = FollowWaypoints.Result()
        result.missed_waypoints = []
        self.get_logger().info("All waypoints completed successfully.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeNav2Server()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
