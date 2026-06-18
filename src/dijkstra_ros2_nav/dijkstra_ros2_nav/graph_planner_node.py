import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
import math

# Importing the core math algorithm
from .dijkstra_adjacency import calculate_dijkstra_graph

class Nav2TopologicalPlanner(Node):
    def __init__(self):
        super().__init__('nav2_topological_planner')

        # Weighted topological graph
        self.graph_map = {
            'A': {'B': 3, 'C': 8, 'D': 6},
            'B': {'E': 9},
            'C': {'E': 4, 'F': 7},
            'D': {'C': 1},
            'E': {'F': 2},
            'F': {}  
        }

        # Translated coordinates
        self.node_coordinates = {
            'A': (0.0, 0.0),   
            'B': (2.0, 2.0),
            'C': (4.0, 0.0),
            'D': (2.0, -2.0),
            'E': (6.0, 2.0),
            'F': (8.0, 0.0)    
        }

        self.current_node = 'A'
        
        #Holds the most recent goal node to help re-plan to the same node on obstacle detection.
        self.current_goal_node = None

        #Holds the path being followed currently
        self.current_path = []

        #Holds the abstract path in the order sent to Nav2
        self.waypoint_nodes = []

        self._nav2_goal_handle = None

        # Listen for RViz clicks
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Listen for any obstacles
        self.obstacle_sub = self.create_subscription(String, '/obstacle_detected', self.obstacle_callback, 10)

        # Action Client to command the Nav2 driver
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info("Nav2 Topological Planner is active. Waiting for an RViz goal...")

    def find_closest_node(self, target_x, target_y):
        closest_node = None
        min_dist = float('inf')
        for node, (nx, ny) in self.node_coordinates.items():
            dist = math.sqrt((nx - target_x)**2 + (ny - target_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node

    def goal_callback(self, msg):
        self.get_logger().info("RViz goal received! Processing...")

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        start_node = self.current_node
        target_node = self.find_closest_node(goal_x, goal_y)
        
        self.plan_and_send(start_node, target_node)

    def plan_and_send(self, start_node, target_node):
        """Runs Dijkstra algorithm from start node to target node and sends the results to Nav2."""
        
        path, cost = calculate_dijkstra_graph(self.graph_map, start_node, target_node)

        if not path:
            self.get_logger().error(f"No path could be found to Node {target_node}.")
            return

        self.get_logger().info(f"Path found: {' -> '.join(path)} (Cost: {cost})")
        
        #Store the chosen goal and path in case of future obstacles
        self.current_goal_node = target_node
        self.current_path = path

        # Send the path to the robot
        self.send_waypoints_to_nav2(path)

    def obstacle_callback(self, msg):
        """Triggered when an obstacle is reported on a specific edge"""

        try: 
            node_a, node_b = msg.data.split(',')
            node_a, node_b = node_a.strip(), node_b.strip()
        except ValueError:
            self.get_logger.error(f"Incorrect format of the message. Expected 'NODE_A,NODE_B'.")
            return

        self.get_logger().warn(f"Obstacle on the edge from {node_a} to {node_b}")

        # Check if the blocked edge is part of the current path
        edge_in_use = any(
            self.current_path[i] == node_a and self.current_path[i + 1] == node_b
            for i in range(len(self.current_path) - 1)
        )

        if not edge_in_use:
            self.get_logger().info("Blocked edge is not on the current path")
            return

        if self.current_goal_node is None:
            self.get_logger().info("No active goal")
            return

        # Remove the blocked edge from the graph
        if node_b in self.graph_map.get(node_a, {}):
            del self.graph_map[node_a][node_b]
            self.get_logger().info(f"Removed edge {node_a} -> {node_b} from the graph.")
        
        # Cancel any existing Nav2 goal plan
        if self._nav2_goal_handle is not None:
            self.get_logger().info("Cancelling current Nav2 plan..")
            cancel_future = self._nav2_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda fut: self._replan_after_cancel())

        else:
            self._replan_after_cancel()

    def _replan_after_cancel(self):
        """Re-plans the route from the current position to the previosuly specified goal after removing the blocked edge."""
        self.get_logger().info(f"Re-planning from {self.current_node} to {self.current_goal_node}")
        self.plan_and_send(self.current_node, self.current_goal_node)


    def send_waypoints_to_nav2(self, abstract_path):
        """Translates the abstract path to physical coordinates and sends to Nav2."""
        
        # Make sure Nav2 is actually running
        if not self.waypoint_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 Waypoint server is not available. Make sure Nav2 is running')
            return

        waypoints = []
        waypoint_nodes = []
        for node in abstract_path:
            # Skip the starting node so the robot does not try to drive to where it already is
            if node == abstract_path[0]:
                continue 

            px, py = self.node_coordinates[node]
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 
            
            waypoints.append(pose)
            waypoint_nodes.append(node)

        if not waypoints:
            self.get_logger().info("Robot is already at the destination.")
            return

        # Helps the feedback callback translate a waypoint index into a node name
        self.waypoint_nodes = waypoint_nodes

        self.get_logger().info(f"Sending {len(waypoints)} physical waypoints to Nav2 driver...")
        
        # Package the waypoints and send them to the Action Server
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        send_future = self.waypoint_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Continuously called by Nav2 to update the nodes reached"""
        current_index = feedback_msg.feedback.current_waypoint
        completed_index = current_index - 1

        if 0 <= completed_index < len(self.waypoint_nodes):
            reached_node = self.waypoint_nodes[completed_index]
            if reached_node != self.current_node:
                self.current_node = reached_node
                self.get_logger().info(f"Robot has reached node {reached_node}.")  

    def _goal_response_callback(self, future):
        """Stores the goal handle."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self._nav2_goal_handle = goal_handle
        self.get_logger().info("Goal accepted")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2TopologicalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()