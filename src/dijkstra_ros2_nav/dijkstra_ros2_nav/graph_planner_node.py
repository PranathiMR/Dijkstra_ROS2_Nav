import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

# Import your core algorithm!
from .dijkstra_adjacency import calculate_dijkstra_graph

class TopologicalPlannerNode(Node):
    def __init__(self):
        super().__init__('topological_planner')

        # 1. THE TOPOLOGICAL GRAPH (The abstract math)
        self.graph_map = {
            'A': {'B': 3, 'C': 8, 'D': 6},
            'B': {'E': 9},
            'C': {'E': 4, 'F': 7},
            'D': {'C': 1},
            'E': {'F': 2},
            'F': {}  
        }

        # 2. THE PHYSICAL TRANSLATION DICTIONARY 
        # Here we assign real-world Gazebo coordinates (x, y) in meters to each Node
        self.node_coordinates = {
            'A': (0.0, 0.0),   # Let's assume the robot spawns here
            'B': (2.0, 2.0),
            'C': (4.0, 0.0),
            'D': (2.0, -2.0),
            'E': (6.0, 2.0),
            'F': (8.0, 0.0)    # The final destination
        }

        # 3. Subscriptions and Publishers
        # Listen for the user clicking "2D Nav Goal" in RViz
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Broadcast the calculated physical path to the robot's local controllers
        self.path_pub = self.create_publisher(Path, '/topological_path', 10)

        self.get_logger().info("Topological Planner Node is awake! Waiting for an RViz goal...")

    def find_closest_node(self, target_x, target_y):
        """Finds the closest abstract Node letter to a physical (x, y) coordinate."""
        closest_node = None
        min_dist = float('inf')

        for node, (nx, ny) in self.node_coordinates.items():
            dist = math.sqrt((nx - target_x)**2 + (ny - target_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_node = node

        return closest_node

    def goal_callback(self, msg):
        """Fires instantly when you click a destination in RViz."""
        self.get_logger().info("RViz goal received! Processing...")

        # Step A: Get physical meters from the click
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        # Step B: Find which abstract node represents the robot's current location and goal
        # (For this example, we assume the robot is currently sitting at Node 'A')
        start_node = 'A'
        target_node = self.find_closest_node(goal_x, goal_y)
        
        self.get_logger().info(f"Translating RViz click ({goal_x:.2f}, {goal_y:.2f}) -> Node '{target_node}'")

        # Step C: Run the pure Python Dijkstra Math!
        path, cost = calculate_dijkstra_graph(self.graph_map, start_node, target_node)

        if not path:
            self.get_logger().error(f"No path could be found to Node {target_node}.")
            return

        self.get_logger().info(f"Mathematical Path found: {' -> '.join(path)} (Cost: {cost})")

        # Step D: Translate the abstract path back into physical waypoints
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for abstract_node in path:
            px, py = self.node_coordinates[abstract_node]
            
            # Create a ROS Pose message for each waypoint
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # Pointing straight ahead
            
            ros_path.poses.append(pose)

        # Step E: Broadcast the path to the rest of the robot
        self.path_pub.publish(ros_path)
        self.get_logger().info("Physical path published to local controllers!\n")

def main(args=None):
    rclpy.init(args=args)
    node = TopologicalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()