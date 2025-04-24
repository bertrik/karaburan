import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from std_msgs.msg import Float64
from boat_interfaces.msg import Waypoints
from nav2_simple_commander.robot_navigator import BasicNavigator
import math

class BoatNavigator(Node):
    def __init__(self):
        super().__init__('boat_navigator')

        self.get_logger().info("Waiting for Basic navigator to be activated")

        # Initialize the Nav2 navigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.get_logger().info("Basic navigator has been activated")

        # Publishers for heading and speed
        self.heading_pub = self.create_publisher(Float64, '/desired_heading', 10)
        self.speed_pub = self.create_publisher(Float64, '/desired_speed', 10)

        # Subscriber for waypoint list
        self.waypoint_sub = self.create_subscription(
            Waypoints,
            '/waypoints',
            self.waypoint_callback,
            10
        )

        self.waypoints = []  # Empty waypoint list, filled dynamically
        self.is_navigating = False  # Prevent navigation before waypoints are received

    def waypoint_callback(self, msg):
        """ Callback to update the waypoint list dynamically. """
        self.waypoints = [(wp.pose.position.y, wp.pose.position.x) for wp in msg.waypoints]
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

        # Start navigation only if not already running
        if not self.is_navigating:
            self.navigate_waypoints()

    def publish_navigation_state(self, speed, heading):
        """ Publish computed speed and heading to be used by the motor controller. """
        speed_msg = Float64()
        heading_msg = Float64()
        
        speed_msg.data = speed
        heading_msg.data = heading
        
        self.speed_pub.publish(speed_msg)
        self.heading_pub.publish(heading_msg)

    def navigate_waypoints(self):
        """ Navigate through a list of GPS waypoints sequentially. """
        if not self.waypoints:
            self.get_logger().warning("No waypoints received yet.")
            return

        self.is_navigating = True  # Lock navigation state

        for goal_lat, goal_lon in self.waypoints:
            self.get_logger().info(f"Navigating to waypoint: {goal_lat}, {goal_lon}")

            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # Coordinate frame
            goal_msg.header.stamp = self.get_clock().now().to_msg()

            goal_msg.pose.position.x = goal_lon  # Longitude as x coordinate
            goal_msg.pose.position.y = goal_lat  # Latitude as y coordinate

            # Send waypoint goal to Nav2
            self.navigator.go_to_pose(goal_msg)

            # Control loop: adjust speed and heading while moving to waypoint
            while not self.navigator.is_task_complete():
                # Get current position from Nav2
                current_position = self.navigator.get_state().get_pose()
                current_lat = current_position.pose.position.y
                current_lon = current_position.pose.position.x
                current_heading = current_position.pose.orientation.z  # Yaw angle
                
                # Compute heading adjustment
                target_heading = self.compute_target_heading(goal_lat, goal_lon, current_lat, current_lon)
                speed = self.compute_speed(goal_lat, goal_lon, current_lat, current_lon)

                # Publish heading and speed
                self.publish_navigation_state(speed, target_heading)

                rclpy.spin_once(self)

            self.get_logger().info(f"Reached waypoint: {goal_lat}, {goal_lon}")

        self.get_logger().info("All waypoints completed!")
        self.is_navigating = False  # Reset navigation state

    def compute_target_heading(self, goal_lat, goal_lon, current_lat, current_lon):
        """ Compute the heading towards the goal based on GPS coordinates. """
        delta_lon = goal_lon - current_lon
        delta_lat = goal_lat - current_lat
        target_heading = math.atan2(delta_lat, delta_lon)
        target_heading = math.degrees(target_heading)  # Convert to degrees
        return target_heading

    def compute_speed(self, goal_lat, goal_lon, current_lat, current_lon):
        """ Compute speed based on distance to target. """
        distance = math.sqrt((goal_lon - current_lon) ** 2 + (goal_lat - current_lat) ** 2)

        # Define max and min speed limits
        max_speed = 1.5  # m/s (example)
        min_speed = 0.3  # m/s

        # Slow down as the boat gets closer to the waypoint
        speed = max_speed * (distance / 0.01)  # Scale based on distance (0.01 is an arbitrary normalization factor)
        speed = max(min_speed, min(speed, max_speed))  # Clamp between min and max

        return speed


def main():
    rclpy.init()
    navigator = BoatNavigator()
    rclpy.spin(navigator)  # Wait for waypoints to be received
    rclpy.shutdown()

if __name__ == '__main__':
    main()

