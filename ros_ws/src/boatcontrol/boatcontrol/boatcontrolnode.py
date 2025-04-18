#python 3
import serial
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Initialize serial communication for actuator control
serial_port = "/dev/ttyUSB0"  # Update with your port
baud_rate = 115200  # Adjust baud rate as per your actuator settings
ser = serial.Serial(serial_port, baud_rate, timeout=1)

class BoatControlNode(Node):

    def __init__(self):
        super().__init__('boat_control_node')
        self.compass_pub = self.create_publisher(Float64, '/compass', 10)
        self.heading_sub = self.create_subscription(Float64, '/desired_heading', self.heading_callback, 10)
        self.speed_sub = self.create_subscription(Float64, '/desired_speed', self.speed_callback, 10)
        self.current_heading = None
        timer_period = 1  # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def start(self):
        time.sleep(0.5)
        response = ser.readline().decode().strip()  # Read response from the actuator
        self.get_logger().info(f"Received: {response}")
        response = ser.readline().decode().strip()  # Read response from the actuator
        self.get_logger().info(f"Received: {response}")
        self.send_enable_command()
        time.sleep(1)
        self.send_calib_command()
        self.send_speed_command(30)

    def timer_callback(self):
        self.send_enable_command()
        current_heading = self.get_heading_command()
        current_heading = self.get_heading_command()# dual reading to flush !

        msg = Float64()
        msg.data = current_heading
        self.compass_pub.publish(msg)

    def heading_callback(self, heading):
        self.send_nav_command(10, heading)

    def speed_callback(self, speed):
        self.send_speed_command(speed)

    def send_command(self, command):
        if ser.is_open:
            ser.write(command.encode())
            time.sleep(1)
            response=""
            while "OK" not in response: 
              response = ser.readline().decode().strip()  # Read response from the actuator
              time.sleep(0.2)
            self.get_logger().info(f"Sent: {command}, Received: {response}")
            return response

    # Function to send enable motor command
    def send_calib_command(self):
        self.send_command(f"POST CALIB")

    # Function to send enable motor command
    def send_enable_command(self):
        self.send_command(f"POST WD 30") #enable motor for 30s
        
    # Function to send command to the actuator
    def send_move_command(self, command):
        self.send_command(f"POST MOVE {command}")

    def send_nav_command(self, duration, heading):
        self.send_command(f"POST NAV {duration} {heading}")

    def send_speed_command(self, speed):
        self.send_command(f"POST SPEED {speed}")

    # Function to send command to the actuator
    def get_heading_command(self):
        response = self.send_command(f"GET HEADING")
            
        # Parse the heading from the response
        try:
            # Find the start and end of the heading value within the response string
            start = response.index("heading:") + len("heading:")
            end = response.index("}", start)
            heading_value = float(response[start:end].strip())  # Convert to float
            return heading_value
        except (ValueError, IndexError) as e:
            self.get_logger().info(f"Error parsing heading: {e}")
            return None

def main():
    rclpy.init()
    node = BoatControlNode()
    node.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Control loop interrupted. Exiting...")
    finally:
        if ser.is_open:
            ser.close()
