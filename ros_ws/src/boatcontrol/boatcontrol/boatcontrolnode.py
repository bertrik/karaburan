#python 3
import serial
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Initialize serial communication for actuator control
serial_port = "/dev/ttyUSB0"  # Update with your port
baud_rate = 115200  # Adjust baud rate as per your actuator settings
ser = serial.Serial(serial_port, baud_rate, timeout=1)

def clip(x, lo, hi):
    return max(lo, min(hi, x))

def to_int8(u):
    return int(clip(round(127 * u), -127, 127))

class BoatControlNode(Node):

    def __init__(self):
        super().__init__('boat_control_node')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.props_callback, 10)

    def start(self):
        time.sleep(0.5)
        response = ser.readline().decode().strip()  # Read response from the actuator
        self.get_logger().info(f"Received: {response}")
        response = ser.readline().decode().strip()  # Read response from the actuator
        self.get_logger().info(f"Received: {response}")
        self.send_enable_command()

    # Controls the propellors for the boat via duty cycle control.
    def props_callback(self, cmd_vel):
        self.send_enable_command()

        # Converting twist message to differntial drive control, includes clipping
        # This is an electronic speed controller
        v = cmd_vel.linear.x     # (m/s)
        w = cmd_vel.angular.z    # (rad/s)

        B = 0.3                  # (m)
        K = 1                    # To be determined!
        left  = (v - w*B/2) / K
        right = (v + w*B/2) / K

        self.send_pwm_command(to_int8(left), to_int8(right))

    def send_command(self, command):
        if ser.is_open:
            ser.write(command.encode())
            time.sleep(0.01)
            response=""
            while "OK" not in response: 
              response = ser.readline().decode().strip()  # Read response from the actuator
              time.sleep(0.01)
            self.get_logger().debug(f"Sent: {command}, Received: {response}")
            return response

    # Function to send enable motor command
    def send_calib_command(self):
        self.send_command(f"POST CALIB")

    # Function to send enable motor command
    def send_enable_command(self):
        self.send_command(f"POST WD 30") #enable motor for 30s
        
    # Function to send command to the actuator
    def send_pwm_command(self, left, right):
        self.send_command(f"POST PWM {left} {right}")

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
