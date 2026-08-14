# python 3
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def to_int8(u):
    return int(clip(round(127 * u), -127, 127))


class BoatControlNode(Node):

    def __init__(self):
        super().__init__('boat_control_node')
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.props_callback, 10)
        self.id = 0
        self.get_logger().info(
            f'Connected to motor controller on {serial_port} at {baud_rate} baud'
        )

    def start(self):
        time.sleep(0.5)
        response = self.ser.readline().decode().strip()
        self.get_logger().info(f'Received: {response}')
        response = self.ser.readline().decode().strip()
        self.get_logger().info(f'Received: {response}')

    # Controls the propellors for the boat via duty cycle control.
    def props_callback(self, cmd_vel):
        # Converting twist message to differntial drive control, includes clipping
        # This is an electronic speed controller
        v = cmd_vel.linear.x     # (m/s)
        w = cmd_vel.angular.z    # (rad/s)

        B = 0.3                  # (m)
        K = 1                    # To be determined!
        left = (v + w * B / 2) / K
        right = (v - w * B / 2) / K

        self.id = self.id + 1

        self.send_pwm_command(to_int8(left), to_int8(right))

    def send_command(self, command):
        if self.ser.is_open:
            self.ser.write(command.encode())
            time.sleep(0.01)
            response = ''
            while 'OK' not in response:
                response = self.ser.readline().decode().strip()
                time.sleep(0.01)
            self.get_logger().info(f'Sent: {command}, Received: {response}')
            return response

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    # Function to send command to the actuator
    def send_pwm_command(self, left, right):
        self.send_command(f'POST PWM {left} {right} {self.id}')


def main():
    rclpy.init()
    node = BoatControlNode()
    try:
        node.start()
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Control loop interrupted. Exiting...')
