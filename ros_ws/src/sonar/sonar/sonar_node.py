#!/usr/bin/env python3
# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>
import math
import threading

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Range


class Sonar:
    def __init__(self, device):
        self.serial = serial.Serial(device, baudrate=230400, timeout=None)

    def open(self) -> None:
        if not self.serial.is_open:
            self.serial.open()

    def poll(self) -> bytes | None:
        return None

    def close(self) -> None:
        self.serial.close()


class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar')
        self.declare_parameter('device', value='/dev/ttyUSB0')
        self.declare_parameter('topic', value='/sonar')
        self.declare_parameter('frame_id', value='sonar')

        device = self.get_parameter('device').get_parameter_value().string_value
        self.sonar = Sonar(device)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(Range, topic, 10)

        # initialise Range message
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id
        self.msg.radiation_type = Range.ULTRASOUND
        self.msg.field_of_view = math.radians(45)
        self.msg.min_range = 0.3  # estimated
        self.msg.max_range = 60.0  # 200 feet actually

        # create and start listen thread
        self.thread = threading.Thread(target=self.node_task())
        self.thread.start()

    def node_task(self) -> None:
        self.sonar.open()
        while rclpy.ok():
            frame = self.sonar.poll()
            if frame:
                self.process_frame(frame)
        self.sonar.close()

    def process_frame(self, frame: bytes) -> None:
        now = self.get_clock().now()

        # publish message
        self.msg.range = 12.34
        self.msg.header.stamp = now.to_msg()
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
