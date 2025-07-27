#!/usr/bin/env python3
# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>
import threading
from enum import Enum

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FrameExtractor:
    class State(Enum):
        HEADER_1 = 1
        HEADER_2 = 2
        COLLECT = 3

    def __init__(self) -> None:
        self.state = self.State.HEADER_1
        self.index = 0
        self.length = 0
        self.data = bytearray(60)

    def add_data(self, b):
        self.data[self.index] = b
        self.index = self.index + 1

    def process(self, b):
        """ processes one byte, returns True if a full frame was received """
        match self.state:
            case self.State.HEADER_1:
                self.index = 0
                if b == 0x55:
                    self.add_data(b)
                    self.state = self.State.HEADER_2

            case self.State.HEADER_2:
                if b == 0xAA:
                    self.add_data(b)
                    self.length = len(self.data)
                    self.state = self.State.COLLECT
                else:
                    self.state = self.State.HEADER_1
                    self.process(b)

            case self.State.COLLECT:
                if self.index < self.length:
                    self.add_data(b)
                if self.index == self.length:
                    # done
                    self.state = self.State.HEADER_1
                    return True
        return False

    def get_data(self):
        return self.data


class MysteryLidar:
    def __init__(self, device):
        self.serial = serial.Serial(device, baudrate=230400, timeout=0.1)
        self.extractor = FrameExtractor()

    def open(self) -> None:
        self.serial.open()

    def poll(self) -> bytes | None:
        num = self.serial.in_waiting
        while num > 0:
            num = num - 1
            b = self.serial.read()
            done = self.extractor.process(b)
            if done:
                return self.extractor.get_data()
        return None

    def close(self) -> None:
        self.serial.close()


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar')
        self.declare_parameter('device', value='/dev/ttyLIDAR')

        device = self.get_parameter('device').get_parameter_value().string_value
        self.lidar = MysteryLidar(device)

        topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(LaserScan, topic_name, 10)

        self.thread = threading.Thread(target=self.node_task())
        self.thread.start()

    def node_task(self):
        self.lidar.open()
        while rclpy.ok():
            frame = self.lidar.poll()
            if frame:
                self.publish_scan(frame)
        self.lidar.close()

    def publish_scan(self, frame: bytes):
        msg = LaserScan()
        # TODO decode frame into msg
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
