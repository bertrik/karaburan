#!/usr/bin/env python3
# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>
import math
import threading
import time
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

    def __add_data(self, b):
        self.data[self.index] = b
        self.index += 1

    def process(self, b) -> bytes | None:
        """ processes one byte, returns True if a full frame was received """
        match self.state:
            case self.State.HEADER_1:
                self.index = 0
                if b == 0x55:
                    self.__add_data(b)
                    self.state = self.State.HEADER_2

            case self.State.HEADER_2:
                if b == 0xAA:
                    self.__add_data(b)
                    self.length = len(self.data)
                    self.state = self.State.COLLECT
                else:
                    self.state = self.State.HEADER_1
                    self.process(b)

            case self.State.COLLECT:
                if self.index < self.length:
                    self.__add_data(b)
                if self.index == self.length:
                    # done
                    self.state = self.State.HEADER_1
                    return self.data
        return None


class MysteryLidar:
    RAYS_PER_ROTATION = 576  # 16 rays per 10 degree packet

    def __init__(self, device):
        self.serial = serial.Serial(device, baudrate=230400, timeout=None)
        self.extractor = FrameExtractor()

    def open(self) -> None:
        if not self.serial.is_open:
            self.serial.open()

    def poll(self) -> bytes | None:
        poll_result = None
        num = self.serial.in_waiting
        if num == 0:
            # chill a bit and let bytes accumulate in the serial buffer
            time.sleep(0.001)
        else:
            data = self.serial.read(num)
            for b in data:
                result = self.extractor.process(b)
                if result:
                    poll_result = result
        return poll_result

    def close(self) -> None:
        self.serial.close()


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar')
        self.declare_parameter('device', value='/dev/ttyUSB0')
        self.declare_parameter('topic', value='/scan')
        self.declare_parameter('frame_id', value='laser')

        device = self.get_parameter('device').get_parameter_value().string_value
        self.lidar = MysteryLidar(device)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(LaserScan, topic, 10)

        # initialise LaserScan message
        self.msg = LaserScan()
        self.msg.header.frame_id = self.frame_id
        self.msg.range_min = 0.02  # radius of scanner head is 2 cm
        self.msg.range_max = 10.0  # assumed
        self.msg.scan_time = 1.0 / 5.8  # time between full 360 sweep: approx 6 rotations per second
        self.msg.intensities = [0.0] * MysteryLidar.RAYS_PER_ROTATION
        self.msg.ranges = [0.0] * MysteryLidar.RAYS_PER_ROTATION
        self.msg.time_increment = self.msg.scan_time / MysteryLidar.RAYS_PER_ROTATION  # time between rays
        self.msg.angle_min = math.radians(0.0)
        self.msg.angle_max = math.radians(360.0)
        self.msg.angle_increment = math.radians(360.0 / MysteryLidar.RAYS_PER_ROTATION)

        # create and start listen thread
        self.thread = threading.Thread(target=self.node_task())
        self.thread.start()

    def node_task(self) -> None:
        self.lidar.open()
        while rclpy.ok():
            frame = self.lidar.poll()
            if frame:
                self.process_frame(frame)
        self.lidar.close()

    def process_frame(self, frame: bytes) -> None:
        now = self.get_clock().now()

        fsa = (frame[6] + (frame[7] << 8)) / 64.0 - 640
        lsa = (frame[56] + (frame[57] << 8)) / 64.0 - 640
        if lsa < fsa:
            lsa += 360.0
        step = (lsa - fsa) / 16

        idx = 8
        angle = fsa
        for i in range(16):
            dist = (frame[idx] + (frame[idx + 1] << 8)) & 0x3FFF
            intensity = frame[idx + 2]
            idx += 3

            ray = int(angle * MysteryLidar.RAYS_PER_ROTATION / 360)
            if ray < MysteryLidar.RAYS_PER_ROTATION:
                self.msg.ranges[-ray] = dist / 1000.0
                self.msg.intensities[-ray] = float(intensity)

            angle += step
            if angle >= 360.0:
                angle -= 360.0

                # publish current message
                self.publisher.publish(self.msg)

                # initialise next message
                self.msg.header.stamp = now.to_msg()


def main(args=None):
    # Set up a transformer using:
    #   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser
    # Visualize using rviz2, add a 'LaserScan' display and choose topic '/scan' and style 'Points'
    #
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
