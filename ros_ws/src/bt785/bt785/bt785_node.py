#!/usr/bin/env python3
# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>
import threading
import time

import rclpy
from karaburan_msgs.msg import ElectricalConductivity
from rclpy.node import Node


class BT785Node(Node):
    def __init__(self):
        super().__init__('bt785')
        self.declare_parameter('device', value='D3:01:01:02:2F:C6')
        self.declare_parameter('topic', value='/bt785')
        self.declare_parameter('frame_id', value='bt785')

        self.device = self.get_parameter('device').get_parameter_value().string_value
        # self.sonar = Sonar(device)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(ElectricalConductivity, topic, 10)

        # initialise Range message
        self.msg = ElectricalConductivity()
        self.msg.header.frame_id = self.frame_id

        # create and start listen thread
        self.thread = threading.Thread(target=self.node_task())
        self.thread.start()

    def node_task(self) -> None:
        while rclpy.ok():
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.temperature = 25.0
            self.msg.conductivity = 50.0
            self.publisher.publish(self.msg)
            time.sleep(10)

    def publish_ec(self, sd: ElectricalConductivity) -> None:
        now = self.get_clock().now()

        # publish message
        self.msg.header.stamp = now.to_msg()
        self.msg.temperature = 25.0
        self.msg.conductivity = 5.0
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    bt785_node = BT785Node()
    rclpy.spin(bt785_node)
    bt785_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

