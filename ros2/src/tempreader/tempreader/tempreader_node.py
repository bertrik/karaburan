#!/usr/bin/env python3

# Copyright 2024 Bertrik Sikken <bertrik@sikken.nl>

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from typing import Optional

SENSOR_TYPE = 'temperature'

def read_temperature(sensor_id: str) -> Optional[float]:
    sensor_file = f"/mnt/1wire/uncached/{sensor_id}/temperature"
    try:
        with open(sensor_file, 'r', encoding="UTF-8") as file:
            temperature = file.read().strip()
            return float(temperature)
    except FileNotFoundError:
#        print(f"Sensor {sensor_id} not found.")
        return None

class Tempreader(Node):
    def __init__(self):
        super().__init__("tempreader")
        self.declare_parameter("sensorId", value="28.7AAB46D42000")
        self.declare_parameter("topic", value="temperature")

        topic_name = self.get_parameter("topic").get_parameter_value().string_value


        self.publisher_ = self.create_publisher(String, topic_name, 10)
        timer_period = 5 # Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        sensor_id = self.get_parameter("sensorId").get_parameter_value().string_value
        msg.data = f"Temperature read {read_temperature(sensor_id)}"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    tempreaderNode = Tempreader()

    rclpy.spin(tempreaderNode)

    tempreaderNode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

