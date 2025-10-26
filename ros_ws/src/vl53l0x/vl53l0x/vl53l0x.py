#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Bibliothèque Adafruit pour VL53L0X
import board
import busio
import adafruit_vl53l0x


class VL53L0XPublisher(Node):
    """
    Nœud ROS2 qui publie la mesure du VL53L0X sur un topic sensor_msgs/Range.
    Paramètres ROS :
      - topic (string): nom du topic (défaut: /tof/distance)
      - frame_id (string): frame TF associée (défaut: tof_link)
      - rate_hz (double): fréquence de publication (défaut: 20.0)
      - max_range (double): portée max en mètres (défaut: 2.0)
      - min_range (double): portée min en mètres (défaut: 0.03)
      - fov_deg (double): champ de vue en degrés (défaut: 25.0)
      - i2c_addr (int): adresse I2C du capteur (défaut: 0x29)
    """

    def __init__(self):
        super().__init__('vl53l0x_publisher')

        # Déclaration des paramètres
        self.declare_parameter('topic', '/tof/distance')
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('fov_deg', 25.0)
        self.declare_parameter('i2c_addr', 0x29)

        # Lecture des paramètres
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        fov_deg = self.get_parameter('fov_deg').get_parameter_value().double_value
        i2c_addr = int(self.get_parameter('i2c_addr').get_parameter_value().integer_value)

        # Publisher
        self.pub = self.create_publisher(Range, topic, 10)

        # Init I2C + capteur
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            # La lib Adafruit ne prend pas directement l'adresse en paramètre,
            # mais on peut en changer si besoin via .set_address() quand dispo.
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
            # Si tu utilises plusieurs capteurs, on pourrait reconfigurer l'adresse ici.
            if hasattr(self.vl53, 'set_address') and i2c_addr != 0x29:
                self.vl53.set_address(i2c_addr)
        except Exception as e:
            self.get_logger().fatal(f"Échec init I2C/VL53L0X : {e}")
            raise

        # Préparer message "Range" constant
        self.msg = Range()
        self.msg.radiation_type = Range.INFRARED
        self.msg.field_of_view = math.radians(fov_deg)
        self.msg.min_range = float(self.min_range)
        self.msg.max_range = float(self.max_range)
        self.msg.header.frame_id = self.frame_id

        # Timer pour publier
        period = 1.0 / max(0.1, rate_hz)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(f"VL53L0X prêt. Publication sur '{topic}' à {1.0/period:.1f} Hz.")

    def timer_cb(self):
        try:
            # La lib Adafruit renvoie la distance en millimètres
            dist_mm = self.vl53.range
            # Sanity check
            if dist_mm is None or dist_mm <= 0:
                # Valeur invalide -> on publie +inf pour signaler out-of-range
                distance_m = float('inf')
            else:
                distance_m = dist_mm / 1000.0

            # Bornes min/max
            if distance_m < self.min_range or distance_m > self.max_range:
                # Hors plage utile : publier quand même, certains consumers préfèrent filtrer
                pass

            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.range = float(distance_m)
            self.pub.publish(self.msg)
        except Exception as e:
            # On journalise mais on continue (le capteur peut rater une lecture de temps en temps)
            self.get_logger().warn(f"Lecture VL53L0X échouée : {e}")


def main():
    rclpy.init()
    node = VL53L0XPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

