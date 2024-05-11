import os
import sys
import time
import math

from gpiozero import DistanceSensor

import numpy as np

from typing import Type, Union, List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Range

from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__("ultrasonic_node")

        echo_pin = self.declare_parameter("echo_pin", 17).value
        trigger_pin = self.declare_parameter("trigger_pin", 4).value
        self.field_of_view = self.declare_parameter("field_of_view", 15.0).value
        self.min_range = self.declare_parameter("min_range", 0.02).value
        self.max_range = self.declare_parameter("max_range", 7.0).value

        self.ultrasonic = DistanceSensor(echo=echo_pin, trigger=trigger_pin)

        self.range_pub = self.create_publisher(Range, "range", 10)

        hz = self.declare_parameter("hz", 30).value
        self.timer_period = 1.0 / hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):

        range_msg = Range()
        range_msg.header.frame_id = "ultrasonic_link"
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.field_of_view
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        range_msg.range = self.ultrasonic.distance

        self.range_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info(
            "Starting " + node.get_name() + ", shut down with CTRL-C"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
