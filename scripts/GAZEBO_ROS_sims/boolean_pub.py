#!/usr/bin/env python3

import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from tbot_EKF_node import tbot_EKF
from std_msgs.msg import Bool
from statistics import random 

class BooleanPublisher(Node):
    def __init__(self, frequency=10):
        super().__init__('boolean_publisher')
        self.publish_bool = self.create_publisher(Bool, '/bool_indicator', 1)
        timer_period = frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = bool( np.random.choice( [True, False] ) )# or False, depending on your requirement
        self.publish_bool.publish(msg)
        self.get_logger().info(f'Publishing jump indicator as: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    boolean_publisher = BooleanPublisher()

    try:
        rclpy.spin(boolean_publisher)
    except KeyboardInterrupt:
        pass

    boolean_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

