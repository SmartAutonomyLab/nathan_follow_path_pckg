#!/usr/bin/env python3

import time
import rclpy
import math
import numpy as np
from rclpy.node import Node
from tbot_EKF_node import tbot_EKF

def follow_path_2():
    # Preps the ROS2 Python Node
    rclpy.init(args=None)
    # circular
    path_list = np.load('circle_points.npy', allow_pickle=True)

    # creates instance of path_P class
    turtle_car = tbot_EKF(path_list)
    # option 1
    rclpy.spin(turtle_car)
    # ROS node cleanup tasks
    turtle_car.destroy_node()
    rclpy.shutdown()

    # Prints "PARKED" to user to indicate end of algorithm.
    turtle_car.get_logger().info('PARKED')
    # for i in range(10):
    #     print(f"Child Script 1: Step {i+1}")
    #     time.sleep(1)  # Simulate work by sleeping
    # for i in range(10):
    #     print(f"Child Script 2: Step {i+1}")
    #     time.sleep(1)  # Simulate work by sleeping

if __name__ == "__main__":
    follow_path_2()
