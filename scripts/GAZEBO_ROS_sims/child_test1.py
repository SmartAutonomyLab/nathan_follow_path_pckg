#!/usr/bin/env python3

import time
import rclpy
import math
import numpy as np
import threading
from rclpy.node import Node
from tbot_EKF_node import tbot_EKF
from boolean_pub import BooleanPublisher
# script that creates and runs node 
# for turtlebot to follow path1
def follow_path_1():
    # Preps the ROS2 Python Node
    rclpy.init(args=None)
    # circular
    # path_list = np.load('circle_points.npy', allow_pickle=True)

    # STRAIGHT LINE
    # Define the range for y-coordinates
    y_start = -3.
    y_end = 3.
    num_points = 5  # Number of points you want to generate

    # Generate linearly spaced y-coordinates
    y_coords = np.linspace(y_start, y_end, num_points)

    # Create the list of ordered pairs
    path_list = [np.array([1, y]) for y in y_coords]

    # creates instance of tbot estimator and controller class
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

if __name__ == "__main__":
    follow_path_1()
