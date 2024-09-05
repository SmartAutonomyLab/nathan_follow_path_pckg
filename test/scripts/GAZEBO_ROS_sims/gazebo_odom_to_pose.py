#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Gazebo_Odom_To_Pose(Node):

    def __init__(self):
        super().__init__("gazebo_odom_to_pose_node")
        self.publisher_pose = self.create_publisher(PoseStamped, "/pose_stamped", 1)
        self.subscriber_odometry = self.create_subscription(Odometry, "/odom", self.callback_odometry, 1)
            
    def callback_odometry(self, msg:Odometry):
        # linear_speed = msg.twist.twist.linear
        # self.get_logger().info(f'Linear speed: x={linear_speed.x:.2f} m/s, y={linear_speed.y:.2f} m/s, z={linear_speed.z:.2f} m/s')
        
        msg_odometry = msg
        msg_posewithcovariance = msg_odometry.pose
        # print('inside odom_to_pose')
        msg_posestamped = PoseStamped()
        msg_posestamped.header = msg_odometry.header
        msg_posestamped.pose = msg_posewithcovariance.pose

        self.publisher_pose.publish(msg_posestamped)


def main(args=None):
    rclpy.init(args=args)

    node = Gazebo_Odom_To_Pose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.get_logger().info('Gracefully destroying node...')
    
    # ROS node cleanup tasks
    node.destroy_node()
    rclpy.shutdown()


# Called when script is run.
if __name__ == "__main__":
    # Calls an instance of the main function (shown directly above).
    main()
