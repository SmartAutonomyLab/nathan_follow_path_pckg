#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from rclpy.time import Time
import math
import numpy as np 
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion


class path_P(Node):
    """
    Contains node to force turtlebot on desired trajectory 
    with P controller.
    """
    def __init__(self):
        """Attributes for the CarPark class; including sub, pub"""
        # Inherits the ROS2 Node Class Functions 
        super().__init__("turtle_car")
        
        # TODO:
        #   Initialize the following class (self.) variables.
        #   A "parked" variable to "False"
        #   Current pose reading variable to a PoseStamped() data type
        #   The current_yaw_euler to "None"
        #   The current_yaw to "None"

        self.parked = False
        '''
        parked variable indicating bot's arrival at final coordinate in goal_path 
        initialized to false
        '''

        self.parked_temp = False
        '''
        parked variable indicating bot's arrival at intermediary coordinates
        in goal_path 
        initialized to false
        '''
        # create path 
        # Define the range for y-coordinates
        y_start = 0
        y_end = 6
        num_points = 10  # Number of points you want to generate

        # Generate linearly spaced y-coordinates
        y_coords = np.linspace(y_start, y_end, num_points)

        # Create the list of ordered pairs
        path_list = [np.array([1, y]) for y in y_coords]
        self.goal_path = path_list
        '''
        list where each item is a np array containing x and y coordinates of desired path
        '''
        self.current_goal = path_list[0]
        self.current_index = 0
        self.goal_yaw  = None 
        self.cur_yaw_euler = None

        # kP constant values.
        self.kP_angular = 0.01
        self.kP_linear = 0.2

        # max linear control actuation 
        # (actuation maxes roughly around 2 meters away)
        self.max_linear_act = 0.4
        # Holds the error between the current pose & goal pose readings
        #   Create two class (self.) variables that are for the pose error and the yaw error.
        #   Set both error values equal to "None".
        self.pose_error = None
        self.yaw_error  = None
        

        # Creates the command in the data type of Twist for Turtlebot hardware directions
        self.command = Twist()

        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        '''
          Create a class (self.) velocity publisher variable to send data to the Turtlebot.
          Use the self.create_publisher() function and publish a message of the Twist
          data type, to the "/cmd_vel" topic that already exists, and a queue size of 1.
        '''

        self.mocap_subscription = self.create_subscription(PoseStamped, "/pose_stamped", self.mocap_callback, 1)
        '''
          Create a class subscriber variable to receive position data from MoCap.
          Use the self.create_subscription() function and sunscribe a PoseStamped message,
          to the "/pose_stamped" topic, using the mocap callback function in this class, 
          followed by the the integer 1 (queue size).
        '''      

        # create timer for spin function
        # self.function_timer = self.create_timer(0.05, self.follow_path)
        '''function to publish velocity commands with every spin'''

        # Copy the name of your subscriber veraible below to avoid the unused variable warning.
        self.mocap_subscription
        
            
    def mocap_callback(self, msg):
        """Gets current position information from the motion capture cameras."""
        # Sets received pose data from turtlebot, and sets to current position
        self.current_pose = msg 

        self.get_logger().info(f'Current Pose inside mocap {self.cur_yaw_euler}( {self.current_pose.pose.position.x, self.current_pose.pose.position.y} ) ')
        # Uncomment the next line when debugging
        # self.get_logger().info(f'MOCAP DATA    \n{msg}\n')
        
    def follow_path(self):
        """
        Function that runs the P controller algorithm until 
        robot reaches end of path.
        """
        if not self.parked:
            if not self.parked_temp:
                self.position_error_calc()
                self.orientation_error_calc()
                self.publish_velocity()  
            else:
                #update current goal and change to new goal 
                self.current_index += 1
                self.current_goal  = self.goal_path[ self.current_index ]
                self.parked_temp   = False   
        else:
            # ROS node cleanup tasks
            self.destroy_node()
            rclpy.shutdown()   

    def position_error_calc(self):
        """Publishes the difference between the current position and goal position"""
        # Get current pose reading's x & y variables.
        current_pose = self.current_pose.pose.position
        goal_coord = self.current_goal
        
        # Calc x & y differences between pose reading & Goal position.
        x_difference = goal_coord[0] - current_pose.x
        y_difference = goal_coord[1] - current_pose.y
        self.get_logger().info(f'goal direction ({x_difference}, {y_difference})')

        # Initialize pose_error attribute to PoseStamped data type
        self.pose_error = PoseStamped()
        
        #   Set the x and y values of the self.pose_error (using variable.pose.position.x syntax)
        #   equal to the differences calculated in the previous task.
        self.pose_error.pose.position.x = x_difference
        self.pose_error.pose.position.y = y_difference

    def orientation_error_calc(self):
        """
        Determines error between bot's current heading and desired heading.
        """
        # Gets current turtlebot_position
        data = self.current_pose
        
        # Sets y_difference and x_difference values for relative_yaw calculation
        x_difference, y_difference = self.pose_error.pose.position.x, self.pose_error.pose.position.y

        # Converts the pose data from quaternion (sensory input) into euler (yaw angular)
        ##### DO NOT CHANGE THE FOLLOW LINES OF CODE.
        ornt_quat = data.pose.orientation
        ornt_quat_list = [ornt_quat.x, ornt_quat.y, ornt_quat.z, ornt_quat.w]
        [roll, pitch, yaw] = euler_from_quaternion(ornt_quat_list)
        self.cur_euler = [roll, pitch, yaw]
        ##### END OF EULER ANGLE CALCULATIONS

        # Calculate the yaw error (amount to rotate)
        # TODO:
        #   The class' current_yaw variable should be set to the third item in the cur_euler array.
        self.current_yaw = self.cur_euler[2]
        # TODO:
        #   Use the math.atan2() function with the y difference and x difference to calculate the relative yaw.
        self.goal_yaw = math.atan2(y_difference, x_difference)
        
        # TODO:
        #   Calculate the yaw error as the difference between the relative and the current yaws.
        self.yaw_error = self.goal_yaw - self.current_yaw

    def publish_velocity(self):
        """Publishes the linear and angular velocity commands to the hardware."""
        # Convert yaw error into degrees (from radians)
        yaw_error_deg = self.yaw_error * (180/math.pi)

        # TODO:
        #   Currently your yaw error is from 0 to 360, but it instead needs to range
        #   from -180 to 180.  Correct the yaw error to be in range -180 to 180 (prevents spinning)
        #   you will need two separate if statements for when the value is above 180 and below -180.
        
        if yaw_error_deg > 180: #above 180
            yaw_error_deg = yaw_error_deg - 360

        if yaw_error_deg < -180: #below -180
            yaw_error_deg = yaw_error_deg + 360

        # TODO:
        #   Calculate & initiate spin of bot towards goal
        #   set command.angular.z equal to your instance's kp value multiplied by the yaw_error_deg.
        self.command.angular.z = self.kP_angular * yaw_error_deg

        # Calculate and initiate forward movement of the robot.
        car = self.pose_error.pose.position

        # TODO:
        #   The linear.x value (of command) can be calculated as the kp * the hypotenuse 
        #   of the x and y error distances.  Use math.sqrt() for this equation.
        #   To access the x and y values, use the syntax: car.x and .y
        #   For squaring a value, use: ** 2 to indicate the 2 is an exponent.
        #   original control actuation
        distance = math.sqrt(car.x ** 2 + car.y ** 2)
        control_act_p = self.kP_linear * distance
        
        # if statement to limit linear actuation commands sent to turtle bot
        if control_act_p < self.max_linear_act:
            #control input is undersaturated (no change is made)
            self.command.linear.x = control_act_p
        else:
            #control input will be oversaturated so change it to max
            self.command.linear.x = self.max_linear_act 
        
        # TODO
        #   Publish the current command value to the bot using the velocity_publisher.publish command.
        self.velocity_publisher.publish( self.command )

        #debugging logger lines
        self.get_logger().info(f'yaw_error_deg {yaw_error_deg}')
        self.get_logger().info(f'distance {distance}')
        self.get_logger().info(f'current goal {self.current_goal}')
        self.get_logger().info(f'Current Pose (yaw, x, y) ( {self.cur_yaw_euler},  {self.current_pose.pose.position.x, self.current_pose.pose.position.y} ) ')
        # TODO
        #   Checks if in proximity to target by comparing to the most recent linear.x and linear.y values.
        #   If the boundaries are met, change the class' parked variable to True.
        if  distance < 0.1:
            self.parked_temp = True
            self.get_logger().info(f'test')
            if np.array_equal(self.current_goal, self.goal_path[-1]):
                self.parked = True
                self.get_logger().info(f'reached final coordinate')


def main(args=None):
    # Preps the ROS2 Python Node
    rclpy.init(args=args)

    # creates instance of path_P class
    turtle_car = path_P()
    # option 1
    # rclpy.spin(turtle_car)
    # # ROS node cleanup tasks
    # turtle_car.destroy_node()
    # rclpy.shutdown()

    # # Prints "PARKED" to user to indicate end of algorithm.
    # turtle_car.get_logger().info('PARKED')

    # option 2
    while not turtle_car.parked:
        # spin instance
        rclpy.spin_once( turtle_car )

        # command car to follow path
        turtle_car.follow_path()

    # When loop ends, indicates robot is in proximity to target. 
    # Prints "PARKED" to user to indicate end of algorithm.
    turtle_car.get_logger().info('PARKED')


# Called when script is run.
if __name__ == "__main__":
    # Calls an instance of the main function (shown directly above).
    main()