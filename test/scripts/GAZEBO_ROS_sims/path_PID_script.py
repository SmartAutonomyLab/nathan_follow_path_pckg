#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
# from rclpy.time import Time
import math
import numpy as np 
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion


class path_P2(Node):
    """
    Contains node to force turtlebot on desired trajectory 
    with P controller.
    """
    def __init__(self, path_list):
        """Attributes for the CarPark class; including sub, pub"""
        # Inherits the ROS2 Node Class Functions 
        super().__init__("turtle_car")

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

        # STRAIGHT LINE
        # # Define the range for y-coordinates
        # y_start = 0
        # y_end = 6
        # num_points = 10  # Number of points you want to generate

        # # Generate linearly spaced y-coordinates
        # y_coords = np.linspace(y_start, y_end, num_points)

        # # Create the list of ordered pairs
        # path_list = [np.array([1, y]) for y in y_coords]

        # CIRCULAR PATH
        # Load the circle points from the .npy file
        # path_list = np.load('circle_points.npy', allow_pickle=True)
        self.goal_path = path_list
        '''
        list where each item is a np array containing x and y coordinates of desired path
        '''
        self.current_goal = path_list[0]
        self.current_index = 0 #current path index, increases as bot reaches next point
        
        #initialize pose values
        self.current_pose = PoseStamped()
        self.goal_yaw  = None 
        self.current_yaw = None

        # Holds the error between the current pose & goal pose readings
        self.pose_error     = None
        self.yaw_error      = None
        self.I_yaw_error    = 0 # integral error
        self.dedt           = 0 # dedt error
        self.prev_yaw_error = None # yaw error at previous time step
        self.init_ind       = True # indicator describing if new goal coordinate has just been obtained
        
        # Controller paramters
        self.kP_angular = 0.01
        self.kI_angular = 5e-3
        self.kD_angular = 5e-3
        self.kP_linear  = 0.2
        self.dt         = 0.05 # time between "spins"
        # desired linear speed (m/s)
        self.desired_lin_speed = 0.2
        # max linear control actuation (actuation maxes roughly around 2 meters away)
        self.max_linear_act = 0.4
        
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
        
        '''
        USE THIS CODE WHEN OPERATING ON REAL TURTLEBOT 
        OR DESIGNATED LAUNCH FILE 
        THAT CONVERTS THE TOPICS INTO THEIR PROPER NAMES
        '''
        # self.velocity_publisher = self.create_publisher(Twist, "/commands/velocity", 1)
        # self.mocap_subscription = self.create_subscription(PoseStamped, "/supercar/nwu/pose_stamped", self.mocap_callback, 1)


        # create timer for spin function
        self.function_timer = self.create_timer(self.dt, self.follow_path)
        '''function to publish velocity commands with every spin'''

        # avoid the unused variable warning.
        self.mocap_subscription
        
            
    def mocap_callback(self, msg):
        """
        Gets current position information from the motion capture cameras.
        """
        self.current_pose = msg 

        # self.get_logger().info(f'Current Pose inside mocap {self.current_yaw}( {self.current_pose.pose.position.x, self.current_pose.pose.position.y} ) ')
        
    def follow_path(self):
        """
        Function that runs the P controller algorithm until 
        robot reaches end of path.
        """
        if not self.parked:
            if not self.parked_temp:
                self.position_error_calc()
                self.orientation_error_calc()
                self.error2control()
                self.publish_velocity()
            else:
                # update current goal, change to new goal, 
                # reset integral yaw error to 0
                self.current_index += 1
                self.current_goal  = self.goal_path[ self.current_index ]
                self.parked_temp   = False   
                self.I_yaw_error   = 0 # initialize to 0 as goal coordinate switches
                self.dedt          = 0 # initialize to 0 as goal coordinate switches
                self.init_ind      = True
        else:
            # bot reached final destination so perform ROS node cleanup tasks
            sys.exit(0)
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
        # self.get_logger().info(f'goal direction ({x_difference}, {y_difference})')

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

        if not self.init_ind:
            self.prev_yaw_error = self.yaw_error_deg

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

        self.current_yaw = self.cur_euler[2]
        self.goal_yaw = math.atan2(y_difference, x_difference)
        
        # TODO:
        #   Calculate the yaw error as the difference between the relative and the current yaws.
        self.yaw_error = self.goal_yaw - self.current_yaw

    def error2control(self):
        '''
        Maps position error and orientation error to linear speed and 
        angular velocity commands
        '''
        # Convert yaw error into degrees (from radians) 
        yaw_error_deg = self.yaw_error * (180/math.pi)

        # Correct the yaw error to be in range -180 to 180 (prevents spinning)
        if yaw_error_deg > 180: #above 180
            yaw_error_deg = yaw_error_deg - 360

        if yaw_error_deg < -180: #below -180
            yaw_error_deg = yaw_error_deg + 360 

        self.yaw_error_deg = yaw_error_deg

        # update dedt and I_e
        self.I_yaw_error += self.yaw_error_deg*self.dt
        if not self.init_ind:
            self.dedt = (self.yaw_error_deg - self.prev_yaw_error )/self.dt

        # Calculate & initiate spin of bot towards current goal point 
        # as a gain
        self.command.angular.z = self.kP_angular * self.yaw_error_deg + \
            self.kI_angular * self.I_yaw_error + \
                self.kD_angular * self.dedt

        # Calculate and initiate forward movement of the robot.
        position_error = self.pose_error.pose.position 

        self.distance = math.sqrt(position_error.x ** 2 + position_error.y ** 2)
        # control_act_p = self.kP_linear * self.distance

        if yaw_error_deg >= abs(90):
            self.command.linear.x = float( 0 )
        else:
            self.command.linear.x = self.desired_lin_speed/90*\
                (90 - abs( yaw_error_deg ) )

        # # if statement to limit linear actuation commands sent to turtle bot
        # if control_act_p < self.max_linear_act:
        #     #control input is undersaturated (no change is made)
        #     self.command.linear.x = control_act_p
        # else:
        #     #control input will be oversaturated so change it to max
        #     self.command.linear.x = self.max_linear_act 

    def publish_velocity(self):
        """
        Publishes the linear and angular velocity commands to the hardware 
        logs debug statements
        checks to see if turtlebot is within tolerance of the current goal point 
        updates initialization variable
        """
        
        self.velocity_publisher.publish( self.command )
        self.init_ind = False
        #debugging logger lines
        # self.get_logger().info(f'yaw_error_deg {self.yaw_error_deg}')
        # self.get_logger().info(f'distance {self.distance}')
        # self.get_logger().info(f'current goal {self.current_goal}')
        # self.get_logger().info(f'current goal yaw {self.goal_yaw*180/math.pi}')
        # self.get_logger().info(f'Current Pose (yaw, x, y) \
                            #    ( {self.current_yaw*180/math.pi},\
                            #         {self.current_pose.pose.position.x, self.current_pose.pose.position.y} ) ')
        # self.get_logger().info(f'P actuation term {self.kP_angular * self.yaw_error_deg}')
        # self.get_logger().info(f'I actuation term {self.kI_angular * self.I_yaw_error}')
        # self.get_logger().info(f'D actuation term {self.kD_angular * self.dedt}')
        
        #   Checks if in proximity to target by checking self.distance.
        if  self.distance < 0.1:
            self.parked_temp = True
            self.get_logger().info(f'Reached intermediate point')
            if np.array_equal(self.current_goal, self.goal_path[-1]):
                # reached final goal point in path
                self.parked = True
                self.get_logger().info(f'reached final coordinate')


def main(args=None):
    # Preps the ROS2 Python Node
    rclpy.init(args=args)
    # circular
    path_list = np.load('circle_points.npy', allow_pickle=True)

    # creates instance of path_P class
    turtle_car = path_P2(path_list)
    # option 1
    rclpy.spin(turtle_car)
    # ROS node cleanup tasks
    turtle_car.destroy_node()
    rclpy.shutdown()

    # Prints "PARKED" to user to indicate end of algorithm.
    turtle_car.get_logger().info('PARKED')

    # # option 2
    # while not turtle_car.parked:
    #     # spin instance
    #     rclpy.spin_once( turtle_car )

    #     # command car to follow path
    #     turtle_car.follow_path()

    # # When loop ends, indicates robot is in proximity to target. 
    # # Prints "PARKED" to user to indicate end of algorithm.
    # turtle_car.get_logger().info('PARKED')


# Called when script is run.
if __name__ == "__main__":
    # Calls an instance of the main function (shown directly above).
    main()