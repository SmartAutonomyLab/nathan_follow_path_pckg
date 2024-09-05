#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
# from rclpy.time import Time
import math
import numpy as np 
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, String
from tf_transformations import euler_from_quaternion
from datetime import datetime 
from EKF_basic import EKF_Discrete

class tbot_EKF(Node):
    """
    Contains node to force turtlebot on desired trajectory 
    with P controller.
    """
    def __init__(self, path_list):
        """Attributes for the tbot EKF class; including sub, pub"""
        # Inherits the ROS2 Node Class Functions 
        super().__init__("turtle_car")

        self.received_mocap = False # indicates if we have received mocap/gazebo data
        self.parked         = False # indicates if we have reached final target point
        self.first_spin     = True  # indicates if we have yet to complete a first spin of pose error calc

        self.parked_temp = False
        '''
        parked variable indicating bot's arrival at intermediary coordinates
        in goal_path initialized to false
        '''
        self.goal_path = path_list
        '''
        list where each item is a np array containing x and y coordinates of desired path
        OR a nx2 numpy array with n x,y coorindates 
        '''
        self.current_goal = path_list[0] # initialize current goal as first point in path
        self.current_index = 0 # current path index, increases as bot reaches next point
        
        #initialize pose values
        self.current_pose = PoseStamped()
        self.goal_yaw  = None 
        self.current_yaw = None
        # Creates the command in the data type of Twist for Turtlebot hardware directions
        self.command = Twist()

        # Holds the error between the current pose & goal pose readings
        self.pose_error     = np.zeros( (3,1) )
        '''
        [x_error (m), y_error (m), yaw_error (deg)]
        '''
        self.yaw_error      = None
        self.I_yaw_error    = float( 0 ) # integral error
        self.dedt           = float( 0 ) # error time derivative
        self.prev_yaw_error = None # yaw error at previous time step
        self.init_ind       = True # indicator describing if new goal coordinate has just been obtained
        
        # CONTROL PARAMETERS 
        self.v_reg = False # Indicates if we are capable of linear speed control
        # Gains for well behaved feedback
        kP_angular = 3.
        kI_angular = 0.
        kD_angular = 3e-1
        self.desired_lin_speed = 0.20 # desired linear speed (m/s)
        # Gains for "jumpy" feedback
        kP_angular_jumpy = 1.
        kI_angular_jumpy = 0.
        kD_angular_jumpy = 1e-2
        self.desired_lin_speed_jumpy = 0.10 # desired linear speed (m/s)

        self.k_angular = np.array([kP_angular, kI_angular, kD_angular])
        self.k_angular_jumpy = np.array([kP_angular_jumpy, kI_angular_jumpy, kD_angular_jumpy])
        # threshhold parameters to determine when to switch to controllers
        dist_jump_threshhold = 0.2
        yaw_jump_threshhold  = 0.2
        
        self.jump_threshhold = np.array([ 
                                        [dist_jump_threshhold], 
                                        [yaw_jump_threshhold],
                                        ])
        # time and frequency paramters
        self.dt_act     = 0.1 # time between control inputs
        self.dt_meas    = 1.0 # time between state measurement/observation

        # approximate covariances
        self.Q  = 1e-3*np.eye(3) * self.dt_act # process covariance scaled by continuous time discretization
        self.R_exp  = 1e-4*np.eye(3) # expected measurement covariance (used in Kalman gain update step)
        self.R_jumpy  = 5e-3*np.eye(3) # measurement covariance
        self.P0 = 1e-3*np.eye(3) # covariance of initial state estimate 
        self.jump_indicator = False # indicates if we are purposefully forcing our estimates to be jumpy
        
        '''
          Create a class (self.) velocity publisher variable to send data to the Turtlebot.
          Use the self.create_publisher() function and publish a message of the Twist
          data type, to the "/cmd_vel" topic that already exists, and a queue size of 1.
        '''
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
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

        # data publishers to data_collector node for plotting
        self.x_true_publisher = self.create_publisher(Float32MultiArray, 'x_true_topic', 10)
        self.x_est_publisher  = self.create_publisher(Float32MultiArray, 'x_est_topic', 10)
        self.P_publisher      = self.create_publisher(Float32MultiArray, 'P_topic', 10)
        self.u_publisher      = self.create_publisher(Float32MultiArray, 'u_topic', 10)

        # create publication for command to collect data 
        # if tbot reaches final destination
        self.save_data_publisher = self.create_publisher(String, 'save_data_topic', 1)

        # create timers for measurement and control functions
        self.control_timer = self.create_timer(self.dt_act, self.follow_path)
        self.meas_timer    = self.create_timer(self.dt_meas, self.meas_ind_callback)

        # avoid the unused variable warning.
        self.mocap_subscription

        # subscription for boolean jumpy indicator outside node that determines 
        # if noise is added.
        self.jump_subscription = self.create_subscription(Bool, '/bool_indicator', self.bool_callback, 1)
        
    def meas_ind_callback(self): 
        # indicates that a measurement/update step is in order if 
        # we are currently receiving mocap data 
        if self.received_mocap: # only measure if received mocap feedback 
            if not self.first_spin: # do not "measure" initial position
                self.get_logger().info(f'Received measurement update')
                if self.jump_indicator:
                    # jumpy sensor so add higher noise
                    meas_noise = np.random.multivariate_normal(np.zeros( 3 ), self.R_jumpy, 1).T
                else:
                    # good sensor so add expected measurement noise 
                    meas_noise = np.random.multivariate_normal(np.zeros( 3 ), self.R_exp, 1).T
                self.z = self.pose_mocap_array + meas_noise
                self.x_newu, self.P_newu = self.tbot_EKF_obj.update(self.z)
        else:
            self.get_logger().info('waiting for mocap data to record a measurement')

    def mocap_callback(self, msg):
        """
        Updates current position information from the motion capture cameras
        to be accessed at prescribed frequency
        """
        self.current_pose = msg 
        mocap_position = self.current_pose.pose.position
        #Q2E CALC
        ornt_quat = self.current_pose.pose.orientation
        ornt_quat_list = [ornt_quat.x, ornt_quat.y, ornt_quat.z, ornt_quat.w]
        [_, _, mocap_yaw] = euler_from_quaternion(ornt_quat_list)
        
        self.pose_mocap_array = np.array([ [mocap_position.x], \
                                            [mocap_position.y], \
                                            [mocap_yaw] ] )    
        # publish "true" position
        pose_floatmulti = np2float32multiarray(self.pose_mocap_array)
        self.x_true_publisher.publish( pose_floatmulti )    
        self.received_mocap = True
        # self.get_logger().info(f'Current Pose inside mocap ( {self.current_pose.pose.position.x, self.current_pose.pose.position.y} ) ')

    def bool_callback(self, msg):
        self.jump_indicator = msg.data
        self.get_logger().info(f'received jump indicator is ({msg.data})')
    
    def follow_path(self):
        """
        Function that runs the P controller algorithm until 
        robot reaches end of path.
        """
        if not self.parked:
            if not self.parked_temp:
                if self.received_mocap:
                # the mocap call_back flag has been called
                    self.pose_error_calc()
                    self.error2control()
                    self.publish_velocity()
                else:
                    self.get_logger().info('control commands are waiting for mocap data')
            else:
                self.update_tasks()
        else:
            # bot reached final destination so perform ROS node cleanup tasks
            self.shutdown_tasks()

    def pose_error_calc(self):
        """
        calculates the difference between the current position and goal position
        """
        if self.first_spin: #read mocap to obtain initial position and initialize EKF object
            self.tbot_EKF_obj = EKF_Discrete(self.pose_mocap_array, self.P0, \
                                            self.f_turtle, self.F_turtle, \
                                            self.h_turtle, self.H_turtle, \
                                            self.Q, self.R_exp)
            self.first_spin = False
        
        if not self.init_ind: #used for dedt calcuation
            self.prev_yaw_error = self.pose_error[2].item()

        self.pose_best = self.tbot_EKF_obj.x # current best estimate for x
        goal_coord = self.current_goal
        
        # Calc x & y differences between pose reading & Goal position.
        x_difference = float( goal_coord[0].item() - self.pose_best[0].item() )
        y_difference = float( goal_coord[1].item() - self.pose_best[1].item() )
        
        # obtain current yaw and goal yaw with atan2
        current_yaw   = self.pose_best[2]
        self.goal_yaw = math.atan2(y_difference, x_difference)
        
        # Calculate the yaw error as the difference between the relative and the current yaws.
        # and Convert yaw error into degrees (from radians) 
        yaw_error     = self.goal_yaw - current_yaw 

        # Correct the yaw error to be in range -180 to 180 (prevents spinning)
        if yaw_error > np.pi: #above 180
            yaw_error -= 2*np.pi

        if yaw_error < -np.pi: #below -180
            yaw_error += 2*np.pi

        # self.get_logger().info(f'goal coard ({goal_coord[0]}, {goal_coord[1]})')
        # self.get_logger().info(f'path index is {self.current_index}')
        # self.get_logger().info(f'yaw error (Deg) ({yaw_error*180/np.pi})')
        
        # update pose error 
        self.pose_error[0] = x_difference 
        self.pose_error[1] = y_difference 
        self.pose_error[2] = yaw_error

        # publish current pose estimate and covariance to data collector
        x_est_floatmulti = np2float32multiarray(self.pose_best)
        P_floatmulti     = np2float32multiarray(self.tbot_EKF_obj.P)
        
        self.x_est_publisher.publish(x_est_floatmulti)
        self.P_publisher.publish(P_floatmulti)

    def error2control(self):
        '''
        Maps position error and orientation error to linear speed and 
        angular velocity commands
        '''
        yaw_error = self.pose_error[2].item()

        # update dedt and I_e
        self.I_yaw_error += yaw_error * self.dt_act
        if not self.init_ind:
            self.dedt = ( (yaw_error - self.prev_yaw_error )/self.dt_act )
            # print(f'dedt type is {type(self.dedt)}')            
        
        euc_jump = self.tbot_EKF_obj.delta[0,0].item()
        yaw_jump = self.tbot_EKF_obj.delta[1,0].item()

        euc_threshhold = self.jump_threshhold[0,0]
        yaw_threshhold = self.jump_threshhold[1,0]

        if euc_jump > euc_threshhold or abs(yaw_jump) > yaw_threshhold:
            # PID Angular velocity control
            # BIG JUMPS SO CHOOSE SMALL GAINS 
            # AND DECREASED LINEAR SPEED
            self.get_logger().info(f'Euclidian jump ({euc_jump})')
            self.get_logger().info(f'Yaw jump    ({yaw_jump})')
            print('jumpy')
            P_gain = self.k_angular_jumpy[0].item() * yaw_error
            I_gain = self.k_angular_jumpy[1].item() * self.I_yaw_error
            D_gain = self.k_angular_jumpy[2].item() * self.dedt   
            desired_lin_speed = self.desired_lin_speed_jumpy         
        else:
            # SMALL MEASUREMENTS SO CHOOSE NORMAL GAINS 
            # AND FASTER SPEEDS
            P_gain = self.k_angular[0].item() * yaw_error
            I_gain = self.k_angular[1].item() * self.I_yaw_error
            D_gain = self.k_angular[2].item() * self.dedt
            desired_lin_speed = self.desired_lin_speed   
        angular_speed = float(P_gain + I_gain + D_gain)
        #check for saturation
        if angular_speed >= 2.84:
            angular_speed = 2.84
        if angular_speed <= -2.84:
            angular_speed = -2.84

        self.command.angular.z = angular_speed
        # linear speed control
        if self.v_reg:
            if abs( yaw_error ) >= np.pi/4:
                # do not move linearly if yaw_error > 90 deg
                act_linear = float( 0 )
            else: 
                # input linear speed scales linearly with yaw_error
                act_linear = float(desired_lin_speed/(np.pi/4) * (np.pi/4- abs( yaw_error ) ) )
        else:
            act_linear = desired_lin_speed

        # act_list = np.array([ [P_gain], \
        #                     [I_gain], \
        #                     [D_gain], \
        #                     [act_linear] ] ) 

        # self.get_logger().info(f'P actuation term {P_gain}')
        # self.get_logger().info(f'I actuation term {I_gain}')
        # self.get_logger().info(f'D actuation term {D_gain}')
        # self.get_logger().info(f'l linear speed term {act_linear}')

        # self.act_list.append( act_list )
        self.command.linear.x = act_linear

        # publish control command for data collection
        u = np.array([
            [ angular_speed], 
            [ act_linear ]
            ])
        u_floatmulti = np2float32multiarray( u )
        self.u_publisher.publish( u_floatmulti )

    def publish_velocity(self):
        """
        1. Publishes the linear and angular velocity commands to the hardware 
        2. Logs debug statements
        3. Propogates pose estimate based on transition functions
        4. Checks to see if turtlebot is within tolerance of the current goal point 
        5. Updates initialization variable
        """
        
        self.velocity_publisher.publish( self.command )

        if self.init_ind:
            self.init_ind = False

        self.control_array = np.array( [ [self.command.linear.x], \
                                   [self.command.angular.z] ])
        # propogation step
        self.x_newp, self.P_newp = self.tbot_EKF_obj.propagate(uk=self.control_array)
        # 
        # debugging logger lines
        # self.get_logger().info(f'yaw_error_deg {self.yaw_error_deg}')
        # self.get_logger().info(f'distance {self.distance}')
        # self.get_logger().info(f'current goal {self.current_goal}')
        # self.get_logger().info(f'current goal yaw {self.goal_yaw*180/math.pi}')
        # self.get_logger().info(f'Desired radial speed {self.command.linear.x}')
        
        #   Checks if in proximity to target by checking self.distance.

        self.distance = math.sqrt(self.pose_error[0].item() ** 2 \
                                  + self.pose_error[1].item() ** 2)

        if  self.distance < 0.1:
            self.parked_temp = True
            self.get_logger().info(f'Reached intermediate point')
            if np.array_equal(self.current_goal, self.goal_path[-1]):
                # reached final goal point in path
                self.parked = True
                self.get_logger().info(f'reached final coordinate')
    
    def f_turtle(self, x, u):
        '''
        x_ps - state in posestamped format
        u_twist - control in twist format

        x - 3x1 numpy array with structure [x, y, yaw] x,y in m and yaw in deg
        u - 2x1 numpy array with structure [speed, yaw_rate] speed in m/s and yaw in rad/s
        
        x_newp = f(xk, uk) where f is state transition function for tbot.
        '''

        # x = np.array( [ [x_ps.pose.position.x], \
        #                 [x_ps.pose.position.y], \
        #                 [self.current_yaw] ] )
        
        # u = np.array( [ [u_twist.linear.x], \
        #                 [u_twist.angular.z] ])
        
        dxdt = np.zeros([3,1])
        dxdt[0] = u[0].item() * np.cos( x[2].item() )
        dxdt[1] = u[0].item() * np.sin( x[2].item() )
        dxdt[2] = u[1].item() 
        return x + self.dt_act * dxdt

    def F_turtle(self, x, u ):
        d_dxdt_dx = np.zeros([3,3])
        d_dxdt_dx[0,2] = -u[0].item() * np.sin( x[2].item() )
        d_dxdt_dx[1,2] =  u[0].item() * np.cos( x[2].item() )
        return np.eye(3) + self.dt_act * d_dxdt_dx

    def h_turtle(self, x ):
        return x 

    def H_turtle(self, x ):
        return np.eye(3)

    def shutdown_tasks( self ):
        # bot reached final destination so perform ROS node cleanup tasks
        self.command.angular.z = 0.
        self.command.linear.x = 0.
        self.get_logger().info(f'current target is {self.current_goal}')
        self.velocity_publisher.publish( self.command )
        msg = String()
        msg.data = 'save'
        self.save_data_publisher.publish(msg=msg)
        sys.exit(0)
        self.destroy_node()
        rclpy.shutdown()   

    def update_tasks( self ):
        # update current goal, change to new goal, 
        # reset integral yaw error to 0
        self.current_index += 1
        self.current_goal  = self.goal_path[ self.current_index ]
        self.parked_temp   = False   
        self.I_yaw_error   = 0 # initialize to 0 as goal coordinate switches
        self.dedt          = 0 # initialize to 0 as goal coordinate switches
        self.init_ind      = True # new path initialization boolean reset to true
        self.get_logger().info(f'current target is {self.current_goal}')

def np2float32multiarray(numpy_array):
    '''Maps numpy arrays --> the ros2 type np2float32multiarray'''
    float32_array = Float32MultiArray()
    float32_array.data = numpy_array.flatten().tolist()
    return float32_array

def main(args=None):
    # Preps the ROS2 Python Node
    rclpy.init(args=args)

    # # STRAIGHT LINE
    # # Define the range for y-coordinates
    # y_start = -3.
    # y_end = 3.
    # num_points = 5  # Number of points you want to generate

    # # Generate linearly spaced y-coordinates
    # y_coords = np.linspace(y_start, y_end, num_points)

    # # Create the list of ordered pairs
    # path_list = [np.array([1, y]) for y in y_coords]


    # # circular
    # path_list = np.load('circle_points.npy', allow_pickle=True)
    
    # FIGURE 8
    path_list = np.load('figure_eight_points.npy', allow_pickle=True)
    path_list = path_list.T

    # creates instance of tbot_EKF class
    turtle_car = tbot_EKF(path_list)
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
