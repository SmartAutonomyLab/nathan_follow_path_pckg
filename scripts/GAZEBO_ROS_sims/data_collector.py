import rclpy
from rclpy.node import Node
# from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class DataCollector(Node):
    def __init__(self):
        super().__init__('DATA_collector')

        "Trajectory subscriptions"
        self.x_true_subscription = self.create_subscription(Float32MultiArray, 
                                                     'x_true_topic', self.x_true_callback, 1)       
        self.x_est_subscription = self.create_subscription(Float32MultiArray, 
                                                     'x_est_topic', self.x_est_callback, 1)
        self.P_subscription = self.create_subscription(Float32MultiArray, 
                                                     'P_topic', self.P_callback, 1)
        self.u_subscription = self.create_subscription(Float32MultiArray, 
                                                     'u_topic', self.u_callback, 1)   
        
        "command to save data subscription"
        self.save_data_subscription = self.create_subscription(String
                                                               , 'save_data_topic', self.save_data_callback, 1)
        
        self.x_true_list = []  # true pose list
        self.x_est_list  = []  # estimated pose list
        self.P_list      = []  # covariance list
        self.u_list      = []  # control input list

    def x_true_callback(self, msg):
        # Append the new data to the list
        timestamp = self.get_clock().now().to_msg().sec
        timestamp_array = np.array([ [timestamp] ])
        x_true_array = np.array( msg.data ).reshape(-1,1) # convert to numpy array\
        # print(f'x true {x_true_array} with shape {np.shape(x_true_array)}')
        x_true_w_time = np.vstack( (timestamp_array, x_true_array) )
        # self.get_logger().info(f'x true with time {x_true_w_time}')
        self.x_true_list.append(x_true_w_time)

    def x_est_callback(self, msg):
        # Append the new data to the list
        timestamp = self.get_clock().now().to_msg().sec
        timestamp_array = np.array([ [timestamp] ])
        x_est_array = np.array( msg.data ).reshape(-1,1) # convert to numpy array
        x_est_w_time = np.vstack( (timestamp_array, x_est_array) )
        self.x_est_list.append(x_est_w_time)

    def P_callback(self, msg):
        # Append the new data to the list
        timestamp = self.get_clock().now().to_msg().sec
        timestamp_array = np.array([ [timestamp] ])
        P_array = np.array( msg.data ).reshape( (3,3) ) # convert to numpy array
        # print(f'P shape is {np.shape(P_array)}')
        self.P_list.append(P_array)   

    def u_callback(self, msg):
        # Append the new data to the list
        timestamp = self.get_clock().now().to_msg().sec
        timestamp_array = np.array([ [timestamp] ])
        u_array = np.array( msg.data ).reshape(-1,1) # convert to numpy array
        u_w_time = np.vstack( (timestamp_array, u_array) )
        self.u_list.append(u_w_time)         

    def save_data_callback(self, msg):
        self.get_logger().info(f'Received save data command: {msg.data}')
        self.save_data() 
        # reset lists to empty 

        self.x_true_list = []  # true pose list
        self.x_est_list  = []  # estimated pose list
        self.P_list      = []  # covariance list
        self.u_list      = []  # control input list

    def save_data(self):
        # Convert each list to a NumPy array and save it to a .npy file
        if self.x_true_list:
            x_true_array = np.hstack(self.x_true_list)
            np.save('x_true.npy', x_true_array)
        if self.x_est_list:
            x_est_array  = np.hstack(self.x_est_list)
            np.save('x_est.npy', x_est_array)
        if self.P_list:
            P_array      = np.stack(self.P_list,axis=2)
            np.save('P.npy', P_array)
        if self.u_list:
            u_array      = np.hstack(self.u_list)
            np.save('u.npy', u_array) 

        self.get_logger().info('Data saved')

def main(args=None):
    rclpy.init(args=args)
    data_collector_node = DataCollector()
    
    try:
        print('Listening for x_true, x_est, u, and P')
        rclpy.spin(data_collector_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        data_collector_node.get_logger().error(f'An error occurred: {e}')
        data_collector_node.save_data()
        raise
    finally:
        data_collector_node.save_data()
        data_collector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
