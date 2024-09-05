import numpy as np
import matplotlib.pyplot as plt

"""
dx = [VcosT, VsinT, w, nx, ny];
A = 0, 0, -Vsin(T), 0, 0
    0, 0,  Vcos(T), 0, 0
    0, 0,        0, 0, 0
    0
    0,
B = cos(T), 0
    sin(T), 0
    0,      1
    0, 0
    0, 0
"""
num_state = 5
num_inputs = 2
num_measure = 3

class tbest:
    def __init__(self, dt, P, Q, R):
        self.dt = dt
        self.x_ii = np.array([
            [0], # X position
            [0], # Y position
            [0], # Orientation
            [1], # Bias X
            [1]  # Bias Y
        ])
        self.H = np.array([
            [1, 0, 0, 1, 0], # Measure X Position + Bias X
            [0, 1, 0, 0, 1], # Measure Y Position + Bias Y
            [0, 0, 1, 0, 0]  # Measure Orientation Directly
        ])

        self.F = np.eye(self.x_ii.shape[0])
        self.G = np.eye(self.x_ii.shape[0], num_inputs)
        self.L = np.eye(self.x_ii.shape[0])
        self.x_ji = self.x_ii
        self.x_jj = self.x_ii

        self.P_ii = P 
        self.P_ji = self.P_ii
        self.P_jj = self.P_ii

        self.Q = Q
        self.R = R

        self.parked      = False #boolean indicator bot reaching final point

    def spinOnce(self, u_ii, z_jj, flag):
        self.propogate(u_ii)
        if flag:
            self.update(z_jj)
            self.x_ii = self.x_jj
            self.P_ii = self.P_jj
        else:
            self.x_ii = self.x_ji
            self.P_ii = self.P_ji
        

    def propogate(self, u_ii):
        A = np.array([
            [0, 0, -u_ii[0,0]*np.sin(self.x_ii[2,0]), 0, 0],
            [0, 0,  u_ii[0,0]*np.cos(self.x_ii[2,0]), 0, 0],
            [0, 0,  0,                                0, 0],
            [0, 0,  0,                                0, 0],
            [0, 0,  0,                                0, 0]
        ])
        B = np.array([
            [np.cos(self.x_ii[2,0]), 0],
            [np.sin(self.x_ii[2,0]), 0], 
            [0,                      1], 
            [0,                      0],
            [0,                      0]
        ])
        C = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        self.F = np.eye(A.shape[0]) + A * self.dt  
        self.G = B * self.dt 
        self.L = C * self.dt

        # Access the process noise directly
        nx = 1*np.sqrt(self.Q[3,3])*np.random.randn()
        ny = 1*np.sqrt(self.Q[4,4])*np.random.randn()
        
        dx_ji = np.array([
            [u_ii[0,0]*np.cos(self.x_ii[2,0])],
            [u_ii[0,0]*np.sin(self.x_ii[2,0])],
            [u_ii[1,0]],
            [nx],
            [ny]
        ])
        
        self.x_ji = dx_ji * self.dt + self.x_ii

        self.P_ji = self.F @ self.P_ii @ self.F.transpose() + self.L @ self.Q @ self.L.transpose()
               
    def update(self, z_jj):
        z_ji = self.x_ji[0:3,:] + np.vstack((self.x_ji[3:5,:], 0))
        y_jj = z_jj - z_ji
        S_jj = self.H @ self.P_ji @ self.H.transpose() + self.R 
        K_jj = self.P_ji @ self.H.transpose() @ np.linalg.inv(S_jj)
        self.x_jj = self.x_ji + K_jj @ y_jj
        self.P_jj = (np.eye(self.F.shape[0]) - K_jj @ self.H) @ self.P_ji

class tbtru:
    def __init__(self, Q, R, dt):
        self.Q = Q[0:3,0:3]
        self.R = R
        self.dt = dt

        self.x = np.array([
            [0], # X Position
            [0], # Y Position
            [0]  # Orientation
        ])
        self.x_i = np.zeros((3,1))
        self.x_j = np.zeros((3,1))
        self.z   = np.zeros((3,1))

    def propogate(self, u_ii):
        self.x_i = self.x
        dx_i = np.array([
            [u_ii[0,0]*np.cos(self.x_i[2,0])],
            [u_ii[0,0]*np.sin(self.x_i[2,0])],
            [u_ii[1,0]]
        ])
        self.x_j = dx_i*self.dt + self.x_i + np.array([np.diag(self.Q)]).transpose()
    
    def sample(self):
        self.z = self.x + np.array([np.diag(self.R)]).transpose()
        return self.z

    def spinOnce(self, u_ii):
        self.propogate(u_ii)
        self.x = self.x_j
        
class ctrl:
    """
    The controller and path planner algorithm.
    """
    def __init__(self, dt, v, K, path_array ):
        self.dt = dt
        self.V = v
        self.t = 0.
        self.K = K
        self.u = np.zeros((2,1))
        self.path_index = 0
        self.path_array = path_array
        self.error_vector   = np.zeros( (3,1) )
        self.prev_yaw_error = 0.
        self.I_yaw_error    = 0.
        self.init_ind       = True # boolean describing if a new goal point ha just been selected
        self.dedt           = 0.
    
        self.arrival_times  = []
        

    def compute_error(self, x):
        """
        x - state in form [x, y, yaw]
        Compute the control action.
        """
        
        # x, y error and the euclidian error
        x_diff = self.current_point[0] - x[0,0]
        y_diff = self.current_point[1] - x[1,0]
        self.distance = np.sqrt(x_diff ** 2 + y_diff ** 2)

        # yaw error
        current_yaw = x[2,0]
        desired_yaw = np.arctan2(y_diff, x_diff)

        yaw_error = desired_yaw - current_yaw

        # ensure yaw_error is within pi and -pi to prevent spinning
        if yaw_error >= np.pi:
            yaw_error -= 2*np.pi 
        if yaw_error <= -np.pi:
            yaw_error += 2*np.pi 

        self.error_vector = np.array([ 
                                     [x_diff], 
                                     [y_diff], 
                                     [yaw_error]
                                     ])

    def compute_command(self):
       
        yaw_error = self.error_vector[2,0]
        
        # update integral and derivateive terms

        if self.init_ind:
            # first spin since new point is chosen 
            # DO NOT UPDATE I AND dedt
            self.init_ind = False
        else:
            # it is not first spin since new target is chosen 
            # UPDATE I AND dedt
            self.dedt = (yaw_error - self.prev_yaw_error)/self.dt
            # print(f'dedt is {self.dedt}')
        self.I_yaw_error += yaw_error * self.dt

        kP = self.K[0].item()
        kI = self.K[1].item()
        kD = self.K[2].item()

        P_gain = kP*yaw_error
        I_gain = kI*self.I_yaw_error
        D_gain = kD*self.dedt 

        self.angular_gain_vector = np.array([ 
                                            [P_gain], 
                                            [I_gain], 
                                            [D_gain], 
                                            ])
        
        w = np.sum( self.angular_gain_vector ) 

        #checks for saturation
        if w >= 2.84:
            w = 2.84
        if w <= -2.84:
            w = -2.84
      
        # calculate linear
        if abs( yaw_error )>= np.pi/4:
            act_linear = float( 0 )
        else:
            act_linear = float( self.V / (np.pi/4) * \
                (np.pi/4 - abs( yaw_error ) ) )
        

        self.u = np.array([
            [act_linear],
            [w]
        ])

        # update previous yaw error for next time step. 
        self.prev_yaw_error = yaw_error
        
    def compute_trajectory(self):
        '''
        compute trajectory from point in path list
        '''
        # set current target
        self.current_point = self.path_array[:,self.path_index]

    def update(self):
        """
        Increment the time.
        """
        self.t += self.dt

    def spinOnce(self, x):
        self.compute_trajectory()
        self.compute_error(x)
        self.compute_command() # Need to add a state measurement here
        self.update()

    def check(self, tolerance, tbest_object):
    # Determines if we are close enough to intermediary point or final point
        if self.distance < tolerance: # close enough to current goal 
            # update boolean to show that it is the fist spin for new target point
            self.init_ind = True
            self.prev_yaw_error = 0. # reset previous yaw error to zero
            self.I_yaw_error    = 0. # reset integral error back to zero
            self.arrival_times.append(self.t) # add current time to list of arrival times
            # print(f'current time {self.t}')
            # print(f'current goal point index {self.path_index}')
            # print(f'current goal point {self.path_array[ :, self.path_index ]}')
            if np.array_equal(self.path_array[ :, self.path_index ]
                              , self.path_array[:, -1] ): #close enough to final goal
                tbest_object.parked = True
            else:
                # proceed to next point
                self.path_index += 1

class plotter:
    """
    Record information for plotting.
    """
    def __init__(self, path_array ):
       
        self.path_array = path_array
        self.t = []
        self.x = [] # The estimator state
        self.r = [] # The true state
        self.P = [] # The covariance of the estimator
        self.u = [] # The control inputs (radial and angular speed)
        self.e = []
        self.angular_gain = []
        self.count = 0


    def spinOnce(self, x, t, r, P, u, e, angular_gain):
        # INCORPORATE GRAPHS OF ERROR_VECTOR AND CONTROL COMPONENTS.
        self.record(x, t, r, P, u, e, angular_gain)
        self.update()

    def record(self, x, t, r, P, u, e, angular_gain):
        """
        Record given information.
        """
        self.x.append( x )
        self.t.append( t )
        self.r.append( r )
        self.P.append( P )
        self.u.append( u )
        self.e.append( e )
        self.angular_gain.append( angular_gain )

    def update(self):
        """
        Increment the record index.
        """
        self.count += 1

    def plot(self, arrival_times, debug = False):
        """
        Plot the stored information.
        """
        self.x = np.hstack( self.x )
        self.t = np.hstack( self.t )
        self.r = np.hstack( self.r )
        self.P = np.stack( self.P, axis=2 )
        self.u = np.hstack( self.u )
        self.e = np.hstack( self.e )
        self.angular_gain = np.hstack( self.angular_gain )
        self.arrival_times = np.array( arrival_times )
        zeros = np.zeros_like(self.arrival_times)
        # print(f"self.t shape: {self.t.shape}")
        # print(f"self.x shape: {self.x.shape}")
        # print(f"self.x[0, :] shape: {self.x[0, :].shape}")

        if debug:
            # This is the plot of state 1 (X Position) estimation with truth, desired, and confidence interval
            plt.figure(1)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.x[0,:],'b', linestyle = ':', label='estimate')
            plt.plot(self.t, self.r[0,:], 'k', label='truth')
            plt.plot(self.t, self.x[0,:]+3*np.sqrt(self.P[0,0,:]), 'r', label='confidence')
            plt.plot(self.t, self.x[0,:]-3*np.sqrt(self.P[0,0,:]), 'r', label='confidence')
            plt.xlabel('Time (s)')
            plt.ylabel('X Pose (m)')
            plt.legend(loc='upper right')
            plt.show()

            # Plot 1 but for state 2 (Y Position)
            plt.figure(2)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.x[1,:], 'b', label='estimate')
            plt.plot(self.t, self.r[1,:], 'k', label='truth')
            plt.plot(self.t, self.x[1,:]+3*np.sqrt(self.P[1,1,:]), 'r', label='confidence')
            plt.plot(self.t, self.x[1,:]-3*np.sqrt(self.P[1,1,:]), 'r', label='confidence')
            plt.xlabel('Time (s)')
            plt.ylabel('Y Pose (m)')
            plt.legend(loc='upper right')
            plt.show()

            # Plot 1 but for state 3 (Theta)
            plt.figure(3)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.x[2,:], 'b', label='estimate')
            plt.plot(self.t, self.r[2,:], 'k', label='truth')
            plt.plot(self.t, self.x[2,:]+3*np.sqrt(self.P[2,2,:]), 'r', label='confidence')
            plt.plot(self.t, self.x[2,:]-3*np.sqrt(self.P[2,2,:]), 'r', label='confidence')
            plt.xlabel('Time (s)')
            plt.ylabel('Theta (rad)')
            plt.legend(loc='upper right')
            plt.show()

            # XY plot showing the estimator, truth, and desired trajectories through time
            plt.figure(4)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.path_array[0,:], self.path_array[1,:], color = 'r', label = 'Desired Path' )
            plt.plot(self.x[0,:], self.x[1,:], color='b', linestyle = ':', linewidth = 2, label='estimate')
            plt.plot(self.r[0,:], self.r[1,:], color='k', label='truth')
            plt.xlabel('X Pose (m)')
            plt.ylabel('Y Pose (m)')
            plt.legend(loc='upper right')
            plt.xlim(-4,4)
            plt.ylim(-4,4)
            plt.show()

            # plot the desired radial speed
            plt.figure(5)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.u[0,:], 'b', label='Desired radial speed')
            plt.xlabel('Time (s)')
            plt.ylabel('Radial speed (m/s)')
            plt.legend(loc='upper right')
            plt.show()

            # plot the desired angular velocity
            plt.figure(6)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.u[1,:], 'r', linestyle =':', label='Desired angular velocity')
            plt.xlabel('Time (s)')
            plt.ylabel('Angular velocity (rad/s)')
            plt.legend(loc='upper right')
            plt.show()

            # plot the x error
            plt.figure(7)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.e[0,:], 'b', label='error in x position')
            plt.xlabel('Time (s)')
            plt.ylabel('Error (m)')
            plt.legend(loc='upper right')
            plt.show()

            # plot the y error
            plt.figure(8)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.e[1,:], 'b', label='error in y position')
            plt.xlabel('Time (s)')
            plt.ylabel('Error (m)')
            plt.legend(loc='upper right')
            plt.show()

            # plot the yaw error
            plt.figure(9)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.e[2,:], 'b', label='error in yaw')
            plt.xlabel('Time (s)')
            plt.ylabel('Error (rad)')
            plt.legend(loc='upper right')
            plt.show()

            # plot the PID terms
            plt.figure(10)
            plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.t, self.angular_gain[0,:], 'r', label='P Gain')
            plt.plot(self.t, self.angular_gain[1,:], 'g', linestyle = '--', label='I Gain')
            plt.plot(self.t, self.angular_gain[2,:], 'b', linestyle = '-.', label='D Gain')
            plt.xlabel('Time (s)')
            plt.ylabel('Control Gains (rad/s)')
            plt.legend(loc='upper right')
            plt.show()
        else:
            # XY plot showing the estimator, truth, and desired trajectories through time
            plt.figure(1)
            # plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
            plt.plot(self.path_array[0,:], self.path_array[1,:], color = 'r', marker = 'o', label = 'Desired Path' )
            plt.plot(self.x[0,:], self.x[1,:], color='b', linestyle = ':', linewidth = 2, label='estimate')
            plt.plot(self.r[0,:], self.r[1,:], color='k', label='truth')
            plt.xlabel('X Pose (m)')
            plt.ylabel('Y Pose (m)')
            plt.legend(loc='upper right')
            plt.xlim(-4,4)
            plt.ylim(-4,4)
            plt.show()

if __name__=="__main__":
    
    # Some simulation constants
    dt = 1e-2    # time step    
    P = (1e-2)*np.eye(num_state)
    Q = (1e-4)*dt*np.eye(num_state)
    Q[3,3] = 1e1*dt   # Random Walk noise should be higher than the others
    Q[4,4] = 1e1*dt   # Random Walk noise should be higher than the others
    R = (1e0)*dt*np.eye(num_measure)
    # CONTROL
    v = 0.2     # desired tbot radial speed (held fixed)
    kP = 5      # P controller gain
    kI = 1e-1   # I controller gain
    kD = 5e-1   # D controller gain
    K  = np.array([ kP, kI, kD ])

    T = 10.      # max time value
    t = 0.       # current time value
    dn = 5.0       # measurement period
    tolerance = 1e-1
    period = 2*np.pi # period of figure 8

    # Load the list of points from the .npy file
    figure_eight_points_array = np.load('figure_eight_points.npy', allow_pickle=True)

    # Convert the loaded array back to a list of NumPy arrays
    figure_eight_points_list = [np.array(point) for point in figure_eight_points_array]
        
    # Initialize the objects
    TBEST = tbest(dt, P, Q, R)
    TBTRU = tbtru(Q, R, dt)
    CTRL = ctrl(dt, v, K, figure_eight_points_array)
    PLOT = plotter(figure_eight_points_array)

    # Run through the simulation and retrieve needed information from each object 
    x = np.array([
            [0], # X position
            [0], # Y position
            [0]  # Orientation
                ])
    
    z = np.array([
            [0], # X position
            [0], # Y position
            [0]  # Orientation
                ])
    flag = False
    while not TBEST.parked:
        # Calculate the control action and error
        CTRL.spinOnce(x)
        e = CTRL.error_vector
        angular_gain_vector = CTRL.angular_gain_vector
        u = CTRL.u
        
        # Calculate the true state dynamics
        TBTRU.spinOnce(u)
        r = TBTRU.x
        
        # Measurement Block
        if (int(t/dt) % int(dn/dt) == 0):
            flag = True
            z = TBTRU.sample()
            
        # Calculate the estimator state dynamics
        TBEST.spinOnce(u,z,flag)
        x = TBEST.x_ii
        P = TBEST.P_ii
        
        
        # Record information for plots
        PLOT.spinOnce(x,t,r,P,u,e=e,angular_gain=angular_gain_vector)

        CTRL.check(tolerance=tolerance,tbest_object=TBEST)

        t += dt
        flag = False
    
    # Plot
    PLOT.plot(CTRL.arrival_times)