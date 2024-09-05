import numpy as np
import matplotlib.pyplot as plt
"""
dx = [VcosT, VsinT, w];
A = 0, 0, -Vsin(T), 
    0, 0,  Vcos(T), 
    0, 0,        0,

B = cos(T), 0
    sin(T), 0
    0,      1

"""
num_state  = 3 # X Y Theta
num_inputs = 2 # forward and angular rates

class tbest:
    """
    The estimator class.
    """
    def __init__(self, dt, P, Q, R):
        self.dt = dt
        self.x_ii = np.array([
            [0], # X position
            [0], # Y position
            [0]  # Orientation
        ])
        self.H = np.array([
            [1, 0, 0], # Measure X Position + Bias X
            [0, 1, 0], # Measure Y Position + Bias Y
            [0, 0, 1]  # Measure Orientation Directly
        ])

        self.F = np.eye(self.x_ii.shape[0])
        self.G = np.eye(self.x_ii.shape[0], num_inputs)
        self.x_ji = self.x_ii
        self.x_jj = self.x_ii

        self.P_ii = P 
        self.P_ji = self.P_ii
        self.P_jj = self.P_ii

        self.Q = Q
        self.R = R

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
        """
        The prediction step.
        """
        A = np.array([
            [0, 0, -u_ii[0,0]*np.sin(self.x_ii[2,0])],
            [0, 0,  u_ii[0,0]*np.cos(self.x_ii[2,0])],
            [0, 0,  0,                              ],
        ])
        B = np.array([
            [np.cos(self.x_ii[2,0]), 0],
            [np.sin(self.x_ii[2,0]), 0], 
            [0,                      1], 
        ])
        self.F = np.eye(A.shape[0]) + A * self.dt  
        self.G = B * self.dt 

        dx_ji = np.array([
            [u_ii[0,0]*np.cos(self.x_ii[2,0])],
            [u_ii[0,0]*np.sin(self.x_ii[2,0])],
            [u_ii[1,0]],
        ])
        self.x_ji = dx_ji * self.dt + self.x_ii

        self.P_ji = self.F @ self.P_ii @ self.F.transpose() + self.Q
               
    def update(self, z_jj):
        """
        The measurement update step.
        """
        z_ji = self.x_ji[0:3,:]
        y_jj = z_jj - z_ji
        S_jj = self.H @ self.P_ji @ self.H.transpose() + self.R 
        K_jj = self.P_ji @ self.H.transpose() @ np.linalg.inv(S_jj)
        self.x_jj = self.x_ji + K_jj @ y_jj
        self.P_jj = (np.eye(self.F.shape[0]) - K_jj @ self.H) @ self.P_ji


class tbtru:
    """
    The true system dynamics.
    """
    def __init__(self, Q, R, dt):
        self.Q = Q
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
        """
        Step forward the dynamics with process noise.
        """
        self.x_i = self.x
        dx_i = np.array([
            [u_ii[0,0]*np.cos(self.x_i[2,0])],
            [u_ii[0,0]*np.sin(self.x_i[2,0])],
            [u_ii[1,0]]
        ])
        self.x_j = dx_i*self.dt + self.x_i + np.array([np.diag(self.Q)]).transpose()
    
    def sample(self):
        """
        Take a measurement of the system.
        """
        self.z = self.x + np.array([np.diag(self.R)]).transpose()
        return self.z

    def spinOnce(self, u_ii):
        self.propogate(u_ii)
        self.x = self.x_j
        

class ctrl:
    """
    The controller and path planner algorithm.
    """
    def __init__(self, dt, v, K):
        self.dt = dt
        self.V = v
        self.t = 0.
        self.K = K
        self.d = np.zeros((3,1))
        self.u = np.zeros((2,1))

    def compute_command(self, a):
        """
        Compute the control action.
        """
        w = -self.K*(self.d[2,0] - a)
        self.u = np.array([
            [self.V],
            [w]
        ])
        
    
    def compute_trajectory(self):
        """
        Compute the current setpoint for the desired path.
        """
        # (2cos(t), sin(2t)), a = arctan(dy/dx)
        x = 2*np.cos(   self.t - np.pi/2)
        y =   np.sin(2*(self.t - np.pi/2))
        
        dx = 2 * -1 * np.sin(self.t - np.pi/2)
        dy = 2 *  1 * np.cos(2*(self.t - np.pi/2))

        a = np.arctan2(dy,dx)

        self.V = np.sqrt(dx**2 + dy**2)

        self.d = np.array([
            [x],
            [y],
            [a]
        ])

    def update(self):
        """
        Increment the time.
        """
        self.t += self.dt

    def spinOnce(self, a):
        self.compute_trajectory()
        self.compute_command(a) # Need to add a state measurement here
        self.update()


class plotter:
    """
    Record information for plotting.
    """
    def __init__(self, dt, T):
        self.t = np.arange(0, T, dt)
        self.x = np.zeros((3,self.t.shape[0]))   # The estimator state
        self.d = np.zeros((3,self.t.shape[0]))   # The desired setpoint
        self.r = np.zeros((3,self.t.shape[0]))   # The true state
        self.P = np.zeros((3,3,self.t.shape[0])) # The covariance of the estimator
        self.count = 0

    def spinOnce(self, x, d, r, P):
        self.record(x, d, r, P)
        self.update()

    def record(self, x, d, r, P):
        """
        Record given information.
        """
        self.x[:,[self.count]] = x
        self.d[:,[self.count]] = d
        self.r[:,[self.count]] = r
        self.P[:,:,self.count] = P
        
    def update(self):
        """
        Increment the record index.
        """
        self.count += 1

    def plot(self):
        """
        Plot the stored information.
        """
        
        # This is the plot of state 1 (X Position) estimation with truth, desired, and confidence interval
        plt.figure(1)
        plt.plot(self.t, self.x[0,:], 'b', label='estimate')
        plt.plot(self.t, self.r[0,:], 'k', label='truth')
        plt.plot(self.t, self.d[0,:], 'm', label='target')
        plt.plot(self.t, self.x[0,:]+3*np.sqrt(self.P[0,0,:]), 'r', label='confidence')
        plt.plot(self.t, self.x[0,:]-3*np.sqrt(self.P[0,0,:]), 'r', label='confidence')
        plt.xlabel('Time (s)')
        plt.ylabel('X Pose (m)')
        plt.legend(loc='upper right')
        plt.show()

        # Plot 1 but for state 2 (Y Position)
        plt.figure(2)
        plt.plot(self.t, self.x[1,:], 'b', label='estimate')
        plt.plot(self.t, self.r[1,:], 'k', label='truth')
        plt.plot(self.t, self.d[1,:], 'm', label='target')
        plt.plot(self.t, self.x[1,:]+3*np.sqrt(self.P[1,1,:]), 'r', label='confidence')
        plt.plot(self.t, self.x[1,:]-3*np.sqrt(self.P[1,1,:]), 'r', label='confidence')
        plt.xlabel('Time (s)')
        plt.ylabel('Y Pose (m)')
        plt.legend(loc='upper right')
        plt.show()

        # Plot 1 but for state 3 (Theta)
        plt.figure(3)
        plt.plot(self.t, self.x[2,:], 'b', label='estimate')
        plt.plot(self.t, self.r[2,:], 'k', label='truth')
        plt.plot(self.t, self.d[2,:], 'm', label='target')
        plt.plot(self.t, self.x[2,:]+3*np.sqrt(self.P[2,2,:]), 'r', label='confidence')
        plt.plot(self.t, self.x[2,:]-3*np.sqrt(self.P[2,2,:]), 'r', label='confidence')
        plt.xlabel('Time (s)')
        plt.ylabel('Theta (rad)')
        plt.legend(loc='upper right')
        plt.show()

        # XY plot showing the estimator, truth, and desired trajectories through time
        plt.figure(4)
        plt.plot(self.x[0,:], self.x[1,:], color='b', label='estimate')
        plt.plot(self.d[0,:], self.d[1,:], color='m', label='target')
        plt.plot(self.r[0,:], self.r[1,:], color='k', label='truth')
        plt.xlabel('X Pose (m)')
        plt.ylabel('Y Pose (m)')
        plt.legend(loc='upper right')
        plt.xlim(-2.5,2.5)
        plt.ylim(-1.5,1.5)
        plt.show()

        


if __name__=="__main__":
    
    # Some simulation constants
    P = (1e-2)*np.eye(num_state)
    Q = (1e-6)*np.eye(num_state)
    R = (1e-2)*np.eye(num_state)
    dt = 1e-2    # time step
    v = 0.1*10*2 # just an initial value
    K = -100.0   # P controller gain
    T = 10.      # max time value
    t = 0.       # current time value
    dn = 1.0     # measurement period

    # Initialize the objects
    TBEST = tbest(dt, P, Q, R)
    TBTRU = tbtru(Q, R, dt)
    CTRL = ctrl(dt, v, K)
    PLOT = plotter(dt, T)

    # Run through the simulation and retrieve needed information from each object 
    a = 0.
    z = 0.
    flag = False
    while (t<T-dt):

        # Calculate the control action
        CTRL.spinOnce(a)
        u = CTRL.u
        d = CTRL.d
        
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
        PLOT.spinOnce(x,d,r,P)
        
        # Update some values for the next simulation
        a = x[2,0]
        t += dt
        flag = False
    
    # Plot
    PLOT.plot()