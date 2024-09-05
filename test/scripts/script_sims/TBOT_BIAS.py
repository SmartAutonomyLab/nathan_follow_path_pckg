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
    def __init__(self, dt, v, K):
        self.dt = dt
        self.V = v
        self.t = 0.
        self.K = K
        self.d = np.zeros((3,1))
        self.u = np.zeros((2,1))

    def compute_command(self, a):
        w = -self.K*(self.d[2,0] - a)
        self.u = np.array([
            [self.V],
            [w]
        ])
        
    
    def compute_trajectory(self):
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
        self.t += self.dt

    def spinOnce(self, a):
        self.compute_trajectory()
        self.compute_command(a) # Need to add a state measurement here
        self.update()


class plotter:
    def __init__(self, dt, T):
        self.t = np.arange(0, T, dt)
        self.x = np.zeros((num_state,self.t.shape[0]))
        self.d = np.zeros((3,self.t.shape[0]))
        self.r = np.zeros((3,self.t.shape[0]))
        self.P = np.zeros((5,5,self.t.shape[0]))
        self.count = 0

    def spinOnce(self, x, d, r, P):
        self.record(x, d, r, P)
        self.update()

    def record(self, x, d, r, P):
        self.x[:,[self.count]] = x
        self.d[:,[self.count]] = d
        self.r[:,[self.count]] = r
        self.P[:,:,self.count] = P
        
    def update(self):
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

        # Plot 1 but for state 4 (X Position Bias)
        plt.figure(4)
        plt.plot(self.t, self.x[3,:], 'b', label='estimate')
        #plt.plot(self.t, self.r[3,:], 'k', label='truth')
        #plt.plot(self.t, self.d[3,:], 'm', label='target')
        plt.plot(self.t, self.x[3,:]+3*np.sqrt(self.P[3,3,:]), 'r', label='confidence')
        plt.plot(self.t, self.x[3,:]-3*np.sqrt(self.P[3,3,:]), 'r', label='confidence')
        plt.xlabel('Time (s)')
        plt.ylabel('X Pose Bias (m)')
        plt.legend(loc='upper right')
        plt.show()

        # Plot 1 but for state 5 (X Position Bias)
        plt.figure(5)
        plt.plot(self.t, self.x[4,:], 'b', label='estimate')
        #plt.plot(self.t, self.r[4,:], 'k', label='truth')
        #plt.plot(self.t, self.d[4,:], 'm', label='target')
        plt.plot(self.t, self.x[4,:]+3*np.sqrt(self.P[4,4,:]), 'r', label='confidence')
        plt.plot(self.t, self.x[4,:]-3*np.sqrt(self.P[4,4,:]), 'r', label='confidence')
        plt.xlabel('Time (s)')
        plt.ylabel('Y Pose Bias (m)')
        plt.legend(loc='upper right')
        plt.show()

        # XY plot showing the estimator, truth, and desired trajectories through time
        plt.figure(6)
        plt.plot(self.x[0,:], self.x[1,:], color='b', label='estimate')
        plt.plot(self.d[0,:],self.d[1,:], color='m', label='target')
        plt.plot(self.r[0,:], self.r[1,:], color='k', label='truth')
        plt.xlabel('X Pose (m)')
        plt.ylabel('Y Pose (m)')
        plt.xlim(-3,3)
        plt.ylim(-2,2)
        plt.legend(loc='upper right')
        plt.show()

        


if __name__=="__main__":
    P = (1e-2)*np.eye(num_state)
    Q = (1e-6)*np.eye(num_state)
    Q[3,3] = 1e-1   # Random Walk noise should be higher than the others
    Q[4,4] = 1e-1   # Random Walk noise should be higher than the others
    R = (1e-2)*np.eye(num_measure)
    dt = 1e-2
    v = 0.1*10*2 # just an initial value
    K = -100.0
    T = 10.
    t = 0.
    dn = 1.0

    TBEST = tbest(dt, P, Q, R)
    TBTRU = tbtru(Q, R, dt)
    CTRL = ctrl(dt, v, K)
    PLOT = plotter(dt, T)

    a = 0.
    z = 0.
    flag = False
    while (t<T-dt):
        CTRL.spinOnce(a)
        u = CTRL.u
        TBTRU.spinOnce(u)
        r = TBTRU.x
        if (int(t/dt) % int(dn/dt) == 0):
            flag = True
            z = TBTRU.sample()
            
        TBEST.spinOnce(u,z,flag)
        x = TBEST.x_ii
        d = CTRL.d
        P = TBEST.P_ii
        a = x[2,0]
        PLOT.spinOnce(x,d,r,P)
        t += dt
        flag = False
    PLOT.plot()