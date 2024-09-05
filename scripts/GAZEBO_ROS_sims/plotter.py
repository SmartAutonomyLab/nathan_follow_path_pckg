import numpy as np
import matplotlib.pyplot as plt 

x_true = np.load('x_true.npy', allow_pickle=True)
x_est = np.load('x_est.npy', allow_pickle=True)
u = np.load('u.npy', allow_pickle=True)
P = np.load('P.npy', allow_pickle=True)

path_list = np.load('circle_points.npy', allow_pickle=True)

# FIGURE 8
path_list2 = np.load('figure_eight_points.npy', allow_pickle=True)

plt.figure(1)
# plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
plt.plot(x_est[0,:], x_est[1,:],'b', linestyle = ':', label='estimate')
plt.plot(x_true[0,:], x_true[1,:], 'k', label='truth')
plt.plot(x_est[0,:], x_est[1,:]+3*np.sqrt(P[0,0,:]), 'r', label='confidence')
plt.plot(x_est[0,:], x_est[1,:]-3*np.sqrt(P[0,0,:]), 'r', label='confidence')
plt.xlabel('Time (s)')
plt.ylabel('X Pose (m)')
plt.legend(loc='upper right')
plt.show()

plt.figure(2)
# plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
plt.plot(x_est[0,:], x_est[2,:],'b', linestyle = ':', label='estimate')
plt.plot(x_true[0,:], x_true[2,:], 'k', label='truth')
plt.plot(x_est[0,:], x_est[2,:]+3*np.sqrt(P[1,1,:]), 'r', label='confidence')
plt.plot(x_est[0,:], x_est[2,:]-3*np.sqrt(P[1,1,:]), 'r', label='confidence')
plt.xlabel('Time (s)')
plt.ylabel('Y Pose (m)')
plt.legend(loc='upper right')
plt.show()

plt.figure(3)
# plt.scatter(self.arrival_times, zeros, color = 'r',  label='Arrival Times')
plt.plot(x_est[0,:], x_est[3,:],'b', linestyle = ':', label='estimate')
plt.plot(x_true[0,:], x_true[3,:], 'k', label='truth')
plt.plot(x_est[0,:], x_est[3,:]+3*np.sqrt(P[2,2,:]), 'r', label='confidence')
plt.plot(x_est[0,:], x_est[3,:]-3*np.sqrt(P[2,2,:]), 'r', label='confidence')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (rad)')
plt.legend(loc='upper right')
plt.show()


# XY plot showing the estimator, truth, and desired trajectories through time
plt.figure(4)
# plt.plot(self.path_array[0,:], self.path_array[1,:], color = 'r', marker = 'o', label = 'Desired Path' )
plt.plot(x_est[1,:], x_est[2,:], color='b', linestyle = ':', linewidth = 2, label='estimate')
plt.plot(x_true[1,:], x_true[2,:], color='k', label='truth')
plt.xlabel('X Pose (m)')
plt.ylabel('Y Pose (m)')
plt.legend(loc='upper right')
plt.xlim(-4.5,4.5)
plt.ylim(-4.5,4.5)
plt.show()