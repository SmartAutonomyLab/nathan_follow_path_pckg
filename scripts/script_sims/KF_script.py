# Clear variables
%reset -f
# Enable interactive plots
%matplotlib inline 
import numpy as np
import matplotlib.pyplot as plt

# Clear plots
plt.close('all')

# Establish Dynamic System
h = np.array([0.4, 0.2, 0.7])  # [m,c,k]
dt = 1e-3  # propagation period
dn = 0.50  # measurement period
T = 10     # full time for simulation

# Mass Spring Damper System
A = np.array([
    [0, 1],
    [-h[2]/h[0], -h[1]/h[0]]
])
B = np.array([
    [0],
    [1/h[0]]
])
F = np.eye(2) + A * dt
G = B * dt
H = np.eye(2)

# Initialize Kalman Filter
P = (1e-2) * np.eye(2) # covariance of estimates    
Q = (1e-3) * np.eye(2) # uncertainty of dynamics (process)
R = (1e-1) * np.eye(2) # uncertainty of measurements

# Control gain
k = 0.2

# Simulate the system with the controller and the filter
X = []
Z = []
C = []
rr = []
Force  = []

u = 0
x_last = np.array([1, 0.4])
P_last = P
r_last = x_last

for t in np.arange(0, T, dt):
    # Get the force input
    f = 1e-2 * np.sin(t) + u
    # f = u
    Force.append(f)
    # Predict
    x_next = F @ x_last + G.flatten() * f
    P_next = F @ P_last @ F.T + Q

    # Truth
    r_next = F @ r_last + G.flatten() * f + Q[0, 0] * np.random.randn(2)

    if t % dn < dt/10:  # Check if it's measurement time
        # print(f'current time is {t}')
        # Fake a measurement
        # z = r_last + R[0, 0] * np.random.randn(2)
        z = r_last + R[0, 0] * np.random.choice([1, -1])
        Z.append(z)
        # Update
        z_err = z - H @ x_next
        S = H @ P_next @ H.T + R
        K = P_next @ H.T @ np.linalg.inv(S)
        x_next = x_next + K @ z_err
        P_next = (np.eye(2) - K @ H) @ P_next

    # Determine the control action
    pd = np.sin(0.9 * t)
    p = x_next[0]
    u = -k * (pd - p)

    # Update for sim
    x_last = x_next
    P_last = P_next
    r_last = r_next

    # Store data for plots
    X.append(x_next)
    C.append(P_next[0, 0])
    rr.append(r_next)

# Convert lists to arrays
X = np.array(X) 
Z = np.array(Z)
C = np.array(C)
rr = np.array(rr)
Force  = np.array(Force)

# Plot
plt.figure(figsize=(15, 10), dpi=300)  # Increase figure size and DPI
plt.plot(np.arange(0, T, dt), X[:, 0], 'b-*', markersize=1, label='Estimate')
plt.plot(np.arange(0, T, dt), rr[:, 0], 'k', label='Truth')
plt.plot(np.arange(0, T, dn), Z[:, 0], 'm', markersize=2, marker='o', linestyle='none', label='Measurement')
plt.plot(np.arange(0, T, dt), X[:, 0] + 3 * np.sqrt(C), 'r', label='Confidence Interval')
plt.plot(np.arange(0, T, dt), X[:, 0] - 3 * np.sqrt(C), 'r')
plt.plot(np.arange(0, T, dt), np.sin(0.9 * np.arange(0, T, dt)), 'r', label='Desired position')
plt.title("Position Plot")
plt.ylabel("x (m)")
plt.xlabel("t (s)")
plt.legend()

plt.figure(figsize=(15, 10), dpi=300)  # Increase figure size and DPI
plt.plot(np.arange(0, T, dt), X[:, 1], 'b-*', markersize=1, label='Estimate')
plt.plot(np.arange(0, T, dt), rr[:, 1], 'k', label='Truth')
plt.plot(np.arange(0, T, dn), Z[:, 1], 'm', markersize=2, marker='o', linestyle='none', label='Measurement')
plt.plot(np.arange(0, T, dt), X[:, 1] + 3 * np.sqrt(C), 'r', label='Confidence Interval')
plt.plot(np.arange(0, T, dt), X[:, 1] - 3 * np.sqrt(C), 'r')
# plt.plot(np.arange(0, T, dt), np.sin(0.9 * np.arange(0, T, dt)), 'r', label='Desired Velocity')
plt.title("Velocity Plot")
plt.ylabel("v (m)")
plt.xlabel("t (s)")
plt.legend()

plt.figure(figsize=(15, 10), dpi=300)  # Increase figure size and DPI
plt.plot(np.arange(0, T, dt), Force, 'b-*', markersize=1, label='Force Input')
plt.title("Actuation Plot")
plt.ylabel("F (N)")
plt.xlabel("t (s)")
plt.legend()

plt.show()