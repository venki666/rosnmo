import numpy as np
import matplotlib.pyplot as plt

# Robot parameters
r = 0.05   # wheel radius [m]
L = 0.2    # wheel separation [m]
dt = 0.01

# Start and goal
x, y, theta = 0.0, 0.0, 0.0
x_goal, y_goal, theta_goal = 1.0, 1.0, np.pi/2

# Controller gains
k_heading = 1.5
k_dist = 0.8

# Functions
def inverse_kinematics(v, omega, r, L):
    vR = v + (L/2)*omega
    vL = v - (L/2)*omega
    return vR/r, vL/r

def forward_kinematics(x, y, theta, wR, wL, r, L, dt):
    v = r*(wR + wL)/2
    omega = r*(wR - wL)/L
    x += v*np.cos(theta)*dt
    y += v*np.sin(theta)*dt
    theta += omega*dt
    return x, y, theta

# Store trajectory
xs, ys = [x], [y]

# Loop until close to goal
for _ in range(5000):
    # Compute distance and heading error
    dx = x_goal - x
    dy = y_goal - y
    dist = np.hypot(dx, dy)
    if dist < 0.01 and abs(theta - theta_goal) < 0.01:
        break
    target_heading = np.arctan2(dy, dx)
    heading_error = np.arctan2(np.sin(target_heading - theta), np.cos(target_heading - theta))
    
    # Control law
    v = k_dist * dist
    omega = k_heading * heading_error
    
    # Inverse kinematics
    wR, wL = inverse_kinematics(v, omega, r, L)
    
    # Forward kinematics
    x, y, theta = forward_kinematics(x, y, theta, wR, wL, r, L, dt)
    
    xs.append(x)
    ys.append(y)

# Plot
plt.figure(figsize=(6,6))
plt.plot(xs, ys, 'b-', label="Path")
plt.scatter([xs[0], x_goal], [ys[0], y_goal], color='red', label="Start/Goal")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Smooth Inverse Kinematics Path with Pure Pursuit Control")
plt.show()
