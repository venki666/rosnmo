import numpy as np
import matplotlib.pyplot as plt

def differential_drive_kinematics(pose, t, wheel_velocities, R, L):
    """
    Calculates the derivatives of the robot's pose for forward kinematics.

    Args:
        pose (list): Current pose [x, y, theta] of the robot.
        t (float): Current time (unused in this time-invariant system, but required by odeint).
        wheel_velocities (tuple): Angular velocities (phi_dot_L, phi_dot_R) of left and right wheels.
        R (float): Wheel radius.
        L (float): Distance between wheels (track width).

    Returns:
        list: Derivatives of pose [x_dot, y_dot, theta_dot].
    """
    x, y, theta = pose
    phi_dot_L, phi_dot_R = wheel_velocities

    v = (R / 2) * (phi_dot_L + phi_dot_R)
    omega = (R / L) * (phi_dot_R - phi_dot_L)

    x_dot = v * np.cos(theta)
    y_dot = v * np.sin(theta)
    theta_dot = omega

    return [x_dot, y_dot, theta_dot]

if __name__ == "__main__":
    from scipy.integrate import odeint

    # Robot parameters
    wheel_radius = 0.05  # meters
    track_width = 0.2    # meters

    # Initial pose [x, y, theta]
    initial_pose = [0.0, 0.0, 0.0] # Start at origin, facing along x-axis

    # Wheel angular velocities (e.g., constant velocities for a straight line)
    # To move straight: phi_dot_L = phi_dot_R
    # To turn: phi_dot_L != phi_dot_R
    left_wheel_angular_velocity = 10.0 # rad/s
    right_wheel_angular_velocity = 5.0 # rad/s
    
    # Time span for simulation
    time_start = 0.0
    time_end = 5.0
    num_steps = 100
    time_points = np.linspace(time_start, time_end, num_steps)

    # Solve the differential equations
    # The 'args' parameter passes the additional arguments (wheel_velocities, R, L) to the function
    solution = odeint(differential_drive_kinematics, initial_pose, time_points, 
                      args=((left_wheel_angular_velocity, right_wheel_angular_velocity), wheel_radius, track_width))

    # Extract results
    x_trajectory = solution[:, 0]
    y_trajectory = solution[:, 1]
    theta_trajectory = solution[:, 2]

    # Plot the trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_trajectory, y_trajectory, label='Robot Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Differential Drive Robot Trajectory')
    plt.grid(True)
    plt.axis('equal') # Ensure equal scaling for x and y axes
    plt.legend()
    plt.show()