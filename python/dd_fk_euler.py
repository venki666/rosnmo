import numpy as np
import matplotlib.pyplot as plt

class DifferentialDriveRobot:
    """
    A simple differential drive robot model.
    """
    def __init__(self, wheelbase, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        self.L = wheelbase  # Distance between the wheels
        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta
        self.history = [(self.x, self.y, self.theta)]

    def update_pose(self, v_L, v_R, dt):
        """
        Calculates and updates the robot's pose for a given time step.

        Args:
            v_L (float): Linear velocity of the left wheel.
            v_R (float): Linear velocity of the right wheel.
            dt (float): Time step for numerical integration.
        """
        # Calculate linear and angular velocities
        v = (v_R + v_L) / 2.0
        omega = (v_R - v_L) / self.L

        # Update pose using a simple Euler integration
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        # Store the new pose
        self.history.append((self.x, self.y, self.theta))

    def plot_trajectory(self):
        """Plots the robot's trajectory."""
        x_history = [h[0] for h in self.history]
        y_history = [h[1] for h in self.history]
        plt.figure()
        plt.plot(x_history, y_history, marker='o', linestyle='-', label='Robot Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Differential Drive Robot Trajectory')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()

# --- Simulation Example ---
if __name__ == "__main__":
    wheelbase = 0.5  # 50 cm
    robot = DifferentialDriveRobot(wheelbase=wheelbase)
    dt = 0.05
    
    print("Simulating different movements...")
    
    # Define a sequence of movements: (left_speed, right_speed, duration)
    movements = [
        (1.0, 1.0, 2.0),   # Move straight forward
        (0.5, 1.5, 2.0),   # Turn left
        (1.5, 0.5, 2.0),   # Turn right
        (-1.0, -1.0, 2.0)  # Move straight backward
    ]

    for v_L, v_R, duration in movements:
        print(f"Executing: v_L={v_L}, v_R={v_R}, duration={duration}s")
        num_steps = int(duration / dt)
        for _ in range(num_steps):
            robot.update_pose(v_L, v_R, dt)
            
    print("\nSimulation complete. Plotting trajectory...")
    robot.plot_trajectory()