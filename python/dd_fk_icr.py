import numpy as np
import matplotlib.pyplot as plt

class DifferentialDriveICR:
    """
    Simulates a differential drive robot using the Instantaneous Center of Rotation (ICR) method.
    """
    def __init__(self, wheelbase, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        """
        Initializes the robot with its physical parameters and initial pose.

        Args:
            wheelbase (float): The distance between the two wheels (L).
            initial_x (float): Initial x-coordinate.
            initial_y (float): Initial y-coordinate.
            initial_theta (float): Initial orientation in radians.
        """
        self.L = wheelbase
        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta
        self.history = [(self.x, self.y, self.theta)]

    def update_pose(self, v_L, v_R, dt):
        """
        Updates the robot's pose for a given time step using the ICR method.

        Args:
            v_L (float): Linear velocity of the left wheel.
            v_R (float): Linear velocity of the right wheel.
            dt (float): Time step for numerical integration.
        """
        # Special case 1: Straight-line motion (v_R == v_L)
        if np.isclose(v_R, v_L):
            # Angular velocity is zero, so ICR is at infinity.
            # Motion is a simple translation.
            v = v_L  # or v_R
            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
            # Orientation remains unchanged
            self.theta += 0.0

        # Special case 2: Rotation in place (v_R == -v_L)
        elif np.isclose(v_R, -v_L):
            # ICR is at the center of the robot.
            omega = (v_R - v_L) / self.L
            self.theta += omega * dt
            # Position remains unchanged
            self.x += 0.0
            self.y += 0.0

        # General case: Rotation around a finite ICR
        else:
            # Calculate angular velocity (omega) and radius of rotation (R)
            omega = (v_R - v_L) / self.L
            R = (self.L / 2) * ((v_R + v_L) / (v_R - v_L))

            # Find the instantaneous center of rotation (ICR)
            icr_x = self.x - R * np.sin(self.theta)
            icr_y = self.y + R * np.cos(self.theta)
            
            # Update the robot's position and orientation by rotating around the ICR
            # New pose is found by a rotation matrix on the vector from ICR to robot
            self.x = icr_x + np.cos(omega * dt) * (self.x - icr_x) - np.sin(omega * dt) * (self.y - icr_y)
            self.y = icr_y + np.sin(omega * dt) * (self.x - icr_x) + np.cos(omega * dt) * (self.y - icr_y)
            self.theta = self.theta + omega * dt

        # Store the history for plotting
        self.history.append((self.x, self.y, self.theta))

    def plot_trajectory(self):
        """Plots the robot's trajectory."""
        x_history = [h[0] for h in self.history]
        y_history = [h[1] for h in self.history]
        
        plt.figure()
        plt.plot(x_history, y_history, marker='o', linestyle='-', label='Robot Trajectory')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Differential Drive Robot Trajectory (ICR Method)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()

# --- Simulation Example ---
if __name__ == "__main__":
    # Robot parameters
    wheelbase = 0.5  # Distance between wheels (L) in meters
    robot = DifferentialDriveICR(wheelbase=wheelbase)
    
    # Simulation parameters
    dt = 0.05
    
    print("Simulating different movements...")
    
    # Define a sequence of movements
    # Each tuple is (v_L, v_R, duration)
    movements = [
        # Special Case 1: Straight line forward
        {'name': 'Straight Forward', 'v_L': 1.0, 'v_R': 1.0, 'duration': 2.0},
        # Special Case 2: Rotation in place
        {'name': 'Rotate CCW', 'v_L': -1.0, 'v_R': 1.0, 'duration': 2.0},
        # General Case: Gentle curve
        {'name': 'Gentle Curve', 'v_L': 0.5, 'v_R': 1.0, 'duration': 3.0},
        # Special Case 3: Pivoting around the left wheel (v_L = 0)
        {'name': 'Pivot on Left Wheel', 'v_L': 0.0, 'v_R': 1.0, 'duration': 2.0},
        # General Case: Moving backward on a curve
        {'name': 'Curve Backward', 'v_L': -0.8, 'v_R': -0.5, 'duration': 3.0},
    ]
    
    for move in movements:
        print(f"\n--- {move['name']} ---")
        num_steps_for_move = int(move['duration'] / dt)
        
        for _ in range(num_steps_for_move):
            robot.update_pose(move['v_L'], move['v_R'], dt)
            
    print("\nSimulation complete. Plotting trajectory...")
    robot.plot_trajectory()