# Assignment 1  
**Course:** [ROS in Motion]  
**Topic:** DC Motor Simulator and Controller (ROS-based Implementation)  

---

## Objective
The goal of this assignment is to develop a **simulator** and a **controller** to regulate the speed of a DC motor. The desired motor speed is provided by the user (via command-line input) within the range **0 – 300 rpm**.  

The assignment requires the implementation of two ROS nodes:  
1. **simulator.cpp** – simulates the electrical and mechanical dynamics of the DC motor.  
2. **controller.cpp** – implements a proportional speed controller.  

The simulator and controller must communicate using ROS topics.  

---

## Requirements
- Desired speed (float) provided as **command-line input** and published as a ROS topic.  
- The **controller node** uses proportional gain (`Kp`) to control motor speed.  
- Integration time step: **0.01 sec**.  
- Follow the simulator–controller framework discussed in class.  
- Submission must include:  
  - ROS package.  
  - Screenshots of successful compilation.  
  - Execution steps.  
  - `rostopic info` outputs.  
  - `rqt_plot` plots showing desired speed vs. controller speed.  

---

## DC Motor Parameters
| Parameter | Symbol | Value | Unit |
|-----------|---------|-------|------|
| Resistance | R | 1 | Ω |
| Inductance | L | 0.5 | H |
| Inertia | J | 0.01 | kg·m² |
| Constant of Proportionality | K | 1 | V/(rad/s) |
| Damping Coefficient | b | 0.1 | N·m/(rad/s) |
| Electromotive Force Constant | Ke | 0.01 | V/(rad/s) |
| Motor Torque Constant | Kt | 0.01 | N·m/A |

---

## Motor Dynamics (Simplified)

### Electrical Dynamics
\[
\frac{dI}{dt} = \frac{V - I \cdot R - K_e \cdot \omega}{L}
\]

### Mechanical Dynamics
\[
\frac{d\omega}{dt} = \frac{K_t \cdot I - b \cdot \omega}{J}
\]

Where:  
- \( I \) = current (A)  
- \( V \) = applied voltage (V)  
- \( \omega \) = angular speed (rad/s)  

---

## Implementation Structure

### ROS Nodes
1. **simulator.cpp**  
   - Subscribes: control voltage `V` (from controller node).  
   - Publishes: actual motor speed `ω` (rad/s).  
   - Uses discrete integration (`dt = 0.01 s`) to update motor dynamics.  

2. **controller.cpp**  
   - Subscribes: actual motor speed from simulator.  
   - Publishes: control voltage `V`.  
   - Implements proportional control law:  
     \[
     V = K_p \cdot ( \omega_{desired} - \omega )
     \]  

### Workflow
1. User inputs desired speed (0–300 rpm).  
2. Controller computes voltage based on proportional gain.  
3. Simulator updates motor states using dynamics equations.  
4. Loop repeats at 100 Hz (dt = 0.01s).  
5. Results visualized using `rqt_plot`.  

---

## Expected Deliverables
1. ROS package including:  
   - `simulator.cpp`  
   - `controller.cpp`  
   - `CMakeLists.txt` and `package.xml`  

2. Documentation including:  
   - **Compilation screenshot** (`catkin_make` or `colcon build`).  
   - **Execution steps** (`rosrun` or `roslaunch`).  
   - **rostopic info** output showing topics exchanged.  
   - **rqt_plot** screenshot showing:  
     - Desired speed vs. simulated motor speed.  

---

## Notes
- Use **Proportional Gain only** (no integral/derivative terms).  
- Ensure unit conversion between **rpm ↔ rad/s** when necessary.  
- Example:  
  \[
  \omega_{rad/s} = \omega_{rpm} \times \frac{2\pi}{60}
  \]  

---

## Submission
- Submit complete ROS package as a compressed file (`.zip` or `.tar.gz`).  
- Include screenshots and plots in a `report.pdf` file.  
- Deadline: [Sept 1st 2025].  

---

