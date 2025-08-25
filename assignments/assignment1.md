# Assignment 1  
**Course:** [ROS in Motion]  
**Topic:** DC Motor Simulator and Controller (ROS-based Implementation)  

## Objective
The goal of this assignment is to **develop a simulator and controller** to control the speed of a DC motor.  
- The **desired speed** is entered by the user (range: **0 – 300 rpm**).  
- Two nodes must be implemented:  
  1. **simulator.cpp** – simulates the dynamics of the motor.  
  2. **controller.cpp** – implements a proportional gain speed controller.  
- The **required speed (float)** is supplied/published as a command line input.  
- The **integration time step** should be **0.01 sec**.  
- The implementation should follow the simulator and controller example discussed in class.  

The final submission must include:  
- Complete ROS package.  
- Screenshots of successful compilation.  
- Execution steps.  
- `rostopic info` output.  
- `rqt_plot` graphs showing both desired speed and controller output speed.  

---

## DC Motor Parameters
| Parameter | Symbol | Value | Unit |
|-----------|---------|-------|------|
| Resistance | R | 1 | Ohm |
| Inductance | L | 0.5 | H |
| Inertia | J | 0.01 | kg·m² |
| Constant of Proportionality | K | 1 | V/(rad/s) |
| Damping Coefficient | b | 0.1 | N·m/(rad/s) |
| Electromotive Force Constant | Ke | 0.01 | V/(rad/s) |
| Motor Torque Constant | Kt | 0.01 | N·m/A |

---

## Motor Dynamics (Simplified)

### Electrical Part:
$$
\frac{dI}{dt} = \frac{V - I \cdot R - K_b \cdot \omega}{L}
$$

### Mechanical Part:
$$
\frac{d\omega}{dt} = \frac{K_t \cdot I - B \cdot \omega}{J}
$$

Where:  
- \( I \) = Armature current (A)  
- \( V \) = Input voltage (V)  
- \( \omega \) = Angular velocity (rad/s)  
- \( R \) = Resistance (Ohm)  
- \( L \) = Inductance (H)  
- \( J \) = Inertia (kg·m²)  
- \( B \) = Damping coefficient (N·m/(rad/s))  
- \( K_b \) = Back EMF constant (V/(rad/s))  
- \( K_t \) = Torque constant (N·m/A)  

---

## Controller Implementation
- Use a **proportional controller** only:  
$$  V = K_p \cdot \( \omega_{desired} - \omega_{actual} \) $$
- \( \omega_{desired} \) = Desired motor speed.  
- \( \omega_{actual} \) = Measured motor speed from the simulator node.  
- \( K_p \) = Proportional gain (tunable).  

---

## Expected Deliverables
1. **ROS Package** containing:  
   - `simulator.cpp` (motor dynamics simulator).  
   - `controller.cpp` (proportional speed controller).  
2. **Screenshots**:  
   - Successful package compilation.  
   - Execution steps in terminal.  
   - `rostopic info` showing topic communication.  
   - `rqt_plot` displaying desired vs actual speed.  

---

## Notes
- Use **0.01 sec integration time** for numerical updates.  
- Ensure proper **ROS message publishing/subscribing** between nodes.  
- The **controller node publishes voltage command** to the simulator node.  
- The **simulator node publishes speed feedback** back to the controller node.  

---


## Submission
- Submit complete ROS package as a compressed file (`.zip` or `.tar.gz`).  
- Include screenshots and plots in a `report.pdf` file.  
- Deadline: [Sept 1st 2025].  

---

