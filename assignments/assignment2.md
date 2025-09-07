# ROS Noetic Assignment — Differential-Drive Kinematics on `turtlesim`

## 1) Overview
You will treat `turtlesim`’s turtle as a **differential-drive robot**. Implement both directions of the kinematics and demonstrate goal-to-goal motion control:

- **(a) Forward Kinematics (FK):** subscribe to a **custom wheel-velocity message** (left/right angular velocities) → convert to `geometry_msgs/Twist` and publish on `/turtle1/cmd_vel`.
- **(b) Inverse Kinematics (IK) + Control:** accept a **goal pose** \((x, y, \theta)\). Use a proportional controller to drive the turtle to the goal by generating a `Twist`. Also compute and display the **left/right wheel velocities** produced by that `Twist` via IK (publish as your custom message).

Submit a complete C++ package, screenshots of compilation and execution, and documentation explaining your design.

---

## 2) Robot Model & Notation
- Wheel radius \( r \) (m)  
- Wheel track (axle length) \( L \) (m)  
- Left/right wheel angular velocities \( $\omega_L$, $\omega_R$ \) (rad/s)  
- Body linear velocity \( v \) (m/s)  
- Body angular velocity \( $\omega$ \) (rad/s)  

**Forward kinematics (wheel → body):**
```math
v = \frac{r}{2}(\omega_R + \omega_L), \quad
\omega = \frac{r}{L}(\omega_R - \omega_L)
```

**Inverse kinematics (body → wheel):**
```math
\omega_R = \frac{1}{r}\Bigl(v + \tfrac{L}{2}\,\omega\Bigr), \quad
\omega_L = \frac{1}{r}\Bigl(v - \tfrac{L}{2}\,\omega\Bigr)
```

Default parameters:
- `wheel_radius = 0.05`
- `wheel_base = 0.20`

---
## 3) Custom message & service
**`msg/WheelVel.msg`**
```ros
float64 omega_left
float64 omega_right
```
---

## 4) Nodes to Implement

fk_wheels_to_twist.cpp
- Sub: ~wheel_vel_in → WheelVel
- Pub: /turtle1/cmd_vel → Twist
- Implements FK.

go_to_goal_controller.cpp
- Sub: /turtle1/pose
- Pub: /turtle1/cmd_vel → Twist
- Implements IK.
- Pub: ~wheel_vel_dbg → WheelVel (from IK)
- Implements proportional controller.

---
## 5) Controller Details

Let the current pose be \((x, y, \theta)\), and the goal pose be \((x_g, y_g, \theta_g)\):

```math
\rho = \sqrt{(x_g-x)^2 + (y_g-y)^2}, \quad
\alpha = \arctan2(y_g-y, x_g-x) - \theta, \quad
\beta = \theta_g - \theta - \alpha
```

**Control law:**
```math
v = K_\rho \,\rho, \quad
\omega = K_\alpha \,\alpha + K_\beta \,\beta
```

- \( v \): linear velocity (m/s)  
- \( $\omega$ \): angular velocity (rad/s)  
- \( $K_\rho, K_\alpha, K_\beta$ \): proportional controller gains  

**Recommended values:**  
- `K_rho = 1.5`  
- `K_alpha = 4.0`  
- `K_beta = -1.0`  

**Stopping criteria:**  
- Stop motion when
- \(  $\rho$ < $pos_{tol}$ \)
-  \(\|($\theta$ - $\theta_g$)\| < $\theta_{tol}$\)

  ---


