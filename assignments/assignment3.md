# ROS Noetic Assignment — STM32F103C8T6 “BluePill” as a ROS Node: Twist → PWM with Feedback

## 0) Overview
Use an **STM32F1 (BluePill)** as a ROS node over serial. The microcontroller shall:

- **Subscribe** to `geometry_msgs/Twist` (use **linear.x** and **angular.z**).
- **Compute two motor PWM commands** (left & right) using differential-drive kinematics and a linear PWM map with saturation.
- **Publish** the resulting PWM commands back to the host for visualization/logging.
- Provide a complete submission: firmware + host ROS package, successful build/run screenshots, and documentation.

---

## 1) Learning Objectives
- Run a microcontroller as a ROS node using **rosserial**.
- Implement a simple **Twist → wheel PWM** mapping.
- Publish telemetry from MCU → ROS and validate with `rqt_plot`/`rostopic echo`.
- Practice robust packaging, launching, and documentation.

---

## 2) System Architecture

**On the PC (ROS Noetic):**
- `rosserial_python/serial_node.py` bridges USB serial ↔ ROS topics.
- Your host package provides a launch file and (optional) test publishers.

**On the BluePill (STM32F103C8T6):**
- `rosserial` firmware subscribes to `/cmd_vel` (`geometry_msgs/Twist`).
- MCU computes `(PWM_L, PWM_R)` and **publishes** `/motor_pwm` (custom message).
- PWM is output on two MCU pins to drive a motor driver (e.g., L298N/TB6612FNG).

---

## 3) Robot & Mapping (Math)

Let \(r\) = wheel radius (m), \(L\) = wheelbase/track (m). From `Twist`:

```math
v = \text{linear.x}, \quad \omega = \text{angular.z}
```

Inverse kinematics (body → wheels, angular wheel speed):

```math
\omega_R = \frac{1}{r}\left(v + \frac{L}{2}\,\omega\right),\quad
\omega_L = \frac{1}{r}\left(v - \frac{L}{2}\,\omega\right)
```

Map wheel angular speed to PWM (signed), with limits:

- Convert \(\omega_{L/R}\) to a **normalized duty** \(\in [-1, 1]\) via a scale \(\omega_{\max}\).
- Then to integer PWM in \([ -\text{PWM}_{\max},\ \text{PWM}_{\max} ]\).

Example:

```math
\text{duty}_{L/R} = \operatorname{clip}\!\left(\frac{\omega_{L/R}}{\omega_{\max}},\ -1,\ 1\right),\qquad
\text{PWM}_{L/R} = \left\lfloor \text{duty}_{L/R}\cdot \text{PWM}_{\max} \right\rfloor
```

**Recommended parameters:**
- `wheel_radius = 0.03 m`
- `wheel_base = 0.16 m`
- `omega_max = 60 rad/s`
- `pwm_max = 1000` (Timer top) or `255` (8-bit)

---

## 4) Hardware Notes
- **Left motor:** PWM → `PA0` (TIM2_CH1), DIR → `PB10`
- **Right motor:** PWM → `PA1` (TIM2_CH2), DIR → `PB11`
- **UART/USB:** USB-TTL adapter at 3.3V. Connect:
  - MCU `PA9` (TX1) → Adapter RX
  - MCU `PA10` (RX1) → Adapter TX
  - GND ↔ GND
- **Power:** 5–12V to motor driver; **3.3V** to BluePill.
- **Common ground required** between MCU, driver, and power supply.

---

