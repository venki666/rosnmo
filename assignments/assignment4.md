# ROS Noetic — URDF Modeling & Diff-Drive Rover (RViz + Gazebo)

## 🧭 Goal
Model your 4-wheel rover in URDF (or Xacro), configure `diff_drive_controller`, and demonstrate teleop motion with visualization in **RViz** and **Gazebo**.

---

## 📁 Base Template
Fork/clone and work **only** inside this package:

```
rosnmo/ros/rosbot_ws/src/ros_mobile_robot
```

👉 [GitHub Template](https://github.com/venki666/rosnmo/tree/main/ros/rosbot_ws/src/ros_mobile_robot)

---

## 🧱 Robot Specifications (convert to SI units)
- **Chassis box**: L = 12" (0.3048 m), W = 8" (0.2032 m), H = 4" (0.1016 m)  
- **Wheels**: 4 wheels, diameter D = 65 mm → radius r = 0.0325 m  
- **Wheel offset**: 15 mm each side  
  - Wheel separation ≈ `0.2032 + 2 × 0.015 = 0.2332 m`  
- **Motor axis height**: 25 mm above bottom of chassis  
- **Payload**: Jetson Nano/RPi4 SBC, ROS controller, battery  
- **RPLIDAR**: top–center–center, elevated 3" (0.0762 m) above chassis top  
- **Camera**: top–center–front of chassis  

> Assumption: Wheels extend 15 mm outward from each side of the chassis.

---

## 🧰 Required Modifications

### 1. URDF/Xacro
- Modify `urdf/rover.urdf` or `urdf/rover.xacro`
- Chassis as box link (L × W × H)
- Four wheel links (cylinder geometry, radius = 0.0325 m, thickness ~ 0.02 m)
- Four revolute joints:
  - **y-offset** = ±(wheel_separation / 2)
  - **z** = +0.025 m
  - **x** = front/rear positions at ±(L/2)
- Add:
  - RPLIDAR (box) at `H/2 + 0.0762 m` above chassis
  - Camera (box) at front-top
- Expose frames: `base_link`, `base_footprint`, `odom`, `laser` (or `rplidar`), `camera_link`

---

### 2. Controllers & Config
`config/controllers.yaml` example:

```yaml
diff_drive_controller:
  type: "diff_drive_controller/DiffDriveController"
  publish_rate: 50
  left_wheel:  ["wheel_fl_joint", "wheel_rl_joint"]
  right_wheel: ["wheel_fr_joint", "wheel_rr_joint"]
  wheel_separation: 0.2332
  wheel_radius: 0.0325
  cmd_vel_timeout: 0.5
  enable_odom_tf: true
  base_frame_id: base_footprint
  odom_frame_id: odom
  publish_cmd: true
  pose_covariance_diagonal: [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]
  twist_covariance_diagonal: [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]
```

- Add `controllers.launch` to load `controller_manager` and spawn controllers.

---

### 3. Gazebo Integration
- Add `<transmission>` tags for each wheel joint.
- Load `gazebo_ros_control` and diff drive plugin or `ros_control` in Gazebo.
- Create `gazebo.launch` to spawn the robot.

---

### 4. RViz
- Display:
  - TF tree
  - Robot model
  - Laser scan
  - Camera frame
- Verify odometry visualization during teleop.

---

### 5. Teleop
- Use `teleop_twist_keyboard` to publish `/cmd_vel`.
- Verify forward/backward/in-place rotation.

---

## 🎥 Demo Requirements
- **RViz**: robot model, TF tree, odom trail during teleop  
- **Gazebo**: rover moves with `diff_drive_controller`  
- Correct wheel directions and odometry

---

## 📦 Deliverables (GitHub Repo)
1. **Code**: URDF/Xacro, launch files, controller config, Gazebo files, RViz config, README with run instructions.  
2. **Document (PDF)**:
   - All key dimensions (SI units)
   - Wheel radius, wheel separation, joint origins
   - Screenshots (RViz, Gazebo, TF tree, rostopic)
   - Notes on tuning
3. **YouTube video** (in README):
   - Launch + RViz + Gazebo
   - Teleop demo (forward, back, turn)
   - Brief explanation of frames and controller

---

## 🧪 Run Instructions (README)
```bash
# Terminal 1
roscore

# Terminal 2 (Gazebo sim + controllers)
roslaunch ros_mobile_robot gazebo.launch

# Terminal 3 (RViz)
roslaunch ros_mobile_robot rviz.launch

# Terminal 4 (teleop)
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

---

## 📝 Grading (100%)
| Component                             | Points |
|---------------------------------------|--------|
| URDF geometry, joints, frames         | 35     |
| Diff-drive configuration + odometry  | 25     |
| Gazebo + RViz + teleop demo          | 25     |
| Documentation + repo + video        | 15     |

---

## ✅ Submission Checklist
- [ ] Dimensions in meters (SI)  
- [ ] Wheel radius = `0.0325 m`  
- [ ] Wheel separation ≈ `0.2332 m`  
- [ ] Wheel joint `z` = `0.025 m`  
- [ ] RPLIDAR top center (`+0.0762 m`)  
- [ ] Camera at top center front  
- [ ] `diff_drive_controller` functional  
- [ ] RViz and Gazebo screenshots included  
- [ ] YouTube demo link in README  
- [ ] PDF uploaded before due date  

---

---
