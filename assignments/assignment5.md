# ROS Noetic — SLAM & Navigation Assignment for Rover (RViz + Gazebo)

## 🧭 Goal
Integrate **SLAM** and **navigation** capabilities into your rover.  
Use `gmapping` for mapping, `amcl` for localization, and `RRT` for autonomous navigation in Gazebo and RViz.

---

## 📁 Base Template
Use the following repositories as starting points:

- 👉 [Assignment Base Template](https://github.com/venki666/rosnmo/tree/main/ros/rosbot_ws/src/armbot_nav)  
- 👉 [TurtleBot3 Reference](https://github.com/ROBOTIS-GIT/turtlebot3/tree/noetic) *(for navigation stack configuration & simulation)*

---

## 🧱 Task Requirements

### 1. URDF & Parameter Modifications
- Replace the URDF file with the **rover URDF/Xacro** created in the previous assignment.
- Adjust `config` parameters to match your rover dimensions (e.g., wheel radius, separation, base frame).
- Update **LiDAR range** to **8 meters**.
- Verify TF tree and sensor frames are correctly aligned.

---

### 2. SLAM Mapping with `gmapping`
- Use `gmapping` to build a **2D occupancy grid map** of any chosen Gazebo world.
- Launch the rover in Gazebo and RViz.
- Move the rover with `teleop_twist_keyboard`.
- Visualize:
  - Real-time laser scan
  - Robot pose
  - Map generation
- Save the generated map (`map.pgm` and `map.yaml`).

**Example Launch Sequence**
```bash
roslaunch armbot_nav armbot_gazebo.launch
roslaunch armbot_nav armbot_gmapping.launch
rosrun armbot_nav armbot_nav_teleop.py
rosrun map_server map_saver -f ~/map
```

---

### 3. Offline Navigation with AMCL
- Use the **AMCL localization stack** to localize the rover within the saved map.
- Configure:
  - `amcl` parameters (scan matching, pose covariance, odometry)
  - `move_base` global & local planners
  - costmaps (global + local)
- Launch navigation stack and RViz.
- Use `2D Nav Goal` to command the rover to navigate autonomously to target locations.
- Visualize:
  - Global and local costmaps
  - Path planning & robot trajectory
  - Real-time pose update and map alignment

**Example Launch Sequence**
```bash
roslaunch armbot_nav armbot_gazebo.launch
roslaunch armbot_nav armbot_offline_nav.launch map_file:=~/map.yaml
roslaunch armbot_nav rviz_nav.launch
```

---

### 4. Online Navigation with RRT
- Install and configure **RRT-based planner** (e.g., `rrt_exploration` or equivalent package).
- Perform **autonomous exploration** in the Gazebo world.
- Visualize the robot’s motion and generated exploration tree in RViz and Gazebo.
- Record:
  - RRT paths
  - Robot trajectory
  - Obstacle avoidance behavior

**Example Launch Sequence**
```bash
roslaunch armbot_nav armbot_gazebo.launch
roslaunch armbot_nav armbot_online_nav.launch
```

---

## 🎥 Demo Requirements
- **Mapping**: Show rover exploring and map being generated live in RViz.
- **AMCL Navigation**: Show the rover localizing and navigating to set goals with visible costmaps.
- **RRT Exploration**: Show the rover autonomously exploring.
- Show both **RViz** and **Gazebo** visualizations in the demonstration.

---

## 📦 Deliverables (GitHub Repo)
1. **Code**  
   - URDF/Xacro, launch files, config files for gmapping, amcl, RRT, costmaps.
2. **Document (PDF)**  
   - Clear description of parameter changes and tuning.
   - Screenshots of mapping, localization, and exploration.
   - Block diagrams or TF trees if applicable.
3. **YouTube Demo Video**  
   - Mapping demo (gmapping)
   - Navigation demo (AMCL)
   - RRT exploration demo
   - Include the link in the README.

---

## 📝 Grading (100%)
| Component                                  | Points |
|--------------------------------------------|--------|
| URDF & parameter configuration             | 20     |
| Mapping (gmapping) & map quality           | 25     |
| AMCL offline navigation                    | 25     |
| RRT online exploration                     | 20     |
| Documentation + repo organization + video | 10     |

---

## ✅ Submission Checklist
- [ ] URDF updated with rover geometry  
- [ ] LIDAR range set to **8 m**  
- [ ] Map generated and saved (`map.yaml`, `map.pgm`)  
- [ ] AMCL localization working with costmaps  
- [ ] RRT navigation installed & demonstrated  
- [ ] PDF report with screenshots & explanation  
- [ ] YouTube demo link in README  
- [ ] Repository organized and documented  

---

## 💡 Tips
- Reference **TurtleBot3 Navigation** configs for structure.
- Tune costmaps carefully to avoid poor path planning.
- Use `tf_monitor` or `rqt_tf_tree` to verify TF tree.
- Always test teleop first before launching navigation.
- Keep launch files modular (`gmapping.launch`, `amcl.launch`, etc.).

---
