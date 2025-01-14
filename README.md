# Homework 4: Robotics Instructions

**Contributors:** Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## Overview

This guide provides step-by-step instructions for working with the robotics package. It covers Gazebo simulation, navigation tasks, mapping and localization, and vision-based navigation. Follow these instructions to ensure a successful setup and operation.

---

## Setup Instructions

### 1. Clone the Repository

Navigate to the `src` directory of your ROS2 workspace and execute:

```bash
cd src
git clone https://github.com/ferd-bot/RL_24_Homework_4_Robotics.git .
```

**Important:**  
The above command (`git clone` with a dot `.`) works only if the target directory is empty. If it's not, you can:

1. Remove all files in the directory:
   ```bash
   rm -rf *
   ```
2. Alternatively, clone the repository without the dot and manually move the contents of the `RL_24_Homework_4_Robotics` folder into the `src` directory.

### 2. Configure and Build the Workspace

Navigate to your ROS2 workspace, clean previous builds, and rebuild:

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## Simulation and Navigation

### 1. Start the Simulation in Gazebo

Launch the Gazebo simulation environment:

```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

### 2. Autonomous Exploration and Mapping

#### Mode 1: Autonomous Exploration

Start autonomous exploration by launching:

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

This will activate the autonomous exploration node and, after a brief delay, Rviz will open with the "explore.rviz" configuration. Once the robot completes the exploration, run the waypoint-following script:

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

- **Waypoints:**  
  The waypoints are specified in the `new_goals.yaml` file located in the `config` directory.  
  To use the waypoints from point 2 of Homework, replace `new_goals.yaml` with `goals.yaml` in the script.

- **Environment:**  
  The navigation operates in the `leonardo_race_field_new.sdf` world with the `mappa_mondo_1.pgm` map.

---

### 3. Vision-based Navigation

For vision-based navigation using AMCL, follow these steps:

1. Start the Gazebo simulation:
   ```bash
   ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
   ```

2. Launch the vision-based navigation node, which includes AMCL, RViz, and SLAM-based localization:
   ```bash
   ros2 launch rl_fra2mo_description fra2mo_navigation_vision.launch.py
   ```

3. Once all nodes are active, start the task script for approaching an obstacle, detecting a marker, and returning to the initial position (for Aruco TF publish, check "Aruco TF Frame":
   ```bash
   ros2 run rl_fra2mo_description task.py
   ```

4. To visualize Aruco marker detection, open `rqt` in another terminal and subscribe to the `/aruco_detect/result` topic:

   ```bash
   rqt
   ```

---

### 4. Aruco TF Frame

To visualize transformation frames during the simulation:

1. While running the `task.py` script, publish and print the Aruco pose using:
   ```bash
   ros2 run rl_fra2mo_description aruco_tf
   ```

   This command publishes the Aruco pose in the global frame.

2. To read the tag pose relative to the camera:
   ```bash
   ros2 topic echo /aruco_detect/pose
   ```

**Note:** These topics can be monitored while running the `task.py` script during the Aruco detection task. This allows you to observe transformations and validate marker tracking dynamically within the simulation.

---

## Additional Notes

1. **Multiple Terminals:**  
   Run each command or node in a separate terminal after sourcing the workspace:
   ```bash
   source install/setup.bash
   ```

2. **Video Demonstrations:**  
   - [Autonomous Navigation with New Goals](https://youtu.be/_O3qW9CAqX8)  
   - [Vision-based Navigation](https://youtu.be/pcC26Ym-iqM)

3. **Troubleshooting:**  
   - Ensure all required nodes are running.
   - Check terminal outputs for error messages.

---

By following these instructions, you can perform advanced robotics tasks such as simulation, autonomous navigation, exploration, and vision-based control for Homework 4.
