
# Homework 4: Robotics Instructions

**Contributors:** Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## Overview

This guide provides step-by-step instructions for working with the robotics package, including Gazebo simulation, navigation tasks, mapping and localization, and vision-based navigation. Follow these guidelines to ensure successful setup and operation.

---

## **Setup Instructions**

### 1. Clone the Repository

To clone the repository, navigate to the `src` directory of your ROS2 workspace and execute:

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

Navigate to your ROS2 workspace and clean previous builds:

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## **Simulation and Navigation**

### 1. Start the Simulation in Gazebo

Launch the Gazebo simulation environment using:

```bash
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

---

### 2. Autonomous Exploration and Mapping

#### Execution Mode 1: Autonomous Exploration

To start autonomous exploration, launch the following:

```bash
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```
To view the robot remember to open Rviz, open "config" and go to "/user/ros2_ws/src/rl_fra2mo_description/rviz_conf/explore.rviz". Once the robot completes the exploration, you can run the waypoint-following script:

```bash
ros2 run rl_fra2mo_description follow_waypoints.py
```

The waypoints are specified in the `new_goals.yaml` file located in the `config` directory. To use the waypoints from Homework 2 instead, replace `new_goals.yaml` with `goals.yaml` in the script.

The navigation operates in a new world, `leonardo_race_field_new.sdf`, with an updated map, `mappa_mondo_1.pgm`.

---

### 3. Vision-based Navigation

For vision-based navigation with AMCL, follow these steps:

1. Start the Gazebo simulation:
   ```bash
   ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
   ```
2. Launch SLAM-based localization:
   ```bash
   ros2 launch rl_fra2mo_description fra2mo_amcl.launch.py
   ```
3. Start the vision-based navigation node, which includes AMCL and RViz:
   ```bash
   ros2 launch rl_fra2mo_description fra2mo_navigation_vision.launch.py
   ```
4. Run the Aruco detection script to search for markers:
   ```bash
   ros2 run rl_fra2mo_description task.py
   ```
To view the robot remember to open Rviz, open "config" and go to "/user/ros2_ws/src/rl_fra2mo_description/rviz_conf/navigation.rviz".
To visualize the Aruco marker's detection, open `rqt` in another terminal and subscribe to the topic `/aruco_detect/result`:

```bash
rqt
```

---

### 4. Visualizing TF Frames

To visualize transformation frames during the simulation, you can use the following commands:

1. **Echo static transformations**:  
   View static transformation frames in real-time:
   ```bash
   ros2 topic echo /tf_static
   ```

2. **Inspect specific frames**:  
   For example, to check the transformation between `camera_link` and `aruco_marker_frame`, run:
   ```bash
   ros2 run tf2_ros tf2_echo camera_link aruco_marker_frame
   ```

**Note**: These topics can be monitored while running the `task.py` script during the Aruco detection task. This allows you to observe the transformations and validate marker tracking dynamically within the simulation.

---

## **Additional Notes**

1. **Multiple Terminals:**  
   Run each command or node in a separate terminal after sourcing the workspace:
   ```bash
   source install/setup.bash
   ```

2. **Video Demonstrations:**  
   - [Autonomous Navigation with New Goals](https://youtu.be/_O3qW9CAqX8)  
   - [Vision-based Navigation](https://youtu.be/pcC26Ym-iqM)

3. **Troubleshooting:**  
   If the simulation or tracking fails:
   - Ensure all required nodes are running.
   - Check terminal outputs for error messages.

---

With these instructions, you are equipped to perform advanced robotics tasks such as simulation, autonomous navigation, exploration, and vision-based control for Homework 4.
