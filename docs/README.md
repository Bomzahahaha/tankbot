# TurtleBot Setup Guide

## Table of Contents
- [TurtleBot Setup Guide](#turtlebot-setup-guide)
  - [Table of Contents](#table-of-contents)
  - [2D Lidar Setup](#2d-lidar-setup)
  - [Multi-Machine Communication](#multi-machine-communication)
  - [Running Laser Lines Nodes for Visualization](#running-laser-lines-nodes-for-visualization)
  - [TurtleBot3 Drive Setup](#turtlebot3-drive-setup)

---

## 2D Lidar Setup

This step configures and verifies that your 2D LiDAR is connected and working properly.

1. **Check which port your LiDAR is connected to.**  
   It should appear as `/dev/ttyACM0` or `/dev/ttyACM1`.

    ```bash
    ls /dev/ttyACM* 
    ```

2. **Check that LiDAR-related ROS packages are installed:**

    ```bash
    ros2 pkg list | grep urg
    ```
    You should see something like:
    ```
    urg_c
    urg_node
    urg_node2
    urg_node_msgs
    ```

3. **Launch the LiDAR driver:**

    ```bash
    ros2 launch urg_node2 urg_node2.launch.py
    ```

4. **If you get an error**, try editing serial_port and launch again:
    ```bash
    nano ~/capstone_ws/src/urg_node2/config/params_serial.yaml
    ```

> ✅ **Check:** You should see laser scan data being published to `/scan`.
"
  
---

## Multi-Machine Communication

This allows your **PC** (for visualization) and **TurtleBot** (the robot) to communicate over the same network using ROS2.

1. **Make sure both devices are connected to the same Wi-Fi.**
2. **Check the IP address** of each device. It should be the same:

    ```bash
        ip -4 addr show | grep -oP 'inet \K[\d.]+/\d+'
    ```
    
3. **Set ROS environment variables** on both TurtleBot and PC:

    ```bash
        export ROS_DOMAIN_ID=30
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_LOCALHOST_ONLY=
    ```

---

## Running Laser Lines Nodes for Visualization
This visualizes LiDAR data as lines

1. **Run node that publishes only `/scan`:**

        ros2 run laser_lines scan_only

2. **Run node that publishes `/best_angle` and `/scan`:**

        ros2 run laser_lines best_angle_scan

---



