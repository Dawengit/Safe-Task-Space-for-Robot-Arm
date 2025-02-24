# AprilTag Safety Zone Monitoring System

![System Architecture](https://example.com/safety_system_arch.png) <!-- Replace with actual diagram -->

## Overview
This ROS2 node provides real-time safety zone monitoring for robotic arms using AprilTag detection. It dynamically calculates a 3D safety zone based on detected AprilTags and verifies if the robot's end-effector stays within defined boundaries.

## Features
- 🎯 Real-time AprilTag detection with OpenCV
- 📦 Dynamic safety zone calculation
- 🤖 Robot end-effector position tracking
- 🔄 TF-based coordinate transformation
- 🚨 Safety boundary violation detection
- 📊 RViz visualization (Safety zone & markers)

## Dependencies
- **ROS2 Humble**
- OpenCV 4.x
- Python packages:
  ```bash
  pip install apriltag numpy


## Reference📌
AprilTag:https://github.com/AprilRobotics/apriltag
