# AprilTag Safety Zone Monitoring System



## Overview
This ROS2 node provides real-time safety zone monitoring for robotic arms using AprilTag detection. It dynamically calculates a 3D safety zone based on detected AprilTags and verifies if the robot's end-effector stays within defined boundaries.

## Features
- 🎯 Real-time AprilTag detection with OpenCV
- 📦 Dynamic safety zone calculation
- 🤖 Robot end-effector position tracking
- 🔄 TF-based coordinate transformation
- 🚨 Safety boundary violation detection
- 📊 RViz visualization (Safety zone & markers)

## System Overview

![System Architecture](system.png)

## Dependencies
- **ROS2 Humble**
- OpenCV 4.x
- Python packages:
  ```bash
  pip install apriltag numpy
  sudo apt install ros-humble-vision-opencv ros-humble-tf2-ros \
  ros-humble-geometry-msgs ros-humble-camera-calibration

## installation
- install
  ```bash
  mkdir -p ~/safety_ws/src
  cd ~/safety_ws/src
  git clone https://github.com/Dawengit/Safe-Task-Space-for-Robot-Arm.git
  cd ~/safety_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  

## Camera Setup
- Camera Parameters(config/camera_params.yaml)
  ```bash
  camera:
    index: 2                # USB camera index
    resolution: [1280, 720] # WxH
    matrix:                 # Intrinsic parameters (pixel)
      fx: 1200              # Focal Length()
      fy: 1195

      cx: 640.5             # Principal Point
      cy: 360.5
    distortion: [0.12, -0.25, 0.001, 0.003, 0.15]

  apriltag:
    size: 0.035            # Tag side length (meters)
    family: "tag36h11"     # Tag family
    ids: [0, 1, 2, 3]      # Target tag IDs

  safety:
    xy_margin: 0.1         # Expansion margin in XY plane
    z_margin: 0.5          # Z-axis safety margin


## Robot integartion
- End-Effector Tracking
  ```bash
  1.Change the name of the robot link in the function end_effector_detector  
  2.Change the static frame parameter in order to calculate the transform coordinate for the system

  
- Coordinate Transformation  

  The node handles TF transformations between:

    - Robot base frame (base_link)

    - Camera frame (camera_frame)

    - AprilTag frames (tag_0, tag_1, etc.)

- Verify transformation chain:
    ```bash
      ros2 run tf2_ros tf2_echo base_link camera_frame
      
## Usage
- launch the system
  ```bash
  ros2 launch apriltag_detection detection.launch.py 
- RViz Monitoring

  Add these displays:

    1. Image: Topic /camera_image

    2. Marker: Topic /safety_zone

    3. TF: Frames base_link and camera_frame

- Operation Tips  

    1. Maintain visibility of at least 4 AprilTags

    2. Recommended tag arrangement: Rectangular Pattern

    3. Optimal working distance: 0.5-1.5 meters from camera(For laser cutting 3.5cm)


## Reference📌
AprilTag:  

https://github.com/AprilRobotics/apriltag

https://github.com/StanleyChueh/AprilTag_Detection/tree/master

