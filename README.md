# TRSA Lab 1 – ROS 2 Image Processing and Object Detection

## Overview
ROS 2 package implementing a complete image-processing pipeline for TRSA Lab 1 at FCT NOVA.  
It provides two main operation modes:

### Mode 1 — Edge Detection
Displays the video feed with enhanced edge detection using grayscale conversion, Gaussian blur, and Canny filtering.

### Mode 2 — Object Detection
Performs color-based object detection (green phone case) using HSV segmentation and bounding boxes.

Features include:
- Frame publishing from video files
- Image processing and feature extraction
- Camera calibration and rectification support
- Real-time visualization through OpenCV or RViz2

---

## Node Structure

| Node | Function | Publishes | Subscribes |
|------|-----------|------------|-------------|
| `camera_driver` | Reads frames from a video and publishes `/camera/image_raw` | `/camera/image_raw` | — |
| `camera_driver2` | Alternative video source for object detection | `/camera2/image_raw` | — |
| `image_convert` | Converts to grayscale, applies blur, detects edges | `/camera/image_processed` | `/camera/image_rect` |
| `image_reader` | Displays raw and processed images | — | `/camera/image_raw`, `/camera/image_processed` |
| `object_detection` | Detects green phone case and draws bounding boxes | `/camera2/object_detected` | `/camera2/image_raw` |

---

## Dependencies

- ROS 2 Humble
- rclpy
- cv_bridge
- opencv-python
- numpy

Install with:
```bash
sudo apt install ros-humble-cv-bridge python3-opencv python3-numpy

## Build
From the ROS 2 workspace root:

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

Execution
Mode 1 — Edge Detection

Start the camera driver (publishes video frames):

ros2 run trsa_lab1 camera_driver


Run the image processing node (grayscale + Canny edges):

ros2 run trsa_lab1 image_convert


View both raw and processed video streams:

ros2 run trsa_lab1 image_reader


This will show two OpenCV windows: one with the original video and another with edge detection results.

Mode 2 — Object Detection (Green Phone Case)

Start the secondary camera driver for the new video:

ros2 run trsa_lab1 camera_driver2


Launch the object detection node:

ros2 run trsa_lab1 object_detection


Display the annotated output:

ros2 run trsa_lab1 image_reader --ros-args -r /camera/image_raw:=/camera2/object_detected


This will display the video feed where the green phone case is automatically detected and highlighted with a bounding box.

Project Layout
trsa_lab1/
 ├── calibration/
 ├── video/
 ├── trsa_lab1/
 │    ├── camera_driver.py
 │    ├── camera_driver2.py
 │    ├── image_convert.py
 │    ├── image_reader.py
 │    ├── object_detection.py
 │    └── ...
 ├── package.xml
 └── setup.py

Notes

Tested on ROS 2 Humble (Ubuntu 22.04 / WSL2) with OpenCV ≥ 4.5

HSV thresholds in object_detection.py may require fine-tuning depending on lighting conditions and object color

Input videos must be stored in video/ and referenced within the corresponding camera driver node
