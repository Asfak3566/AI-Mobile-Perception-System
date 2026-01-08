# Mobile Mast Project

## Overview
The Mobile Mast project is a multi-sensor perception and tracking system designed for high-altitude surveillance using a mobile mast platform. It utilizes a combination of **RoboSense RS16 LiDAR** and **4x Axis Bullet Cameras** to detect, localize, and track objects in a global 3D frame.

The system is distributed across three **NVIDIA Orin** devices to handle the heavy computational load of real-time point cloud processing, AI-based object detection (YOLO11), and global multi-object tracking.

---

## System Architecture

The project follows a modular architecture where processing is split by stage across the Orin devices:


### Detailed Breakdown

1.  **Orin 1 (Drivers & Pre-processing)**:
    *   Runs the LiDAR driver and filters raw points.
    *   Performs background subtraction and clustering.
    *   Hosts the camera streaming drivers.
2.  **Orin 2 (AI Perception)**:
    *   Handles image rectification using NVIDIA VPI.
    *   Runs YOLO11 inference for person/vehicle detection.
    *   Projects 2D detections into 3D using LiDAR-camera fusion.
3.  **Orin 3 (State Estimation)**:
    *   Runs EKF-based trackers for each sensor stream.
    *   Fuses local tracks into a single global state.
    *   Publishes markers for visualization.

---

## Prerequisites

- **NVIDIA Jetson Orin (AGX / NX)**
- **ROS 2 Humble**
- **NVIDIA VPI 2.x/3.x**
- **Ultralytics (YOLO11)**
- **OpenCV & NumPy**
- **linuxptp** (for time synchronization)

---

## Installation

1.  **Clone the workspace**:
    ```bash
    git clone <repository_url> mobile_mast_ws
    cd mobile_mast_ws
    ```

2.  **Install dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the project**:
    ```bash
    colcon build --symlink-install
    ```

---

## Usage

### 1. Time Synchronization (PTP)
The system requires microsecond-level synchronization across all Orin devices. Run the setup script on each device:
```bash
sudo ./setup_ptp.sh
```

### 2. Launching the System
To launch the entire pipeline from a master controller:
```bash
ros2 launch mobile_mast mobile_mast_all.launch.py
```

To launch specific stages (on individual Orins):
- **Orin 1**: `ros2 launch mobile_mast mobile_mast_orin1.launch.py`
- **Orin 2**: `ros2 launch mobile_mast mobile_mast_orin2.launch.py`
- **Orin 3**: `ros2 launch mobile_mast mobile_mast_orin3.launch.py`
---

## Configuration
Config files for LiDAR clustering, camera intrinsics/extrinsics, and trackers are located in:
`src/mobile_mast/config/`

---

## Recording & Data Analysis
To record data for offline analysis, use the `record` argument:
```bash
ros2 launch mobile_mast mobile_mast_all.launch.py record:=True
```
Bag files are timestamped and stored in the package directory.
