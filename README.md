# Mobile Mast Perception System

## Overview
The Mobile Mast Perception System is a distributed, multi-sensor tracking platform designed for high-altitude traffic monitoring and infrastructure perception. It integrates a **RoboSense RS16 LiDAR** and **4x Axis Bullet Cameras** (expandable to PTZ and Panorama) to detect and localize objects in a global 3D coordinate system.

The system utilizes a cluster of three **NVIDIA Jetson Orin** modules to achieve real-time performance through hardware-accelerated deep learning (YOLOv11 via TensorRT), image processing (NVIDIA VPI), and sub-microsecond time synchronization (IEEE 1588 PTP).

---

## System Architecture

The project follows a distributed ROS 2 architecture where processing is offloaded across three Orin modules:

1.  **Orin 1 (Sensing & Sync)**:
    *   IP: `192.168.6.100`
    *   Handles high-bandwidth sensor drivers (LiDAR, Cameras).
    *   Runs initial point cloud preprocessing and background subtraction.
    *   Acts as the PTP synchronization slave to the GPS-disciplined LiDAR clock.
2.  **Orin 2 (AI Perception)**:
    *   IP: `192.168.6.101`
    *   Performs VPI-accelerated image undistortion.
    *   Runs YOLOv11 inference via a TensorRT FP16 engine (~38ms latency).
    *   Projects 2D detections into 3D space using the camera-LiDAR fusion model.
3.  **Orin 3 (Global Tracking & Fusion)**:
    *   IP: `192.168.6.102`
    *   Runs a local tracker for each camera stream.
    *   Consolidates detections into a unified Global Track List using the Hungarian algorithm and Euclidean distance metrics.
    *   Publishes visualization markers for real-time monitoring.

---

## Key Features

- **Mission Control GUI**: A centralized PyQt5-based dashboard (`qt_calibrator.py`) for managing the entire cluster, monitoring node health, and fine-tuning manual extrinsic calibrations.
- **Latency-Optimized Pipeline**: End-to-end perception latency of ~90ms, enabling stable 10Hz operation.
- **PTP Synchronization**: Anchors all visual and spatial data to a unified GPS-disciplined time-base with <1µs jitter.
- **High-Throughput Transport**: Uses specialized `image_transport` republishing to keep high-bandwidth raw data local to the inference modules.

---

## Prerequisites

- **NVIDIA Jetson Orin (AGX / NX)**
- **ROS 2 Humble**
- **NVIDIA VPI 2.x/3.x**
- **Ultralytics (YOLO11)**
- **TensorRT 8.6+**
- **linuxptp** (for PTP synchronization)

---

## Installation & Setup

1.  **Setup Network**: Configure your local machine and Orins on the `192.168.6.x` subnet.
2.  **Clone & Build**:
    ```bash
    mkdir -p ~/mobile_mast_ws/src
    cd ~/mobile_mast_ws/src
    git clone https://github.com/Asfak3566/AI-Mobile-Perception-System.git .
    cd ..
    colcon build --symlink-install --packages-select mobile_mast
    ```
3.  **Time Sync**: Run the PTP configuration script on each Orin:
    ```bash
    sudo ./setup_ptp.sh
    ```

---

## Usage

### 1. Launching the System
Launch the specialized nodes on each Orin:
- **Orin 1**: `ros2 launch mobile_mast mobile_mast_orin1.launch.py`
- **Orin 2**: `ros2 launch mobile_mast mobile_mast_orin2.launch.py`
- **Orin 3**: `ros2 launch mobile_mast mobile_mast_orin3.launch.py`

Alternatively, use the master controller launch file (if configured):
```bash
ros2 launch mobile_mast mobile_mast_all.launch.py
```

### 2. Mission Control
Start the management GUI from your base station:
```bash
python3 src/mobile_mast/mobile_mast/qt_calibrator.py
```

