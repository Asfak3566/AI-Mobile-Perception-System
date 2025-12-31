# ...existing code...
import sys
import os
import requests
import threading
import numpy as np
import yaml
import traceback
from scipy.spatial.transform import Rotation as R


# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2


# PyQt5 Imports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QLabel, QPushButton, QSlider,
                            QGroupBox, QDoubleSpinBox, QScrollArea, QFrame,
                            QSizePolicy, QComboBox, QFormLayout, QFileDialog, QMessageBox, QListWidget, QTabWidget, QAction)
import subprocess
import signal
from dataclasses import dataclass
from typing import List, Optional
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QFont


import cv2


# --- CONFIGURATION ---
CAMERA_IPS = {
   "Camera 1 (Bullet)": "192.168.6.41",
   "Camera 2 (Bullet)": "192.168.6.42",
   "Camera 3 (Bullet)": "192.168.6.43",
   "Camera 4 (Bullet)": "192.168.6.44",
   "Fisheye": "192.168.6.47",  # M4317-PLVE (Panorama)
   "PTZ": "192.168.6.45"       # Q6135-LE (PTZ 1)
}


CAMERA_TOPICS = {
   "Camera 1 (Bullet)": "/mast/orin1/bullet1/image_raw/compressed",
   "Camera 2 (Bullet)": "/mast/orin1/bullet2/image_raw/compressed",
   "Camera 3 (Bullet)": "/mast/orin1/bullet3/image_raw/compressed",
   "Camera 4 (Bullet)": "/mast/orin1/bullet4/image_raw/compressed",
   "Fisheye": "/mast/orin1/fisheye/raw_data/compressed",
   "PTZ": "/mast/orin1/ptz/raw_data/compressed"
}


LIDAR_TOPIC = "/rslidar_points"
CAM_USER = "root"
CAM_PASS = "pass"


# --- STYLE SHEET (PRO LEVEL MATERIAL DESIGN 3) ---
LIGHT_STYLESHEET = """
/* Global Reset */
QMainWindow { background-color: #f8f9fa; }
QWidget { color: #202124; font-family: 'Segoe UI', 'Inter', sans-serif; font-size: 14px; }


/* Tabs */
QTabWidget::pane { border: 1px solid #e0e0e0; background: #ffffff; border-radius: 12px; margin-top: -1px; }
QTabBar::tab { background: transparent; padding: 12px 24px; margin-right: 4px; color: #5f6368; font-weight: 600; border-bottom: 3px solid transparent; }
QTabBar::tab:selected { color: #1a73e8; border-bottom: 3px solid #1a73e8; }
QTabBar::tab:hover:!selected { color: #1a73e8; background: #f1f3f4; border-radius: 4px 4px 0 0; }


/* Cards & Groups */
QGroupBox {
   background-color: #ffffff;
   border: 1px solid #e0e0e0;
   border-radius: 12px;
   margin-top: 24px;
   padding-top: 24px;
   font-weight: 700;
   font-size: 15px;
}
QGroupBox::title {
   subcontrol-origin: margin;
   left: 16px;
   padding: 0 8px;
   color: #1a73e8;
   background-color: #ffffff;
}


/* Buttons */
QPushButton {
   background-color: #ffffff;
   border: 1px solid #dadce0;
   border-radius: 8px;
   padding: 10px 20px;
   font-weight: 600;
   color: #3c4043;
   font-size: 14px;
}
QPushButton:hover { background-color: #f8f9fa; border-color: #1a73e8; color: #1a73e8; }
QPushButton:pressed { background-color: #e8f0fe; border-color: #1a73e8; }


/* Primary Action Buttons */
QPushButton#btn_save, QPushButton#btn_primary {
   background-color: #1a73e8;
   color: #ffffff;
   border: none;
   font-weight: 700;
   font-size: 15px;
   padding: 12px 24px;
}
QPushButton#btn_save:hover, QPushButton#btn_primary:hover { background-color: #1765cc; }


/* Inputs */
QDoubleSpinBox, QComboBox, QLineEdit {
   background-color: #ffffff;
   border: 1px solid #dadce0;
   border-radius: 6px;
   padding: 8px 12px;
   color: #202124;
   font-size: 14px;
   font-weight: 500;
}
QDoubleSpinBox:focus, QComboBox:focus, QLineEdit:focus { border: 2px solid #1a73e8; padding: 7px 11px; }
QComboBox::drop-down { border: none; width: 24px; }


/* Sliders */
QSlider::groove:horizontal { border: 1px solid #dadce0; background: #e8eaed; height: 6px; border-radius: 3px; }
QSlider::handle:horizontal { background: #ffffff; border: 2px solid #1a73e8; width: 18px; height: 18px; margin: -7px 0; border-radius: 9px; }
QSlider::handle:horizontal:hover { background: #1a73e8; border-color: #1a73e8; }


/* Scroll Area */
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical { border: none; background: #f1f3f4; width: 10px; border-radius: 5px; }
QScrollBar::handle:vertical { background: #bdc1c6; border-radius: 5px; }
QScrollBar::handle:vertical:hover { background: #9aa0a6; }
"""
# ...existing code...
def run_local(cmd: list) -> tuple:
   try:
       print(f"DEBUG: Running {cmd}")
       p = subprocess.run(cmd, capture_output=True, text=True)
       out = (p.stdout + "\n" + p.stderr).strip()
       return p.returncode, out
   except Exception as e:
       return -1, str(e)


def run_ssh(user: str, ip: str, remote_cmd: str) -> tuple:
   # Use BatchMode to fail instead of prompting for password
   return run_local(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no", f"{user}@{ip}", remote_cmd])


class SshWorker(QThread):
   finished = pyqtSignal(int, str)
  
   def __init__(self, user, ip, cmd):
       super().__init__()
       self.user = user
       self.ip = ip
       self.cmd = cmd
      
   def run(self):
       code, out = run_ssh(self.user, self.ip, self.cmd)
       self.finished.emit(code, out)


@dataclass
class Component:
   name: str
   start_cmd: str
   stop_cmd: str
   check_node: Optional[str] = None
   check_topic: Optional[str] = None


@dataclass
class OrinConfig:
   name: str
   ip: str
   user: str
   components: List[Component]
   launch_cmd: str
   tmux_session: str
   vnc_cmd: Optional[str] = None


class ComponentRow(QWidget):
   def __init__(self, orin: OrinConfig, comp: Component):
       super().__init__()
       self.orin = orin
       self.comp = comp


       self.status = QLabel("—")
       self.status.setFixedWidth(100)


       btn_start = QPushButton("▶ Start")
       btn_stop  = QPushButton("■ Stop")
       btn_check = QPushButton("🔍 Check")
      
       # Style buttons
       btn_start.setStyleSheet("background-color: #1b5e20; color: white;")
       btn_stop.setStyleSheet("background-color: #b71c1c; color: white;")
       btn_check.setStyleSheet("background-color: #0d47a1; color: white;")


       btn_start.clicked.connect(self.start)
       btn_stop.clicked.connect(self.stop)
       btn_check.clicked.connect(self.check)


       layout = QHBoxLayout()
       layout.setContentsMargins(0, 2, 0, 2)
       lbl_name = QLabel(comp.name)
       lbl_name.setFixedWidth(200)
       layout.addWidget(lbl_name)
       layout.addWidget(btn_start)
       layout.addWidget(btn_stop)
       layout.addWidget(btn_check)
       layout.addWidget(self.status)
       self.setLayout(layout)


   def start(self):
       self.status.setText("Starting...")
       QApplication.processEvents()
       code, out = run_ssh(self.orin.user, self.orin.ip, self.comp.start_cmd)
       if code == 0:
           self.status.setText("Started ✅")
           self.status.setStyleSheet("color: green; font-weight: bold;")
       else:
           self.status.setText("Failed ❌")
           self.status.setStyleSheet("color: red; font-weight: bold;")
           QMessageBox.warning(self, "Start failed", out[:2000])


   def stop(self):
       self.status.setText("Stopping...")
       QApplication.processEvents()
       code, out = run_ssh(self.orin.user, self.orin.ip, self.comp.stop_cmd)
       if code == 0:
           self.status.setText("Stopped ✅")
           self.status.setStyleSheet("color: orange; font-weight: bold;")
       else:
           self.status.setText("Failed ❌")
           self.status.setStyleSheet("color: red; font-weight: bold;")
           QMessageBox.warning(self, "Stop failed", out[:2000])


   def check(self):
       self.status.setText("Checking...")
       QApplication.processEvents()
       checks = []
       if self.comp.check_node:
           checks.append(("node", f"ros2 node list | grep -F '{self.comp.check_node}'"))
       if self.comp.check_topic:
           checks.append(("topic", f"ros2 topic list | grep -F '{self.comp.check_topic}'"))


       if not checks:
           self.status.setText("No check set")
           return


       ok = True
       details = []
       for kind, cmd in checks:
           code, out = run_ssh(self.orin.user, self.orin.ip, cmd)
           if code == 0 and out.strip():
               details.append(f"{kind}: OK")
           else:
               ok = False
               details.append(f"{kind}: NOT FOUND")


       if ok:
           self.status.setText("OK ✅")
           self.status.setStyleSheet("color: green; font-weight: bold;")
       else:
           self.status.setText("Issue ⚠️")
           self.status.setStyleSheet("color: red; font-weight: bold;")
           QMessageBox.information(self, "Check result", "\n".join(details))


class OrinPanel(QGroupBox):
   def __init__(self, cfg: OrinConfig):
       super().__init__(cfg.name)
       self.cfg = cfg


       top = QHBoxLayout()
       top.addWidget(QLabel(f"IP: {cfg.ip}"))
       top.addStretch(1)
      
       if cfg.vnc_cmd:
           btn_vnc = QPushButton("🖥 Open VNC")
           btn_vnc.clicked.connect(lambda: subprocess.Popen(cfg.vnc_cmd, shell=True))
           top.addWidget(btn_vnc)


       btn_term = QPushButton("🐚 Open Terminal")
       btn_term.setCursor(Qt.PointingHandCursor)
       btn_term.setStyleSheet("background-color: #e8f0fe; color: #1a73e8; font-weight: bold; border: 1px solid #1a73e8; padding: 6px 12px; border-radius: 4px;")
       btn_term.clicked.connect(self.open_terminal)
       top.addWidget(btn_term)

       # Start/Stop Launch Buttons
       btn_launch_start = QPushButton("▶ Start Launch")
       btn_launch_stop = QPushButton("■ Stop Launch")
       btn_launch_start.setStyleSheet("background-color: #1a73e8; color: white; font-weight: 700;")
       btn_launch_stop.setStyleSheet("background-color: #5f6368; color: white; font-weight: 700;")
       btn_launch_start.clicked.connect(self.start_launch)
       btn_launch_stop.clicked.connect(self.stop_launch)
       top.addWidget(btn_launch_start)
       top.addWidget(btn_launch_stop)

       # Quick Commands Area
       sys_box = QGroupBox("Quick Commands")
       sys_box.setStyleSheet("QGroupBox { border: 1px solid #e0e0e0; border-radius: 8px; margin-top: 12px; padding-top: 10px; } QGroupBox::title { color: #1a73e8; font-weight: bold; }")
       sys_l = QHBoxLayout()
       
       self.cmd_input = QComboBox()
       self.cmd_input.setEditable(False)
       self.cmd_input.addItems([
           "Build Workspace (colcon build ...)",
           "Check Disk Space (df -h ~)",
           "Check Uptime (uptime)",
           "List ROS2 Nodes (ros2 node list | head -n 20)",
           "List ROS2 Topics (ros2 topic list | head -n 20)",
           "Check Time Sync (chronyc tracking)",
           "System Sensors (sensors)"
       ])
       self.cmd_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
       self.cmd_input.setStyleSheet("padding: 6px;")
       
       self.btn_run = QPushButton("Run")
       self.btn_run.setFixedWidth(120)
       self.btn_run.setCursor(Qt.PointingHandCursor)
       self.btn_run.setStyleSheet("background-color: #1a73e8; color: white; font-weight: bold; border-radius: 4px; padding: 6px;")
       self.btn_run.clicked.connect(self.run_quick_cmd)
       
       sys_l.addWidget(self.cmd_input)
       sys_l.addWidget(self.btn_run)
       sys_box.setLayout(sys_l)


       # Components List
       scroll = QScrollArea()
       scroll.setWidgetResizable(True)
       scroll.setStyleSheet("border: 1px solid #eee;")
      
       container = QWidget()
       v = QVBoxLayout(container)
       v.setSpacing(5)
       for comp in cfg.components:
           v.addWidget(ComponentRow(cfg, comp))
       v.addStretch(1)
       scroll.setWidget(container)


       layout = QVBoxLayout()
       layout.addLayout(top)
       layout.addWidget(sys_box)
       layout.addWidget(scroll)
       self.setLayout(layout)


   def open_terminal(self):
       # Launches gnome-terminal with SSH session
       cmd = f"gnome-terminal --title='SSH: {self.cfg.name}' -- ssh {self.cfg.user}@{self.cfg.ip}"
       subprocess.Popen(cmd, shell=True)

   def start_launch(self):
       sess = self.cfg.tmux_session
       launch_cmd = self.cfg.launch_cmd
       # Remote command to start launch in tmux session
       remote_cmd = (
           f"tmux has-session -t {sess} 2>/dev/null && echo 'already running' || "
           f"tmux new -d -s {sess} \"bash -lc 'source /opt/ros/humble/setup.bash; "
           f"source ~/mobile_mast_ws/install/setup.bash; {launch_cmd}'\""
       )
       self._run(remote_cmd, f"{self.cfg.name} :: Start Launch")

   def stop_launch(self):
       sess = self.cfg.tmux_session
       # Remote command to stop launch by sending Ctrl-C to tmux session
       remote_cmd = f"tmux send-keys -t {sess} C-c || true"
       self._run(remote_cmd, f"{self.cfg.name} :: Stop Launch")

   def run_quick_cmd(self):
       txt = self.cmd_input.currentText()
       if "Build Workspace" in txt:
           # Run colcon build in a way that preserves the environment
           cmd = "bash -lc 'source /opt/ros/humble/setup.bash; source ~/mobile_mast_ws/install/setup.bash; colcon build --symlink-install --packages-select mobile_mast'"
           title = "Build Workspace"
       elif "Disk Space" in txt:
           cmd = "df -h ~"
           title = "Disk Space"
       elif "Uptime" in txt:
           cmd = "uptime"
           title = "System Uptime"
       elif "ROS2 Nodes" in txt:
           cmd = "bash -lc 'source /opt/ros/humble/setup.bash; ros2 node list | head -n 20'"
           title = "ROS2 Nodes"
       elif "ROS2 Topics" in txt:
           cmd = "bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic list | head -n 20'"
           title = "ROS2 Topics"
       elif "Time Sync" in txt:
           cmd = "chronyc tracking"
           title = "Time Sync"
       elif "Sensors" in txt:
           cmd = "sensors"
           title = "System Sensors"
       else:
           return

       self._run(cmd, f"{self.cfg.name} :: {title}")

   def _run(self, remote_cmd: str, title: str):
       if self.worker and self.worker.isRunning():
           return

       self.btn_run.setEnabled(False)
       self.btn_run.setText("Running...")
       self.worker = SshWorker(self.cfg.user, self.cfg.ip, remote_cmd)
       self.worker.finished.connect(lambda code, out: self._on_cmd_finished(code, out, title))
       self.worker.start()

   def _on_cmd_finished(self, code: int, out: str, title: str):
       self.btn_run.setEnabled(True)
       self.btn_run.setText("Run")
       
       msg = QMessageBox(self)
       msg.setWindowTitle(title)
       if code == 0:
           msg.setIcon(QMessageBox.Information)
           msg.setText(f"Success (Exit Code: {code})")
       else:
           msg.setIcon(QMessageBox.Warning)
           msg.setText(f"Failed (Exit Code: {code})")
           
       msg.setInformativeText(out[:5000])
       msg.setStandardButtons(QMessageBox.Ok)
       msg.show()


class ProcessControlTab(QWidget):
   def __init__(self):
       super().__init__()
       USER = "ashfaq" # Updated to your user
      
       self.orins = [
           OrinConfig(
               name="Orin1 (Sensors)",
               ip="192.168.6.101",
               user=USER,
               launch_cmd="ros2 launch mobile_mast mobile_mast_orin1.launch.py",
               tmux_session="orin1_launch",
               vnc_cmd="remmina -c vnc://192.168.6.101",
               components=[
                   Component("LiDAR Driver", "bash ~/scripts/start_lidar_driver.sh", "bash ~/scripts/stop_lidar_driver.sh", check_topic="/mast/orin1/lidar/points"),
                   Component("LiDAR Preprocess", "bash ~/scripts/start_lidar_preprocess.sh", "bash ~/scripts/stop_lidar_preprocess.sh", check_topic="/mast/orin1/lidar/points_filtered"),
                   Component("Background Sub", "bash ~/scripts/start_lidar_bgsub.sh", "bash ~/scripts/stop_lidar_bgsub.sh", check_topic="/mast/orin1/lidar/foreground"),
                   Component("LiDAR Cluster", "bash ~/scripts/start_lidar_cluster.sh", "bash ~/scripts/stop_lidar_cluster.sh", check_topic="/mast/orin1/lidar/objects_3d"),
                   Component("Bullet1 Cam", "bash ~/scripts/start_bullet1_cam.sh", "bash ~/scripts/stop_bullet1_cam.sh", check_topic="/mast/orin1/bullet1/image_raw/compressed"),
                   Component("Bullet2 Cam", "bash ~/scripts/start_bullet2_cam.sh", "bash ~/scripts/stop_bullet2_cam.sh", check_topic="/mast/orin1/bullet2/image_raw/compressed"),
                   Component("Bullet3 Cam", "bash ~/scripts/start_bullet3_cam.sh", "bash ~/scripts/stop_bullet3_cam.sh", check_topic="/mast/orin1/bullet3/image_raw/compressed"),
                   Component("Bullet4 Cam", "bash ~/scripts/start_bullet4_cam.sh", "bash ~/scripts/stop_bullet4_cam.sh", check_topic="/mast/orin1/bullet4/image_raw/compressed"),
               ],
           ),
           OrinConfig(
               name="Orin2 (Detection)",
               ip="192.168.6.102",
               user=USER,
               launch_cmd="ros2 launch mobile_mast mobile_mast_orin2.launch.py",
               tmux_session="orin2_launch",
               vnc_cmd="remmina -c vnc://192.168.6.102",
               components=[
                   # Bullet 1
                   Component("VPI Undistort B1", "bash ~/scripts/start_vpi_bullet1.sh", "bash ~/scripts/stop_vpi_bullet1.sh", check_topic="/mast/orin1/bullet1/image_rect"),
                   Component("YOLO B1", "bash ~/scripts/start_yolo_bullet1.sh", "bash ~/scripts/stop_yolo_bullet1.sh", check_topic="/mast/orin1/bullet1/detections_2d"),
                   Component("2D-3D Proj B1", "bash ~/scripts/start_proj_bullet1.sh", "bash ~/scripts/stop_proj_bullet1.sh", check_topic="/mast/orin1/bullet1/detections_3d"),
                   # Bullet 2
                   Component("VPI Undistort B2", "bash ~/scripts/start_vpi_bullet2.sh", "bash ~/scripts/stop_vpi_bullet2.sh", check_topic="/mast/orin1/bullet2/image_rect"),
                   Component("YOLO B2", "bash ~/scripts/start_yolo_bullet2.sh", "bash ~/scripts/stop_yolo_bullet2.sh", check_topic="/mast/orin1/bullet2/detections_2d"),
                   Component("2D-3D Proj B2", "bash ~/scripts/start_proj_bullet2.sh", "bash ~/scripts/stop_proj_bullet2.sh", check_topic="/mast/orin1/bullet2/detections_3d"),
                   # Bullet 3
                   Component("VPI Undistort B3", "bash ~/scripts/start_vpi_bullet3.sh", "bash ~/scripts/stop_vpi_bullet3.sh", check_topic="/mast/orin1/bullet3/image_rect"),
                   Component("YOLO B3", "bash ~/scripts/start_yolo_bullet3.sh", "bash ~/scripts/stop_yolo_bullet3.sh", check_topic="/mast/orin1/bullet3/detections_2d"),
                   Component("2D-3D Proj B3", "bash ~/scripts/start_proj_bullet3.sh", "bash ~/scripts/stop_proj_bullet3.sh", check_topic="/mast/orin1/bullet3/detections_3d"),
                   # Bullet 4
                   Component("VPI Undistort B4", "bash ~/scripts/start_vpi_bullet4.sh", "bash ~/scripts/stop_vpi_bullet4.sh", check_topic="/mast/orin1/bullet4/image_rect"),
                   Component("YOLO B4", "bash ~/scripts/start_yolo_bullet4.sh", "bash ~/scripts/stop_yolo_bullet4.sh", check_topic="/mast/orin1/bullet4/detections_2d"),
                   Component("2D-3D Proj B4", "bash ~/scripts/start_proj_bullet4.sh", "bash ~/scripts/stop_proj_bullet4.sh", check_topic="/mast/orin1/bullet4/detections_3d"),
               ],
           ),
           OrinConfig(
               name="Orin3 (Tracking)",
               ip="192.168.6.103",
               user=USER,
               launch_cmd="ros2 launch mobile_mast mobile_mast_orin3.launch.py",
               tmux_session="orin3_launch",
               vnc_cmd="remmina -c vnc://192.168.6.103",
               components=[
                   Component("Tracker B1", "bash ~/scripts/start_tracker_bullet1.sh", "bash ~/scripts/stop_tracker_bullet1.sh", check_topic="/mast/orin1/bullet1/tracks_3d"),
                   Component("Tracker B2", "bash ~/scripts/start_tracker_bullet2.sh", "bash ~/scripts/stop_tracker_bullet2.sh", check_topic="/mast/orin1/bullet2/tracks_3d"),
                   Component("Tracker B3", "bash ~/scripts/start_tracker_bullet3.sh", "bash ~/scripts/stop_tracker_bullet3.sh", check_topic="/mast/orin1/bullet3/tracks_3d"),
                   Component("Tracker B4", "bash ~/scripts/start_tracker_bullet4.sh", "bash ~/scripts/stop_tracker_bullet4.sh", check_topic="/mast/orin1/bullet4/tracks_3d"),
                   Component("Global Fusion", "bash ~/scripts/start_fusion.sh", "bash ~/scripts/stop_fusion.sh", check_topic="/mast/global/tracks_3d"),
                   Component("RViz Markers", "bash ~/scripts/start_markers.sh", "bash ~/scripts/stop_markers.sh", check_topic="/mast/global/tracks_markers"),
               ],
           ),
       ]
      
       layout = QVBoxLayout()
       layout.setContentsMargins(0, 0, 0, 0) # Revert margins
       self.tabs = QTabWidget()
       for cfg in self.orins:
           self.tabs.addTab(OrinPanel(cfg), cfg.name)
       layout.addWidget(self.tabs)
       self.setLayout(layout)


# --- ROS 2 BACKEND ---
class RosWorker(Node):
   def __init__(self):
       super().__init__('qt_calibrator_node')
       self.latest_cv_image = None
       self.latest_lidar_points = None
      
       # Start with Camera 2 (Bullet) as default
       self.current_topic = CAMERA_TOPICS["Camera 2 (Bullet)"]
       self.create_subs()


   def create_subs(self):
       # Best Effort QoS to ensure connection to sensors
       qos_profile = QoSProfile(
           reliability=ReliabilityPolicy.BEST_EFFORT,
           history=HistoryPolicy.KEEP_LAST,
           depth=10
       )


       if hasattr(self, 'img_sub'):
           self.destroy_subscription(self.img_sub)
          
       self.img_sub = self.create_subscription(
           CompressedImage,
           self.current_topic,
           self.img_cb,
           qos_profile)
          
       if not hasattr(self, 'lidar_sub'):
           self.lidar_sub = self.create_subscription(
               PointCloud2,
               LIDAR_TOPIC,
               self.lidar_cb,
               qos_profile)
      
       self.get_logger().info(f"Subscribed to {self.current_topic} (Best Effort)")
       self.get_logger().info(f"Subscribed to {LIDAR_TOPIC} (Best Effort)")


   def img_cb(self, msg):
       try:
           np_arr = np.frombuffer(msg.data, np.uint8)
           self.latest_cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
           self.get_logger().info("Image frame received!")
       except Exception as e:
           self.get_logger().error(f"Error decoding image: {e}")


   def lidar_cb(self, msg):
       try:
           # Read x, y, z and skip NaNs
           gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
          
           # --- ROBUST FIX FOR STRUCTURED ARRAYS ---
           points_list = [[p[0], p[1], p[2]] for p in gen]
          
           if len(points_list) > 0:
               points = np.array(points_list, dtype=np.float32)
              
               if points.ndim == 2 and points.shape[1] >= 3:
                   # Filter points behind sensor (x < 0.5m) and downsample
                   self.latest_lidar_points = points[points[:,0] > 0.5][::5]
                   self.get_logger().info(f"LiDAR points: {len(self.latest_lidar_points)}")
               else:
                   self.get_logger().warning("LiDAR data shape incorrect, skipping frame")
                  
       except Exception as e:
           self.get_logger().error(f"Error parsing LiDAR: {e}")


   def switch_camera(self, cam_name):
       if cam_name in CAMERA_TOPICS:
           self.current_topic = CAMERA_TOPICS[cam_name]
           self.create_subs()


# --- GUI FRONTEND ---
class CalibratorApp(QMainWindow):
   def __init__(self, ros_node):
       super().__init__()
       self.ros_node = ros_node
       self.bag_process = None
       self.record_process = None
       self.current_bag_dir = os.getcwd()
       # --- INTRINSICS (FISHEYE MODEL) ---
       self.K = np.array([
           [1097.2, 0.0, 960.0],
           [0.0, 1097.2, 540.0],
           [0.0, 0.0, 1.0]
       ])
      
       self.D = np.array([-0.1204641, -0.00018423, 0.00731092, -0.00474879])


       # --- SLIDER STATE ---
       self.spinners = {}
       self.sliders = {}
       # Initial guess values
       self.param_centers = {
           'x': 0.1, 'y': 0.0, 'z': 0.2,
           'roll': -1.57, 'pitch': 0.0, 'yaw': -1.57
       }
       self.current_range_pos = 1.0
       self.current_range_rot = 10.0


       # VAPIX State
       self.current_cam_ip = None
       self.vapix_timer = QTimer()
       self.vapix_timer.setSingleShot(True)
       self.vapix_timer.timeout.connect(self.send_vapix)
       self.pending_cmd = None
      
       self.setup_ui()
      
       self.timer = QTimer()
       self.timer.timeout.connect(self.update_display)
       self.timer.start(33)


   def setup_ui(self):
       self.setWindowTitle("Mobile Mast | LiDAR-Camera Calibration & Control")
       self.resize(1400, 950)
       self.setStyleSheet(LIGHT_STYLESHEET)
       self.setFont(QFont("Segoe UI", 11))
      
       # Menu
       menubar = self.menuBar()
       file_menu = menubar.addMenu("&File")
       exit_action = QAction("Exit", self)
       exit_action.triggered.connect(self.close)
       file_menu.addAction(exit_action)




      
       # Status bar
       self.statusBar().showMessage("System initializing...")
      
       # Main Layout
       main_widget = QWidget()
       self.setCentralWidget(main_widget)
       self.main_layout = QVBoxLayout(main_widget)
      
       # --- HEADER ---
       header = QFrame()
       header.setFixedHeight(90)
       header.setStyleSheet("background: #ffffff; border-bottom: 1px solid #e0e0e0;")
       h_layout = QHBoxLayout(header)
      
       # Logo
       logo_lbl = QLabel()
       logo_path = os.path.join(os.path.dirname(__file__), "mobile_mast_logo.png")
       if os.path.exists(logo_path):
           pix = QPixmap(logo_path).scaled(80, 80, Qt.KeepAspectRatio, Qt.SmoothTransformation)
           logo_lbl.setPixmap(pix)
       else:
           logo_lbl.setText("📡")
           logo_lbl.setStyleSheet("font-size: 40px;")
      
       title_v = QVBoxLayout()
       title_lbl = QLabel("MOBILE MAST")
       title_lbl.setStyleSheet("font-size: 24px; font-weight: 800; color: #1a73e8; letter-spacing: 1px;")
       subtitle_lbl = QLabel("Multi-Orin Distributed Sensor Fusion & Control")
       subtitle_lbl.setStyleSheet("font-size: 13px; color: #5f6368; font-weight: 500;")
       title_v.addWidget(title_lbl)
       title_v.addWidget(subtitle_lbl)
      
       h_layout.addWidget(logo_lbl)
       h_layout.addLayout(title_v)
       h_layout.addStretch()
      
       # Quick Stats
       self.lbl_status = QLabel("● SYSTEM ONLINE")
       self.lbl_status.setStyleSheet("color: #137333; font-weight: 700; font-size: 13px; padding: 8px 16px; border: 1px solid #ceead6; border-radius: 16px; background: #e6f4ea;")
       h_layout.addWidget(self.lbl_status)
      
       self.main_layout.addWidget(header)


       self.tabs = QTabWidget()
       self.main_layout.addWidget(self.tabs)
      
       # --- TAB 1: CALIBRATION & VIEW ---
       self.tab_calib = QWidget()
       self.layout = QVBoxLayout(self.tab_calib)
      
       content = QHBoxLayout()
       self.layout.addLayout(content)
      
       # Sidebar
       scroll = QScrollArea()
       scroll.setFixedWidth(460)
       scroll.setWidgetResizable(True)
       scroll.setStyleSheet("border: none; background: transparent;")
      
       self.controls = QWidget()
       self.c_layout = QVBoxLayout(self.controls)
       self.c_layout.setSpacing(30)
       self.c_layout.setContentsMargins(10,10,20,10)
      
       self.create_groups()
      
       self.c_layout.addStretch()
       scroll.setWidget(self.controls)
       content.addWidget(scroll)
      
       # Viewport
       self.create_viewport(content)
       self.update_sensitivity(2)
      
       self.tabs.addTab(self.tab_calib, "Calibration & View")
      
       # --- TAB 2: PROCESS CONTROL ---
       self.tab_process = ProcessControlTab()
       self.tabs.addTab(self.tab_process, "Process Control")


       self.statusBar().showMessage("Ready")


   def create_top_bar(self):
       f = QFrame()
       f.setFixedHeight(60)
       f.setStyleSheet("background: #ffffff; border-radius: 8px; border: 1px solid #ddd;")
       l = QHBoxLayout(f)
      
       btn_rst = QPushButton("↺ Reset All")
       btn_rst.setFixedWidth(120)
       btn_rst.clicked.connect(self.reset_all)
      
       btn_save = QPushButton("💾 Save Config")
       btn_save.setObjectName("btn_save")
       btn_save.setFixedWidth(140)
       btn_save.clicked.connect(self.save_config)
      
       l.addWidget(btn_rst); l.addWidget(btn_save); l.addStretch()
       self.layout.addWidget(f)


   def create_groups(self):
       # DATA
       # CAMERA SELECTION (TOP PRIORITY)
       g_cam = QGroupBox("Camera Selection")
       l_cam = QHBoxLayout()
       self.combo = QComboBox()
       self.combo.addItems(list(CAMERA_IPS.keys()))
       self.combo.setFixedHeight(30)
       self.combo.setCurrentText("Camera 2 (Bullet)")
       self.combo.currentTextChanged.connect(self.change_camera)
       l_cam.addWidget(self.combo)
       g_cam.setLayout(l_cam)
       self.c_layout.addWidget(g_cam)


       # DATA SOURCE (LIVE vs PLAYBACK)
       g_data = QGroupBox("Data Source")
       l_data = QVBoxLayout()
      
       h_btns = QHBoxLayout()
       btn_live = QPushButton("🔴 Live Feed")
       btn_live.clicked.connect(self.switch_to_live)
      
       btn_browse = QPushButton("📂 Open & Play Bag")
       btn_browse.setObjectName("btn_primary")
       # Force style directly to ensure visibility
       btn_browse.setStyleSheet("""
           QPushButton {
               background-color: #1a73e8;
               color: #ffffff;
               border: none;
               font-weight: bold;
               border-radius: 8px;
               padding: 12px 20px;
               font-size: 15px;
           }
           QPushButton:hover { background-color: #1765cc; }
           QPushButton:pressed { background-color: #1a73e8; }
       """)
       btn_browse.clicked.connect(self.browse_and_play)
      
       h_btns.addWidget(btn_live)
       h_btns.addWidget(btn_browse)
       l_data.addLayout(h_btns)
      
       g_data.setLayout(l_data)
       self.c_layout.addWidget(g_data)


       # ROSBAG RECORDING (SEPARATE SECTION)
       g_rec = QGroupBox("Rosbag Recording")
       l_rec = QVBoxLayout()
      
       h_rec_btns = QHBoxLayout()
       self.btn_rec_start = QPushButton("⏺ Start Record")
       self.btn_rec_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
       self.btn_rec_start.clicked.connect(self.start_recording)
      
       self.btn_rec_stop = QPushButton("⏹ Stop Record")
       self.btn_rec_stop.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
       self.btn_rec_stop.clicked.connect(self.stop_recording)
       self.btn_rec_stop.setEnabled(False)
      
       h_rec_btns.addWidget(self.btn_rec_start)
       h_rec_btns.addWidget(self.btn_rec_stop)
       l_rec.addLayout(h_rec_btns)
      
       self.lbl_rec_status = QLabel("Status: Idle")
       self.lbl_rec_status.setAlignment(Qt.AlignCenter)
       self.lbl_rec_status.setStyleSheet("font-weight: bold; color: #aaa;")
       l_rec.addWidget(self.lbl_rec_status)
      
       g_rec.setLayout(l_rec)
       self.c_layout.addWidget(g_rec)
      
       # Populate initially (hidden logic removed)
      
       # LENS
       g_lens = QGroupBox("Lens Control (VAPIX)")
       l_lens = QFormLayout()
      
       h_zoom = QHBoxLayout()
       s_zoom = QSlider(Qt.Horizontal); s_zoom.setRange(0, 10000)
       l_zval = QLabel("0")
       s_zoom.valueChanged.connect(lambda v: self.queue_vapix('zoom', v))
       s_zoom.valueChanged.connect(lambda v: l_zval.setText(str(v)))
       h_zoom.addWidget(s_zoom); h_zoom.addWidget(l_zval)
      
       h_focus = QHBoxLayout()
       s_focus = QSlider(Qt.Horizontal); s_focus.setRange(0, 10000)
       l_fval = QLabel("0")
       s_focus.valueChanged.connect(lambda v: self.queue_vapix('focus', v))
       s_focus.valueChanged.connect(lambda v: l_fval.setText(str(v)))
       h_focus.addWidget(s_focus); h_focus.addWidget(l_fval)
      
       l_lens.addRow("Zoom:", h_zoom)
       l_lens.addRow("Focus:", h_focus)
       g_lens.setLayout(l_lens)
       self.c_layout.addWidget(g_lens)
      
       # ALIGNMENT
       g_pos = QGroupBox("Spatial Alignment")
       l_pos = QVBoxLayout()
       l_pos.setSpacing(15)
      
       l_pos.addWidget(QLabel("POSITION (Meters)"))
       l_pos.addWidget(self.make_row("X", "x", "pos"))
       l_pos.addWidget(self.make_row("Y", "y", "pos"))
       l_pos.addWidget(self.make_row("Z", "z", "pos"))
      
       l_pos.addWidget(QLabel("ORIENTATION (Degrees)"))
       l_pos.addWidget(self.make_row("Roll", "roll", "rot"))
       l_pos.addWidget(self.make_row("Pitch", "pitch", "rot"))
       l_pos.addWidget(self.make_row("Yaw", "yaw", "rot"))
      
       # Sensitivity
       f_sens = QFrame()
       f_sens.setStyleSheet("background: #ffffff; border: 1px solid #ddd; border-radius: 5px;")
       l_sens = QVBoxLayout(f_sens)
       l_sens.addWidget(QLabel("CONTROL SENSITIVITY"))
       s_sens = QSlider(Qt.Horizontal)
       s_sens.setObjectName("sens_slider")
       s_sens.setRange(0, 4); s_sens.setValue(2)
       self.lb_sens = QLabel("Normal")
       s_sens.valueChanged.connect(self.update_sensitivity)
       l_sens.addWidget(s_sens); l_sens.addWidget(self.lb_sens)
      
       l_pos.addWidget(f_sens)
       g_pos.setLayout(l_pos)
       self.c_layout.addWidget(g_pos)


   def make_row(self, label, key, type_tag):
       w = QWidget(); h = QHBoxLayout(w); h.setContentsMargins(0,0,0,0)
       lbl = QLabel(label); lbl.setFixedWidth(50); lbl.setStyleSheet("font-weight: 700; color: #1a73e8; font-size: 14px;")
      
       sl = QSlider(Qt.Horizontal); sl.setRange(0, 1000); sl.setValue(500)
      
       sp = QDoubleSpinBox()
       sp.setRange(-999, 999); sp.setFixedHeight(40); sp.setButtonSymbols(QDoubleSpinBox.UpDownArrows)
       sp.setDecimals(3)
       # Stylesheet handled globally now for consistency
       val = self.param_centers[key]
       sp.setValue(val if type_tag=="pos" else np.degrees(val))
      
       self.spinners[key] = sp
       self.sliders[key] = sl
      
       sl.valueChanged.connect(lambda v: self.on_slider_change(v, key, type_tag))
       sp.valueChanged.connect(lambda v: self.on_spinbox_change(v, key, type_tag))
      
       h.addWidget(lbl); h.addWidget(sl); h.addWidget(sp)
       return w


   def on_slider_change(self, value, key, type_tag):
       rng = self.current_range_pos if type_tag == "pos" else self.current_range_rot
       center = self.param_centers[key]
       if type_tag == "rot": center = np.degrees(center)
       norm = (value - 500) / 500.0
       new_val = center + (norm * rng)
       self.spinners[key].blockSignals(True)
       self.spinners[key].setValue(new_val)
       self.spinners[key].blockSignals(False)


   def on_spinbox_change(self, value, key, type_tag):
       if type_tag == "pos": self.param_centers[key] = value
       else: self.param_centers[key] = np.radians(value)
       self.sliders[key].blockSignals(True)
       self.sliders[key].setValue(500)
       self.sliders[key].blockSignals(False)


   def update_sensitivity(self, val):
       # 1. Capture current values as the new center BEFORE changing range
       for key, spinner in self.spinners.items():
           current_val = spinner.value()
           if key in ['roll', 'pitch', 'yaw']:
               self.param_centers[key] = np.radians(current_val)
           else:
               self.param_centers[key] = current_val


       # 2. Update Ranges
       pos_ranges = [0.05, 0.25, 1.0, 5.0, 20.0]
       rot_ranges = [0.5, 2.5, 10.0, 45.0, 180.0]
       labels = [
           "Very Fine (Range: 10cm / 1°)",
           "Fine (Range: 50cm / 5°)",
           "Normal (Range: 2m / 20°)",
           "Coarse (Range: 10m / 90°)",
           "Very Coarse (Range: 40m / 360°)"
       ]
       # If called from the initial setup, val may be a QObjectSignal; ensure int
       if not isinstance(val, int):
           val = 2
       self.current_range_pos = pos_ranges[val]
       self.current_range_rot = rot_ranges[val]
       self.lb_sens.setText(labels[val])
      
       # 3. Reset all sliders to middle (500)
       for k, s in self.sliders.items():
           s.blockSignals(True)
           s.setValue(500)
           s.blockSignals(False)


   def update_display(self):
       img = self.ros_node.latest_cv_image
       pts = self.ros_node.latest_lidar_points
      
       if img is None:
           return
      
       try:
           disp = img.copy()
           if pts is not None and pts.size > 0:
               x = self.spinners['x'].value()
               y = self.spinners['y'].value()
               z = self.spinners['z'].value()
              
               r = np.radians(self.spinners['roll'].value())
               p = np.radians(self.spinners['pitch'].value())
               yw = np.radians(self.spinners['yaw'].value())
              
               r_vec = R.from_euler('xyz', [r, p, yw], degrees=False).as_rotvec().astype(np.float64)
               t_vec = np.array([x, y, z], dtype=np.float64)
              
               # Reshape for Fisheye ProjectPoints (N, 1, 3)
               pts_to_proj = pts.reshape(-1, 1, 3).astype(np.float64)
              
               # --- FISHEYE PROJECTION (MATCHES FILE) ---
               p2d, _ = cv2.fisheye.projectPoints(pts_to_proj, r_vec, t_vec, self.K.astype(np.float64), self.D.astype(np.float64))
              
               h, w = disp.shape[:2]
              
               for p in p2d:
                   px, py = int(p[0][0]), int(p[0][1])
                   if 0 <= px < w and 0 <= py < h:
                       cv2.circle(disp, (px, py), 2, (0, 255, 0), -1)
          
           # --- RGB CORRECTION (FIXES BLUE FACES) ---
           rgb = cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)
           qt_img = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1]*3, QImage.Format_RGB888)
           pix = QPixmap.fromImage(qt_img).scaled(self.view_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
           self.view_lbl.setPixmap(pix)
       except Exception as e:
           print(f"Visualization Error: {e}")
           traceback.print_exc()


   def create_viewport(self, parent):
       f = QFrame()
       f.setStyleSheet("background: #000; border: 2px solid #ddd; border-radius: 10px;")
       f.setMinimumSize(640, 480)
      
       l = QVBoxLayout(f)
       l.setContentsMargins(0, 0, 0, 0)
      
       self.view_lbl = QLabel("WAITING FOR DATA STREAM...")
       self.view_lbl.setStyleSheet("color: #666; font-weight: bold; font-size: 20px;")
       self.view_lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
       self.view_lbl.setScaledContents(True)
      
       l.addWidget(self.view_lbl)
       parent.addWidget(f)


   # --- LOGIC ---
   def change_camera(self, text):
       self.current_cam_ip = CAMERA_IPS.get(text)
       self.ros_node.switch_camera(text)
       self.lbl_status.setText(f"Active: {text}")
       self.statusBar().showMessage(f"Switched camera: {text}")


   def reset_all(self):
       for k in self.param_centers:
           self.param_centers[k] = 0.0
           if k in ['roll','pitch','yaw']: self.param_centers[k] = np.radians(0.0)
          
       for s in self.spinners.values(): s.setValue(0.0)
       self.lbl_status.setText("Reset All to Zero")
       self.statusBar().showMessage("Parameters reset")


   def browse_and_play(self):
       d = QFileDialog.getExistingDirectory(self, "Select Bag Directory", self.current_bag_dir)
       if d:
           # keep the selected folder and basename
           self.current_bag_dir = d
           self.selected_bag = os.path.basename(d)
           self.play_selected_bag()


   def populate_bag_list(self, path):
       # Kept for internal use if needed, but UI list is gone
       pass


   def on_bag_selected(self, item):
       pass


   def play_selected_bag(self):
       if not hasattr(self, 'selected_bag') or not self.selected_bag:
           return
      
       self.stop_bag()
       bag_path = os.path.join(self.current_bag_dir, self.selected_bag)
      
       # Run ros2 bag play in a separate process
       cmd = ['ros2', 'bag', 'play', bag_path, '--loop']
       try:
           self.bag_process = subprocess.Popen(cmd)
           self.lbl_status.setText(f"Playing: {self.selected_bag}")
           self.statusBar().showMessage(f"Playing bag: {self.selected_bag}")
       except Exception as e:
           self.lbl_status.setText(f"Error: {e}")
           self.statusBar().showMessage("Error starting bag playback")


   def stop_bag(self):
       if self.bag_process:
           self.bag_process.send_signal(signal.SIGINT)
           try:
               self.bag_process.wait(timeout=2)
           except subprocess.TimeoutExpired:
               self.bag_process.kill()
           self.bag_process = None
       self.lbl_status.setText("Playback Stopped")
       self.statusBar().showMessage("Playback stopped")


   def switch_to_live(self):
       self.stop_bag()
       self.lbl_status.setText("Switched to Live Feed")
       self.statusBar().showMessage("Live feed active")


   def start_recording(self):
       if self.record_process:
           return


       # Topics to record
       topics = [
           '/rslidar_points',
           '/mast/orin1/bullet1/image_raw/compressed',
           '/mast/orin1/bullet2/image_raw/compressed',
           '/mast/orin1/bullet3/image_raw/compressed',
           '/mast/orin1/bullet4/image_raw/compressed',
           '/tf',
           '/tf_static'
       ]
      
       # Generate bag name with timestamp
       import datetime
       timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
       bag_name = f"rosbag2_{timestamp}"
      
       cmd = ['ros2', 'bag', 'record', '-o', bag_name] + topics
      
       try:
           self.record_process = subprocess.Popen(cmd, cwd=self.current_bag_dir)
           self.lbl_rec_status.setText(f"Recording: {bag_name}")
           self.lbl_rec_status.setStyleSheet("font-weight: bold; color: #d32f2f;") # Red text
           self.btn_rec_start.setEnabled(False)
           self.btn_rec_stop.setEnabled(True)
           self.lbl_status.setText(f"Started Recording: {bag_name}")
           self.statusBar().showMessage(f"Recording: {bag_name}")
       except Exception as e:
           self.lbl_status.setText(f"Record Error: {e}")
           self.statusBar().showMessage("Record error")


   def stop_recording(self):
       if self.record_process:
           self.record_process.send_signal(signal.SIGINT)
           try:
               self.record_process.wait(timeout=5)
           except subprocess.TimeoutExpired:
               self.record_process.kill()
           self.record_process = None
          
           self.lbl_rec_status.setText("Status: Idle")
           self.lbl_rec_status.setStyleSheet("font-weight: bold; color: #555;")
           self.btn_rec_start.setEnabled(True)
           self.btn_rec_stop.setEnabled(False)
           self.lbl_status.setText("Recording Stopped")
           self.statusBar().showMessage("Recording stopped")
          
           # Recording Stopped
           pass


   def queue_vapix(self, p, v):
       self.pending_cmd = (p, v); self.vapix_timer.start(200)


   def send_vapix(self):
       if not self.pending_cmd or not self.current_cam_ip: return
       p, v = self.pending_cmd
       try: requests.get(f"http://{self.current_cam_ip}/axis-cgi/com/ptz.cgi?{p}={v}", auth=(CAM_USER, CAM_PASS), timeout=0.5)
       except: pass


   def save_config(self):
       path, _ = QFileDialog.getSaveFileName(self, "Save", "calibration.yaml", "YAML (*.yaml)")
       if path:
           data = {'calibration': {k: float(s.value()) for k, s in self.spinners.items()}}
           with open(path, 'w') as f: yaml.dump(data, f)
           QMessageBox.information(self, "Saved", f"Saved to {path}")
           self.statusBar().showMessage(f"Saved calibration: {os.path.basename(path)}")


def main():
   rclpy.init()
   node = RosWorker()
   t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
   t.start()
  
   app = QApplication(sys.argv)
   app.setStyle("Fusion")
   w = CalibratorApp(node)
   w.show()
   sys.exit(app.exec_())


if __name__ == "__main__":
   main()
