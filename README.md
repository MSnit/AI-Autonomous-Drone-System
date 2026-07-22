# Autonomous AI Swarm Intelligence Architecture

An enterprise-grade autonomous drone swarm tracking system built in Python. This project bridges deep learning and physical kinematics by utilizing a custom-trained YOLOv8 neural network to drive a visual servoing P-Controller, enabling real-time, GPS-denied target pursuit within the Unreal Engine AirSim environment.

## 🧠 Core Architecture

* **Custom AI Edge Inference:** A YOLOv8 Convolutional Neural Network trained from scratch on custom-harvested simulator datasets to detect and bound dynamic aerial targets.
* **Visual Servoing Kinematics:** A Proportional Controller (P-Controller) that translates raw pixel coordinates into physical yaw and pitch velocities, creating a fully autonomous hunter-killer tracking loop.
* **Dynamic Spatial Navigation:** A* pathfinding algorithms that recalculate spatial matrices to evade dynamic obstacles.
* **Stereo Vision Depth Mapping:** Epipolar geometry and SIFT feature matching to extract depth data from 2D image frames.
* **Asynchronous Ground Control:** A multithreaded Ground Control Station (GCS) providing real-time telemetry, GPS radar, and HUD overlays.

## 🛠️ Tech Stack

* **Language:** Python 3.10+
* **Deep Learning:** Ultralytics YOLOv8, PyTorch
* **Computer Vision:** OpenCV, NumPy
* **Simulation Engine:** Microsoft AirSim (Unreal Engine 4/5)
* **RPC Protocol:** Msgpack

## 🚀 Execution Pipeline

### 1. Data Harvesting
The system autonomously deploys Drone 1 to strafe and capture raw RGB frames of Drone 2. Background/negative samples are retained to train out false positives.
`python swarm_data_harvester.py`

### 2. Neural Network Training
Automatically structures the annotated YOLO dataset and trains the `yolov8n.pt` base model, exporting a highly optimized `best.pt` weights file for edge inference.
`python swarm_yolo_trainer.py`

### 3. Live Visual Servoing (Live Inference)
Injects the trained AI weights into Drone 1's visual cortex. The system calculates the centroid of the AI bounding box and feeds the error delta directly into the drone's velocity motors to maintain target lock.
`python swarm_visual_servoing.py`

## ⚙️ Control Systems Math (The P-Controller)

The visual servoing loop relies on proportional error calculation to drive the drone without GPS waypoints:
* **Yaw Control:** `yaw_rate = (box_center_x - image_center_x) * k_yaw`
* **Pitch Control:** `forward_velocity = (target_box_area - current_box_area) * k_pitch`

## 👨‍💻 Developer
**Manthan Sontakke**  
