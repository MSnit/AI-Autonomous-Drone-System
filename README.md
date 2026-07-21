# Autonomous Multi-Agent Drone Swarm Architecture

An end-to-end simulation environment for autonomous UAV (Unmanned Aerial Vehicle) operations, featuring dynamic cooperative pathfinding, real-time machine perception, and a custom Ground Control Station (GCS). Built using Python, OpenCV, and Unreal Engine (AirSim).

## 🚀 Core Architecture

* **Dynamic A* Evasion (Multi-Agent Spatial Logic):** 
  Implemented a continuous recalculation loop using A* pathfinding. The primary UAV dynamically routes around moving obstacles (secondary UAVs) while maintaining a strict mathematical safety margin.
* **RANSAC Visual Target Lock (Machine Perception):** 
  Engineered a computer vision pipeline extracting raw RGB matrices from the UAV's virtual downward/forward cameras. Utilized ORB feature extraction and Random Sample Consensus (RANSAC) to calculate Homography matrices, maintaining a continuous visual lock on moving targets against noisy backgrounds.
* **Autonomous Ground Control Station (GCS):** 
  Developed a multi-threaded desktop application using Tkinter. Features an interactive map that translates real-world GPS coordinates into local Cartesian grid vectors, dispatching flight commands to the swarm via a background thread to prevent UI blocking.
* **Cloud Telemetry Pipeline:** 
  Built an edge-to-cloud telemetry system streaming live kinematics data (velocity, orientation, position) to a remote dashboard.

## 🛠️ Technology Stack
* **Languages:** Python
* **Vision & Math:** OpenCV, NumPy, SciPy
* **Simulation:** Unreal Engine, Microsoft AirSim API
* **Interfaces:** Tkinter, TkinterMapView, multithreading

## ⚙️ Module Execution

**1. Ground Control Station Dispatch:**
```bash
python gcs_autonomous_dispatch.py