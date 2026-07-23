# 🚁 Autonomous Drone Logistics & Swarm Ground Control Station (GCS)

A robust Python-based desktop Ground Control Station (GCS) for autonomous drone delivery, swarm behavior simulation, and machine learning data harvesting. Built on top of Microsoft's **AirSim** and **Unreal Engine**, this project bridges the gap between simulated physics and real-world logistics applications, with a strong focus on AI integration and cybernetics.

## ✨ Key Features

* **Interactive GIS Dashboard:** Utilizes `tkintermapview` for a real-time, Google Maps-style interface. 
* **Live Telemetry & Tracking:** Independent, thread-safe telemetry pinging that translates AirSim Cartesian coordinates (X, Y, Z) into real-world GPS coordinates (Lat, Lon) for live mapping.
* **Autonomous Dispatch:** Point-and-click deployment. The drone automatically calculates the route, takes off, navigates at a calculated cruising altitude, and lands at the destination.
* **Swarm Data Harvesting:** Includes automated scripts (`swarm_data_harvester.py`) for collecting multi-angle dataset images.
* **Computer Vision Integration:** Ready pipeline for training custom YOLO object detection models on the harvested dataset (`swarm_yolo_trainer.py`).

## 📂 Repository Structure

```text
├── dataset/                     # Harvested image data for YOLO training
├── runs/                        # YOLO training weights and validation outputs
├── delivery_dashboard.py        # Core autonomous logistics Ground Control UI
├── swarm_dashboard.py           # Multi-drone swarm simulation controller
├── swarm_data_harvester.py      # Automated script for generating ML datasets
├── swarm_yolo_trainer.py        # Object detection model training pipeline
└── README.md                    # Project documentation