import os
import shutil
import yaml
from ultralytics import YOLO

print("Initializing YOLOv8 Training Pipeline...")

# 1. Define Paths
base_dir = "dataset"
raw_dir = os.path.join(base_dir, "raw_images")
images_dir = os.path.join(base_dir, "images", "train")
labels_dir = os.path.join(base_dir, "labels", "train")

# Create YOLO directories
os.makedirs(images_dir, exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)

# 2. Sort and Move Files
print("Structuring Dataset...")
for filename in os.listdir(raw_dir):
    file_path = os.path.join(raw_dir, filename)
    if filename.endswith(".jpg"):
        shutil.copy(file_path, os.path.join(images_dir, filename))
    elif filename.endswith(".txt") and filename != "classes.txt":
        shutil.copy(file_path, os.path.join(labels_dir, filename))

# 3. Generate data.yaml configuration
yaml_path = os.path.join(base_dir, "data.yaml")
yaml_content = {
    "train": os.path.abspath(images_dir),
    "val": os.path.abspath(images_dir), # Using train for val in this micro-dataset
    "nc": 1,
    "names": ["drone"]
}

with open(yaml_path, 'w') as f:
    yaml.dump(yaml_content, f)

print(f"Configuration saved to {yaml_path}")

# 4. Initialize and Train YOLOv8
print("\nBooting Neural Network...")
# Load a pre-trained nano model (fastest for edge inference)
model = YOLO('yolov8n.pt') 

print("Starting Training Loop (Epochs: 50)...")
results = model.train(
    data=yaml_path,
    epochs=50,       # Number of times the AI reviews the dataset
    imgsz=640,       # Standard image resolution for YOLO
    batch=4,         # Number of images processed simultaneously
    name="swarm_drone_detector"
)

print("\nTraining Complete! The AI brain has been secured.")