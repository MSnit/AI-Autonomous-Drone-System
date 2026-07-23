import airsim
import cv2
import numpy as np
import time
import os

print("Initializing Targeted Data Harvester...")

# 1. Setup Dataset Directory
dataset_dir = "dataset/raw_images"
os.makedirs(dataset_dir, exist_ok=True)

# 2. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

print("Deploying Swarm for Photo Session...")
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2").join()

# Position Drone 2 right in front of Drone 1
client.moveToPositionAsync(5, 0, -3, 2, vehicle_name="Drone2")
client.moveToPositionAsync(0, 0, -3, 2, vehicle_name="Drone1").join()

image_count = 0

# 3. Strafe and Capture (Drone 1 slides left and right while looking at Drone 2)
waypoints = [
    (0, 3, -3),   # Slide Right
    (0, -3, -3),  # Slide Left
    (0, 0, -3)    # Back to center
]

for wp in waypoints:
    client.moveToPositionAsync(wp[0], wp[1], wp[2], 2, vehicle_name="Drone1").join()
    
    # Take a rapid burst of 10 photos at each position
    for _ in range(10):
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8) 
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        
        filename = os.path.join(dataset_dir, f"actual_drone_{image_count:04d}.jpg")
        cv2.imwrite(filename, img_rgb)
        print(f"Captured: {filename}")
        image_count += 1
        time.sleep(0.5)

# 4. Secure Swarm
print(f"\nHarvesting Complete. Total valid images: {image_count}")
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()

for drone in ["Drone1", "Drone2"]:
    client.armDisarm(False, vehicle_name=drone)
    client.enableApiControl(False, vehicle_name=drone)