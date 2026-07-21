import airsim
import cv2
import numpy as np
import time

print("Initializing Swarm Machine Perception...")

# 1. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

print("Accessing Drone 1 Primary Camera...")

# 2. Continuous Vision Loop
while True:
    # Request RGB image from Drone 1's front camera (Camera ID "0", Scene Type)
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], vehicle_name="Drone1")
    
    response = responses[0]
    
    # Ensure valid image data is received
    if response.width == 0 or response.height == 0:
        print("Waiting for camera initialization...")
        time.sleep(0.5)
        continue

    # 3. Matrix Transformation
    # Convert raw bytes to a 1D numpy array
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    
    # Reshape to a 3D matrix (Height x Width x Channels)
    img_rgb = img1d.reshape(response.height, response.width, 3)
    
    # AirSim returns RGB, but OpenCV processes in BGR
    img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    
    # 4. Render the Feed
    cv2.imshow("Drone 1 - Optical Sensor Feed", img_bgr)
    
    # Press 'q' in the video window to safely exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Terminating vision feed...")
        break

cv2.destroyAllWindows()