import airsim
import cv2
import numpy as np
from ultralytics import YOLO

print("Initializing Live Edge Inference Engine...")

# 1. Load the Custom AI Brain
model_path = "runs/detect/swarm_drone_detector/weights/best.pt"
model = YOLO(model_path)
print("Neural weights loaded successfully.")

# 2. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

print("Deploying Swarm...")
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2").join()

# Position Drone 2 directly in front of Drone 1's camera
client.moveToPositionAsync(5, 0, -3, 2, vehicle_name="Drone2")
client.moveToPositionAsync(0, 0, -3, 2, vehicle_name="Drone1").join()

print("Activating Neural Vision (Press 'q' in the video window to quit)...")

# 3. Live Inference Loop
while True:
    # Pull raw frame from Drone 1
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
    response = responses[0]
    
    # Convert bytes to an OpenCV image matrix
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
    frame = img1d.reshape(response.height, response.width, 3)
    
    # Pass the frame through your trained YOLOv8 model (conf=0.6 means 60% confidence threshold)
    results = model.predict(source=frame, conf=0.6, verbose=False)
    
    # Render the AI's bounding boxes and confidence scores onto the image
    annotated_frame = results[0].plot()
    
    # Display the HUD
    cv2.imshow("Drone 1: Neural Target Tracking", annotated_frame)
    
    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 4. Secure Swarm
print("Mission aborted. Grounding Swarm...")
cv2.destroyAllWindows()

client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()

for drone in ["Drone1", "Drone2"]:
    client.armDisarm(False, vehicle_name=drone)
    client.enableApiControl(False, vehicle_name=drone)