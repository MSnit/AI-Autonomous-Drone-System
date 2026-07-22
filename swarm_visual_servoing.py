import airsim
import cv2
import numpy as np
import time
from ultralytics import YOLO

print("Initializing Visual Servoing Tracking Loop...")

# 1. Load the AI Brain
model = YOLO("runs/detect/swarm_drone_detector/weights/best.pt")

# 2. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

print("Deploying Swarm...")
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2").join()

# Position Drone 2 directly in front of Drone 1 (4 meters ahead at same altitude)
client.moveToPositionAsync(0, 0, -3, 2, vehicle_name="Drone1")
client.moveToPositionAsync(4, 0, -3, 2, vehicle_name="Drone2").join()

# Pause briefly so Drone 1's camera is pointing straight at Drone 2
time.sleep(1.0)

# Start Drone 2 on a gentle drift (0.5 m/s)
client.moveByVelocityAsync(0, 0.5, 0, 30, vehicle_name="Drone2")

print("Engaging Visual Lock. Press 'q' to abort.")

# Proportional Control Constants
k_yaw = 0.15    # INCREASED: Drone 1 will now turn 3x faster to track lateral movement
k_pitch = 0.005 
target_area = 25000

while True:
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
    response = responses[0]
    
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
    frame = img1d.reshape(response.height, response.width, 3).copy()
    img_h, img_w, _ = frame.shape
    img_center_x = img_w / 2
    
    # Lowered conf to 0.4 so motion blur doesn't break the visual lock
    results = model.predict(source=frame, conf=0.4, verbose=False)
    
    yaw_rate = 0.0
    v_x = 0.0 
    target_found = False
    
    if len(results[0].boxes) > 0:
        for box_obj in results[0].boxes:
            box = box_obj.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = box
            
            box_area = (x2 - x1) * (y2 - y1)
            frame_area = img_w * img_h
            
            # Filter out whole-screen false positives
            if box_area > (0.5 * frame_area):
                continue
                
            box_center_x = (x1 + x2) / 2
            
            # Visual Servoing P-Controller
            error_x = box_center_x - img_center_x
            yaw_rate = error_x * k_yaw
            
            error_area = target_area - box_area
            v_x = error_area * k_pitch
            v_x = np.clip(v_x, -1.5, 1.5)
            
            # Draw HUD
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (int(box_center_x), int((y1+y2)/2)), 5, (0, 0, 255), -1)
            cv2.putText(frame, "TARGET LOCKED", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            target_found = True
            break
            
    if not target_found:
        cv2.putText(frame, "SEARCHING...", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # Slowly rotate (yaw) to scan for Drone 2 if lost
        yaw_rate = 0.3 
        v_x = 0.0
        
    # Execute Flight Command (cast to native floats for msgpack RPC)
    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate))
    client.moveByVelocityAsync(float(v_x), 0.0, 0.0, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode, vehicle_name="Drone1")
    
    cv2.imshow("Drone 1: Visual Servoing HUD", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()

for drone in ["Drone1", "Drone2"]:
    client.armDisarm(False, vehicle_name=drone)
    client.enableApiControl(False, vehicle_name=drone)

# 3. Live Servoing Loop
while True:
    # Pull raw frame from Drone 1
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
    response = responses[0]
    
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
    frame = img1d.reshape(response.height, response.width, 3).copy()
    img_h, img_w, _ = frame.shape
    img_center_x = img_w / 2
    
    # Process through YOLOv8
    results = model.predict(source=frame, conf=0.6, verbose=False)
    
    yaw_rate = 0.0
    v_x = 0.0 # Forward velocity
    
    if len(results[0].boxes) > 0:
        # Get the pixel coordinates of the first detected target
        box = results[0].boxes[0].xyxy[0].cpu().numpy()
        x1, y1, x2, y2 = box
        
        box_center_x = (x1 + x2) / 2
        box_area = (x2 - x1) * (y2 - y1)
        
        # --- SENSOR FUSION MATH ---
        # 1. Yaw Error: Difference between image center and bounding box center
        error_x = box_center_x - img_center_x
        yaw_rate = error_x * k_yaw
        
        # 2. Distance Error: Difference between ideal box size and current box size
        error_area = target_area - box_area
        v_x = error_area * k_pitch
        v_x = np.clip(v_x, -1.5, 1.5) # Cap max speed for safety
        
        # Draw Targeting HUD
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.circle(frame, (int(box_center_x), int((y1+y2)/2)), 5, (0, 0, 255), -1)
        cv2.putText(frame, "TARGET LOCKED", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "SEARCHING...", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
    # --- FLIGHT EXECUTION ---
    # Convert our calculated yaw rate and forward velocity into physical motor commands
    # Convert our calculated yaw rate and forward velocity into physical motor commands
    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate))
    client.moveByVelocityAsync(float(v_x), 0.0, 0.0, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode, vehicle_name="Drone1")
    
    # Display HUD
    cv2.imshow("Drone 1: Visual Servoing HUD", frame)
    
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