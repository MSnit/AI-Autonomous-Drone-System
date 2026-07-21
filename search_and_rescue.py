import airsim
import cv2
import numpy as np
import time
import math
from ultralytics import YOLO

print("Initializing Search & Rescue Protocol...")
yolo = YOLO('yolov8n.pt')

# 1. Connect and initialize
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()

# 2. FSM and Control Variables
mode = "PATROL"
TARGET_CLASS = 0  # 0 = Person
Kp = 0.5          # Visual Servoing Yaw Gain
SPEED = 3.0       # Patrol Speed

# Define a 20x20 meter search grid
waypoints = [(20, 0), (20, 20), (0, 20), (0, 0)]
wp_index = 0

print("Commencing Grid Patrol. Press 'q' to abort.")

try:
    while True:
        # SENSE: Pull Camera Frame
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8).copy()
        frame = img.reshape(responses[0].height, responses[0].width, 3)
        frame_center_x = frame.shape[1] // 2
        
        # PROCESS: YOLO Object Detection
        results = yolo(frame, stream=True, verbose=False)
        target_locked = False
        error_x = 0
        
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) == TARGET_CLASS:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    box_center_x = (x1 + x2) // 2
                    error_x = box_center_x - frame_center_x
                    target_locked = True
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    break # Focus on the first detected person
            if target_locked:
                break
                
        # ACTUATE: The Finite State Machine
        if target_locked:
            if mode == "PATROL":
                print(">>> TARGET DETECTED: Switching to INTERCEPT mode!")
                mode = "INTERCEPT"
                
            # Intercept Logic: Calculate yaw to lock on, push forward at 1.5 m/s
            yaw_rate = Kp * (error_x / frame.shape[1]) * 100 
            client.moveByVelocityBodyFrameAsync(1.5, 0, 0, 0.1, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate))
                
            cv2.putText(frame, f"INTERCEPTING | Error: {error_x}px", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
        else:
            if mode == "INTERCEPT":
                print("<<< Target Lost. Resuming PATROL grid.")
                mode = "PATROL"
                
            # Patrol Logic: Calculate vector to next waypoint
            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            target_x, target_y = waypoints[wp_index]
            
            dist = math.sqrt((target_x - pos.x_val)**2 + (target_y - pos.y_val)**2)
            
            if dist < 1.5:
                # Cycle to the next waypoint in the list
                wp_index = (wp_index + 1) % len(waypoints)
                print(f"Waypoint reached. Routing to: {waypoints[wp_index]}")
            else:
                # Calculate heading and push velocity
                dir_x = (target_x - pos.x_val) / dist
                dir_y = (target_y - pos.y_val) / dist
                yaw = math.atan2(dir_y, dir_x)
                
                client.moveByVelocityAsync(dir_x * SPEED, dir_y * SPEED, 0, 0.1,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(yaw)))
                    
            cv2.putText(frame, "PATROLLING", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Output Telemetry and manage CPU thermals
        cv2.imshow("Main Drone Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Flight sequence aborted.")

# Safe Landing Sequence
print("Landing safely...")
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()