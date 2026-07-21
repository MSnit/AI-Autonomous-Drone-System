import airsim
import cv2
import numpy as np
import time
from ultralytics import YOLO

print("Initializing Visual Servoing System...")

# 1. Load YOLOv8 for Target Acquisition
yolo = YOLO('yolov8n.pt')

# 2. Connect to the Simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off to scanning altitude...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()

# 3. Control Loop Parameters
# Kp is the Proportional Gain. If it tracks too slow, increase it. If it wobbles, decrease it.
Kp = 0.5 
TARGET_CLASS = 0  # 0 = Person, 2 = Car (COCO dataset)

print("Searching for target... Press 'q' to abort.")

try:
    while True:
        # SENSE: Grab the camera frame
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8).copy()
        frame = img1d.reshape(responses[0].height, responses[0].width, 3)
        
        frame_center_x = frame.shape[1] // 2
        
        # PROCESS: Run Object Detection
        results = yolo(frame, stream=True, verbose=False)
        target_locked = False
        
        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) == TARGET_CLASS:
                    # Calculate the center of the target's bounding box
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    box_center_x = (x1 + x2) // 2
                    
                    # Calculate the pixel error from the center of the screen
                    error_x = box_center_x - frame_center_x
                    
                    # ACTUATE: Apply Proportional Control to calculate Yaw Rate
                    # Normalize the error based on frame width so yaw rate isn't extreme
                    yaw_rate = Kp * (error_x / frame.shape[1]) * 100 
                    
                    # Send rotational command while holding altitude
                    client.moveByVelocityBodyFrameAsync(0, 0, 0, 0.1, 
                        yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate))
                    
                    # Visual Telemetry
                    cv2.circle(frame, (box_center_x, (y1+y2)//2), 5, (0, 0, 255), -1)
                    cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (0, 255, 0), 1)
                    cv2.putText(frame, f"LOCKED | Yaw Rate: {yaw_rate:.2f}", (20, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    target_locked = True
                    break # Track the first detected target
            
            if target_locked:
                break
                
        if not target_locked:
            # Hover in place if target is lost
            client.moveByVelocityBodyFrameAsync(0, 0, 0, 0.1, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0))
            cv2.putText(frame, "SEARCHING...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Visual Servoing Target Lock", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        # CPU Thermal Throttle
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Tracking aborted via terminal.")

# Safe Landing Sequence
print("Landing...")
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()