import airsim
import cv2
import numpy as np
import os
import time

print("Initializing Custom Ground Control Station...")

# 1. Create Dataset Directory
dataset_dir = "custom_dataset"
target_dir = os.path.join(dataset_dir, "background")
os.makedirs(target_dir, exist_ok=True)

# 2. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-2, 2).join()

print("\n=== FLIGHT CONTROLS (Click the Video Window to fly) ===")
print("[W] / [S] : Fly Forward / Backward")
print("[A] / [D] : Rotate Left / Right")
print("[R] / [F] : Fly Up / Down")
print("[C]       : CAPTURE PHOTO")
print("[Q]       : QUIT & LAND")
print("=========================================================\n")

image_count = 0

try:
    while True:
        # SENSE: Pull Camera Frame
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        if not responses or not responses[0].image_data_uint8:
            continue
            
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8).copy()
        frame = img1d.reshape(responses[0].height, responses[0].width, 3)
        
        # Display the live feed
        display_frame = frame.copy()
        cv2.putText(display_frame, f"Photos Captured: {image_count}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Ground Control Station", display_frame)
        
        # SENSE: Read keyboard inputs from OpenCV (50ms refresh rate)
        key = cv2.waitKey(50) & 0xFF
        
        # Default hovering state (0 velocity)
        vx, vz, yaw_rate = 0.0, 0.0, 0.0
        
        if key == ord('w'): vx = 2.0             # Forward
        elif key == ord('s'): vx = -2.0          # Backward
        elif key == ord('a'): yaw_rate = -40.0   # Rotate Left
        elif key == ord('d'): yaw_rate = 40.0    # Rotate Right
        elif key == ord('r'): vz = -1.5          # Up (Z is negative in aerospace!)
        elif key == ord('f'): vz = 1.5           # Down 
        elif key == ord('c'):
            # CAPTURE PHOTO
            filename = os.path.join(target_dir, f"target_{image_count:04d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Captured: {filename}")
            image_count += 1
        elif key == ord('q'):
            break
            
        # ACTUATE: Push dynamic velocities to the drone
        client.moveByVelocityBodyFrameAsync(
            vx, 0.0, vz, 
            duration=0.1, 
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate)
        )

except KeyboardInterrupt:
    print("Flight aborted.")

# Safe Landing Sequence
print("Landing safely...")
client.hoverAsync().join()
client.landAsync()       # Start the landing sequence
time.sleep(4)            # Give it 4 seconds to physically reach the floor!

client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()
print("API connection closed.")