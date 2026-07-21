import airsim
import cv2
import numpy as np
import joblib
from skimage.feature import hog
import time

print("Initializing Autonomous Hunter Protocol v2 (With Collision Evasion)...")

# 1. Load the Custom AI Brain
model_filename = "drone_target_svm.pkl"
try:
    svm_model = joblib.load(model_filename)
    print(">>> AI Brain loaded successfully.")
except Exception as e:
    print(f"ERROR: Could not load model. Details: {e}")
    exit()

IMG_SIZE = (64, 64)

# 2. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-2, 2).join()

print("Initiating Search Pattern...")

try:
    while True:
        # SENSE 1: Check for physical collisions first!
        collision_info = client.simGetCollisionInfo()
        
        if collision_info.has_collided:
            print(f"IMPACT DETECTED with {collision_info.object_name}! Executing evasion...")
            
            # Flash a warning on the video feed
            dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(dummy_frame, "COLLISION! REVERSING...", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            cv2.imshow("Drone AI Vision", dummy_frame)
            cv2.waitKey(1)
            
            # ACTUATE (Override): Step 1 - Reverse at 2 m/s for 1.5 seconds (3 meters total)
            client.moveByVelocityBodyFrameAsync(-2.0, 0.0, 0.0, 1.5, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0)).join()
            
            # ACTUATE (Override): Step 2 - Spin to face a new clear path
            client.moveByVelocityBodyFrameAsync(0.0, 0.0, 0.0, 1.5, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=60.0)).join()
            
            print("Evasion complete. Resuming AI control.")
            continue # Skip the rest of the loop and start fresh
            

        # SENSE 2: Pull live camera frame for the AI
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        if not responses or not responses[0].image_data_uint8:
            continue
            
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8).copy()
        frame = img1d.reshape(responses[0].height, responses[0].width, 3)
        
        # PROCESS: Prepare image for the SVM
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, IMG_SIZE)
        features = hog(resized, orientations=9, pixels_per_cell=(8, 8), 
                       cells_per_block=(2, 2), visualize=False).reshape(1, -1)
        
        # Predict (0 = background, 1 = target)
        prediction = svm_model.predict(features)[0]
        
        display_frame = frame.copy()
        
        # ACTUATE: Flight Logic based on AI classification
        if prediction == 1:
            cv2.putText(display_frame, "TARGET DETECTED - PURSUING", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            client.moveByVelocityBodyFrameAsync(2.0, 0.0, 0.0, 0.1, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0))
        else:
            cv2.putText(display_frame, "SEARCHING...", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            client.moveByVelocityBodyFrameAsync(0.0, 0.0, 0.0, 0.1, 
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=30.0))
        
        cv2.imshow("Drone AI Vision", display_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Flight aborted by user.")

# Safe Landing Sequence
print("Landing safely...")
client.hoverAsync().join()
client.landAsync()
time.sleep(4)
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()
print("System powered down.")