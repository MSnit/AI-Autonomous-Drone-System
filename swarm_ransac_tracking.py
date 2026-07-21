import airsim
import cv2
import numpy as np
import time

print("Initializing RANSAC Visual Target Lock...")

# 1. Connect and Launch Swarm
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

print("Synchronized Takeoff...")
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2").join()

client.moveToZAsync(-3, 2, vehicle_name="Drone1")
client.moveToZAsync(-3, 2, vehicle_name="Drone2").join()

# Move Drone 2 directly in front of Drone 1's camera
print("Positioning Target (Drone 2) in Field of View...")
client.moveToPositionAsync(5, 0, -3, 2, vehicle_name="Drone2").join()
time.sleep(2) # Allow camera to stabilize

# 2. Acquire Template (Target Lock Initialization)
print("Acquiring Initial Target Template...")
responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
template_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
template_gray = cv2.cvtColor(template_rgb, cv2.COLOR_RGB2GRAY)

# Define the central region where Drone 2 is hovering
height, width = template_gray.shape
h_start, h_end = int(height*0.4), int(height*0.6)
w_start, w_end = int(width*0.4), int(width*0.6)
target_roi = template_gray[h_start:h_end, w_start:w_end]

# 3. Initialize ORB Detector
orb = cv2.ORB_create(nfeatures=500)
kp_template, des_template = orb.detectAndCompute(target_roi, None)

# Set Drone 2 in motion across the field of view
print("Initiating Target Evasive Maneuvers...")
client.moveToPositionAsync(5, 10, -3, 1.5, vehicle_name="Drone2")

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# 4. Continuous Tracking Loop
while True:
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
    if responses[0].width == 0: continue
    
    img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    frame_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
    frame_gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    # Extract features from live frame
    kp_frame, des_frame = orb.detectAndCompute(frame_gray, None)
    
    if des_template is not None and des_frame is not None:
        matches = bf.match(des_template, des_frame)
        # Sort matches by distance (best matches first)
        matches = sorted(matches, key=lambda x: x.distance)

        # We need at least 4 matches to calculate a homography matrix
        if len(matches) > 10:
            # Extract coordinates of good matches
            src_pts = np.float32([kp_template[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

            # Apply RANSAC to filter outliers and find the Homography matrix
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            if H is not None:
                # Get dimensions of the target ROI
                h, w = target_roi.shape
                pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                
                # Project the bounding box onto the live frame
                dst = cv2.perspectiveTransform(pts, H)
                frame_bgr = cv2.polylines(frame_bgr, [np.int32(dst)], True, (0, 255, 0), 3, cv2.LINE_AA)
                cv2.putText(frame_bgr, "TARGET LOCKED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame_bgr, "SEARCHING...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Drone 1 - Tactical Targeting System", frame_bgr)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()