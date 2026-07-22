import airsim
import cv2
import numpy as np
import time

print("Initializing Stereo Vision Depth Perception...")

# 1. Connect and Sync Drones
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

# 2. Establish Stereo Baseline (Position drones side-by-side)
print("Positioning Swarm for Stereo Capture...")
client.takeoffAsync(vehicle_name="Drone1")
client.takeoffAsync(vehicle_name="Drone2").join()

client.moveToPositionAsync(0, -2, -3, 2, vehicle_name="Drone1") # Left eye
client.moveToPositionAsync(0, 2, -3, 2, vehicle_name="Drone2").join() # Right eye
time.sleep(2) # Stabilize cameras

# 3. Simultaneous Image Capture
print("Capturing Left and Right Visual Feeds...")
response1 = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
response2 = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone2")

img1_1d = np.frombuffer(response1[0].image_data_uint8, dtype=np.uint8)
img2_1d = np.frombuffer(response2[0].image_data_uint8, dtype=np.uint8)

img1_rgb = img1_1d.reshape(response1[0].height, response1[0].width, 3)
img2_rgb = img2_1d.reshape(response2[0].height, response2[0].width, 3)

img1_gray = cv2.cvtColor(img1_rgb, cv2.COLOR_RGB2GRAY)
img2_gray = cv2.cvtColor(img2_rgb, cv2.COLOR_RGB2GRAY)

# 4. Feature Extraction (SIFT provides high accuracy for depth)
sift = cv2.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1_gray, None)
kp2, des2 = sift.detectAndCompute(img2_gray, None)

# 5. Feature Matching (FLANN based matcher for speed)
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)

good_matches = []
pts1 = []
pts2 = []

# Lowe's ratio test to filter out bad matches
for i, (m, n) in enumerate(matches):
    if m.distance < 0.8 * n.distance:
        good_matches.append(m)
        pts1.append(kp1[m.queryIdx].pt)
        pts2.append(kp2[m.trainIdx].pt)

pts1 = np.int32(pts1)
pts2 = np.int32(pts2)

# 6. Calculate the Fundamental Matrix using RANSAC
print(f"Analyzing {len(good_matches)} geometric anchor points...")
F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC)

print("\n--- FUNDAMENTAL MATRIX CALCULATED ---")
print(F)
print("-------------------------------------")

# We select only the inlier points calculated by the matrix
pts1 = pts1[mask.ravel() == 1]
pts2 = pts2[mask.ravel() == 1]

# Display the matched geometric features
matched_img = cv2.drawMatches(img1_rgb, kp1, img2_rgb, kp2, good_matches[:50], None, flags=2)
cv2.imshow("Stereo Vision Feature Mapping", matched_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 7. Secure Swarm
print("Grounding Swarm.")
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()