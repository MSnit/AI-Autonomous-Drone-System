import airsim
import cv2
import torch
import numpy as np

print("Loading MiDaS AI Depth Engine...")
model_type = "MiDaS_small"
midas = torch.hub.load("intel-isl/MiDaS", model_type)
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.small_transform

# 1. Connect and initialize the multirotor
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off to a safe altitude...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join() # Ascend to exactly 3 meters

print("Beginning autonomous patrol! Press 'q' to abort.")

# Flight parameters
SAFE_DISTANCE_THRESHOLD = 180  # Max depth brightness (0-255)
FORWARD_SPEED = 3.0            # m/s
DODGE_SPEED = 2.0              # m/s

try:
    while True:
        # 2. Sense: Grab the raw camera frame
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        frame = img1d.reshape(responses[0].height, responses[0].width, 3)

        # 3. Process: Generate the Depth Map
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = transform(img_rgb).to(device)

        with torch.no_grad():
            prediction = midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1), size=img_rgb.shape[:2], mode="bicubic", align_corners=False
            ).squeeze()

        output = prediction.cpu().numpy()
        
        # Normalize the map so 0 is infinitely far and 255 is a physical collision
        depth_min, depth_max = output.min(), output.max()
        if depth_max - depth_min > 0:
            output = (output - depth_min) / (depth_max - depth_min)
        else:
            output = np.zeros_like(output)
        
        depth_map = (output * 255).astype(np.uint8)

        # 4. Define the targeting reticle (Middle 40% of the screen)
        h, w = depth_map.shape
        roi_top, roi_bottom = int(h * 0.3), int(h * 0.7)
        roi_left, roi_right = int(w * 0.3), int(w * 0.7)
        
        center_roi = depth_map[roi_top:roi_bottom, roi_left:roi_right]
        mean_depth = np.mean(center_roi)
        
        # Draw the reticle so you can see exactly what the drone is analyzing
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 0), 2)

        # 5. Actuate: The Control Loop
        if mean_depth > SAFE_DISTANCE_THRESHOLD:
            # Threat detected: Kill forward velocity (vx=0) and strafe right (vy=DODGE_SPEED)
            cv2.putText(frame, f"OBSTACLE! Evading...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            client.moveByVelocityBodyFrameAsync(0, DODGE_SPEED, 0, 0.5)
        else:
            # Path clear: Push forward
            cv2.putText(frame, f"Clear (Proximity: {mean_depth:.1f})", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            client.moveByVelocityBodyFrameAsync(FORWARD_SPEED, 0, 0, 0.5)

        # 6. Output the telemetry
        depth_colormap = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)
        cv2.imshow("Drone Main Camera", frame)
        cv2.imshow("AI Spatial Mapping", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Flight sequence aborted via terminal.")

# 7. Safe Landing Sequence
print("Landing safely...")
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()