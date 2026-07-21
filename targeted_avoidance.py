import airsim
import cv2
import torch
import numpy as np
from ultralytics import YOLO

print("Initializing Dual-AI Vision System...")
# 1. Load YOLOv8 (Object Detection)
yolo = YOLO('yolov8n.pt')

# 2. Load MiDaS (Depth Estimation)
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()
transform = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform

# 3. Connect to Drone
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()

# Flight parameters
SAFE_DEPTH_THRESHOLD = 170  
FORWARD_SPEED = 3.0
DODGE_SPEED = 2.0

try:
    while True:
        # 4. SENSE: Pull Camera Frame
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        frame = img1d.reshape(responses[0].height, responses[0].width, 3)

        # 5. PROCESS: YOLO Object Detection
        yolo_results = yolo(frame, stream=True, verbose=False)
        
        # 6. PROCESS: MiDaS Depth Map
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = transform(img_rgb).to(device)
        with torch.no_grad():
            prediction = midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1), size=img_rgb.shape[:2], mode="bicubic", align_corners=False
            ).squeeze()
        
        output = prediction.cpu().numpy()
        depth_min, depth_max = output.min(), output.max()
        if depth_max - depth_min > 0:
            output = (output - depth_min) / (depth_max - depth_min)
        else:
            output = np.zeros_like(output)
            
        depth_map = (output * 255).astype(np.uint8)
        
        threat_detected = False

        # 7. FUSION: Map YOLO Boxes to Depth Data
        for r in yolo_results:
            boxes = r.boxes
            for box in boxes:
                # Get Class ID (0 = Person, 2 = Car in COCO dataset)
                cls_id = int(box.cls[0])
                if cls_id in [0, 2]: 
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Extract the depth data strictly inside the bounding box
                    roi_depth = depth_map[y1:y2, x1:x2]
                    if roi_depth.size > 0:
                        mean_proximity = np.mean(roi_depth)
                        
                        # Draw YOLO box on the frame
                        label = f"{yolo.names[cls_id]} | Prox: {mean_proximity:.1f}"
                        color = (0, 0, 255) if mean_proximity > SAFE_DEPTH_THRESHOLD else (0, 255, 0)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        
                        if mean_proximity > SAFE_DEPTH_THRESHOLD:
                            threat_detected = True

        # 8. ACTUATE: Flight Control Loop
        if threat_detected:
            cv2.putText(frame, "TARGET EVASION!", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            client.moveByVelocityBodyFrameAsync(0, DODGE_SPEED, 0, 0.5)
        else:
            client.moveByVelocityBodyFrameAsync(FORWARD_SPEED, 0, 0, 0.5)

        # 9. Output Telemetry
        depth_colormap = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)
        cv2.imshow("Targeting & Fusion", frame)
        cv2.imshow("Spatial Map", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Aborting flight...")

client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()
