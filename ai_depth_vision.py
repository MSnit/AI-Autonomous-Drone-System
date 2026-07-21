import airsim
import cv2
import torch
import numpy as np

print("Loading MiDaS Depth Estimation Model...")
# 1. Load the lightweight MiDaS model from PyTorch Hub
model_type = "MiDaS_small"
midas = torch.hub.load("intel-isl/MiDaS", model_type)

# Move the model to the GPU if you have one, otherwise use CPU
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval() # Set model to evaluation mode

# 2. Load the specific image transformations required by MiDaS
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.small_transform

# 3. Connect to the AirSim drone
client = airsim.MultirotorClient()
client.confirmConnection()
print("Connected! Press 'q' to close the video windows.")

while True:
    # 4. Pull the front-facing RGB camera feed from AirSim
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    response = responses[0]

    # Convert raw bytes to a 3D OpenCV image array
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    frame = img1d.reshape(response.height, response.width, 3)
    
    # 5. Format the image for the AI
    # OpenCV uses BGR, but MiDaS expects RGB
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_batch = transform(img_rgb).to(device)

    # 6. Run the Depth Estimation Prediction
    with torch.no_grad():
        prediction = midas(input_batch)
        
        # Resize the output prediction to match our original AirSim camera resolution
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=img_rgb.shape[:2],
            mode="bicubic",
            align_corners=False,
        ).squeeze()

    # 7. Normalize the depth map so we can visualize it
    output = prediction.cpu().numpy()
    
    # Scale the output values between 0 and 255 for OpenCV
    depth_min = output.min()
    depth_max = output.max()
    if depth_max - depth_min > 0:
        output = (output - depth_min) / (depth_max - depth_min)
    else:
        output = np.zeros_like(output)
        
    output = (output * 255).astype(np.uint8)
    
    # Apply a heat-map color scheme (Magma looks fantastic for depth)
    depth_colormap = cv2.applyColorMap(output, cv2.COLORMAP_MAGMA)

    # 8. Display both the original feed and the AI Depth feed side-by-side
    cv2.imshow("AirSim RGB Feed", frame)
    cv2.imshow("MiDaS AI Depth Estimation", depth_colormap)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()