import airsim
import cv2
import numpy as np

# 1. Connect to the simulator
client = airsim.MultirotorClient()
client.confirmConnection()

print("Connecting to drone camera... Press 'q' to close the video window.")

# 2. Start a loop to continuously pull frames
while True:
    # Request the front camera feed (Camera ID "0", Scene image)
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ])
    
    response = responses[0]

    # Convert the raw bytes into a 1D NumPy array
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    
    # Reshape the 1D array into a 3D image array (Height x Width x 3 color channels)
    # AirSim returns images in BGRA format, but OpenCV uses BGR, so we just take the first 3 channels
    img_bgr = img1d.reshape(response.height, response.width, 3)

    # 3. Display the live feed
    cv2.imshow("Drone Live Feed", img_bgr)

    # 4. Break the loop if the user presses 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()