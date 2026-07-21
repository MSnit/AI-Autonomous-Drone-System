import airsim
import time
import math
import requests

print("Starting Edge Node Telemetry Broadcaster...")

# The API endpoint of our dashboard server
# If you use ngrok later, you will replace this with your public ngrok URL
CLOUD_URL = "http://127.0.0.1:5000/api/telemetry/update"

# Connect to the drone (Edge device)
client = airsim.MultirotorClient()
client.confirmConnection()

try:
    while True:
        # SENSE: Read local IMU and GPS data
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        orientation = state.kinematics_estimated.orientation
        
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)
        speed = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2)
        
        # PACKAGE: Format the data as a JSON payload
        payload = {
            "status": "LIVE (EDGE LINK)",
            "altitude": round(-pos.z_val, 2),
            "speed": round(speed, 2),
            "pitch": round(math.degrees(pitch), 1),
            "roll": round(math.degrees(roll), 1),
            "yaw": round(math.degrees(yaw), 1)
        }
        
        # TRANSMIT: Beam the data to the web server
        try:
            response = requests.post(CLOUD_URL, json=payload)
            print(f"Data pushed -> Alt: {payload['altitude']}m | Spd: {payload['speed']}m/s | Server: {response.status_code}")
        except requests.exceptions.ConnectionError:
            print("ERROR: Could not connect to the cloud server. Is cloud_dashboard.py running?")
            
        # Transmit at 2Hz (twice a second) to save bandwidth
        time.sleep(0.5) 
        
except KeyboardInterrupt:
    print("Edge transmission terminated.")