import airsim
import time
import math

# 1. Connect and initialize
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off and stabilizing...")
client.takeoffAsync().join()
target_z = -3.0
client.moveToZAsync(target_z, 2).join()

# 2. Define the target destination (50m North, 20m East)
target_x = 50.0
target_y = 20.0
speed = 3.0

print(f"Target locked at coordinates X:{target_x}m, Y:{target_y}m.")
print("Beginning dynamic vector navigation...")

try:
    while True:
        # 3. Pull real-time telemetry
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        
        # Calculate the Euclidean distance to the target
        dist = math.sqrt((target_x - pos.x_val)**2 + (target_y - pos.y_val)**2)
        
        # Stop condition: if we are within 1 meter of the target, we have arrived
        if dist < 1.0:
            print("Destination reached! Holding position.")
            break
            
        # 4. Calculate the normalized velocity vectors
        dir_x = (target_x - pos.x_val) / dist
        dir_y = (target_y - pos.y_val) / dist
        
        # Push the velocity command to the physics engine
        client.moveByVelocityAsync(dir_x * speed, dir_y * speed, 0, 1)
        
        # 5. CPU Throttle: Restrict Python loop to 20 FPS to prevent overheating
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("Flight sequence aborted by user.")

# 6. Safe Landing Sequence
print("Executing landing protocol...")
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print("System powered down.")