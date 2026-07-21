import airsim
import time

# 1. Connect to the simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()

# Climb to 3 meters altitude (-3 in NED frame)
target_altitude = -3.0
client.moveToZAsync(target_altitude, 2).join()
print(f"Stabilized at altitude: {-target_altitude} meters.")

# 2. Define 3D Waypoints (Vector3r objects in NED frame)
# Square patrol perimeter: (10m North) -> (10m East) -> (Back South) -> (Back Home)
waypoints = [
    airsim.Vector3r(10, 0, target_altitude),   # Waypoint 1: (10, 0, -3)
    airsim.Vector3r(10, 10, target_altitude),  # Waypoint 2: (10, 10, -3)
    airsim.Vector3r(0, 10, target_altitude),   # Waypoint 3: (0, 10, -3)
    airsim.Vector3r(0, 0, target_altitude)     # Waypoint 4: (0, 0, -3) Return Home
]

SPEED = 3.0  # Speed in m/s

print("\n--- Starting Waypoint Patrol Mission ---")

# 3. Point-to-Point Execution Loop
for i, wp in enumerate(waypoints):
    print(f"Navigating to Waypoint {i+1}: X={wp.x_val}m | Y={wp.y_val}m | Z={wp.z_val}m...")
    
    # moveToPositionAsync(x, y, z, velocity)
    # .join() blocks execution until the target position is reached
    client.moveToPositionAsync(wp.x_val, wp.y_val, wp.z_val, SPEED).join()
    
    print(f"-> Reached Waypoint {i+1}! Holding position for 2 seconds...")
    time.sleep(2)

# 4. Safe Mission Wrap-up
print("\n--- Mission Complete! Executing Landing ---")
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print("Drone safely disarmed and on the ground.")