import airsim
import time

print("Initializing Multi-Agent Swarm Protocol...")

# 1. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# 2. Assume Control of BOTH Drones
for drone_name in ["Drone1", "Drone2"]:
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)

print("Swarm taking off...")
# We use Async without .join() immediately so they rise simultaneously
f1 = client.takeoffAsync(vehicle_name="Drone1")
f2 = client.takeoffAsync(vehicle_name="Drone2")
f1.join()
f2.join()

# Climb to identical altitude
client.moveToZAsync(-3, 2, vehicle_name="Drone1")
client.moveToZAsync(-3, 2, vehicle_name="Drone2").join()

print("Swarm airborne. Commencing formation flight...")

# Send the Leader (Drone1) flying straight North at 2 m/s
client.moveByVelocityAsync(2.0, 0, 0, 20, vehicle_name="Drone1")

# Wingman offset target: 2 meters behind (-X) and 2 meters right (+Y) of the Leader
OFFSET_X = -2.0
OFFSET_Y = 2.0
Kp = 1.2 # Proportional gain for the wingman's catch-up speed

try:
    while True:
        # 3. SENSE: Query Leader's exact coordinates
        state1 = client.getMultirotorState(vehicle_name="Drone1")
        pos1 = state1.kinematics_estimated.position
        
        # 4. SENSE: Query Wingman's exact coordinates
        state2 = client.getMultirotorState(vehicle_name="Drone2")
        pos2 = state2.kinematics_estimated.position
        
        # 5. PROCESS: Calculate Wingman's target destination
        target_x = pos1.x_val + OFFSET_X
        target_y = pos1.y_val + OFFSET_Y
        target_z = pos1.z_val
        
        # Calculate the 3D error vectors (Distance to target)
        err_x = target_x - pos2.x_val
        err_y = target_y - pos2.y_val
        err_z = target_z - pos2.z_val
        
        # 6. ACTUATE: Fire Wingman thrusters proportionally to the error
        client.moveByVelocityAsync(
            err_x * Kp, 
            err_y * Kp, 
            err_z * Kp, 
            0.1, 
            vehicle_name="Drone2"
        )
        
        # CPU Thermal Throttle (Mandatory for laptop stability)
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("Swarm sequence aborted.")

# 7. Safe Swarm Landing Sequence
print("Landing swarm...")
client.hoverAsync(vehicle_name="Drone1")
client.hoverAsync(vehicle_name="Drone2")
time.sleep(1)

f1 = client.landAsync(vehicle_name="Drone1")
f2 = client.landAsync(vehicle_name="Drone2")
f1.join()
f2.join()

for drone_name in ["Drone1", "Drone2"]:
    client.armDisarm(False, drone_name)
    client.enableApiControl(False, drone_name)

print("Swarm safely grounded.")