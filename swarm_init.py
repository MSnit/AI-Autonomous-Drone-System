import airsim
import time

print("Initializing Swarm Protocol...")

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# 1. Take Control of the Swarm
for drone_id in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone_id)
    client.armDisarm(True, vehicle_name=drone_id)
    print(f"{drone_id} Armed and Ready.")

print("Executing Synchronized Takeoff...")

# 2. Asynchronous Launch
# We do not use .join() immediately so they lift off at the exact same time
task1 = client.takeoffAsync(vehicle_name="Drone1")
task2 = client.takeoffAsync(vehicle_name="Drone2")

# Now we wait for both to finish ascending
task1.join()
task2.join()

print("Swarm airborne. Hovering...")
time.sleep(5)

print("Executing Synchronized Landing...")
land1 = client.landAsync(vehicle_name="Drone1")
land2 = client.landAsync(vehicle_name="Drone2")
land1.join()
land2.join()

# 3. Secure the Swarm
for drone_id in ["Drone1", "Drone2"]:
    client.armDisarm(False, vehicle_name=drone_id)
    client.enableApiControl(False, vehicle_name=drone_id)

print("Swarm successfully grounded.")