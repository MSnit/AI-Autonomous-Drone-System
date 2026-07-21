import airsim
import heapq
import math
import time

print("Initializing Dynamic Swarm Evasion Protocol...")

def heuristic(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(start, goal, obstacle):
    """Calculates path with a larger safety margin for moving targets."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    # Increased safety margin since the target is moving
    safe_margin = 3.0 
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if heuristic(current, goal) < 1.0:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
            
        # 4-directional movement for tighter dynamic recalculation
        neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]),
                     (current[0], current[1]+1), (current[0], current[1]-1)]
                     
        for neighbor in neighbors:
            if heuristic(neighbor, obstacle) < safe_margin:
                continue 
                
            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []

# Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()

for drone in ["Drone1", "Drone2"]:
    client.enableApiControl(True, vehicle_name=drone)
    client.armDisarm(True, vehicle_name=drone)

print("Synchronized Takeoff...")
t1 = client.takeoffAsync(vehicle_name="Drone1")
t2 = client.takeoffAsync(vehicle_name="Drone2")
t1.join()
t2.join()

client.moveToZAsync(-3, 2, vehicle_name="Drone1").join()
client.moveToZAsync(-3, 2, vehicle_name="Drone2").join()

# Set Drone 2 on a continuous collision-course vector (No .join() so it runs in background)
print("Drone 2 entering patrol vector...")
client.moveToPositionAsync(5, 10, -3, 1.5, vehicle_name="Drone2")

# Drone 1 Mission Parameters
goal_pos = (10, 0)
print("Drone 1 initiating dynamic recalculation loop...")

while True:
    # 1. Ping Current States
    pos1 = client.getMultirotorState(vehicle_name="Drone1").kinematics_estimated.position
    pos2 = client.getMultirotorState(vehicle_name="Drone2").kinematics_estimated.position
    
    current_pos = (round(pos1.x_val), round(pos1.y_val))
    obs_pos = (round(pos2.x_val), round(pos2.y_val))
    
    # Check if goal reached
    if heuristic(current_pos, goal_pos) < 1.5:
        print("Target Destination Reached Successfully!")
        break
        
    # 2. Run A* against the *current* moving obstacle position
    path = astar(current_pos, goal_pos, obs_pos)
    
    if path:
        next_waypoint = path[0]
        print(f"Evasion Active | Obstacle at {obs_pos} | Rerouting to {next_waypoint}")
        # 3. Move exactly ONE node forward, then loop again
        client.moveToPositionAsync(next_waypoint[0], next_waypoint[1], -3, 4, vehicle_name="Drone1").join()
    else:
        print(f"WARNING: Collision imminent. Evasive hover.")
        time.sleep(0.5)

print("Mission Complete. Grounding Swarm.")
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()