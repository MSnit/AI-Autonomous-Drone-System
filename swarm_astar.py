import airsim
import heapq
import math
import time

print("Initializing Cooperative A* Pathfinding...")

# 1. A* Algorithm Core
def heuristic(a, b):
    # Euclidean distance formula
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(start, goal, obstacle):
    """Calculates the shortest path avoiding the obstacle."""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    # Safe distance to keep away from the obstacle (Drone 2)
    safe_margin = 2.0 
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        # If we reached the goal, reconstruct the path
        if heuristic(current, goal) < 1.0:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
            
        # Check 8 directions (N, S, E, W and diagonals)
        neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]),
                     (current[0], current[1]+1), (current[0], current[1]-1),
                     (current[0]+1, current[1]+1), (current[0]-1, current[1]-1),
                     (current[0]+1, current[1]-1), (current[0]-1, current[1]+1)]
                     
        for neighbor in neighbors:
            # Check if this neighbor is too close to Drone 2
            if heuristic(neighbor, obstacle) < safe_margin:
                continue # Obstacle detected, skip this node!
                
            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
                
    return [] # No path found

# 2. Connect and Launch Swarm
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

# 3. Define the Mission
start_pos = (0, 0)
goal_pos = (10, 2) # 10 meters forward, 2 meters right
drone2_pos = (5, 1) # Drone 2 is parked in the middle as an obstacle

print(f"Moving Drone 2 to static obstacle position: {drone2_pos}")
client.moveToPositionAsync(drone2_pos[0], drone2_pos[1], -3, 2, vehicle_name="Drone2").join()

print("Calculating A* Path for Drone 1...")
# Calculate path avoiding Drone 2
path = astar(start_pos, goal_pos, drone2_pos)

if path:
    print(f"Path calculated with {len(path)} waypoints. Executing...")
    for waypoint in path:
        # Fly to each calculated waypoint at 3 m/s
        client.moveToPositionAsync(waypoint[0], waypoint[1], -3, 3, vehicle_name="Drone1").join()
        
    print("Mission Complete. Drone 1 reached goal.")
else:
    print("ERROR: No safe path could be found.")

# Ground the swarm
print("Landing...")
client.landAsync(vehicle_name="Drone1")
client.landAsync(vehicle_name="Drone2").join()