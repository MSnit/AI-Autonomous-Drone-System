import tkinter as tk
import tkintermapview
import math
import heapq
import airsim
import threading
import time

print("Initializing Advanced GCS with Live Telemetry...")

HOME_LAT = 21.1458
HOME_LON = 79.0882

# 1. Coordinate Translation Engines
def gps_to_cartesian(target_lat, target_lon):
    R = 6378137.0
    lat1, lat2 = math.radians(HOME_LAT), math.radians(target_lat)
    lon1, lon2 = math.radians(HOME_LON), math.radians(target_lon)
    x = (lon2 - lon1) * R * math.cos((lat1 + lat2) / 2)
    y = (lat2 - lat1) * R
    return round(x, 2), round(y, 2)

def cartesian_to_gps(x, y):
    R = 6378137.0
    lat_offset = (y / R) * (180 / math.pi)
    lon_offset = (x / (R * math.cos(math.radians(HOME_LAT)))) * (180 / math.pi)
    return HOME_LAT + lat_offset, HOME_LON + lon_offset

# 2. A* Pathfinding Logic
def heuristic(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def astar(start, goal, obstacles):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from, g_score = {}, {start: 0}
    f_score = {start: heuristic(start, goal)}
    safe_margin = 3.0 
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if heuristic(current, goal) < 1.5:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
            
        neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]),
                     (current[0], current[1]+1), (current[0], current[1]-1),
                     (current[0]+1, current[1]+1), (current[0]-1, current[1]-1),
                     (current[0]+1, current[1]-1), (current[0]-1, current[1]+1)]
                     
        for neighbor in neighbors:
            if any(heuristic(neighbor, obs) < safe_margin for obs in obstacles):
                continue
                
            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return []


# 3. Live Telemetry Thread (The Radar)
def update_telemetry(drone_marker):
    """Runs on its own independent AirSim client to prevent IOLoop crashes."""
    telemetry_client = airsim.MultirotorClient()
    telemetry_client.confirmConnection()
    
    while telemetry_active:
        state = telemetry_client.getMultirotorState(vehicle_name="Drone1")
        pos = state.kinematics_estimated.position
        
        # Convert simulator X/Y back to GPS Latitude/Longitude
        live_lat, live_lon = cartesian_to_gps(pos.x_val, pos.y_val)
        
        # THREAD-SAFE GUI UPDATE: Schedule the marker move on the main Tkinter thread
        root.after(0, drone_marker.set_position, live_lat, live_lon)
        
        time.sleep(0.5) # Poll at 2Hz

# 4. The Smart Flight Thread
def execute_smart_mission(target_x, target_y, drone_marker):
    global telemetry_active
    telemetry_active = True
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Start the radar ping (Removed 'client' argument, passing only the marker)
    threading.Thread(target=update_telemetry, args=(drone_marker,), daemon=True).start()
    
    status_label.config(text=f"STATUS: Calculating Route...", fg="yellow")
    known_obstacles = [(10, 10)] 
    start_pos = (0, 0)
    goal_pos = (int(target_x), int(target_y))
    
    path = astar(start_pos, goal_pos, known_obstacles)
    
    if not path:
        status_label.config(text="ERROR: No safe path.", fg="red")
        telemetry_active = False
        return
        
    status_label.config(text=f"STATUS: Airborne. Following {len(path)} waypoints.", fg="orange")
    
    client.enableApiControl(True, vehicle_name="Drone1")
    client.armDisarm(True, vehicle_name="Drone1")
    client.takeoffAsync(vehicle_name="Drone1").join()
    client.moveToZAsync(-5, 2, vehicle_name="Drone1").join()
    
    for waypoint in path:
        client.moveToPositionAsync(waypoint[0], waypoint[1], -5, 4, vehicle_name="Drone1").join()
    
    client.moveToPositionAsync(target_x, target_y, -5, 2, vehicle_name="Drone1").join()
    
    status_label.config(text="STATUS: Mission Complete. Grounded.", fg="green")
    client.landAsync(vehicle_name="Drone1").join()
    client.armDisarm(False, vehicle_name="Drone1")
    client.enableApiControl(False, vehicle_name="Drone1")
    telemetry_active = False # Shut down the radar

# 5. Dispatch Trigger
def trigger_dispatch(coordinates):
    target_lat, target_lon = coordinates
    map_widget.delete_all_marker()
    map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home")
    map_widget.set_marker(target_lat, target_lon, text="Target")
    
    # Create the live tracker icon
    drone_marker = map_widget.set_marker(HOME_LAT, HOME_LON, text="🚁 Drone 1 (LIVE)", text_color="blue")
    
    x, y = gps_to_cartesian(target_lat, target_lon)
    threading.Thread(target=execute_smart_mission, args=(x, y, drone_marker)).start()

# GUI Setup
root = tk.Tk()
root.geometry("900x700")
root.title("Swarm Control - Command Center")
status_label = tk.Label(root, text="STATUS: System Ready. Awaiting Input.", font=("Courier", 12, "bold"), bg="black", fg="white")
status_label.pack(fill="x")
map_widget = tkintermapview.TkinterMapView(root, width=900, height=600, corner_radius=0)
map_widget.pack(fill="both", expand=True)
map_widget.set_position(HOME_LAT, HOME_LON) 
map_widget.set_zoom(18)
map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home")
map_widget.add_right_click_menu_command(label="Deploy Swarm", command=trigger_dispatch, pass_coords=True)
telemetry_active = False

root.mainloop()