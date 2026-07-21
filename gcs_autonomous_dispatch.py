import tkinter as tk
import tkintermapview
import math
import airsim
import threading

print("Initializing Autonomous Ground Control Station...")

HOME_LAT = 21.1458
HOME_LON = 79.0882

# 1. The Drone Backend (Runs on a separate thread)
def execute_flight_mission(target_x, target_y):
    print("\n--- FLIGHT SYSTEMS ACTIVE ---")
    status_label.config(text=f"STATUS: Connecting to Swarm...", fg="yellow")
    
    # Connect to simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    client.enableApiControl(True, vehicle_name="Drone1")
    client.armDisarm(True, vehicle_name="Drone1")
    
    status_label.config(text=f"STATUS: Swarm Airborne. Routing to X:{target_x}m, Y:{target_y}m", fg="orange")
    
    # Takeoff and climb
    client.takeoffAsync(vehicle_name="Drone1").join()
    client.moveToZAsync(-5, 2, vehicle_name="Drone1").join()
    
    # Fly to the translated map coordinates
    client.moveToPositionAsync(target_x, target_y, -5, 5, vehicle_name="Drone1").join()
    
    status_label.config(text="STATUS: Destination Reached. Executing Auto-Land.", fg="green")
    print("Mission Complete. Grounding Drone.")
    
    # Land and secure
    client.landAsync(vehicle_name="Drone1").join()
    client.armDisarm(False, vehicle_name="Drone1")
    client.enableApiControl(False, vehicle_name="Drone1")

# 2. The Translation Middleware
def gps_to_cartesian(target_lat, target_lon):
    R = 6378137.0
    lat1, lat2 = math.radians(HOME_LAT), math.radians(target_lat)
    lon1, lon2 = math.radians(HOME_LON), math.radians(target_lon)
    
    x_meters = (lon2 - lon1) * R * math.cos((lat1 + lat2) / 2)
    y_meters = (lat2 - lat1) * R
    
    return round(x_meters, 2), round(y_meters, 2)

# 3. The Dispatch Trigger
def trigger_dispatch(coordinates):
    target_lat, target_lon = coordinates
    
    map_widget.delete_all_marker()
    map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home")
    map_widget.set_marker(target_lat, target_lon, text="Target")
    
    x, y = gps_to_cartesian(target_lat, target_lon)
    
    # Launch the flight sequence in the background so the map doesn't freeze!
    flight_thread = threading.Thread(target=execute_flight_mission, args=(x, y))
    flight_thread.start()

# 4. The GUI Frontend
root = tk.Tk()
root.geometry("900x700")
root.title("Swarm Control - Command Center")

status_label = tk.Label(root, text="STATUS: System Ready. Awaiting Map Input.", font=("Courier", 12, "bold"), bg="black", fg="white")
status_label.pack(fill="x")

map_widget = tkintermapview.TkinterMapView(root, width=900, height=600, corner_radius=0)
map_widget.pack(fill="both", expand=True)

map_widget.set_position(HOME_LAT, HOME_LON) 
map_widget.set_zoom(17) # Zoomed in closer for realistic flight distances
map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home")

map_widget.add_right_click_menu_command(label="Dispatch Swarm", command=trigger_dispatch, pass_coords=True)

root.mainloop()