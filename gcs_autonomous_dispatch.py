import tkinter as tk
import tkintermapview
import math
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

# 2. Live Telemetry Thread (The Radar)
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

# 3. The Direct Flight Thread
def execute_smart_mission(target_x, target_y, drone_marker):
    global telemetry_active
    telemetry_active = True
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Start the radar ping on the GUI
    threading.Thread(target=update_telemetry, args=(drone_marker,), daemon=True).start()
    
    # --- DRONE 1: SMOOTH DIRECT FLIGHT ---
    status_label.config(text=f"STATUS: Airborne. Proceeding directly to target...", fg="orange")
    client.enableApiControl(True, vehicle_name="Drone1")
    client.armDisarm(True, vehicle_name="Drone1")
    client.takeoffAsync(vehicle_name="Drone1").join()
    client.moveToZAsync(-5, 2, vehicle_name="Drone1").join()
    
    # Direct, smooth flight to the GPS target (speed: 8 m/s)
    client.moveToPositionAsync(target_x, target_y, -5, 8, vehicle_name="Drone1").join()
    
    status_label.config(text="STATUS: Target Reached. Grounding Drone.", fg="green")
    
    # Secure the drone
    client.landAsync(vehicle_name="Drone1").join()
    client.armDisarm(False, vehicle_name="Drone1")
    client.enableApiControl(False, vehicle_name="Drone1")
        
    telemetry_active = False

# 4. Dispatch Trigger
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
root.title("Drone Logistics - Command Center")
status_label = tk.Label(root, text="STATUS: System Ready. Awaiting Input.", font=("Courier", 12, "bold"), bg="black", fg="white")
status_label.pack(fill="x")

map_widget = tkintermapview.TkinterMapView(root, width=900, height=600, corner_radius=0)
map_widget.pack(fill="both", expand=True)
map_widget.set_position(HOME_LAT, HOME_LON) 
map_widget.set_zoom(18)
map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home")

# Right-click to deploy
map_widget.add_right_click_menu_command(label="Deploy Drone", command=trigger_dispatch, pass_coords=True)

telemetry_active = False
root.mainloop()