import tkinter as tk
import tkintermapview
import math

print("Initializing GCS Translation Engine...")

# 1. Define Drone Spawning Location (Home Base)
HOME_LAT = 21.1458
HOME_LON = 79.0882

# 2. The Math Engine: GPS to Meters
def gps_to_cartesian(target_lat, target_lon):
    R = 6378137.0 # Radius of Earth in meters
    
    # Convert GPS degrees to Radians for mathematical functions
    lat1 = math.radians(HOME_LAT)
    lat2 = math.radians(target_lat)
    lon1 = math.radians(HOME_LON)
    lon2 = math.radians(target_lon)
    
    # Calculate X (East-West) and Y (North-South) distance in meters
    x_meters = (lon2 - lon1) * R * math.cos((lat1 + lat2) / 2)
    y_meters = (lat2 - lat1) * R
    
    # Round to 2 decimal places for clean A* grid ingestion
    return round(x_meters, 2), round(y_meters, 2)

# 3. GUI Setup
root = tk.Tk()
root.geometry("900x700")
root.title("Autonomous Swarm - Ground Control Station")

# Add a live status bar at the top
status_label = tk.Label(root, text="System Ready. Right-click map to set mission waypoint.", font=("Courier", 12, "bold"), bg="black", fg="green")
status_label.pack(fill="x")

map_widget = tkintermapview.TkinterMapView(root, width=900, height=600, corner_radius=0)
map_widget.pack(fill="both", expand=True)

# Center on Home Base
map_widget.set_position(HOME_LAT, HOME_LON) 
map_widget.set_zoom(15)
map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home Base")

# 4. Triggering the Translation on Click
def add_waypoint(coordinates):
    target_lat, target_lon = coordinates
    
    # Reset map markers
    map_widget.delete_all_marker()
    map_widget.set_marker(HOME_LAT, HOME_LON, text="Drone Home Base")
    map_widget.set_marker(target_lat, target_lon, text="Target Destination")
    
    # Run the Math Engine
    x, y = gps_to_cartesian(target_lat, target_lon)
    
    # Update UI and Backend
    status_label.config(text=f"TARGET ACQUIRED | Grid Coordinates: X = {x}m, Y = {y}m")
    print(f"\n[DISPATCH] Sending grid coordinates (X: {x}, Y: {y}) to A* Navigation Engine...")

map_widget.add_right_click_menu_command(label="Deploy Swarm to Destination", command=add_waypoint, pass_coords=True)

root.mainloop()