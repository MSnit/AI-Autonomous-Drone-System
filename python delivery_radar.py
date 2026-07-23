import cv2
import numpy as np
import airsim
import threading
import math
import time

# --- Radar Map Configuration ---
RADAR_SIZE = 800
CENTER_X = RADAR_SIZE // 2
CENTER_Y = RADAR_SIZE // 2
SCALE = 2.0  # 1 pixel = 2 meters in AirSim
ALTITUDE = -15.0
VELOCITY = 8.0

# --- Global State Variables ---
target_pixel = None
drone_pixel = (CENTER_X, CENTER_Y)
status = "SYSTEM BOOTING..."
distance_m = 0.0
speed_ms = 0.0

# --- Coordinate Translation ---
def pixel_to_airsim(px, py):
    # AirSim X is North (Up in image, so -Y)
    # AirSim Y is East (Right in image, so +X)
    airsim_x = (CENTER_Y - py) * SCALE
    airsim_y = (px - CENTER_X) * SCALE
    return airsim_x, airsim_y

def airsim_to_pixel(ax, ay):
    px = int(CENTER_X + (ay / SCALE))
    py = int(CENTER_Y - (ax / SCALE))
    return px, py

# --- Flight Controller (Background Thread) ---
def execute_flight(x, y):
    global status
    try:
        status = "EN ROUTE TO TARGET..."
        # Using default simple move command to prevent toilet-bowl orbiting
        client.moveToPositionAsync(
            float(x), float(y), float(ALTITUDE), 
            float(VELOCITY), 
            timeout_sec=3600,
            vehicle_name="Drone1"
        ).join()
        status = "ARRIVED / HOVERING"
    except Exception as e:
        status = f"FLIGHT ERROR: {e}"

# --- OpenCV Mouse Callback ---
def mouse_click(event, x, y, flags, param):
    global target_pixel, status
    
    # Left click to set delivery destination
    if event == cv2.EVENT_LBUTTONDOWN:
        target_pixel = (x, y)
        target_ax, target_ay = pixel_to_airsim(x, y)
        
        # Launch flight in a background thread so OpenCV doesn't freeze
        threading.Thread(target=execute_flight, args=(target_ax, target_ay), daemon=True).start()

# --- Main Setup ---
print("Connecting to AirSim...")
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name="Drone1")
client.armDisarm(True, vehicle_name="Drone1")

print("Taking off...")
status = "TAKING OFF..."
client.takeoffAsync(vehicle_name="Drone1").join()
client.moveToPositionAsync(0, 0, ALTITUDE, 5, vehicle_name="Drone1").join()
status = "AIRBORNE. AWAITING ORDERS."

# --- OpenCV UI Setup ---
cv2.namedWindow("Blinkit Tactical Radar")
cv2.setMouseCallback("Blinkit Tactical Radar", mouse_click)

print("\n--- SYSTEM READY ---")
print("Click anywhere on the Radar window to dispatch the drone.")
print("Press 'q' to quit and land.")

# --- Main Telemetry & Rendering Loop ---
while True:
    # 1. Create a blank dark radar background
    radar_img = np.zeros((RADAR_SIZE, RADAR_SIZE, 3), dtype=np.uint8)
    
    # 2. Draw Radar Grid
    for i in range(0, RADAR_SIZE, 50):
        cv2.line(radar_img, (i, 0), (i, RADAR_SIZE), (30, 30, 30), 1)
        cv2.line(radar_img, (0, i), (RADAR_SIZE, i), (30, 30, 30), 1)
        
    # Draw Crosshairs
    cv2.line(radar_img, (CENTER_X, 0), (CENTER_X, RADAR_SIZE), (50, 70, 50), 2)
    cv2.line(radar_img, (0, CENTER_Y), (RADAR_SIZE, CENTER_Y), (50, 70, 50), 2)
    cv2.circle(radar_img, (CENTER_X, CENTER_Y), 200, (50, 70, 50), 1)
    cv2.circle(radar_img, (CENTER_X, CENTER_Y), 400, (50, 70, 50), 1)

    # 3. Fetch AirSim Telemetry
    try:
        state = client.getMultirotorState(vehicle_name="Drone1")
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        
        # Calculate speed and distance
        speed_ms = math.sqrt(vel.x_val**2 + vel.y_val**2)
        distance_m = math.sqrt(pos.x_val**2 + pos.y_val**2)
        
        # Convert physical position to pixel coordinates
        drone_pixel = airsim_to_pixel(pos.x_val, pos.y_val)
    except Exception:
        pass # Ignore dropped telemetry frames

    # 4. Draw Markers
    # Base (Center)
    cv2.circle(radar_img, (CENTER_X, CENTER_Y), 6, (255, 255, 0), -1) 
    cv2.putText(radar_img, "BASE", (CENTER_X + 10, CENTER_Y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    # Target
    if target_pixel:
        cv2.drawMarker(radar_img, target_pixel, (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(radar_img, "TARGET", (target_pixel[0] + 10, target_pixel[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Drone
    cv2.circle(radar_img, drone_pixel, 8, (0, 255, 255), -1) # Yellow Dot
    cv2.circle(radar_img, drone_pixel, 15, (0, 255, 255), 1) # Yellow Ring
    cv2.putText(radar_img, "DRONE", (drone_pixel[0] + 15, drone_pixel[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    # 5. Draw HUD Text
    cv2.putText(radar_img, f"STATUS: {status}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(radar_img, f"SPEED: {speed_ms:.1f} m/s", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(radar_img, f"DIST FROM BASE: {distance_m:.0f} m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Render Window
    cv2.imshow("Blinkit Tactical Radar", radar_img)

    # Quit logic (Press 'q')
    if cv2.waitKey(30) & 0xFF == ord('q'):
        print("Landing drone and shutting down...")
        break

# --- Shutdown Sequence ---
cv2.destroyAllWindows()
client.landAsync(vehicle_name="Drone1").join()
client.armDisarm(False, vehicle_name="Drone1")
client.enableApiControl(False, vehicle_name="Drone1")