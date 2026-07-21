import airsim
import numpy as np
import cv2
import time
import math

print("Initializing SLAM Occupancy Grid Mapper...")

# 1. Connect to Simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

print("Taking off...")
client.takeoffAsync().join()
client.moveToZAsync(-3, 2).join()

# 2. Map Configuration
# We create an 800x800 pixel image to represent our world map
MAP_SIZE = 800
# Scale: How many pixels represent 1 meter?
PIXELS_PER_METER = 10 
# Map origin (Center the drone's starting point in the middle of the image)
origin_x, origin_y = MAP_SIZE // 2, MAP_SIZE // 2

# Create a blank black map
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)

print("Mapping started! Fly the drone using the simulator (Page Up/Arrows). Press 'q' to stop.")

try:
    while True:
        # 3. SENSE: Get Kinematics (Position and Orientation)
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        orientation = state.kinematics_estimated.orientation
        
        # Convert quaternion to yaw angle (in radians)
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)

        # 4. SENSE: Get raw LiDAR point cloud
        lidar_data = client.getLidarData(lidar_name="Lidar1")
        
        # Calculate drone's global pixel position on our map
        drone_map_x = int(origin_x + (pos.x_val * PIXELS_PER_METER))
        drone_map_y = int(origin_y + (pos.y_val * PIXELS_PER_METER))
        
        # Fade the map slightly over time to show active vs old scans (optional trail effect)
        # occupancy_grid = cv2.addWeighted(occupancy_grid, 0.99, np.zeros_like(occupancy_grid), 0.01, 0)

        # Draw the drone's flight path (blue trail)
        if 0 <= drone_map_x < MAP_SIZE and 0 <= drone_map_y < MAP_SIZE:
            cv2.circle(occupancy_grid, (drone_map_x, drone_map_y), 1, (255, 0, 0), -1)

        # 5. PROCESS: Coordinate Transformation
        if len(lidar_data.point_cloud) >= 3:
            # Reshape flat array into [X, Y, Z]
            points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            
            for point in points:
                local_x = point[0]
                local_y = point[1]
                
                # Apply 2D Rotation Matrix to align LiDAR points with global compass heading
                global_x = (local_x * math.cos(yaw) - local_y * math.sin(yaw)) + pos.x_val
                global_y = (local_x * math.sin(yaw) + local_y * math.cos(yaw)) + pos.y_val
                
                # Convert global meters to map pixels
                map_x = int(origin_x + (global_x * PIXELS_PER_METER))
                map_y = int(origin_y + (global_y * PIXELS_PER_METER))
                
                # 6. ACTUATE: Draw the obstacle (White dots)
                if 0 <= map_x < MAP_SIZE and 0 <= map_y < MAP_SIZE:
                    cv2.circle(occupancy_grid, (map_x, map_y), 1, (255, 255, 255), -1)

        # Display the live SLAM map
        # We overlay a green dot representing the drone's current exact location
        display_grid = occupancy_grid.copy()
        cv2.circle(display_grid, (drone_map_x, drone_map_y), 4, (0, 255, 0), -1)
        
        cv2.imshow("LiDAR SLAM Occupancy Grid", display_grid)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
        # CPU Thermal Throttle
        time.sleep(0.05)

except KeyboardInterrupt:
    print("SLAM sequence aborted.")

# Save the final map to your hard drive
cv2.imwrite("final_slam_map.png", occupancy_grid)
print("Final map saved as 'final_slam_map.png'.")

# Safe Landing Sequence
client.hoverAsync().join()
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
cv2.destroyAllWindows()