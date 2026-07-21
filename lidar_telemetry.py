import airsim
import numpy as np
import time

client = airsim.MultirotorClient()
client.confirmConnection()

print("Streaming 3D LiDAR Point Cloud Data... Press Ctrl+C to exit.")

try:
    while True:
        # Get raw LiDAR point cloud data from AirSim
        lidar_data = client.getLidarData(lidar_name="Lidar1")
        
        if len(lidar_data.point_cloud) < 3:
            print("No LiDAR points detected in range...")
        else:
            # Reshape flat array into (N, 3) representing [X, Y, Z] relative coordinates
            points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            
            # Compute distance of all detected points from the drone
            distances = np.linalg.norm(points, axis=1)
            closest_point_dist = np.min(distances)
            
            print(f"Captured {len(points)} 3D Points | Closest Obstacle: {closest_point_dist:.2f} meters")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLiDAR stream stopped.")