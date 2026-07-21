import airsim
import time
import os

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

print("Establishing Telemetry Link... Press Ctrl+C to stop.")
time.sleep(1)

try:
    while True:
        # Clear the terminal for a clean, updating dashboard
        os.system('cls' if os.name == 'nt' else 'clear')
        
        # 1. Kinematics (Position in 3D Space)
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        
        # 2. IMU Data (Linear Acceleration and Angular Velocity)
        imu = client.getImuData()
        accel = imu.linear_acceleration
        gyro = imu.angular_velocity
        
        # 3. Barometer Data (Absolute Altitude)
        barometer = client.getBarometerData()
        alt = barometer.altitude
        
        # Print the formatted dashboard
        print("=== DRONE TELEMETRY DASHBOARD ===")
        print(f"Position (m):   X: {pos.x_val:6.2f} | Y: {pos.y_val:6.2f} | Z: {pos.z_val:6.2f}")
        print(f"Altitude (m):   {alt:6.2f}")
        print(f"Accel (m/s^2):  X: {accel.x_val:6.2f} | Y: {accel.y_val:6.2f} | Z: {accel.z_val:6.2f}")
        print(f"Gyro (rad/s):   X: {gyro.x_val:6.2f} | Y: {gyro.y_val:6.2f} | Z: {gyro.z_val:6.2f}")
        print("=================================")
        
        # Update 10 times per second
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nTelemetry link closed.")