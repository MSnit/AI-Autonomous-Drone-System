import customtkinter as ctk
import tkintermapview
import airsim
import threading
import time
import math

# --- Configuration ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class DeliveryDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        # --- Window Setup ---
        self.title("Blinkit Autonomous Delivery Network - Ground Control")
        self.geometry("1200x700")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- Drone & GPS State ---
        self.client = None
        self.is_connected = False
        self.drone_marker = None
        self.target_marker = None
        self.delivery_thread = None
        self.running = True
        
        # Base Coordinates (Simulated origin in Nagpur)
        self.base_lat = 21.1458
        self.base_lon = 79.0882
        self.flight_altitude = -15 # Fly at 15 meters high

        # --- 1. Left Control Panel ---
        self.sidebar = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_rowconfigure(5, weight=1)

        self.logo = ctk.CTkLabel(self.sidebar, text="🚁 BLINKIT AIR", font=ctk.CTkFont(size=24, weight="bold"))
        self.logo.grid(row=0, column=0, padx=20, pady=(20, 30))

        self.btn_connect = ctk.CTkButton(self.sidebar, text="1. Initialize Fleet", command=self.connect_drone)
        self.btn_connect.grid(row=1, column=0, padx=20, pady=10)

        self.btn_takeoff = ctk.CTkButton(self.sidebar, text="2. Launch Drone", command=self.takeoff, state="disabled")
        self.btn_takeoff.grid(row=2, column=0, padx=20, pady=10)

        self.btn_rtl = ctk.CTkButton(self.sidebar, text="Return to Base", command=self.return_to_base, fg_color="#8B0000", hover_color="#600000", state="disabled")
        self.btn_rtl.grid(row=3, column=0, padx=20, pady=40)

        self.instr_label = ctk.CTkLabel(self.sidebar, text="Right-Click Map\nto Set Delivery Location", font=ctk.CTkFont(size=12), text_color="gray")
        self.instr_label.grid(row=6, column=0, padx=20, pady=20, sticky="s")

        # --- 2. Center Interactive Map ---
        self.map_frame = ctk.CTkFrame(self)
        self.map_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        
        # Embed the map
        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, corner_radius=10)
        self.map_widget.pack(fill="both", expand=True)
        
        # Set initial map view to Nagpur
        self.map_widget.set_position(self.base_lat, self.base_lon)
        self.map_widget.set_zoom(15)
        
        # Add Right-Click Event for Delivery
        self.map_widget.add_right_click_menu_command(label="Set Delivery Destination", command=self.set_destination, pass_coords=True)

        # --- 3. Right Telemetry Panel ---
        self.telem_frame = ctk.CTkFrame(self, width=250, corner_radius=10)
        self.telem_frame.grid(row=0, column=2, padx=10, pady=10, sticky="nsew")
        
        self.lbl_title = ctk.CTkLabel(self.telem_frame, text="FLIGHT TELEMETRY", font=ctk.CTkFont(size=16, weight="bold"))
        self.lbl_title.pack(pady=(20, 10), padx=20)

        self.lbl_status = ctk.CTkLabel(self.telem_frame, text="Status: OFFLINE", text_color="red")
        self.lbl_status.pack(pady=10, padx=20, anchor="w")

        self.lbl_dist = ctk.CTkLabel(self.telem_frame, text="Distance: 0 m")
        self.lbl_dist.pack(pady=10, padx=20, anchor="w")
        
        self.lbl_speed = ctk.CTkLabel(self.telem_frame, text="Speed: 0 m/s")
        self.lbl_speed.pack(pady=10, padx=20, anchor="w")

    # --- Connection & Setup ---
    def connect_drone(self):
        self.lbl_status.configure(text="Status: CONNECTING...", text_color="yellow")
        self.update()
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            
            # Target Drone1 explicitly (Drone 2 is now removed from settings.json)
            self.client.enableApiControl(True, vehicle_name="Drone1")
            self.client.armDisarm(True, vehicle_name="Drone1")
            
            self.is_connected = True
            self.btn_takeoff.configure(state="normal")
            self.btn_connect.configure(state="disabled", text="Fleet Online")
            self.lbl_status.configure(text="Status: READY", text_color="green")
            
            # Place Base Marker and Drone Marker
            self.map_widget.set_marker(self.base_lat, self.base_lon, text="Warehouse Base")
            self.drone_marker = self.map_widget.set_marker(self.base_lat, self.base_lon, text="Drone", marker_color_circle="black", marker_color_outside="blue")
            
            # Start telemetry loop
            threading.Thread(target=self.telemetry_loop, daemon=True).start()
            
        except Exception as e:
            self.lbl_status.configure(text="Status: CONNECTION FAILED", text_color="red")
            print(e)

    def takeoff(self):
        if self.is_connected:
            self.lbl_status.configure(text="Status: TAKING OFF...", text_color="yellow")
            # Clear collision state and launch
            self.client.takeoffAsync(vehicle_name="Drone1").join()
            self.client.moveToPositionAsync(0, 0, self.flight_altitude, 5, vehicle_name="Drone1").join()
            self.lbl_status.configure(text="Status: AIRBORNE (Awaiting Orders)", text_color="green")
            self.btn_rtl.configure(state="normal")

    # --- Delivery Logistics Logic ---
    def set_destination(self, coords):
        if not self.is_connected:
            return
            
        target_lat, target_lon = coords
        
        # Place Delivery Marker on Map
        if self.target_marker:
            self.target_marker.delete()
        self.target_marker = self.map_widget.set_marker(target_lat, target_lon, text="Delivery Location", marker_color_outside="red")
        
        # Convert Map Lat/Lon to AirSim X/Y coordinates
        target_x, target_y = self.lat_lon_to_airsim(target_lat, target_lon)
        
        self.lbl_status.configure(text="Status: EN ROUTE TO DELIVERY", text_color="orange")
        
        # Launch flight in a background thread to prevent Python from cancelling the async task
        threading.Thread(target=self.fly_to_target, args=(target_x, target_y), daemon=True).start()

    def fly_to_target(self, x, y):
        try:
            # Explicitly cast to Python floats
            safe_x = float(x)
            safe_y = float(y)
            safe_z = float(self.flight_altitude)
            
            # Simplified flight command to prevent AirSim from dropping the physics task
            self.client.moveToPositionAsync(
                safe_x, safe_y, safe_z, 
                8.0, 
                timeout_sec=3600,
                vehicle_name="Drone1"
            ).join() 
            
            # Update status when it physically arrives
            self.lbl_status.configure(text="Status: ARRIVED / HOVERING", text_color="green")
        except Exception as e:
            print(f"Flight error: {e}")

    def return_to_base(self):
        if self.is_connected:
            self.lbl_status.configure(text="Status: RETURNING TO BASE", text_color="orange")
            if self.target_marker:
                self.target_marker.delete()
            
            # Launch return flight in background thread
            threading.Thread(target=self.fly_to_target, args=(0, 0), daemon=True).start()

    # --- Background Math & Telemetry ---
    def telemetry_loop(self):
        while self.running and self.is_connected:
            try:
                # 1. Get Drone state from AirSim
                state = self.client.getMultirotorState(vehicle_name="Drone1")
                pos = state.kinematics_estimated.position
                vel = state.kinematics_estimated.linear_velocity
                
                # 2. Update Map Position
                current_lat, current_lon = self.airsim_to_lat_lon(pos.x_val, pos.y_val)
                self.drone_marker.set_position(current_lat, current_lon)
                
                # 3. Calculate Speed (Scaled up 15x for map realism)
                speed = math.sqrt(vel.x_val**2 + vel.y_val**2) * 15.0
                
                # 4. Calculate Distance from Base (Scaled up 15x)
                dist = math.sqrt((pos.x_val * 15.0)**2 + (pos.y_val * 15.0)**2)
                
                # Update UI
                self.lbl_speed.configure(text=f"Speed: {speed:.1f} m/s")
                self.lbl_dist.configure(text=f"Dist from Base: {dist:.0f} m")
                
                time.sleep(0.5)
            except Exception as e:
                print(e)
                time.sleep(1)

    # --- Geographic Translators ---
    # AirSim uses NED (North=X, East=Y). 1 degree Lat/Lon is approx 111,320 meters.
    def lat_lon_to_airsim(self, target_lat, target_lon):
        dx_meters = (target_lon - self.base_lon) * 111320 * math.cos(math.radians(self.base_lat))
        dy_meters = (target_lat - self.base_lat) * 111320
        
        # Scale real-world map distances down by 15x to prevent hitting invisible simulator walls
        sim_x = dy_meters / 15.0
        sim_y = dx_meters / 15.0
        return sim_x, sim_y

    def airsim_to_lat_lon(self, airsim_x, airsim_y):
        # Scale the simulated coordinates back up 15x so the UI map marker moves correctly
        real_x = airsim_x * 15.0
        real_y = airsim_y * 15.0
        
        lat = self.base_lat + (real_x / 111320)
        lon = self.base_lon + (real_y / (111320 * math.cos(math.radians(self.base_lat))))
        return lat, lon

    def on_closing(self):
        self.running = False
        if self.is_connected:
            self.client.armDisarm(False, vehicle_name="Drone1")
            self.client.enableApiControl(False, vehicle_name="Drone1")
        self.destroy()

if __name__ == "__main__":
    app = DeliveryDashboard()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()