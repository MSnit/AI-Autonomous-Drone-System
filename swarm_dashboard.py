import customtkinter as ctk
from PIL import Image, ImageTk
import cv2
import threading
import airsim
import numpy as np
import time
from ultralytics import YOLO

# --- Configuration ---
ctk.set_appearance_mode("Dark")  # Enterprise Dark Mode
ctk.set_default_color_theme("blue")

class SwarmDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()

        # --- Window Setup ---
        self.title("AI Drone Intelligence System - Ground Control")
        self.geometry("1200x700")
        
        # --- Drone State Variables ---
        self.client = None
        self.is_connected = False
        self.ai_engaged = False
        self.running = True
        self.model = None

        # --- Layout Architecture ---
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # 1. Left Control Panel
        self.sidebar_frame = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(6, weight=1)

        self.logo_label = ctk.CTkLabel(self.sidebar_frame, text="SWARM COMMAND", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 30))

        self.btn_connect = ctk.CTkButton(self.sidebar_frame, text="Connect AirSim", command=self.connect_drone)
        self.btn_connect.grid(row=1, column=0, padx=20, pady=10)

        self.btn_takeoff = ctk.CTkButton(self.sidebar_frame, text="Takeoff Swarm", command=self.takeoff_swarm, state="disabled")
        self.btn_takeoff.grid(row=2, column=0, padx=20, pady=10)

        self.btn_land = ctk.CTkButton(self.sidebar_frame, text="Land Swarm", command=self.land_swarm, state="disabled")
        self.btn_land.grid(row=3, column=0, padx=20, pady=10)

        self.btn_ai = ctk.CTkButton(self.sidebar_frame, text="Engage AI Tracker", command=self.toggle_ai, fg_color="#8B0000", hover_color="#600000", state="disabled")
        self.btn_ai.grid(row=4, column=0, padx=20, pady=40)

        # Manual Control Instructions
        self.manual_label = ctk.CTkLabel(self.sidebar_frame, text="Manual Overrides:\nW/S: Pitch\nA/D: Yaw\nUp/Down: Throttle", font=ctk.CTkFont(size=12), text_color="gray")
        self.manual_label.grid(row=7, column=0, padx=20, pady=20, sticky="s")

        # 2. Center Video Feed
        self.video_frame = ctk.CTkFrame(self)
        self.video_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        
        self.video_label = ctk.CTkLabel(self.video_frame, text="System Offline.\nAwaiting AirSim Connection...", font=ctk.CTkFont(size=24, weight="bold"))
        self.video_label.pack(expand=True, fill="both", padx=10, pady=10)

        # 3. Right Telemetry Panel
        self.telemetry_frame = ctk.CTkFrame(self, width=200, corner_radius=10)
        self.telemetry_frame.grid(row=0, column=2, padx=20, pady=20, sticky="nsew")
        
        self.lbl_telem_title = ctk.CTkLabel(self.telemetry_frame, text="LIVE TELEMETRY", font=ctk.CTkFont(size=16, weight="bold"))
        self.lbl_telem_title.pack(pady=(20, 10), padx=20)

        self.lbl_status = ctk.CTkLabel(self.telemetry_frame, text="Status: OFFLINE", text_color="red")
        self.lbl_status.pack(pady=10, padx=20, anchor="w")

        self.lbl_target = ctk.CTkLabel(self.telemetry_frame, text="Target: N/A")
        self.lbl_target.pack(pady=10, padx=20, anchor="w")

        # --- Key Bindings for Manual Control ---
        self.bind("<KeyPress>", self.key_press)
        self.bind("<KeyRelease>", self.key_release)
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_vz = 0.0
        self.manual_yaw = 0.0

    def connect_drone(self):
        self.lbl_status.configure(text="Status: CONNECTING...", text_color="yellow")
        self.update()
        
        try:
            # 1. Boot YOLO
            print("Loading YOLOv8 Model...")
            self.model = YOLO("runs/detect/swarm_drone_detector/weights/best.pt")
            
            # 2. Connect AirSim
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            
            for drone in ["Drone1", "Drone2"]:
                self.client.enableApiControl(True, vehicle_name=drone)
                self.client.armDisarm(True, vehicle_name=drone)

            self.is_connected = True
            self.lbl_status.configure(text="Status: CONNECTED", text_color="green")
            
            # Enable Buttons
            self.btn_takeoff.configure(state="normal")
            self.btn_land.configure(state="normal")
            self.btn_ai.configure(state="normal")
            self.btn_connect.configure(state="disabled", text="System Online")

            # Start Background Threads
            self.video_thread = threading.Thread(target=self.drone_brain_loop, daemon=True)
            self.video_thread.start()

        except Exception as e:
            self.lbl_status.configure(text="Status: FAILED", text_color="red")
            print(f"Connection Error: {e}")

    def takeoff_swarm(self):
        if self.is_connected:
            self.client.takeoffAsync(vehicle_name="Drone1")
            self.client.takeoffAsync(vehicle_name="Drone2").join()
            
            # Position them for the demo
            self.client.moveToPositionAsync(0, 0, -3, 2, vehicle_name="Drone1")
            self.client.moveToPositionAsync(4, 0, -3, 2, vehicle_name="Drone2").join()

    def land_swarm(self):
        if self.is_connected:
            self.ai_engaged = False
            self.update_ai_button()
            self.client.landAsync(vehicle_name="Drone1")
            self.client.landAsync(vehicle_name="Drone2").join()

    def toggle_ai(self):
        self.ai_engaged = not self.ai_engaged
        self.update_ai_button()
        if self.ai_engaged:
            # Start target drifting
            self.client.moveByVelocityAsync(0, 0.5, 0, 30, vehicle_name="Drone2")
        else:
            # Stop target
            self.client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name="Drone2")

    def update_ai_button(self):
        if self.ai_engaged:
            self.btn_ai.configure(text="DISABLE AI", fg_color="red", hover_color="#8B0000")
            self.lbl_target.configure(text="Target: SEARCHING", text_color="yellow")
        else:
            self.btn_ai.configure(text="Engage AI Tracker", fg_color="#8B0000", hover_color="#600000")
            self.lbl_target.configure(text="Target: N/A", text_color="white")

    # --- Manual Flight Controls ---
    def key_press(self, event):
        if self.ai_engaged or not self.is_connected:
            return
        char = event.keysym.lower()
        if char == 'w': self.manual_vx = 2.0
        elif char == 's': self.manual_vx = -2.0
        elif char == 'a': self.manual_yaw = -0.5
        elif char == 'd': self.manual_yaw = 0.5
        elif char == 'up': self.manual_vz = -2.0
        elif char == 'down': self.manual_vz = 2.0

    def key_release(self, event):
        char = event.keysym.lower()
        if char in ['w', 's']: self.manual_vx = 0.0
        elif char in ['a', 'd']: self.manual_yaw = 0.0
        elif char in ['up', 'down']: self.manual_vz = 0.0

    # --- The Core Intelligence & Vision Thread ---
    def drone_brain_loop(self):
        k_yaw = 0.15    
        k_pitch = 0.005 
        target_area = 25000 

        while self.running and self.is_connected:
            try:
                # 1. Pull Image
                responses = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")
                response = responses[0]
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
                frame = img1d.reshape(response.height, response.width, 3).copy()
                img_h, img_w, _ = frame.shape
                img_center_x = img_w / 2

                # 2. Logic Split: AI vs Manual
                if self.ai_engaged:
                    results = self.model.predict(source=frame, conf=0.4, verbose=False)
                    yaw_rate = 0.0
                    v_x = 0.0 
                    target_found = False

                    if len(results[0].boxes) > 0:
                        for box_obj in results[0].boxes:
                            box = box_obj.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = box
                            box_area = (x2 - x1) * (y2 - y1)
                            
                            if box_area > (0.5 * (img_w * img_h)):
                                continue
                                
                            box_center_x = (x1 + x2) / 2
                            
                            # Visual Servoing Math
                            error_x = box_center_x - img_center_x
                            yaw_rate = error_x * k_yaw
                            v_x = np.clip((target_area - box_area) * k_pitch, -1.5, 1.5)
                            
                            # Draw HUD
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(frame, "LOCKED", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                            
                            self.lbl_target.configure(text="Target: LOCKED", text_color="green")
                            target_found = True
                            break
                            
                    if not target_found:
                        self.lbl_target.configure(text="Target: SEARCHING...", text_color="yellow")
                        yaw_rate = 0.3 

                    # Execute AI Flight
                    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=float(yaw_rate))
                    self.client.moveByVelocityAsync(float(v_x), 0.0, 0.0, 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode, vehicle_name="Drone1")
                
                else:
                    # Execute Manual Flight
                    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=float(self.manual_yaw))
                    self.client.moveByVelocityAsync(float(self.manual_vx), float(self.manual_vy), float(self.manual_vz), 0.1, airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode, vehicle_name="Drone1")
                    cv2.putText(frame, "MANUAL OVERRIDE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 165, 0), 2)

                # 3. Update GUI Video Feed
                # Convert BGR (OpenCV) to RGB (Pillow)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                
                # Resize dynamically to fit the frame
                img = img.resize((640, 480)) 
                img_tk = ImageTk.PhotoImage(image=img)
                
                self.video_label.configure(image=img_tk, text="")
                self.video_label.image = img_tk 

            except Exception as e:
                print(f"Vision loop error: {e}")
            
            time.sleep(0.03) # Limit to ~30 FPS to save CPU

    def on_closing(self):
        self.running = False
        if self.is_connected:
            for drone in ["Drone1", "Drone2"]:
                self.client.armDisarm(False, vehicle_name=drone)
                self.client.enableApiControl(False, vehicle_name=drone)
        self.destroy()

if __name__ == "__main__":
    app = SwarmDashboard()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()