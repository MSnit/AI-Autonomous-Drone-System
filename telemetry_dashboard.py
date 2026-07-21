import airsim
from flask import Flask, jsonify, render_template_string
import threading
import time
import math

# 1. Initialize the Flask Web App
app = Flask(__name__)

# Global dictionary to store live drone data
telemetry_data = {
    "status": "OFFLINE",
    "altitude": 0.0,
    "speed": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0
}

# 2. Background Task: Pull AirSim Telemetry
def update_telemetry():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    telemetry_data["status"] = "CONNECTED"
    
    while True:
        try:
            state = client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            orientation = state.kinematics_estimated.orientation
            
            pitch, roll, yaw = airsim.to_eularian_angles(orientation)
            speed = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2)
            
            # Update the global JSON payload
            telemetry_data["altitude"] = round(-pos.z_val, 2)
            telemetry_data["speed"] = round(speed, 2)
            telemetry_data["pitch"] = round(math.degrees(pitch), 1)
            telemetry_data["roll"] = round(math.degrees(roll), 1)
            telemetry_data["yaw"] = round(math.degrees(yaw), 1)
            
        except Exception as e:
            telemetry_data["status"] = "ERROR / DISCONNECTED"
            
        # CPU Thermal Throttle
        time.sleep(0.1)

# 3. Web Endpoints
@app.route('/api/telemetry')
def get_telemetry():
    """Returns the live data as a JSON payload."""
    return jsonify(telemetry_data)

@app.route('/')
def index():
    """Serves the HTML Dashboard Frontend."""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Drone Telemetry Dashboard</title>
        <style>
            body { font-family: 'Courier New', Courier, monospace; background-color: #0d1117; color: #58a6ff; text-align: center; margin-top: 50px; }
            .grid { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; max-width: 800px; margin: 0 auto; }
            .card { background-color: #161b22; border: 1px solid #30363d; padding: 20px; border-radius: 8px; width: 200px; }
            .value { font-size: 2em; color: #7ee787; font-weight: bold; margin-top: 10px; }
            h1 { color: #c9d1d9; }
        </style>
    </head>
    <body>
        <h1>Mission Control | Live Telemetry</h1>
        <p id="status" style="color: #ff7b72;">Status: CONNECTING...</p>
        
        <div class="grid">
            <div class="card">Altitude<div id="alt" class="value">0.0 m</div></div>
            <div class="card">Airspeed<div id="spd" class="value">0.0 m/s</div></div>
            <div class="card">Pitch<div id="pitch" class="value">0.0&deg;</div></div>
            <div class="card">Roll<div id="roll" class="value">0.0&deg;</div></div>
            <div class="card">Yaw<div id="yaw" class="value">0.0&deg;</div></div>
        </div>

        <script>
            // Fetch the JSON payload every 200ms and update the DOM
            setInterval(() => {
                fetch('/api/telemetry')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('status').innerText = "Status: " + data.status;
                        document.getElementById('status').style.color = (data.status === "CONNECTED") ? "#7ee787" : "#ff7b72";
                        document.getElementById('alt').innerText = data.altitude.toFixed(2) + " m";
                        document.getElementById('spd').innerText = data.speed.toFixed(2) + " m/s";
                        document.getElementById('pitch').innerText = data.pitch.toFixed(1) + "°";
                        document.getElementById('roll').innerText = data.roll.toFixed(1) + "°";
                        document.getElementById('yaw').innerText = data.yaw.toFixed(1) + "°";
                    });
            }, 200);
        </script>
    </body>
    </html>
    """
    return render_template_string(html)

if __name__ == '__main__':
    # Boot up the AirSim polling thread
    t = threading.Thread(target=update_telemetry, daemon=True)
    t.start()
    
    # Start the local web server
    print("\n>>> Server running. Open http://127.0.0.1:5000 in your web browser.\n")
    app.run(host='127.0.0.1', port=5000, debug=False)