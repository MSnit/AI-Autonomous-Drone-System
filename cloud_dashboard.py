from flask import Flask, jsonify, request, render_template_string

app = Flask(__name__)

# Global dictionary holding the latest telemetry
telemetry_data = {
    "status": "AWAITING CONNECTION...",
    "altitude": 0.0,
    "speed": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0
}

@app.route('/api/telemetry/update', methods=['POST'])
def update_telemetry():
    """Endpoint for the local drone to push data to the cloud."""
    global telemetry_data
    incoming_data = request.json
    if incoming_data:
        telemetry_data.update(incoming_data)
        return jsonify({"status": "success", "message": "Telemetry updated"}), 200
    return jsonify({"status": "error", "message": "No data received"}), 400

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    """Endpoint for the web browser to fetch live data."""
    return jsonify(telemetry_data)

@app.route('/')
def index():
    """The HTML Frontend."""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>AWS Cloud Fleet Command</title>
        <style>
            body { font-family: 'Courier New', Courier, monospace; background-color: #0d1117; color: #58a6ff; text-align: center; margin-top: 50px; }
            .grid { display: flex; justify-content: center; gap: 20px; flex-wrap: wrap; max-width: 800px; margin: 0 auto; }
            .card { background-color: #161b22; border: 1px solid #30363d; padding: 20px; border-radius: 8px; width: 200px; }
            .value { font-size: 2em; color: #7ee787; font-weight: bold; margin-top: 10px; }
            h1 { color: #c9d1d9; }
        </style>
    </head>
    <body>
        <h1>AWS Global Fleet Command | Live Telemetry</h1>
        <p id="status" style="color: #ff7b72;">Status: CONNECTING...</p>
        
        <div class="grid">
            <div class="card">Altitude<div id="alt" class="value">0.0 m</div></div>
            <div class="card">Airspeed<div id="spd" class="value">0.0 m/s</div></div>
            <div class="card">Pitch<div id="pitch" class="value">0.0&deg;</div></div>
            <div class="card">Roll<div id="roll" class="value">0.0&deg;</div></div>
            <div class="card">Yaw<div id="yaw" class="value">0.0&deg;</div></div>
        </div>

        <script>
            setInterval(() => {
                fetch('/api/telemetry')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('status').innerText = "Status: " + data.status;
                        document.getElementById('status').style.color = (data.status === "LIVE (AWS)") ? "#7ee787" : "#ff7b72";
                        document.getElementById('alt').innerText = data.altitude.toFixed(2) + " m";
                        document.getElementById('spd').innerText = data.speed.toFixed(2) + " m/s";
                        document.getElementById('pitch').innerText = data.pitch.toFixed(1) + "°";
                        document.getElementById('roll').innerText = data.roll.toFixed(1) + "°";
                        document.getElementById('yaw').innerText = data.yaw.toFixed(1) + "°";
                    });
            }, 500);
        </script>
    </body>
    </html>
    """
    return render_template_string(html)

if __name__ == '__main__':
    # Binding to 0.0.0.0 exposes the server to the public internet
    app.run(host='0.0.0.0', port=5000, debug=False)