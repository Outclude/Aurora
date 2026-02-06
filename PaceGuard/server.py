import os
import asyncio
from flask import Flask, send_from_directory, jsonify
from bleak import BleakScanner

# Configure Flask to serve static files from the 'public' directory
# static_url_path='' ensures files are served at root (e.g., /style.css)
app = Flask(__name__, static_folder='public', static_url_path='')

async def scan_ble_devices_async():
    """Scan for BLE devices using BleakScanner."""
    print("Starting BLE scan...")
    try:
        devices_dict = await BleakScanner.discover(return_adv=True, timeout=5.0)
        results = []
        for address, (device, adv_data) in devices_dict.items():
            results.append({
                "name": device.name if device.name else "Unknown Device",
                "address": device.address,
                "rssi": adv_data.rssi
            })
        # Sort by RSSI (strongest first)
        results.sort(key=lambda x: x['rssi'], reverse=True)
        return results
    except Exception as e:
        print(f"BLE Scan Error: {e}")
        return {"error": str(e)}

@app.route('/api/scan-ble')
def scan_ble():
    """API endpoint to scan for BLE devices."""
    try:
        # Create a new event loop for the async task if needed, or use asyncio.run
        results = asyncio.run(scan_ble_devices_async())
        return jsonify(results)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/')
def serve_index():
    """Serve index.html at the root URL"""
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/<path:path>')
def serve_static(path):
    """
    Serve static files if they exist.
    Fallback to index.html for SPA routing (if any).
    """
    full_path = os.path.join(app.static_folder, path)
    if os.path.exists(full_path) and os.path.isfile(full_path):
        return send_from_directory(app.static_folder, path)
    
    # Fallback for SPA
    return send_from_directory(app.static_folder, 'index.html')

if __name__ == '__main__':
    # Run on port 2333 to match previous configuration
    print("Starting PaceGuard Server (Python/Flask)...")
    print("Running at http://localhost:2333")
    app.run(host='0.0.0.0', port=2333, debug=True)
