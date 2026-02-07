import os
import asyncio
import threading
import queue
import time
from flask import Flask, send_from_directory, jsonify, request
from bleak import BleakScanner, BleakClient

# Configure Flask to serve static files from the 'public' directory
app = Flask(__name__, static_folder='public', static_url_path='')

# UUIDs from cest.py
UART_RX_CHAR_UUID = "0000FFF1-0000-1000-8000-00805F9B34FB"
UART_TX_CHAR_UUID = "0000FFF2-0000-1000-8000-00805F9B34FB"

class BLEManager:
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.client = None
        self.connected = False
        self.device_name = None
        self.msg_queue = queue.Queue()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        
    def connect(self, address, name=None):
        if self.connected and self.client:
             # Already connected
             return True
        self.device_name = name
        future = asyncio.run_coroutine_threadsafe(self._connect(address), self.loop)
        try:
            return future.result(timeout=10) # 10s timeout
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    async def _connect(self, address):
        try:
            print(f"Connecting to {address}...")
            self.client = BleakClient(address)
            await self.client.connect()
            self.connected = self.client.is_connected
            if self.connected:
                print(f"Connected to {address}")
                await self.client.start_notify(UART_TX_CHAR_UUID, self._notification_handler)
            return self.connected
        except Exception as e:
            print(f"Error connecting: {e}")
            self.connected = False
            return False

    def _notification_handler(self, sender, data):
        msg = data.decode(errors='ignore').strip()
        if msg:
            print(f"[Hardware -> PC]: {msg}")
            self.msg_queue.put({"sender": "device", "content": msg, "time": time.time()})
            
    def send(self, message):
        if not self.connected:
            return False
        future = asyncio.run_coroutine_threadsafe(self._send(message), self.loop)
        try:
            return future.result(timeout=5)
        except:
            return False
        
    async def _send(self, message):
        if self.client and self.connected:
            try:
                await self.client.write_gatt_char(UART_RX_CHAR_UUID, message.encode())
                print(f"[PC -> Hardware]: {message}")
                return True
            except Exception as e:
                print(f"Error sending: {e}")
                return False
        return False

    def get_messages(self):
        msgs = []
        while not self.msg_queue.empty():
            msgs.append(self.msg_queue.get())
        return msgs
        
    def disconnect(self):
         future = asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
         try:
             future.result(timeout=5)
         except:
             pass

    async def _disconnect(self):
        if self.client:
            try:
                await self.client.disconnect()
            except:
                pass
        self.connected = False
        self.device_name = None
        print("Disconnected")

ble_manager = BLEManager()

@app.route('/api/connect', methods=['POST'])
def connect_device():
    data = request.json
    address = data.get('address')
    name = data.get('name')
    if not address:
        return jsonify({"error": "Address required"}), 400
    
    success = ble_manager.connect(address, name)
    if success:
        return jsonify({"status": "connected", "address": address, "name": name})
    else:
        return jsonify({"error": "Connection failed"}), 500

@app.route('/api/disconnect', methods=['POST'])
def disconnect_device():
    ble_manager.disconnect()
    return jsonify({"status": "disconnected"})

@app.route('/api/send', methods=['POST'])
def send_message():
    data = request.json
    message = data.get('message')
    if not message:
        return jsonify({"error": "Message required"}), 400
    
    success = ble_manager.send(message)
    if success:
        return jsonify({"status": "sent", "message": message})
    else:
        return jsonify({"error": "Send failed"}), 500

@app.route('/api/messages', methods=['GET'])
def get_messages():
    msgs = ble_manager.get_messages()
    return jsonify(msgs)

@app.route('/api/status', methods=['GET'])
def connection_status():
    return jsonify({"connected": ble_manager.connected, "name": ble_manager.device_name})

# Existing scanning logic
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
        # We can use asyncio.run here because scanning doesn't depend on the persistent connection loop
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
    # Run on port 2334 to match previous configuration
    print("Starting PaceGuard Server (Python/Flask)...")
    print("Running at http://localhost:2334")
    app.run(host='0.0.0.0', port=2334, debug=True)
