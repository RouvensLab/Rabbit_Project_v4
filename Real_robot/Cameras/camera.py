import cv2
import numpy as np
import requests
import websocket
import json
import threading
import time


CAMERA_URL = "http://192.168.4.1/api/v1/stream"
WS_URL = "ws://192.168.4.1/ws"

# Accelerometer callback
def on_message(ws, message):
    data = json.loads(message)
    print(f"Accelerometer -> X: {data['ax']}, Y: {data['ay']}, Z: {data['az']}")

# Start WebSocket in a thread
def start_ws():
    ws = websocket.WebSocketApp(WS_URL, on_message=on_message)
    ws.run_forever()

# Start WebSocket thread to receive accelerometer data asynchronously
ws_thread = threading.Thread(target=start_ws)
ws_thread.daemon = True
ws_thread.start()

# Stream camera
def stream_camera():
    while True:
        try:
            response = requests.get(CAMERA_URL, stream=True)
            if response.status_code == 200:
                bytes_data = bytes()
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            cv2.imshow("M5AtomS3 M12 Camera", img)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
            else:
                print(f"Failed to connect to camera stream: {response.status_code}")
        except requests.RequestException as e:
            print(f"Error fetching camera stream: {e}")
        
        time.sleep(0.1)  # Add a short delay to avoid excessive requests

# Start camera stream
stream_camera()
