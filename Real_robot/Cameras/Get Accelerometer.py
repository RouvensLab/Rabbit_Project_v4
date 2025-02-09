import websocket
import json



class Camera:
    def __init__(self):
        self.ATOM_IP = "192.168.4.1"  # Replace with your device's IP address
        self.URL = f"ws://{self.ATOM_IP}/api/v1/ws/imu_data"
        self.ws = websocket.WebSocketApp(self.URL,
                                         on_message=self.on_message,
                                         on_error=self.on_error,
                                         on_close=self.on_close)
        self.ws.on_open = self.on_open
        self.accelerometer_data = None

    def on_message(self, ws, message):
        # Decode the JSON message received
        data = json.loads(message)
        print(f"IMU Data: {data}")
        # Example: Access individual sensor data
        self.accelerometer_data = data
        # ax, ay, az = data["ax"], data["ay"], data["az"]
        # print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")

    def on_error(self, ws, error):
        print(f"Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        print("### closed ###")

    def on_open(self, ws):
        print("WebSocket connection opened")

    def run(self):
        self.ws.run_forever()

    def get_accelerometer_data(self):
        return self.accelerometer_data

# Usage example
camera = Camera()
camera.run()

# To get the accelerometer data, you can call:
data = camera.get_accelerometer_data()
# print(data)