import websocket
import json
import cv2
import numpy as np
import threading
import requests
import time


class Camera:
    def __init__(self):
        self.ATOM_IP = "192.168.4.1"  # Replace with your device's IP address
        self.accel_url = f"ws://{self.ATOM_IP}/api/v1/ws/imu_data"
        self.cam_url = f"http://{self.ATOM_IP}/api/v1/stream"

        self.accelerometer_data = None
        self.cam_data = None  # Store the latest video frame
        
        # Initialize WebSocket for IMU data
        self.ws = websocket.WebSocketApp(
            self.accel_url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.on_open = self.on_open

        # Start WebSocket threads
        self.start_ws()

    def on_message(self, _, message):
        """Handles accelerometer data from WebSocket."""
        data = json.loads(message)
        self.accelerometer_data = data
        #print(f"Accelerometer Data: {data}")

    # Stream cameras
    def stream_camera(self):
        while True:
            try:
                response = requests.get(self.cam_url, stream=True)
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
                                self.cam_data = img
                                print(f"Frame received: shape={img.shape}")
                else:
                    print(f"Failed to connect to camera stream: {response.status_code}")
            except requests.RequestException as e:
                print(f"Error fetching camera stream: {e}")
            time.sleep(0.1)  # Add a short delay to avoid excessive requests

    def on_error(self, _, error):
        print(f"WebSocket Error: {error}")

    def on_close(self, _, close_status_code, close_msg):
        print(f"WebSocket closed: {close_status_code}, {close_msg}")

    def on_open(self, _):
        print("WebSocket connection opened")



    def get_accelerometer_data(self, dt=0.01):
        """Returns the latest orientation (rad), angular velocity (rad/s), and linear acceleration (m/s²).
        
        Args:
            dt (float): Time step in seconds for integrating gyroscope data.
        
        Returns:
            tuple: (orientation (rad), angular_velocity (rad/s), linear_acceleration (m/s²))
        """
        if self.accelerometer_data is None:
            return None, None, None

        # Extract raw sensor values
        ax, ay, az = self.accelerometer_data["ax"], self.accelerometer_data["ay"], self.accelerometer_data["az"]
        gx, gy, gz = self.accelerometer_data["gx"], self.accelerometer_data["gy"], self.accelerometer_data["gz"]
        mx, my, mz = self.accelerometer_data["mx"], self.accelerometer_data["my"], self.accelerometer_data["mz"]

        # Convert accelerometer readings from g to m/s² (1g ≈ 9.81 m/s²)
        linear_acceleration = np.array([ax, ay, az]) * 9.81

        # Convert gyroscope readings from degrees/s to radians/s
        angular_velocity = np.radians([gx, gy, gz])

        # Estimate roll (ϕ) and pitch (θ) using accelerometer
        roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))  # Rotation around X-axis
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))  # Rotation around Y-axis

        # Tilt-compensated yaw (ψ) calculation using magnetometer
        # Step 1: Normalize magnetometer readings
        mag_norm = np.sqrt(mx**2 + my**2 + mz**2)
        if mag_norm == 0:  # Avoid division by zero
            mx, my, mz = 0, 0, 0
        else:
            mx, my, mz = mx / mag_norm, my / mag_norm, mz / mag_norm

        # Step 2: Apply tilt compensation
        mag_x = mx * np.cos(pitch) + mz * np.sin(pitch)
        mag_y = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
        
        yaw = np.arctan2(mag_y, mag_x)  # Rotation around Z-axis

        # Step 3: Gyroscope Integration (Optional: Use Kalman/Madgwick Filter for better results)
        if hasattr(self, "previous_orientation"):  # Integrate if previous data exists
            gyro_roll = self.previous_orientation[0] + angular_velocity[0] * dt
            gyro_pitch = self.previous_orientation[1] + angular_velocity[1] * dt
            gyro_yaw = self.previous_orientation[2] + angular_velocity[2] * dt

            # Complementary filter (95% gyro, 5% accelerometer/magnetometer)
            roll = 0.95 * gyro_roll + 0.05 * roll
            pitch = 0.95 * gyro_pitch + 0.05 * pitch
            yaw = 0.95 * gyro_yaw + 0.05 * yaw

        self.previous_orientation = np.array([roll, pitch, yaw])  # Store for next iteration

        orientation = np.array([roll, pitch, yaw])  # Orientation in radians

        return orientation, angular_velocity, linear_acceleration


    def get_frame(self):
        """Returns the latest video frame."""
        return self.cam_data

    def start_ws(self):
        """Runs WebSockets in separate threads."""
        print("Starting WebSocket threads...Accelerometer")
        threading.Thread(target=self.ws.run_forever, daemon=True).start()
        # print("Starting WebSocket threads...Camera")
        threading.Thread(target=self.stream_camera, daemon=True).start()

    def show_cam_stream(self):
        """Continuously displays the latest video frame."""
        while True:
            frame = self.get_frame()
            if frame is not None:
                cv2.imshow("M5AtomS3 M12 Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                pass
                #print(f"No frame received: frame={frame}")


if __name__ == "__main__":
    # Usage example
    camera = Camera()

    # Get accelerometer data
    #data = camera.get_accelerometer_data()
    #print("Accelerometer Data:", data)

    # Start camera stream
    camera.show_cam_stream()
