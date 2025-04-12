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
        self.initial_orientation = np.array([0, 0, 0])
        
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
        # while True:
        #     try:
        #         response = requests.get(self.cam_url, stream=True)
        #         if response.status_code == 200:
        #             bytes_data = bytes()
        #             for chunk in response.iter_content(chunk_size=1024):
        #                 bytes_data += chunk
        #                 a = bytes_data.find(b'\xff\xd8')
        #                 b = bytes_data.find(b'\xff\xd9')
        #                 if a != -1 and b != -1:
        #                     jpg = bytes_data[a:b+2]
        #                     bytes_data = bytes_data[b+2:]
        #                     img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        #                     if img is not None:
        #                         self.cam_data = img
        #                         #print(f"Frame received: shape={img.shape}")
        #         else:
        #             #print(f"Failed to connect to camera stream: {response.status_code}")
        #             pass
        #     except requests.RequestException as e:
        #         #print(f"Error fetching camera stream: {e}")
        #         pass
        #     time.sleep(0.1)  # Add a short delay to avoid excessive requests
        try:
            response = requests.get(self.cam_url, stream=True)
            if response.status_code == 200:
                bytes_data = bytes()
                for chunk in response.iter_content(chunk_size=1024):
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')  # JPEG start marker
                    b = bytes_data.find(b'\xff\xd9')  # JPEG end marker
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            self.cam_data = img
            else:
                print(f"Failed to connect to camera stream: {response.status_code}")
        except requests.RequestException as e:
            print(f"Error fetching camera stream: {e}")

    def on_error(self, _, error):
        print(f"WebSocket Error: {error}")

    def on_close(self, _, close_status_code, close_msg):
        print(f"WebSocket closed: {close_status_code}, {close_msg}")

    def on_open(self, _):
        print("WebSocket connection opened")


    def euler_to_rotation_matrix(self, euler):
        """Convert Euler angles (roll, pitch, yaw) to a rotation matrix (sensor to world)."""
        roll, pitch, yaw = euler
        cr = np.cos(roll)
        sr = np.sin(roll)
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])
        return R

    def get_accelerometer_data(self, dt=0.01):
        """Returns the latest orientation (rad), angular velocity (rad/s), and linear acceleration (m/s²).
        
        Args:
            dt (float): Time step in seconds for integrating gyroscope data.
        
        Returns:
            tuple: (orientation (rad), angular_velocity (rad/s), linear_acceleration (m/s²))
                All quantities are in the world frame, compatible with PyBullet.
        """
        if self.accelerometer_data is None:
            return np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0])

        # Extract raw sensor values
        ax, ay, az = self.accelerometer_data["ax"], self.accelerometer_data["ay"], self.accelerometer_data["az"]
        gx, gy, gz = self.accelerometer_data["gx"], self.accelerometer_data["gy"], self.accelerometer_data["gz"]
        mx, my, mz = self.accelerometer_data["mx"], self.accelerometer_data["my"], self.accelerometer_data["mz"]

        # Convert units
        linear_acceleration_sensor = np.array([ax, ay, az]) * 9.81  # g to m/s², sensor frame
        angular_velocity_sensor = np.radians([gx, gy, gz])          # deg/s to rad/s, sensor frame

        # Estimate roll and pitch from accelerometer
        roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Normalize magnetometer readings
        mag_norm = np.sqrt(mx**2 + my**2 + mz**2)
        if mag_norm == 0:
            mx, my, mz = 0, 0, 0
        else:
            mx, my, mz = mx / mag_norm, my / mag_norm, mz / mag_norm

        # Tilt-compensated yaw from magnetometer
        mag_x = mx * np.cos(pitch) + mz * np.sin(pitch)
        mag_y = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
        yaw = np.arctan2(mag_y, mag_x)

        orientation_abs = np.array([roll, pitch, yaw])  # Absolute orientation (rad), sensor frame
        orientation_rel = np.array([roll, pitch, yaw]) - self.initial_orientation  # Relative orientation (rad), sensor frame
        orientation_rel = np.mod(orientation_rel + np.pi, 2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Apply complementary filter if previous orientation exists
        if hasattr(self, "previous_orientation"):
            gyro_roll = self.previous_orientation[0] + angular_velocity_sensor[0] * dt
            gyro_pitch = self.previous_orientation[1] + angular_velocity_sensor[1] * dt
            gyro_yaw = self.previous_orientation[2] + angular_velocity_sensor[2] * dt
            orientation_abs = 0.95 * np.array([gyro_roll, gyro_pitch, gyro_yaw]) + 0.05 * orientation_abs

        self.previous_orientation = orientation_abs

        # Convert orientation to rotation matrix
        R = self.euler_to_rotation_matrix(orientation_abs)

        # Transform angular velocity to world frame
        angular_velocity_world = R @ angular_velocity_sensor

        # Transform linear acceleration to world frame, adjusting for gravity
        g_world = np.array([0, 0, -9.81])  # Gravity in world frame (Z-up)
        linear_acceleration_world = R @ linear_acceleration_sensor + g_world

        return orientation_rel, angular_velocity_world, linear_acceleration_world, orientation_abs
    
    def set_initial_orientation(self):
        """Sets the initial orientation of the camera."""
        while self.accelerometer_data is None:
            # Wait for accelerometer data to be available
            time.sleep(0.5)
            print("Waiting for accelerometer data...")
        if self.accelerometer_data is not None:
            roll, pitch, yaw = self.get_accelerometer_data()[3]
            self.initial_orientation = np.array([roll, pitch, yaw])
            print(f"Initial orientation set: {self.initial_orientation}")
        else:
            print("No accelerometer data available to set initial orientation. Despite waiting.")

    def get_relative_data(self):
        """Returns the relative orientation of the camera with respect to the initial orientation."""
        relative_orientation, angular_vel_world, linear_acc_world, orn_abs = self.get_accelerometer_data()
        return relative_orientation, angular_vel_world, linear_acc_world




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

    def close(self):
        """Closes the WebSocket connection."""
        self.ws.close()


if __name__ == "__main__":
    # Usage example
    camera = Camera()
    camera.set_initial_orientation()

    # Get accelerometer data
    #data = camera.get_accelerometer_data()
    #print("Accelerometer Data:", data)

    # Start camera stream
    #camera.show_cam_stream()$
    while True:
        #clear print
        print("\033c", end="")
        # print(np.array(camera.get_accelerometer_data()), 3)
        #rounding to 3 decimal places
        print(np.array(camera.get_relative_data()[0]*180/np.pi).round(1))
        print(np.array(camera.get_relative_data()[1]*180/np.pi).round(1))
        print(camera.get_relative_data()[2].round(3))

        time.sleep(0.1)

        
