#!/usr/bin/env python

import serial
import time

COMPORT = "COM5"  # Adjust as needed (e.g., "COM6", "/dev/ttyUSB0")
BAUDRATE = 921600

# Define STServo register addresses
STS_PRESENT_POSITION_L = 56
STS_PRESENT_POSITION_H = 57
STS_PRESENT_SPEED_L = 58
STS_PRESENT_SPEED_H = 59
STS_PRESENT_LOAD_L = 60
STS_PRESENT_LOAD_H = 61
STS_PRESENT_VOLTAGE = 62
STS_PRESENT_TEMPERATURE = 63
STS_MOVING = 66
STS_PRESENT_CURRENT_L = 69
STS_PRESENT_CURRENT_H = 70

class SimpleServoHandler:
    def __init__(self, port, baudrate=921600, timeout=0.1):
        """
        Initialize the serial connection.
        
        Args:
            port (str): Serial port (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows).
            baudrate (int): Baud rate for communication (default: 921,600 bps).
            timeout (float): Serial timeout in seconds (default: 0.1s).
        """
        self.BROADCAST_ID = 0xFE  # Broadcast ID for all servos
        self.INST_SYNC_WRITE = 0x83  # Sync Write instruction
        self.INST_SYNC_READ = 0x82  # Sync Read instruction
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
        except serial.SerialException as e:
            raise Exception(f"Failed to open serial port {port}: {e}")

    def syncWrite(self, start_address, data_length, servo_data):
        """
        Fast SyncWrite for multiple servos.
        
        Args:
            start_address (int): Memory address to write to (e.g., 42 for position).
            data_length (int): Number of bytes per servo (e.g., 2 for position).
            servo_data (list): List of [servo_id, value] pairs, e.g., [[1, 1000], [2, 2000]].
        
        Returns:
            bool: True if transmission succeeded, False otherwise.
        """
        param_length = len(servo_data) * (1 + data_length)
        total_length = param_length + 4
        packet = [0] * (total_length + 4)

        packet[0] = 0xFF
        packet[1] = 0xFF
        packet[2] = self.BROADCAST_ID
        packet[3] = total_length
        packet[4] = self.INST_SYNC_WRITE
        packet[5] = start_address
        packet[6] = data_length

        idx = 7
        for servo_id, value in servo_data:
            packet[idx] = servo_id
            idx += 1
            for i in range(data_length):
                packet[idx + i] = (value >> (8 * i)) & 0xFF
            idx += data_length

        checksum = 0
        for i in range(2, total_length + 3):
            checksum += packet[i]
        packet[-1] = ~checksum & 0xFF

        try:
            self.serial.flush()
            self.serial.write(bytes(packet))
            return True
        except serial.SerialException:
            return False

    def syncRead(self, start_address, data_length, servo_ids):
        """
        Fast SyncRead from multiple servos.
        
        Args:
            start_address (int): Starting memory address to read from (e.g., 56 for position).
            data_length (int): Number of bytes to read per servo (e.g., 4 for position+speed).
            servo_ids (list): List of servo IDs to read from, e.g., [1, 2, 3].
        
        Returns:
            dict: {servo_id: [byte1, byte2, ...], ...} or None if failed.
        """
        param_length = len(servo_ids)
        total_length = param_length + 4
        packet = [0] * (total_length + 4)

        packet[0] = 0xFF
        packet[1] = 0xFF
        packet[2] = self.BROADCAST_ID
        packet[3] = total_length
        packet[4] = self.INST_SYNC_READ
        packet[5] = start_address
        packet[6] = data_length

        for i, servo_id in enumerate(servo_ids):
            packet[7 + i] = servo_id

        checksum = 0
        for i in range(2, total_length + 3):
            checksum += packet[i]
        packet[-1] = ~checksum & 0xFF

        try:
            self.serial.flush()
            self.serial.write(bytes(packet))
        except serial.SerialException:
            return None

        response_data = {}
        expected_packet_size = 6 + data_length
        timeout_start = time.time()

        while len(response_data) < len(servo_ids) and (time.time() - timeout_start) < self.serial.timeout:
            if self.serial.in_waiting >= expected_packet_size:
                response = self.serial.read(expected_packet_size)
                if len(response) == expected_packet_size and response[0] == 0xFF and response[1] == 0xFF:
                    servo_id = response[2]
                    packet_len = response[3]
                    error = response[4]
                    if packet_len == data_length + 2 and error == 0:
                        data = list(response[5:5 + data_length])
                        response_data[servo_id] = data
                    checksum = sum(response[2:5 + data_length]) & 0xFF
                    if (~checksum & 0xFF) != response[-1]:
                        response_data.pop(servo_id, None)

        return response_data if response_data else None

    def close(self):
        """Close the serial connection."""
        if self.serial.is_open:
            self.serial.close()

def process_read_data( data, start_address):
    current_time = time.time()
    # delta_time = (current_time - self.last_time) * 1000  # ms
    # self.last_time = current_time
    # self.update_rate_signal.emit(delta_time)

    for servo_id, bytes_data in data.items():
        parsed_data = {'servo_id': servo_id, 'timestamp': current_time}

        if start_address == STS_PRESENT_POSITION_L:
            if len(bytes_data) == 2:
                parsed_data['angle'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
            elif len(bytes_data) == 14:
                parsed_data.update({
                    'angle': int.from_bytes(bytes_data[0:2], byteorder='little', signed=True),
                    'speed': int.from_bytes(bytes_data[2:4], byteorder='little', signed=True),
                    'load': int.from_bytes(bytes_data[4:6], byteorder='little'),
                    'voltage': bytes_data[6],
                    'temperature': bytes_data[7],
                    'async_write_flag': bytes_data[8],
                    'servo_status': bytes_data[9],
                    'mobile_sign': bytes_data[10],
                    'current': int.from_bytes(bytes_data[12:14], byteorder='little', signed=True)
                })
        elif start_address == STS_PRESENT_SPEED_L:
            parsed_data['speed'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
        elif start_address == STS_PRESENT_LOAD_L:
            parsed_data['load'] = int.from_bytes(bytes_data[0:2], byteorder='little')
        elif start_address == STS_PRESENT_VOLTAGE:
            parsed_data['voltage'] = bytes_data[0]
        elif start_address == STS_PRESENT_TEMPERATURE:
            parsed_data['temperature'] = bytes_data[0]
        elif start_address == STS_PRESENT_CURRENT_L:
            parsed_data['current'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)

        return parsed_data

# Example usage
if __name__ == "__main__":
    # Initialize the servo handler
    try:
        servo_handler = SimpleServoHandler(port=COMPORT, baudrate=BAUDRATE)
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

    # Define servo data for writing positions
    servo_data = [
        [[1, 100],  [2, 1000], [3, 1000]],  # State 0
        [[1, 1000], [2, 2000], [3, 3000]],  # State 1
        [[1, 2000], [2, 1000], [3, 3000]]   # State 2
    ]
    servo_ids = [1, 2]
    i = 0

    try:
        while True:
            # Sync Write positions
            success = servo_handler.syncWrite(start_address=42, data_length=2, servo_data=servo_data[i])
            print(f"Transmission {'succeeded' if success else 'failed'}")

            # Wait for servos to move
            #time.sleep(1)

            # Sync Read position and speed (4 bytes: 56-59)
            print(f"{'Servo ID':<10}{'Timestamp':<20}{'Angle':<10}{'Speed':<10}{'Load':<10}{'Voltage':<10}{'Temperature':<15}{'Current':<10}")
                      
            for j in range(20):
                data = servo_handler.syncRead(start_address=STS_PRESENT_POSITION_L, data_length=14, servo_ids=servo_ids)
                if data:
                    '''
                    for servo_id, bytes_data in data.items():
                        # Parse position (bytes 0-1) and speed (bytes 2-3)
                        position = bytes_data[0] | (bytes_data[1] << 8)
                        speed = bytes_data[2] | (bytes_data[3] << 8)
                        # Handle signed speed (assuming 16-bit signed integer)
                        if speed > 32767:
                            speed -= 65536
                        print(f"Servo {servo_id}: Position = {position}, Speed = {speed}")
                    '''
                    parsed_data = process_read_data(data, STS_PRESENT_POSITION_L)
                    if parsed_data:
                        print(f"{parsed_data['servo_id']:<10}{parsed_data['timestamp']:<20}{parsed_data.get('angle', 'N/A'):<10}{parsed_data.get('speed', 'N/A'):<10}{parsed_data.get('load', 'N/A'):<10}{parsed_data.get('voltage', 'N/A'):<10}{parsed_data.get('temperature', 'N/A'):<15}{parsed_data.get('current', 'N/A'):<10}")
                else:
                    print("Failed to read servo data")

            # Cycle through states
            i = (i + 1) % 3
            

    except KeyboardInterrupt:
        print("\nStopped by user")
    
    # Clean up
    servo_handler.close()