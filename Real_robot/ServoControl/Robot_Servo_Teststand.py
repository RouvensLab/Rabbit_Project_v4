#!/usr/bin/env python

import serial
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from queue import Queue

COMPORT = "COM3"  # Adjust as needed
BAUDRATE = 921600
SERVOS = [1, 2, 3, 4]
SERIAL_TIMEOUT = 0.03

# Instruction for STS Protocol
INST_PING       = 1
INST_READ       = 2
INST_WRITE      = 3
INST_REG_WRITE  = 4
INST_ACTION     = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ  = 130  # 0x82

# Memory table definition (relevant addresses only)
STS_ID                  = 5
STS_PRESENT_POSITION_L  = 56
STS_PRESENT_SPEED_L     = 58
STS_PRESENT_LOAD_L      = 60
STS_PRESENT_VOLTAGE     = 62
STS_PRESENT_TEMPERATURE = 63
STS_MOVING              = 66
STS_PRESENT_CURRENT_L   = 69

# Define data types with their starting addresses and lengths
DATA_TYPES = {
    'id': (STS_ID, 1),
    'position': (STS_PRESENT_POSITION_L, 2),
    'speed': (STS_PRESENT_SPEED_L, 2),
    'load': (STS_PRESENT_LOAD_L, 2),
    'voltage': (STS_PRESENT_VOLTAGE, 1),
    'temperature': (STS_PRESENT_TEMPERATURE, 1),
    'moving': (STS_MOVING, 1),
    'current': (STS_PRESENT_CURRENT_L, 2)
}

class Serial_Master:
    def __init__(self, serial_port, baud_rate):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.last_time = time.time()
        self.servo_ids = SERVOS
        self.exit_flag = False

    def calculate_read_params(self, desired_data_types):
        """Calculate the start address and length to cover all desired data types."""
        if not desired_data_types:
            return None, None
        min_addr = min(DATA_TYPES[dt][0] for dt in desired_data_types)
        max_addr = max(DATA_TYPES[dt][0] + DATA_TYPES[dt][1] - 1 for dt in desired_data_types)
        start_address = min_addr
        data_length = max_addr - min_addr + 1
        return start_address, data_length

    def process_read_data(self, data, start_address, desired_data_types):
        """Parse only the desired data types from the response."""
        current_time = time.time()
        delta_time = (current_time - self.last_time) * 1000  # ms
        self.last_time = current_time

        transformed_data = []

        for servo_id, bytes_data in data.items():
            parsed_data = {'servo_id': servo_id, 'timestamp': current_time}
            for data_type in desired_data_types:
                addr, length = DATA_TYPES[data_type]
                offset = addr - start_address
                if 0 <= offset < len(bytes_data) and offset + length <= len(bytes_data):
                    value_bytes = bytes_data[offset:offset + length]
                    if data_type in ['position', 'speed', 'current']:
                        parsed_data[data_type] = int.from_bytes(value_bytes, byteorder='little', signed=True)
                    elif data_type == 'load':
                        parsed_data[data_type] = int.from_bytes(value_bytes, byteorder='little')
                    elif data_type in ['voltage', 'temperature', 'moving', 'id']:
                        parsed_data[data_type] = value_bytes[0]
            transformed_data.append(parsed_data)
        
        return transformed_data
        


    def syncReadDirect(self, servo_ids, desired_data_types):
        start_address, data_length = self.calculate_read_params(desired_data_types)
        if start_address is not None and data_length is not None:
            data = self.syncRead(start_address, data_length, servo_ids)
            if data:
                data = self.process_read_data(data, start_address, desired_data_types)
                return data
        return None

    def syncRead(self, start_address, data_length, servo_ids, timeout=0.005):
        """Send syncRead command and collect responses."""
        param_length = len(servo_ids)
        total_length = param_length + 4
        packet = [0xFF, 0xFF, 0xFE, total_length, INST_SYNC_READ, start_address, data_length]
        packet.extend(servo_ids)
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)

        try:
            if not self.ser or not self.ser.is_open:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=timeout)
            self.ser.flush()
            self.ser.write(bytes(packet))
        except serial.SerialException:
            return None

        response_data = {}
        expected_packet_size = 6 + data_length
        timeout_start = time.time()

        while len(response_data) < len(servo_ids) and (time.time() - timeout_start) < timeout:
            if self.ser.in_waiting >= expected_packet_size:
                response = self.ser.read(expected_packet_size)
                if len(response) == expected_packet_size and response[0] == 0xFF and response[1] == 0xFF:
                    servo_id = response[2]
                    packet_len = response[3]
                    error = response[4]
                    if packet_len == data_length + 2 and error == 0:
                        data = response[5:5 + data_length]
                        response_data[servo_id] = data
                    # Checksum validation could be added here if needed

        return response_data if response_data else None
    
    def setTorque(self, servo_ids, enable):
        """Enable or disable torque for a list of servos using syncWrite."""
        if not servo_ids:
            return

        # Ensure the serial port is open
        if not self.ser or not self.ser.is_open:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=SERIAL_TIMEOUT)

        # Torque Enable register address (decimal 40, hex 0x28)
        register_address = 40
        # Data length per servo: 1 byte (Torque Enable is a 1-byte value)
        data_length = 1
        # Value to write: 1 to enable, 0 to disable
        value = 1 if enable else 0

        # Construct the syncWrite packet
        packet = [0xFF, 0xFF, 0xFE]  # Broadcast ID 0xFE
        N = len(servo_ids)
        total_length = 4 + (1 + data_length) * N  # Length: 4 base bytes + (ID + data) per servo
        packet.append(total_length)
        packet.append(INST_SYNC_WRITE)
        packet.append(register_address)
        packet.append(data_length)

        # Add servo IDs and the torque value for each
        for servo_id in servo_ids:
            packet.append(servo_id)
            packet.append(value)

        # Calculate checksum
        checksum = sum(packet[2:]) & 0xFF
        checksum = (~checksum) & 0xFF
        packet.append(checksum)

        # Send the packet
        self.ser.write(bytes(packet))
        
        

if __name__ == "__main__":
    sm = Serial_Master(COMPORT, BAUDRATE)
    time.sleep(5)
    sm.setTorque(SERVOS, False)
    while True:
        # Example: Read position and temperature from all servos
        desired_data_types = ['position', 'temperature']
        #syncRead
        print(sm.syncReadDirect(SERVOS, desired_data_types))
        time.sleep(0.1)