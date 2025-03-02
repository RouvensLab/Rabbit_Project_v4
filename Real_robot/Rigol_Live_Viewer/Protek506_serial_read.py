#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Read serial output from a device (e.g., RS-232 at 1200,N,7,2) and handle user input.
Based on BASIC program logic for serial communication.

Author: Georg (adapted by Grok)
Date: 2025-02-23
"""

import datetime
import msvcrt  # For Windows keyboard input (non-blocking)
import signal
import sys
import serial
import time
import random
from pathlib import Path
from typing import Optional

# Configuration
SERIAL_CONFIG = {
    'port': 'COM6',  # Adjust to your system's serial port (e.g., '/dev/ttyUSB0' on Linux)
    'baudrate': 1200,
    'bytesize': serial.SEVENBITS,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_TWO,
    'timeout': 1,
    'xonxoff': False,
    'rtscts': False
}

LOG_FILE = Path("serial_log.txt")
SAMPLE_INTERVAL = 1.0  # seconds between readings

class SerialReader:
    """Class to handle reading and logging from a serial device with user interaction."""
    
    def __init__(self) -> None:
        self.ser: Optional[serial.Serial] = None
        self.running: bool = False
        self.log_file = None
        
    def setup(self) -> None:
        """Initialize serial connection and log file."""
        try:
            self.ser = serial.Serial(**SERIAL_CONFIG)
            self.log_file = LOG_FILE.open("a", encoding="utf-8")
            self.running = True
            print("WELCOME!")
            print(f"RS232 EXAMPLE PROGRAM BY BASIC - {SERIAL_CONFIG['baudrate']} BAUD")
            print("1 NONE PARITY")
            print("7 DATA BIT")
            print("2 STOP BIT")
            print("COM PORT")
            print("PLEASE")
            print("PRESS ANY KEY TO EXECUTE OR ESC TO STOP")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)
        except IOError as e:
            print(f"Failed to open log file: {e}")
            sys.exit(1)

    def cleanup(self) -> None:
        """Cleanly close connections and files."""
        self.running = False
        try:
            if self.log_file:
                self.log_file.close()
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("\nProgram terminated cleanly")
        except Exception as e:
            print(f"Error during cleanup: {e}")

    def signal_handler(self, sig: int, frame) -> None:
        """Handle Ctrl+C and other termination signals."""
        print('\nReceived shutdown signal')
        self.cleanup()
        sys.exit(0)

    def check_keyboard(self) -> bool:
        """Check for keyboard input (Windows-specific). Returns True if ESC is pressed."""
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\x1b':  # ESC key
                return True
        return False

    def read_measurement(self) -> Optional[str]:
        """Read random data from the serial port, simulating BASIC's RND and INPUT."""
        try:
            if not self.ser or not self.ser.is_open:
                return None
                
            # Simulate random input similar to BASIC's RND (though serial data isn't random)
            self.ser.write(b"\n")  # Trigger measurement
            data = self.ser.read(1)  # Read first byte
            if data:
                # Read any remaining bytes in buffer
                data += self.ser.read(self.ser.in_waiting or 10)
                timestamp = datetime.datetime.now().isoformat()
                reading = data.decode('ascii', errors='replace').strip()
                # Simulate BASIC's random number check (TS = INPUT$(1))
                if reading and random.random() < 0.5:  # Simple simulation of RND condition
                    return f"{timestamp}  {reading}"
            return None
                
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None

    def run(self) -> None:
        """Main loop to continuously read serial data and handle user input."""
        # Register signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.setup()
        
        try:
            i = 1  # Counter similar to BASIC's loop variable
            while self.running:
                # Check for ESC key press (simulating BASIC's INKEY$)
                if self.check_keyboard():
                    print("\nStopping program...")
                    break
                
                # Read measurement (simulating BASIC's INPUT and RND logic)
                measurement = self.read_measurement()
                if measurement:
                    print(f"RS$(i) = {measurement}")  # Simulate BASIC array RS$(i)
                    self.log_file.write(f"{measurement}\n")
                    self.log_file.flush()
                    i += 1  # Increment counter
                    
                    # Simulate BASIC's IF RS$(i) = CHR$(13) THEN 250 logic
                    if measurement.endswith('\r'):  # CHR$(13) is carriage return
                        print("End of line detected")
                else:
                    print("No data received")
                
                time.sleep(SAMPLE_INTERVAL)
                
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            self.cleanup()

def main() -> None:
    """Entry point of the program."""
    reader = SerialReader()
    reader.run()

if __name__ == "__main__":
    main()
