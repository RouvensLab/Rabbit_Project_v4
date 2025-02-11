import socket
import time
import serial

# Define the serial port and baud rate.
SERIAL_PORT = '/dev/ttyUSB0'  # Update this to your serial port
BAUD_RATE = 1000000

# Initialize serial communication
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# WiFi settings
HOST = '192.168.1.100'  # Update this to your server's IP address
PORT = 12345  # Update this to your server's port

# Initialize WiFi communication
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

def read_servo_feedback():
    ser.write(b'feedback_command')  # Replace with actual command to get feedback
    time.sleep(0.1)
    response = ser.readline().decode('utf-8').strip()
    return response

def main():
    while True:
        feedback = read_servo_feedback()
        if feedback:
            print("Feedback:", feedback)
            sock.sendall(feedback.encode('utf-8'))
        else:
            print("FeedBack err")
            time.sleep(0.5)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
