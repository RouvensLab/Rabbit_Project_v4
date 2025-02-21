import serial
import time
import scs 

class SCServo:
    def __init__(self, port_name="COM3", baudrate=921600):
        #self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)

        self.ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            # parity = serial.PARITY_ODD,
            # stopbits = serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=0)


    def send_command(self, command):
        """Send a command to the servo and return the response."""
        self.ser.write(command)
        time.sleep(0.01)
        return self.ser.read(10)  # Adjust the read length as needed

    def read_feedback(self, servo_id):
        """Request feedback from the servo."""
        command = bytes([servo_id])  # Replace with actual feedback request command
        return self.send_command(command)
    
    def read_position(self, servo_id):
        """Read position of the servo."""
        return self.read_feedback(servo_id)
    
    def read_speed(self, servo_id):
        """Read speed of the servo."""
        return self.read_feedback(servo_id)
    
    def read_load(self, servo_id):
        """Read load of the servo."""
        return self.read_feedback(servo_id)
    
    def read_voltage(self, servo_id):
        """Read voltage of the servo."""
        return self.read_feedback(servo_id)
    
    def read_temperature(self, servo_id):
        """Read temperature of the servo."""
        return self.read_feedback(servo_id)
    
    def read_move(self, servo_id):
        """Read movement status of the servo."""
        return self.read_feedback(servo_id)



class GroupSyncRead:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:  # len(self.data_dict.keys()) == 0:
            return

        self.param = []

        for scs_id in self.data_dict:
            self.param.append(scs_id)

    def addParam(self, scs_id):
        if scs_id in self.data_dict:  # scs_id already exist
            return False

        self.data_dict[scs_id] = []  # [0] * self.data_length

        self.is_param_changed = True
        return True

    def removeParam(self, scs_id):
        if scs_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[scs_id]

        self.is_param_changed = True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncReadTx(self.port, self.start_address, self.data_length, self.param,
                                  len(self.data_dict.keys()) * 1)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for scs_id in self.data_dict:
            self.data_dict[scs_id], result, _ = self.ph.readRx(self.port, scs_id, self.data_length)
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):
        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, scs_id, address, data_length):
        #if self.last_result is False or scs_id not in self.data_dict:
        if scs_id not in self.data_dict:
            return False

        if (address < self.start_address) or (self.start_address + self.data_length - data_length < address):
            return False

        if len(self.data_dict[scs_id])<data_length:
            return False
        return True

    def getData(self, scs_id, address, data_length):
        if not self.isAvailable(scs_id, address, data_length):
            return 0

        if data_length == 1:
            return self.data_dict[scs_id][address - self.start_address]
        elif data_length == 2:
            return SCS_MAKEWORD(self.data_dict[scs_id][address - self.start_address],
                                self.data_dict[scs_id][address - self.start_address + 1])
        elif data_length == 4:
            return SCS_MAKEDWORD(SCS_MAKEWORD(self.data_dict[scs_id][address - self.start_address + 0],
                                              self.data_dict[scs_id][address - self.start_address + 1]),
                                 SCS_MAKEWORD(self.data_dict[scs_id][address - self.start_address + 2],
                                              self.data_dict[scs_id][address - self.start_address + 3]))
        else:
            return 0


# Initialize servo communication
sc = SCServo(port_name="COM3")  # Adjust the port accordingly

def main():
    while True:
        try:
            servo_id = 1
            pos = sc.read_position(servo_id)
            speed = sc.read_speed(servo_id)
            load = sc.read_load(servo_id)
            voltage = sc.read_voltage(servo_id)
            temp = sc.read_temperature(servo_id)
            move = sc.read_move(servo_id)

            print(f"ID:{servo_id}   Position:   {pos}")
            print(f"ID:{servo_id}   Speed:      {speed}")
            print(f"ID:{servo_id}   Load:       {load}")
            print(f"ID:{servo_id}   Voltage:    {voltage}")
            print(f"ID:{servo_id}   Temperature:{temp}")
            print(f"ID:{servo_id}   Move:       {move}")
            print()

            time.sleep(0.5)
        except KeyboardInterrupt:
            print("Exiting...")
            break

if __name__ == "__main__":
    main()
