import threading
from scservo_sdk_t import *
import time
import math
#imprort queue
import queue
import numpy as np
from prettytable import PrettyTable

class ServoSupervisor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.showUpdates = True
        self.showErrors = True
        self.servoIDs = [1, 2, 3, 4, 5, 6, 7, 8]
        self.upToDate_dataTypes = []
        self.GroupSyncReads = {}
        self.Data_Stack = {}

        #For sending comands to the servos
        self.action_queue = queue.Queue()
        self.current_actions = {"positions": [0]*8, "speeds": [0]*8}

        self._init_servocontroler()

    def start_run(self):
        self.start()

    def _init_servocontroler(self):
        # some global defined variables. Not changable-------------------
        self.SERIAL_PORT = 'COM3'
        self.BAUDRATE = 921600                #115200
        self.SCS_MOVING_ACC = 255
        self.SCS_MOVABLE_RANGE = (0, 4094)
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_SPEED = 46
        self.protocol_end = 0 # SCServo bit end(STS/SMS=0, SCS=1)
        self.initializing_pause = 3.5

        #[index, start_value, address, length, unit]
        self.servo_state_structure = {"Position": [0, 0, SMS_STS_PRESENT_POSITION_L, 2, [0, 4094, 0, 2*math.pi]], 
                                        "Velocity": [1, 0, SMS_STS_PRESENT_SPEED_L, 2, [0, 4094, 0, 2*math.pi]], 
                                        "Load": [2, 0, SMS_STS_PRESENT_LOAD_L, 2, 0.1], 
                                        "Voltage": [3, 0, SMS_STS_PRESENT_VOLTAGE, 1, 0.1], 
                                        "Current": [4, 0, SMS_STS_PRESENT_CURRENT_L, 2, 0.0065], 
                                        "Temperature": [5, 0, SMS_STS_PRESENT_TEMPERATURE, 1, 1], 
                                        "TorqueEnable": [6, 1, SMS_STS_TORQUE_ENABLE, 1, 1],
                                        "MobileSign": [7, 0, SMS_STS_MOVING, 1, 1]
                                        }

        # Initialize PortHandler and PacketHandler instances
        self.portHandler = PortHandler(self.SERIAL_PORT)
        #self.packetHandler = SMS_STS_PacketHandler(self.portHandler, self.protocol_end)
        self.packetHandler = STS_PacketHandler(self.portHandler, self.protocol_end)

        

        # connect to the port
        self.portHandler.setBaudRate(self.BAUDRATE)#first set the baudrate
        self.portHandler.openPort()#then open the port
        time.sleep(self.initializing_pause)

        #scaning the servos
        self.servoIDs = self.ScanServos()

        #reading Data -----------------------------------------------------

    def ScanServos(self):
        '''
        Scan for servos and returns a list with all the active servos and their id
    
        '''
        #self.servo_maintainer.stop()#stopt the thread, so that the servos can be initialized.
        if self.portHandler is None:
            print("Error: portHandler is not initialized.")
            return {}
        # Initialize the port with the baud rate
        print(f"Scanning for servos in the range 0 to 20 at {self.BAUDRATE} baud rate...")
        servo_pres = {}
        for servo_id in self.servoIDs:
            model_number ,result, error = self.packetHandler.ping(servo_id)    
            print(f"Servo ID: {servo_id}: Model Number {model_number}, Result: {result}, Error: {error}")
            if result == COMM_SUCCESS:
                print(f"Servo ID: {servo_id}: Model Number {model_number}")
                servo_pres[servo_id] = True
            else:
                print(f"Servo ID: {servo_id} Not present")
                servo_pres[servo_id] = None

        return servo_pres

    def _map_array(self, x:np.ndarray, in_min:float, in_max:float, out_min:float, out_max:float):
        # In order to map array the in_min, in_max, out_min, out_max should be to an array
        in_min_array = np.full_like(x, in_min)
        out_min_array = np.full_like(x, out_min)
        epsilon = 1e-6
        return (x - in_min_array) * (out_max - out_min) / (in_max - in_min + epsilon) + out_min_array
    
    def create_get_informations(self, data_types):
        """
        Create a get information function that returns all the desired data_types in order from all the servos.
        Following will be the Units for the data_types:
        - Position: rad
        - Velocity: rad/s
        - Load: Nm
        - Voltage: V
        - Current: A
        - Temperature: °C
        - TorqueEnable: bool
        - MobileSign: bool

        Args:
            data_type (str): The type of data to be read. e.g. 
        Returns:
            function: A function that returns the desired data type for all servos. ==> [servo1_data, servo2_data, ...] in order of data_types
        """
        #create a list of all the status types
        lambda_list = []
        for state_type in data_types:
            if "Position" == state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: self._map_array(np.array(self.Data_Stack[state_type]), in_min=0, in_max=4094, out_min=-math.pi, out_max=math.pi))
            elif "Velocity" == state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: self.Data_Stack[state_type])
            elif "Load"== state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: np.array(self.Data_Stack[state_type])*0.1)
            elif "Voltage"== state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: np.array(self.Data_Stack[state_type])*0.1)
            elif "Current"== state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: np.array(self.Data_Stack[state_type])*0.0065)
            elif "Temperature"== state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: self.Data_Stack[state_type])
            elif "TorqueEnable"== state_type:
                self.add_dataType(state_type)
                #converts the 1 to True and 0 to False
                lambda_list.append(lambda: np.array(self.Data_Stack[state_type])==1)
            elif "MobileSign"== state_type:
                self.add_dataType(state_type)
                lambda_list.append(lambda: np.array(self.Data_Stack[state_type])==1)


        def get_informations():
            return [func() for func in lambda_list]

        return get_informations
    
    def set_action_queue(self, action_queue:queue.Queue):
        """Set the action queue to send commands of position, speed, ..."""
        self.action_queue = action_queue
    
    def get_ServoStates(self, dataType):
        """returns for every servo the desired data type"""
        if dataType in self.upToDate_dataTypes:
            return self.Data_Stack[dataType]
        else:
            return None
        

    def add_dataType(self, data_type):
        #check if the data_type is already in the list
        if data_type not in self.upToDate_dataTypes:
            self.upToDate_dataTypes.append(data_type)
            #add the data_type to the GroupSyncReads
            print(self.servo_state_structure[data_type][2], self.servo_state_structure[data_type][3])
            groupSyncRead_obj = GroupSyncRead(self.packetHandler, self.servo_state_structure[data_type][2], self.servo_state_structure[data_type][3])
            groupSyncRead_obj.clearParam()
            for servoID in self.servoIDs:
                scs_addparam_result = groupSyncRead_obj.addParam(servoID)#false means that the scs_id was already added
                if scs_addparam_result != True:
                    print("[ID:%03d] groupSyncRead addparam failed" % servoID) if self.showErrors else None
            self.GroupSyncReads[data_type] = groupSyncRead_obj
            self.Data_Stack[data_type] = [0]*len(self.servoIDs)
        
    def remove_dataType(self, data_type):
        if data_type in self.upToDate_dataTypes:
            self.upToDate_dataTypes.remove(data_type)
            #remove the data_type from the GroupSyncReads
            self.GroupSyncReads.pop(data_type)
            self.Data_Stack.pop(data_type)

    def update_states(self):
        if len(self.upToDate_dataTypes) == 0:
            print("Error: No data types to be read")
            return
        for dataType, groupSync_obj in self.GroupSyncReads.items():
            scs_comm_result = groupSync_obj.txRxPacket()#send the packet and receive the data
            if scs_comm_result != COMM_SUCCESS:
                print(f"Error reading {dataType} data") if self.showErrors else None

            data_address = self.servo_state_structure[dataType][2]
            data_length = self.servo_state_structure[dataType][3]
            #print(groupSync_obj.data_dict)

            for index, scs_id in enumerate(self.servoIDs):
                scs_data_result, scs_error = groupSync_obj.isAvailable(scs_id, data_address, data_length)
                if scs_data_result != True:
                    print(f"Error reading {dataType} data from servo {scs_id}") if self.showErrors else None
                else:
                    data = groupSync_obj.getData(scs_id, data_address, data_length)
                    self.Data_Stack[dataType][index] = data
            groupSync_obj.clearParam()
            

    def send_command(self, new_positions, new_speeds):
        #Checks if the new_positions and new_speeds are of the same length as the number of servos
        if len(new_positions) != len(self.servoIDs) or len(new_speeds) != len(self.servoIDs):
            print("Error: The number of positions and speeds should be equal to the number of servos")
            return
        #check if the commands are the same as the previous commands
        if new_positions == self.current_actions["positions"] and new_speeds == self.current_actions["speeds"]:
            return

        for index, scs_id in enumerate(self.servoIDs):
            position = new_positions[index]
            speed = new_speeds[index]
            scs_addparam_result = self.packetHandler.SyncWritePosEx(scs_id, position, speed, self.SCS_MOVING_ACC)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % scs_id) if self.showErrors else None

        scs_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.packetHandler.groupSyncWrite.clearParam()

    def print_beautifullTable(self):
        """
        Print the current states in a beautifull table
        """
        if len(self.upToDate_dataTypes) == 0:
            print("Error: No data types to be read")
            return
        table = PrettyTable()
        table.field_names = ["Servo ID"] + self.upToDate_dataTypes
        for index, scs_id in enumerate(self.servoIDs):
            table.add_row([scs_id]+[self.Data_Stack[dataType][index] for dataType in self.upToDate_dataTypes])
        print("\033c", end="")  # Clear the console
        print(table)

    def run(self):
        print("Starting thread ????")
        while True:
            if not self.action_queue.empty():
                # Execute the actions in the queue
                action = self.action_queue.get()
                if action is not None:
                    # Execute the action
                    self.send_command(action["positions"], action["speeds"])
                    self.action_queue.task_done()
                time.sleep(0.005)
            else:
                # If no actions are available, update the states
                self.update_states()
            
            #show the current states
            if self.showUpdates:
                self.print_beautifullTable()



if __name__ == "__main__":
    # Usage example
    supervisor = ServoSupervisor()
    getFunction = supervisor.create_get_informations(["Position", "Velocity", "Load", "Current", "TorqueEnable"])
    supervisor.start_run()

    for i in range(10):
        time.sleep(1)
        supervisor.action_queue.put({"positions": [2000]*8, "speeds": [1000]*8})
        time.sleep(1)
        supervisor.action_queue.put({"positions": [3000]*8, "speeds": [1000]*8})