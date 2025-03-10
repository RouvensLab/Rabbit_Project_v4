import threading
from scservo_sdk_t import *
import time
import math
#imprort queue
import queue
import numpy as np
from prettytable import PrettyTable

class ServoSupervisor(threading.Thread):
    def __init__(self, servo_id_order=[1, 2, 3, 4, 5, 6, 7, 8]):
        threading.Thread.__init__(self)
        self.showUpdates = False
        self.showErrors = True
        self.no_close = True
        self.pause = True
        self.servoIDs = servo_id_order
        self.upToDate_dataTypes = []
        self.GroupSyncReads = {}
        self.Data_Stack = {}
        self.parti_Update_state = 0

        self.servos_info = {
            1: {"min": 0, "max": 4094, "orientation": 0},
            2: {"min": 0, "max": 4094, "orientation": 1},
            3: {"min": 0, "max": 4094, "orientation": 0},
            4: {"min": 0, "max": 4094, "orientation": 1},
            5: {"min": 0, "max": 4094, "orientation": 0},
            6: {"min": 0, "max": 4094, "orientation": 1},
            7: {"min": 0, "max": 4094, "orientation": 1},
            8: {"min": 0, "max": 4094, "orientation": 0},
        }

        #For sending comands to the servos
        self.action_queue = queue.Queue()
        self.current_actions = {"positions": [0]*8, "speeds": [0]*8}

        self._init_servocontroler()

    def start_run(self):
        self.pause = False
        self.start()

    def stop_run(self):
        self.pause = True
    def continiu_run(self):
        self.pause = False
    
    def close(self):
        #joint the thread
        self.no_close = False
        self.join()
        #close the port
        self.portHandler.closePort()

    def _init_servocontroler(self):
        # some global defined variables. Not changable-------------------
        self.SERIAL_PORT = 'COM3'
        self.BAUDRATE = 921600                #115200
        self.SCS_MOVING_ACC = 255
        self.SCS_MOVABLE_RANGE = (0, 4080)#(0, 4094)
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_SPEED = 46
        self.protocol_end = 0 # SCServo bit end(STS/SMS=0, SCS=1)
        self.initializing_pause = 3.5

        #[index, start_value, address, length, unit]
        self.servo_state_structure = {"Position":       [0, 0, SMS_STS_PRESENT_POSITION_L   , 2, [0, 4094, 0, 2*math.pi]], 
                                        "Velocity":     [1, 0, SMS_STS_PRESENT_SPEED_L      , 2, [0, 4094, 0, 2*math.pi]], 
                                        "Load":         [2, 0, SMS_STS_PRESENT_LOAD_L       , 2, 0.1], 
                                        "Voltage":      [3, 0, SMS_STS_PRESENT_VOLTAGE      , 1, 0.1], 
                                        "Current":      [4, 0, SMS_STS_PRESENT_CURRENT_L    , 2, 0.0065], 
                                        "Temperature":  [5, 0, SMS_STS_PRESENT_TEMPERATURE  , 1, 1], 
                                        "TorqueEnable": [6, 1, SMS_STS_TORQUE_ENABLE        , 1, 1],
                                        "MobileSign":   [7, 0, SMS_STS_MOVING               , 1, 1]
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
        self.servo_pres = self.ScanServos()

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
    
    def create_get_inf(self, state_type):
        if "Position" == state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: [self.calculate_int_to_rad(self.servoIDs[index], sts_pos) for index, sts_pos in enumerate(self.Data_Stack[state_type])]
        elif "Velocity" == state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.Data_Stack[state_type]
        elif "Load"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: np.array(self.Data_Stack[state_type])*0.1
        elif "Voltage"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda: np.array(self.Data_Stack[state_type])*0.1
        elif "Current"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: np.array(self.Data_Stack[state_type])*0.0065
        elif "Temperature"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.Data_Stack[state_type]
        elif "TorqueEnable"== state_type:
            self.add_dataType(state_type)
            #converts the 1 to True and 0 to False
            lambda_func = lambda state_type=state_type: np.array(self.Data_Stack[state_type])==1
        elif "MobileSign"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda: np.array(self.Data_Stack[state_type])==1
        elif "All"== state_type:
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.Data_Stack[state_type]
        return lambda_func
    
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
            lambda_list.append(self.create_get_inf(state_type))


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
            # print(f"ADR: {self.servo_state_structure[data_type][2]}, Lenght: {self.servo_state_structure[data_type][3]}")
            # groupSyncRead_obj = GroupSyncRead(ph=self.packetHandler, 
            #                                   start_address=self.servo_state_structure[data_type][2], 
            #                                   data_length=self.servo_state_structure[data_type][3])
            # #groupSyncRead_obj.clearParam()
            # for servoID in self.servoIDs:
            #     scs_addparam_result = groupSyncRead_obj.addParam(servoID)#false means that the scs_id was already added
            #     if scs_addparam_result != True:
            #         print("[ID:%03d] groupSyncRead addparam failed" % servoID) if self.showErrors else None
            # self.GroupSyncReads[data_type] = groupSyncRead_obj
            self.Data_Stack[data_type] = [0]*len(self.servoIDs)
        
    def remove_dataType(self, data_type):
        if data_type in self.upToDate_dataTypes:
            self.upToDate_dataTypes.remove(data_type)
            #remove the data_type from the GroupSyncReads
            # self.GroupSyncReads.pop(data_type)
            self.Data_Stack.pop(data_type)

    def update_states_serial(self):
        """
        Updates the states of the servos by reading data from each servo and storing it in the Data_Stack.
        This method performs the following steps:
        1. Checks if there are any data types to be read. If not, prints an error message and returns.
        2. Iterates over each data type and its corresponding GroupSyncRead object.
        3. Sends a packet and receives data using the GroupSyncRead object.
        4. Checks if the communication was successful. If not, prints an error message.
        5. For each servo ID, checks if the data is available. If not, prints an error message.
        6. If the data is available, retrieves the data and stores it in the Data_Stack.
        7. Clears the parameters of the GroupSyncRead object.
        Attributes:
            upToDate_dataTypes (list): List of data types to be read.
            GroupSyncReads (dict): Dictionary mapping data types to GroupSyncRead objects.
            servo_state_structure (dict): Dictionary containing servo state structure information.
            servoIDs (list): List of servo IDs.
            Data_Stack (dict): Dictionary to store the read data.
            showErrors (bool): Flag to indicate whether to show error messages.
        Returns:
            None
        """
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

    def update_singel_state(self, dataType, scs_id):
        if dataType == "Position":
            data, sts_comm_result, sts_error = self.packetHandler.ReadPos(scs_id)
        elif dataType == "Velocity":
            data, sts_comm_result, sts_error = self.packetHandler.ReadSpeed(scs_id)
        elif dataType == "Load":
            data, sts_comm_result, sts_error = self.packetHandler.ReadLoad(scs_id)
        elif dataType == "Voltage":
            data, sts_comm_result, sts_error = self.packetHandler.ReadVoltage(scs_id)
        elif dataType == "Current":
            data, sts_comm_result, sts_error = self.packetHandler.ReadCurrent(sts_id=scs_id)
        elif dataType == "Temperature":
            data, sts_comm_result, sts_error = self.packetHandler.ReadTemperature(sts_id=scs_id)
        elif dataType == "TorqueEnable":
            data, sts_comm_result, sts_error = self.packetHandler.ReadTorqueEnable(sts_id=scs_id)
        elif dataType == "MobileSign":
            data, sts_comm_result, sts_error = self.packetHandler.ReadMoving(sts_id=scs_id)
        elif dataType == "All":
            data, sts_comm_result, sts_error = self.packetHandler.ReadAll(sts_id=scs_id)
        else:
            print(f"Error: Unknown data type {dataType}")
        return data
    

    def update_states(self):
        for dataType, groupSync_obj in self.Data_Stack.items():
            for index, scs_id in enumerate(self.servoIDs):
                data = self.update_singel_state(dataType, scs_id)
                self.Data_Stack[dataType][index] = data
    
    def partial_update_states(self, time_for_update = 0.1):
        """This updates only a few data pices. Dependent on the time per update"""
        #check for possible problems
        if len(self.upToDate_dataTypes) == 0:
            print("Error: No data types to be read")
            return
        if len(self.servoIDs) == 0:
            print("Error: No servos to read from")
            return
        
        used_time_for_one_update = 0.016
        number_of_updates = int(time_for_update/used_time_for_one_update)
        for i in range(number_of_updates):
            data_type = self.upToDate_dataTypes[self.parti_Update_state%len(self.upToDate_dataTypes)]
            scs_id = self.servoIDs[self.parti_Update_state%len(self.servoIDs)]
            index = self.parti_Update_state%len(self.servoIDs)
            data = self.update_singel_state(data_type, scs_id)
            self.Data_Stack[data_type][index] = data
            self.parti_Update_state += 1

        if self.parti_Update_state >= len(self.servoIDs)*len(self.upToDate_dataTypes):
            self.parti_Update_state = 0
            
    def set_torque_state(self, scs_id, enable=True):
        """Disable the torque of the servo"""
        if enable:
            scs_comm_result = self.packetHandler.Write_Torque(scs_id, 1)
        else:
            scs_comm_result = self.packetHandler.Write_Torque(scs_id, 0)
        if scs_comm_result != COMM_SUCCESS:
            print(f"Error: Could not set the torque of servo {scs_id} to {enable}, Result: {scs_comm_result}")
        



    def _map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def send_command(self, new_positions, new_speeds):
        """
        The new position must be between 0 and 4094
        The new speed must be between 0 and 4094

        return: True if the new command is different from the previous command
        """
        #Checks if the new_positions and new_speeds are of the same length as the number of servos
        if len(new_positions) != len(self.servoIDs) or len(new_speeds) != len(self.servoIDs):
            print("Error: The number of positions and speeds should be equal to the number of servos")
            return 
        # #check if the commands are the same as the previous commands
        # if new_positions == self.current_actions["positions"] and new_speeds == self.current_actions["speeds"]:
        #     return False
        self.current_actions["positions"] = new_positions
        self.current_actions["speeds"] = new_speeds
        

        #map the new_positions to the range of 0 to 4094
        for index, scs_id in enumerate(self.servoIDs):
            position = self.calculate_rad_to_int(scs_id, new_positions[index])
            speed = new_speeds[index]
            print(f"ID: {scs_id}, Position: {position}, Speed: {speed}")
            scs_addparam_result = self.packetHandler.SyncWritePosEx(scs_id, position, speed, self.SCS_MOVING_ACC)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % scs_id) if self.showErrors else None

        scs_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.packetHandler.groupSyncWrite.clearParam()
        return True
    
    def calculate_rad_to_int(self, sts_id, new_position):
        """
        Calculate the servo positions from the new_position
        """
        if self.servos_info[sts_id]["orientation"] == 1:
            return int(np.clip(self._map(new_position, math.pi, -math.pi, *self.SCS_MOVABLE_RANGE), self.servos_info[sts_id]["min"], self.servos_info[sts_id]["max"]))
        else:
            return int(np.clip(self._map(new_position, -math.pi, math.pi, *self.SCS_MOVABLE_RANGE), self.servos_info[sts_id]["min"], self.servos_info[sts_id]["max"]))
    
    def calculate_int_to_rad(self, sts_id, new_position):
        """
        Calculate the radiant position from the servo position which is between 0 and 4094
        """
        if self.servos_info[sts_id]["orientation"] == 1:
            return self._map(new_position, *self.SCS_MOVABLE_RANGE, math.pi, -math.pi)
        else:
            return self._map(new_position, *self.SCS_MOVABLE_RANGE, -math.pi, math.pi)

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

        print(table)


    def run(self, send_pace=0.8):
        print("Starting thread ????")
        
        while self.no_close:
            while self.pause == False:
                
                if not self.action_queue.empty():
                    # Execute the actions in the queue
                    action = self.action_queue.get()
                    if action is not None:
                        # Execute the action
                        not self.send_command(action["positions"], action["speeds"])
                        print("Action executed")
                        self.action_queue.task_done()
                else:
                    # If no actions are available, update the states
                    start_time = time.time()
                    self.update_states()
                    print(f"Time: {time.time()-start_time}")
                   # self.partial_update_states(send_pace)
                
                #show the current states
                if self.showUpdates:
                    now_time = time.time()
                    print("\033c", end="")  # Clear the console
                    print(f"Time: {now_time-start_time}")
                    start_time = now_time
                    self.print_beautifullTable()
            time.sleep(0.1)

    def close(self):
        self.no_close = False
        self.join()
        self.portHandler.closePort()



if __name__ == "__main__":
    # Usage example
    supervisor = ServoSupervisor()
    getFunction = supervisor.create_get_informations(["All"])

    supervisor.set_torque_state(4, True)
    supervisor.set_torque_state(6, False)
    supervisor.start_run()
    

    for i in range(10):
        # time.sleep(1)
        # supervisor.action_queue.put({"positions": [0]*8, "speeds": [4000]*8})
        time.sleep(1)
        action_input = [0]*8
        supervisor.action_queue.put({"positions": action_input, "speeds": [4000]*8})
        print(action_input)
        print(getFunction())