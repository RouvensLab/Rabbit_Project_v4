import threading
from scservo_sdk_t import *
import time
import math
import queue
import numpy as np
from prettytable import PrettyTable

# STS Protocol Instructions
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ = 130  # 0x82

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
        self.servo_pres = {}
       
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

        self.action_queue = queue.Queue()
        self.current_actions = {"positions": [0]*8, "speeds": [0]*8, "torque": [1]*8}
        self.serial_lock = threading.Lock()
        self.feedback_queue = queue.Queue()

        self._init_servocontroler()



    def _init_servocontroler(self):
        self.SERIAL_PORT = 'COM3'
        self.BAUDRATE = 921600
        self.SCS_MOVING_ACC = 255
        self.SCS_MOVABLE_RANGE = (0, 4080)
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_SPEED = 46
        self.protocol_end = 0
        self.initializing_pause = 3.5


        self.servo_values = {}

        self.position = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.speed = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.load = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.voltage = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.temperature = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.asynchronous_write_flag = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.servo_status = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.mobile_sign = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        self.current = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0}
        
        self.allPinged = [False]*8

        self.servo_state_structure = {
            "Position": [0, 0, SMS_STS_PRESENT_POSITION_L, 2, [0, 4094, 0, 2*math.pi]], 
            "Velocity": [1, 0, SMS_STS_PRESENT_SPEED_L, 2, [0, 4094, 0, 2*math.pi]], 
            "Load": [2, 0, SMS_STS_PRESENT_LOAD_L, 2, 0.1], 
            "Voltage": [3, 0, SMS_STS_PRESENT_VOLTAGE, 1, 0.1], 
            "Current": [4, 0, SMS_STS_PRESENT_CURRENT_L, 2, 0.0065], 
            "Temperature": [5, 0, SMS_STS_PRESENT_TEMPERATURE, 1, 1], 
            "TorqueEnable": [6, 1, SMS_STS_TORQUE_ENABLE, 1, 1],
            "MobileSign": [7, 0, SMS_STS_MOVING, 1, 1]
        }

        self.portHandler = PortHandler(self.SERIAL_PORT)
        self.packetHandler = STS_PacketHandler(self.portHandler, self.protocol_end)

        try:
            with self.serial_lock:
                self.portHandler.setBaudRate(self.BAUDRATE)
                if not self.portHandler.openPort():
                    raise Exception("Failed to open port")
            time.sleep(self.initializing_pause)
            
        except Exception as e:
            print(f"Error initializing servo controller: {e}")
            self.portHandler = None

        # make first update

    #region RUN
    def run(self):
        #print("Starting servo supervisor thread")
        
        cycle_time = 0.07
        while self.no_close:
            acttime = time.time()
            

            # while not self.pause:
            if not self.action_queue.empty():
                action = self.action_queue.get()
                if action is not None:
                    if "type" in action:  # Check if it's an STS instruction
                        result = self.process_instruction(action)
                        #print(f"Instruction {action['type']} executed, result: {result}")
                    self.action_queue.task_done()
            else:
                #print(f"Update time: {time.time() - start_time}")
                if all(self.allPinged):
                    self.action_queue.put({"type": INST_SYNC_READ, "servo_id": self.servoIDs, "address": SMS_STS_PRESENT_POSITION_L, "length": 6})
            
            while not self.feedback_queue.empty():
                feedback = self.feedback_queue.get()
                # if isinstance(feedback.get("result"), dict) and len(feedback.get('result')) == 8:
                #     # print(f"Feedback: {feedback.get('result')[7]}")
                #     print(f"Feedbac2: {self.load[7]}")
            acttime2 = time.time()
            #print(f"Cycle time: {(acttime2 - acttime)*1000} ms")
            while time.time() - acttime < cycle_time:
                time.sleep(0.001)
        print("Servo supervisor thread terminated")


    def ScanServos(self):
        servo_pres = {}
        for servo_id in self.servoIDs:
            self.action_queue.put({"type": INST_PING, "servo_id": servo_id})
            servo_pres[servo_id] = True
        return servo_pres

    
    def start_run(self):
        self.pause = False
        if not self.is_alive():
            self.start()
            self.ScanServos()
            
            
            
           

    def stop_run(self):
        self.pause = True

    def continue_run(self):
        self.pause = False
    
    def close(self):
        self.no_close = False
        self.pause = True
        if self.is_alive():
            self.join()
        if self.portHandler:
            with self.serial_lock:
                self.portHandler.closePort()

    def _map_array(self, x: np.ndarray, in_min: float, in_max: float, out_min: float, out_max: float):
        in_min_array = np.full_like(x, in_min)
        out_min_array = np.full_like(x, out_min)
        epsilon = 1e-6
        return (x - in_min_array) * (out_max - out_min) / (in_max - in_min + epsilon) + out_min_array
    

    def absolute_to_pos_neg(self, array: np.ndarray, wechsel=1000):
        return np.where(array > wechsel, -(array-wechsel), array)
    
    def convert_to_order(self, list, order_index):
        """Convert the list to the order of the order_index"""
        if len(list) != len(order_index):
            raise ValueError("The list and the order_index have to have the same length")
        return [list[i-1] for i in order_index]
    
    def create_get_inf(self, state_type):
        if state_type == "Position":
            self.add_dataType(state_type)
            lambda_func = lambda: self.convert_to_order(np.array([self.calculate_int_to_rad(i, value) for i, value in self.position.items()]), [1, 2,   3, 5,   4, 6,   7, 8])
        elif state_type == "Velocity":
            self.add_dataType(state_type)
            lambda_func = lambda: self.convert_to_order(self.absolute_to_pos_neg(np.array(list(self.speed.values())), wechsel=32766)/4096*2*math.pi, [1, 2,   3, 5,   4, 6,   7, 8])
        elif state_type == "Load":
            self.add_dataType(state_type)
            lambda_func = lambda: self.convert_to_order(self.absolute_to_pos_neg(np.array(list(self.load.values())), wechsel=1024)*0.1, [1, 2,   3, 5,   4, 6,   7, 8])
        elif state_type == "Voltage":
            self.add_dataType(state_type)
            lambda_func = lambda: self.absolute_to_pos_neg(np.array(self.Data_Stack[state_type]))*0.1
        elif state_type == "Current":
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.absolute_to_pos_neg(np.array(self.Data_Stack[state_type]))*0.0065
        elif state_type == "Temperature":
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.absolute_to_pos_neg(self.Data_Stack[state_type])
        elif state_type == "TorqueEnable":
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: np.array(self.Data_Stack[state_type]) == 1
        elif state_type == "MobileSign":
            self.add_dataType(state_type)
            lambda_func = lambda: np.array(self.Data_Stack[state_type]) == 1
        elif state_type == "All":
            self.add_dataType(state_type)
            lambda_func = lambda state_type=state_type: self.Data_Stack[state_type]
        return lambda_func
    
    def create_get_informations(self, data_types):
        lambda_list = [self.create_get_inf(state_type) for state_type in data_types]
        def get_informations():
            return [func() for func in lambda_list]
        return get_informations
    
    def set_action_queue(self, action_queue: queue.Queue):
        self.action_queue = action_queue
    
    def get_ServoStates(self, dataType):
        if dataType in self.upToDate_dataTypes:
            return self.Data_Stack[dataType]
        return None

    def add_dataType(self, data_type):
        if data_type not in self.upToDate_dataTypes:
            self.upToDate_dataTypes.append(data_type)
            self.Data_Stack[data_type] = [0]*len(self.servoIDs)
        
    def remove_dataType(self, data_type):
        if data_type in self.upToDate_dataTypes:
            self.upToDate_dataTypes.remove(data_type)
            self.Data_Stack.pop(data_type)
            
    def setTorque(self, scs_id, enable=True):
        if enable:
            self.action_queue.put({"type": INST_WRITE, "servo_id": scs_id, "address": SMS_STS_TORQUE_ENABLE, "value": 1, "length": 1})
        else:
            self.action_queue.put({"type": INST_WRITE, "servo_id": scs_id, "address": SMS_STS_TORQUE_ENABLE, "value": 0, "length": 1})

    def _map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def send_command(self, new_positions, new_speeds):
        """
        new_positions: A list of new positions for the servos. The new positions must be in radians
        new_speeds: A list of new speeds for the servos. The new speeds must be in rad/s

        return: True if the new command is different from the previous command
        """
        if len(new_positions) != len(self.servoIDs) or len(new_speeds) != len(self.servoIDs):
            print("Error: Number of positions and speeds must match number of servos")
            return False
            
        if not self.packetHandler:
            print("Error: Packet handler not initialized")
            return False

       
        #self.packetHandler.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_STS_GOAL_POSITION, 4)
        #self.packetHandler.groupSyncWrite = GroupSyncWrite(self.packetHandler, self.ADDR_STS_GOAL_POSITION, 4)

        for index, scs_id in enumerate(self.servoIDs):
            if self.current_actions["torque"][index] == 1:
                position = self.calculate_rad_to_int(scs_id, new_positions[index])
                speed = int(abs(new_speeds[index]))
                # param = [SCS_LOBYTE(position), SCS_HIBYTE(position), 
                #         SCS_LOBYTE(speed), SCS_HIBYTE(speed)]
                #scs_addparam_result = self.packetHandler.groupSyncWrite.addParam(scs_id, param)
                scs_addparam_result = self.packetHandler.SyncWritePosEx(scs_id, position, speed, self.SCS_MOVING_ACC)
                if not scs_addparam_result:
                    print(f"[ID:{scs_id:03d}] groupSyncWrite addparam failed") if self.showErrors else None

        scs_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"Sync write failed: {self.packetHandler.getTxRxResult(scs_comm_result)}")
            return False

        self.packetHandler.groupSyncWrite.clearParam()
        
        self.current_actions["positions"] = new_positions.copy()
        self.current_actions["speeds"] = new_speeds.copy()
        return True
    
    def calculate_rad_to_int(self, sts_id, new_position):
        if self.servos_info[sts_id]["orientation"] == 1:
            return int(np.clip(self._map(new_position, math.pi, -math.pi, *self.SCS_MOVABLE_RANGE), 
                              self.servos_info[sts_id]["min"], self.servos_info[sts_id]["max"]))
        else:
            return int(np.clip(self._map(new_position, -math.pi, math.pi, *self.SCS_MOVABLE_RANGE), 
                              self.servos_info[sts_id]["min"], self.servos_info[sts_id]["max"]))
    
    def calculate_int_to_rad(self, sts_id, new_position):
        if self.servos_info[sts_id]["orientation"] == 1:
            return self._map(new_position, *self.SCS_MOVABLE_RANGE, math.pi, -math.pi)
        else:
            return self._map(new_position, *self.SCS_MOVABLE_RANGE, -math.pi, math.pi)

    def print_beautifullTable(self):
        if len(self.upToDate_dataTypes) == 0:
            print("Error: No data types to be read")
            return
        table = PrettyTable()
        table.field_names = ["Servo ID"] + self.upToDate_dataTypes
        for index, scs_id in enumerate(self.servoIDs):
            table.add_row([scs_id] + [self.Data_Stack[dataType][index] for dataType in self.upToDate_dataTypes])
        print(table)

    def process_instruction(self, instruction):
        """Process a single STS protocol instruction from the queue."""
        if not self.packetHandler:
            print("Error: Packet handler not initialized")
            return False

        inst_type = instruction.get("type")
        if inst_type == INST_PING:
            scs_id = instruction.get("servo_id")
            _, result, error = self.packetHandler.ping(scs_id)
            if result == COMM_SUCCESS:
                self.feedback_queue.put({"type": INST_PING, "instruction": instruction, "result": True})
                self.servo_pres[scs_id] = True

                self.allPinged[scs_id-1] = True
                return True
            else:
                self.feedback_queue.put({"type": INST_PING, "instruction": instruction, "result": False})
                self.servo_pres[scs_id] = False

                self.allPinged[scs_id-1] = True
                return False

        elif inst_type == INST_READ:
            scs_id = instruction.get("servo_id")
            addr = instruction.get("address")
            length = instruction.get("length", 1)
            with self.serial_lock:
                if length == 1:
                    data, result, error = self.packetHandler.read1ByteTxRx(scs_id, addr)
                elif length == 2:
                    data, result, error = self.packetHandler.read2ByteTxRx(scs_id, addr)
                else:
                    print(f"Unsupported read length: {length}")
                    return False
            if result == COMM_SUCCESS:
                self.feedback_queue.put({"type": INST_READ, "instruction": instruction, "result": data})
                return data
            else:
                self.feedback_queue.put({"type": INST_READ, "instruction": instruction, "result": None})
                return None

        elif inst_type == INST_WRITE:
            scs_id = instruction.get("servo_id")
            addr = instruction.get("address")
            value = instruction.get("value")
            length = instruction.get("length", 1)
            with self.serial_lock:
                if length == 1:
                    result, error = self.packetHandler.write1ByteTxRx(scs_id, addr, value)
                elif length == 2:
                    result, error = self.packetHandler.write2ByteTxRx(scs_id, addr, value)
                else:
                    print(f"Unsupported write length: {length}")
                    return False
            if result == COMM_SUCCESS:
                self.feedback_queue.put({"type": INST_WRITE, "instruction": instruction, "result": True})
                return True
            else:
                self.feedback_queue.put({"type": INST_WRITE, "instruction": instruction, "result": False})
                return False

        elif inst_type == INST_REG_WRITE:
            # Similar to INST_WRITE but queues the write for later execution with INST_ACTION
            scs_id = instruction.get("servo_id")
            addr = instruction.get("address")
            value = instruction.get("value")
            length = instruction.get("length", 1)
            with self.serial_lock:
                if length == 1:
                    result, error = self.packetHandler.regWriteTxRx(scs_id, addr, length, [value])
                elif length == 2:
                    result, error = self.packetHandler.regWriteTxRx(scs_id, addr, length, 
                                                                   [SCS_LOBYTE(value), SCS_HIBYTE(value)])
                else:
                    print(f"Unsupported reg write length: {length}")
                    return False
            if result == COMM_SUCCESS:
                self.feedback_queue.put({"type": INST_REG_WRITE, "instruction": instruction, "result": True})
                return True
            else:
                self.feedback_queue.put({"type": INST_REG_WRITE, "instruction": instruction, "result": False})
                return False

        elif inst_type == INST_ACTION:
            scs_id = instruction.get("servo_id")
            if scs_id is None:
                self.feedback_queue.put({"type": INST_ACTION, "instruction": instruction, "result": False})
                return False
            with self.serial_lock:
                result = self.packetHandler.action(scs_id)
            if result == COMM_SUCCESS:
                self.feedback_queue.put({"type": INST_ACTION, "instruction": instruction, "result": True})
                return True
            else:
                self.feedback_queue.put({"type": INST_ACTION, "instruction": instruction, "result": False})
                return False

        elif inst_type == INST_SYNC_WRITE:
            positions = instruction.get("positions")
            speeds = instruction.get("speeds")
            result = self.send_command(positions, speeds)
            self.feedback_queue.put({"type": INST_SYNC_WRITE, "instruction": instruction, "result": result})
            return result

        elif inst_type == INST_SYNC_READ:
            scs_ids = instruction.get("servo_id")
            addr = instruction.get("address")
            length = instruction.get("length", 1)
            
            if not self.packetHandler:
                print("Error: Packet handler not initialized")
                self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result": False})
                return False

            # Initialize GroupSyncRead
            groupSyncRead = GroupSyncRead(self.packetHandler, addr, length)
            
            # Add parameter storage for each servo
            for scs_id in scs_ids:
                if not groupSyncRead.addParam(scs_id):
                    print(f"Failed to add parameter storage for servo ID {scs_id}")
                    self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result": False})
                    return False

            # Perform sync read
            result = groupSyncRead.txRxPacket()
            if result != COMM_SUCCESS:
                print(f"Sync read failed: {self.packetHandler.getTxRxResult(result)}")
                self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result": False})
                return False

            # Check if data is available for each servo
            data = {}
            for scs_id in scs_ids:
                if not groupSyncRead.isAvailable(scs_id, addr, length):
                    print(f"Data not available for servo ID {scs_id}")
                    #self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result": False})
                    return False
                # Ensure data is not None before accessing
                if groupSyncRead.data_dict[scs_id] is None:
                    print(f"No data for servo ID {scs_id}")
                    #self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result": False})
                    return False
                # Get data for each servo
                servo_data = []
                #for i in range(length):
                #    servo_data.append(groupSyncRead.getData(scs_id, addr + i, 1))
                

                #Reg.   Description	            Length
                #0x2A	Current position   	    2
                #0x2C	Current speed	        2
                #0x2E	Current load	        2
                #0x30	Current voltage	        1
                #0x31	Current temperature	    1
                #0x32	Asynchronous write flag	1
                #0x33	Servo status	        1
                #0x34	Mobile sign	            1
                #0x35	Current current	        2

                self.position[scs_id] = groupSyncRead.getData(scs_id, addr, 2)
                self.speed[scs_id] = groupSyncRead.getData(scs_id, addr + 2, 2)
                self.load[scs_id] = groupSyncRead.getData(scs_id, addr + 4, 2)
                
                if len(servo_data) > 6:
                    self.voltage[scs_id] = groupSyncRead.getData(scs_id, addr + 6, 1)
                    self.temperature[scs_id] = groupSyncRead.getData(scs_id, addr + 7, 1)
                    self.asynchronous_write_flag[scs_id] = groupSyncRead.getData(scs_id, addr + 8, 1)
                    self.servo_status[scs_id] = groupSyncRead.getData(scs_id, addr + 9, 1)
                    self.mobile_sign[scs_id] = groupSyncRead.getData(scs_id, addr + 10, 1)
                    self.current[scs_id] = groupSyncRead.getData(scs_id, addr + 11, 2)

                else:
                    self.voltage[scs_id] = None
                    self.temperature[scs_id] = None
                    self.asynchronous_write_flag[scs_id] = None
                    self.servo_status[scs_id] = None
                    self.mobile_sign[scs_id] = None
                    self.current[scs_id] = None
                
                self.servo_values[scs_id] = self.position[scs_id], self.speed[scs_id], self.load[scs_id], self.voltage[scs_id], self.temperature[scs_id], self.asynchronous_write_flag[scs_id], self.servo_status[scs_id], self.mobile_sign[scs_id], self.current[scs_id]
                #data[scs_id] = servo_data
                # process data[scs_id] to position speed load voltage temperature
                #print(f"Servo ID {scs_id}: Position: {self.position[scs_id]}, Speed: {self.speed[scs_id]}, Load: {self.load[scs_id]}, Voltage: {self.voltage[scs_id]}, Temperature: {self.temperature[scs_id]}")
            
           
            self.feedback_queue.put({"type": INST_SYNC_READ, "instruction": instruction, "result":self.servo_values })
            return data

        else:
            self.feedback_queue.put({"type": "UNKNOWN", "instruction": instruction, "result": False})
            return False

    

    #Basic functions
    def update_servoStates(self):
        self.action_queue.put({"type": INST_SYNC_READ, "servo_id": self.servoIDs, "address": SMS_STS_PRESENT_POSITION_L, "length": 6})

    def set_PositionSpeed(self, positions, speeds):
        self.action_queue.put({"type": INST_SYNC_WRITE, "positions": positions, "speeds": speeds})
     

if __name__ == "__main__":
    supervisor = ServoSupervisor()
    #getFunction = supervisor.create_get_informations(["Position"])
    supervisor.start_run()
    

    # Example STS instructions to queue
    #supervisor.action_queue.put({"type": INST_PING, "servo_id": 1})
    #supervisor.action_queue.put({"type": INST_READ, "servo_id": 1, "address": SMS_STS_PRESENT_POSITION_L, "length": 2})
    for servo_id in supervisor.servoIDs:
       supervisor.setTorque(servo_id, enable=False)

    
    #supervisor.action_queue.put({"type": INST_REG_WRITE, "servo_id": 1, "address": SMS_STS_GOAL_POSITION_L, "value": 2048, "length": 2})
    #supervisor.action_queue.put({"type": INST_ACTION})
    #supervisor.action_queue.put({"type": INST_SYNC_WRITE, "positions": [0]*8, "speeds": [4000]*8})
    
    try:
        cycle_time = 1

        while True:
            acttime = time.time()   
           
            while time.time() - acttime < cycle_time:
                time.sleep(0.01)
           


    except KeyboardInterrupt:
        supervisor.close()