from scservo_sdk_t import *

import os
import time
import sys
import math
import threading
import numpy as np
from prettytable import PrettyTable

#from PySide6.QtWidgets import QApplication

class ServoControler:
    def __init__(self):
        # some global defined variables. Not changable-------------------
        self.SERIAL_PORT = 'COM3'
        self.BAUDRATE = 115200                #115200
        self.SCS1_ID = 1
        self.SCS2_ID = 5
        self.SCS_MOVING_ACC = 255
        self.SCS_MOVABLE_RANGE = (0, 4094)
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_SPEED = 46
        self.protocol_end = 0
        self.initializing_pause = 3.5

                #-------EPROM(read & write)--------These values tell what should be changed. EveryServo information has a different address.
        self.SMS_STS_ID = 5
        self.SMS_STS_BAUD_RATE = 6
        self.SMS_STS_MIN_ANGLE_LIMIT_L = 9
        self.SMS_STS_MIN_ANGLE_LIMIT_H = 10
        self.SMS_STS_MAX_ANGLE_LIMIT_L = 11
        self.SMS_STS_MAX_ANGLE_LIMIT_H = 12
        self.SMS_STS_CW_DEAD = 26
        self.SMS_STS_CCW_DEAD = 27
        self.SMS_STS_OFS_L = 31
        self.SMS_STS_OFS_H = 32
        self.SMS_STS_MODE = 33

        self.SMS_STS_PROTECTIVE_TORQUE = 34
        self.SMS_STS_PROTECTIVE_TIME = 35
        self.SMS_STS_OVERLOAD_TORQUE = 36


        # Servo protocol details
        self.ADDR_SCS_TORQUE_ENABLE = 40
        self.ADDR_STS_GOAL_ACC = 41
        self.ADDR_STS_GOAL_POSITION = 42
        self.ADDR_STS_GOAL_SPEED = 46
        self.ADDR_SCS_PRESENT_POSITION = 56
        self.SCS_MINIMUM_POSITION_VALUE = 100
        self.SCS_MAXIMUM_POSITION_VALUE = 4000
        self.SCS_MOVING_STATUS_THRESHOLD = 20
        self.SCS_MOVING_ACC = 0
        self.protocol_end = 0            # SCServo bit end(STS/SMS=0, SCS=1)
        self.scs_goal_position = [self.SCS_MINIMUM_POSITION_VALUE, self.SCS_MAXIMUM_POSITION_VALUE]




        # Servo properties
        self.servos_info_limitation = {
            1: {"min": 0, "max": 360, "orientation": 1},
            2: {"min": 0, "max": 360, "orientation": 0},
            3: {"min": 0, "max": 360, "orientation": 0},
            4: {"min": 0, "max": 360, "orientation": 1},
            5: {"min": 0, "max": 360, "orientation": 0},
            6: {"min": 0, "max": 360, "orientation": 1},
            7: {"min": 0, "max": 360, "orientation": 0},
            8: {"min": 0, "max": 360, "orientation": 1},
        }
        self.num_servos = int(len(self.servos_info_limitation))
        print("Num_Servos: ", self.num_servos)
        #[index, start_value, address, length, unit]
        self.servo_state_structure = {"Position": [0, 0, SMS_STS_PRESENT_POSITION_L, 2, [0, 4094, 0, 2*math.pi]],
                                        "Velocity": [1, 0, SMS_STS_PRESENT_SPEED_L, 2, [0, 4094, 0, 2*math.pi]],
                                        "Load": [2, 0, SMS_STS_PRESENT_LOAD_L, 2, 0.1],
                                        "Voltage": [3, 0, SMS_STS_PRESENT_VOLTAGE, 1, 0.1],
                                        "Current": [4, 0, SMS_STS_PRESENT_CURRENT_L, 2, 0.0065],
                                        "Temperature": [5, 0, SMS_STS_PRESENT_TEMPERATURE, 1, 1],
                                        "torque_enable": [6, 1, SMS_STS_TORQUE_ENABLE, 1, 1],
                                        "mobile_sign": [7, 0, SMS_STS_MOVING, 1, 1]
                                        }





        # Changable variables with important information-----------------
        self.start_time = time.time()

        class ServoStack:
            def __init__(self):
                #units are in rad
                #[index, start_value, address, length, unit]
                self.servo_state_structure = {"Position": [0, 0, SMS_STS_PRESENT_POSITION_L, 2, [0, 4094, 0, 2*math.pi]],
                                              "Velocity": [1, 0, SMS_STS_PRESENT_SPEED_L, 2, [0, 4094, 0, 2*math.pi]],
                                              "Load": [2, 0, SMS_STS_PRESENT_LOAD_L, 2, 0.1],
                                              "Voltage": [3, 0, SMS_STS_PRESENT_VOLTAGE, 1, 0.1],
                                              "Current": [4, 0, SMS_STS_PRESENT_CURRENT_L, 2, 0.0065],
                                              "Temperature": [5, 0, SMS_STS_PRESENT_TEMPERATURE, 1, 1],
                                              "torque_enable": [6, 1, SMS_STS_TORQUE_ENABLE, 1, 1],
                                              "mobile_sign": [7, 0, SMS_STS_MOVING, 1, 1]
                                              }
                self.data_types = list(self.servo_state_structure.keys())

                self.servo_state_stack = {}

                self.servo_action_structure = {"angle": [0, 0], "speed": [1, 0], "acc": [2, 0]}
                self.servo_action_stack = {}

            def add_servo(self, servo_id):
                start_states = [i[1] for i in self.servo_state_structure.values()]
                self.servo_state_stack[servo_id] = start_states


                self.servo_action_stack[servo_id] = [i[1] for i in self.servo_action_structure.values()]


            def remove_servo(self, servo_id):
                return self.servo_state_stack.pop(servo_id)

            def update_state(self, servo_id, data_type, state):
                if data_type in self.data_types:
                    data_type_index = self.servo_state_structure[data_type][0]
                    self.servo_state_stack[servo_id][data_type_index] = state


            def update_action(self, servo_id, action_type, action):
                if action_type in self.servo_action_structure.keys():
                    data_type_index = self.servo_action_structure[action_type][0]
                    self.servo_action_stack[servo_id][data_type_index] = action


            def get_servo_states(self, servo_id):
                return self.servo_state_stack[servo_id]

            def get_state(self, servo_id, data_type, origin = False):
                """returns the for every servo the desired data type"""
                if data_type in self.data_types:
                    data_type_index = self.servo_state_structure[data_type][0]
                    if origin:
                        return self.servo_state_stack[servo_id][data_type_index]
                    else:
                        if isinstance(self.servo_state_structure[data_type][4], list):
                            #map the value to the right range
                            return self.map(self.servo_state_stack[servo_id][data_type_index], *self.servo_state_structure[data_type][4])
                        else:
                            return self.servo_state_stack[servo_id][data_type_index] * self.servo_state_structure[data_type][4]
                else:
                    print("Data type not found")
                    return None

            def get_servo_actions(self, servo_id):
                return self.servo_action_stack[servo_id]

            def get_action(self, servo_id, action_type):
                """returns the for every servo the desired data type"""
                if action_type in self.servo_action_structure.keys():
                    return self.servo_action_stack[servo_id][action_type]
                else:
                    print("Action type not found")
                    return None

            def map(self, x, in_min, in_max, out_min, out_max):
                return round((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, 5)


        self.servoStack = ServoStack()
        for servo_id in self.servos_info_limitation.keys():
            self.servoStack.add_servo(servo_id)



        # Initialize PortHandler and PacketHandler instances
        self.portHandler = PortHandler(self.SERIAL_PORT)
        self.packetHandler = SMS_STS_PacketHandler(self.portHandler, self.protocol_end)



        # connect to the port
        self.portHandler.openPort()
        self.portHandler.setBaudRate(self.BAUDRATE)
        time.sleep(self.initializing_pause)

    def _map(self, x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)





    def get_Sync_ServosInfo(self):
        """
        Retrieve and update the state information of all servos.
        This method iterates through the servo state structure, initializes a GroupSyncRead instance,
        adds parameters for each servo, sends a read request, and updates the servo state with the received data.
        Returns:
            dict: Updated servo state information.
        """
        '''
        Get all the information of the servos
        '''

        for data_type, values in self.servoStack.servo_state_structure.items():
            # Initialize GroupSyncRead instance
            self.groupSyncRead = GroupSyncRead(self.packetHandler, values[2], values[3])
            # Add parameter storage for SC Servo#1~10 present position value
            for scs_id in self.servos_info_limitation.keys():
                scs_addparam_result = self.groupSyncRead.addParam(scs_id)#false means that the scs_id was already added
                if scs_addparam_result != True:
                    #print("[ID:%03d] groupSyncRead addparam failed" % scs_id)
                    pass

            scs_comm_result = self.groupSyncRead.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                #print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                pass

            #print(self.groupSyncRead.data_dict)#Here is all the data that was received

            for scs_id in self.servos_info_limitation.keys():
                # Check if groupsyncread data of SC Servo#1~10 is available
                #Check if the data is available
                #print(scs_id, data_type, values[2])
                scs_data_result, scs_error = self.groupSyncRead.isAvailable(scs_id, values[2], values[3])
                if scs_data_result == True:
                    self.servoStack.update_state(scs_id, data_type, self.groupSyncRead.getData(scs_id, values[2], values[3]))
                else:
                    #No Update,
                    #print("[ID:%03d] groupSyncRead getdata failed" % scs_id)
                    pass

            # Clear syncread parameter storage
            self.groupSyncRead.clearParam()
        return self.servoStack.servo_state_stack


    def print_servo_data_table(self,servo_data):
        """
        Print the servo data in a table format with parameters as column headers and servo IDs as rows
        """
        # Define the column headers
        headers = ["Servo ID", "Position", "Speed", "Load", "Voltage", "Current", "Temperature", "TorqueEnable"]

        # Create the table
        table = PrettyTable()
        table.field_names = headers

        # Add rows to the table
        for servo_id, data in servo_data.items():
            row = [
                servo_id,
                data.get("Position", "N/A"),
                data.get("Speed", "N/A"),
                data.get("Load", "N/A"),
                data.get("Voltage", "N/A"),
                data.get("Current", "N/A"),
                data.get("Temperature", "N/A"),
                data.get("TorqueEnable", "N/A")
            ]
            table.add_row(row)

        # Print the table
        print(table)

    def read_servo_data(self, servo_id, address, length):
        if length == 1:
            data, scs_comm_result, scs_error = self.packetHandler.read1ByteTxRx(servo_id, address)
        elif length == 2:
            data, scs_comm_result, scs_error = self.packetHandler.read2ByteTxRx(servo_id, address)
        elif length == 4:
            data, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(servo_id, address)
        else:
            raise ValueError("Invalid length specified")

        # Check if the returned values are valid
        # if scs_comm_result != 0 or scs_error != 0:
        #     raise Exception(f"Communication error: {scs_comm_result}, Error: {scs_error}")
        return data

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
        for servo_id in self.servos_info_limitation.keys():
            model_number ,result, error = self.packetHandler.ping(servo_id)
            print(f"Servo ID: {servo_id}: Model Number {model_number}, Result: {result}, Error: {error}")
            if result == COMM_SUCCESS:
                print(f"Servo ID: {servo_id}: Model Number {model_number}")
                servo_pres[servo_id] = True
            else:
                print(f"Servo ID: {servo_id} Not present")
                servo_pres[servo_id] = None

        return servo_pres


    def setSingleServoPosSpeedAcc(self, Servo_ID, angle, speed, safety_check = False):
        '''
        Set the position of a single servo
        '''
        if safety_check:
            #check if the angle is out of the limitation
            angle = self.safety_check(Servo_ID, angle)

        if self.servos_info_limitation[Servo_ID]["orientation"] == 1:
            servo_pos = int(self._map(angle, 180, -180, *self.SCS_MOVABLE_RANGE))
        else:
            servo_pos = int(self._map(angle, -180, 180, *self.SCS_MOVABLE_RANGE))


        #print(f"Servo ID: {Servo_ID} Angle: {angle}, Servo Pos: {servo_pos}")

        #actualize the global servos_actions dict with the new information
        self.servoStack.update_action(Servo_ID, "angle", angle)
        self.servoStack.update_action(Servo_ID, "speed", speed)
        self.servoStack.update_action(Servo_ID, "acc", self.SCS_MOVING_ACC)

        # Add SC Servo#1~10 goal position\moving speed\moving accc value to the Syncwrite parameter storage
        scs_addparam_result = self.packetHandler.SyncWritePosEx(Servo_ID, servo_pos, speed, self.SCS_MOVING_ACC)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % Servo_ID)



        #make safety_limitation
        #self.setSafetyLimitation(Servo_ID)

    def run_sync_write_commands(self):
        '''
        Run the sync write commands
        '''
        # Syncwrite goal position
        scs_comm_result = self.packetHandler.groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.packetHandler.groupSyncWrite.clearParam()
        #time.sleep(0.002) #wait for servo status moving=1


    def safety_check(self, Servo_ID, angle):
        '''
        Check if the servos are in the right position range
        '''
        if len(list(self.pres_servos_actions.keys())) == 8:

            #check if servo 3 and 5 don't collide
            max_angle_dif = -25
            min_angle_dif = -50

            if Servo_ID == 3:
                angle_dif = angle - self.servoStack.get_action(5, "angle")
                if angle_dif > max_angle_dif:
                    return self.servoStack.get_action(5, "angle") + max_angle_dif
                elif angle_dif < min_angle_dif:
                    return self.servoStack.get_action(5, "angle")+ min_angle_dif
                else:
                    return angle
            elif Servo_ID == 5:
                angle_dif = self.servoStack.get_action(3, "angle") - angle
                if angle_dif > max_angle_dif:
                    return self.servoStack.get_action(3, "angle") - max_angle_dif
                elif angle_dif < min_angle_dif:
                    return self.servoStack.get_action(3, "angle") - min_angle_dif
                else:
                    return angle

            #check if servo 4 and 6 don't collide
            elif Servo_ID == 4:
                angle_dif = angle - self.servoStack.get_action(6, "angle")
                #print(angle_dif)
                if angle_dif > max_angle_dif:
                    return self.servoStack.get_action(6, "angle") + max_angle_dif
                elif angle_dif < min_angle_dif:
                    return self.servoStack.get_action(6, "angle") + min_angle_dif
                else:
                    return angle

            elif Servo_ID == 6:
                angle_dif = self.servoStack.get_action(4, "angle") - angle
                if angle_dif > max_angle_dif:
                    return  self.servoStack.get_action(4, "angle") - max_angle_dif
                elif angle_dif < min_angle_dif:
                    return  self.servoStack.get_action(4, "angle") - min_angle_dif
                else:
                    return angle

            else:
                return angle
        else:
            return angle


    def setGroupSync_ServoPosSpeedAcc(self, Servo_IDs:list, angles:list, speed = 4000, accs = 255):
        '''
        Set the position of the servos synchronical

        accs: is not used in this version
        '''
        for i in range(len(Servo_IDs)):
            self.servoStack.update_action(Servo_IDs[i], "angle", angles[i])
            self.servoStack.update_action(Servo_IDs[i], "speed", speed)
            self.servoStack.update_action(Servo_IDs[i], "acc", accs)

        for i in range(len(Servo_IDs)):
            self.setSingleServoPosSpeedAcc(Servo_IDs[i], angles[i], speed)
        #run the syncwrite commands
        self.run_sync_write_commands()

    def torque_state(self, Servo_ID, torque_state):
        """
        Writes the torque enable value to the specified servo.

        Args:
            scs_id (int): The ID of the servo.
            torque_enable (int): The torque enable value to be written.

        Returns:
            int: The result of the write operation.
        """
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, Servo_ID, self.ADDR_SCS_TORQUE_ENABLE, torque_state)

    def WriteOFS(self, scs_id, ofs):
        """
        Writes the offset value to the specified servo motor.

        Parameters:
        - scs_id (int): The ID of the servo motor.
        - ofs (int): The offset value to be written.

        Returns:
        - int: The result of the write operation.

        """
        self.LockEprom(scs_id)
        self.packetHandler.write2ByteTxRx(self.portHandler, scs_id, SMS_STS_OFS_L, ofs)
        self.unLockEprom(scs_id)

    def LockEprom(self, scs_id):
        return self.packetHandler.write1ByteTxRx(self.portHandler, scs_id, SMS_STS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.packetHandler.write1ByteTxRx(self.portHandler, scs_id, SMS_STS_LOCK, 0)

    def WriteMiddlePos(self, scs_id, angle, type_degree = True):
        '''
        Set a given old angle to the new middle position. So 0 degree is the new middle position.

        scs_id: the id of the servo
        angle: the old angle in degree
        '''
        if type_degree:
            angle = int(self._map(angle, 0, 360, *self.SCS_MOVABLE_RANGE))
        return self.WriteOFS(scs_id, angle)

    def defineInitialPosition(self, Servo_ID):
        '''
            When calling this function, the servo will disable the torque, so you can move the servo to the initial position.
            After that, you can call the returned function to set the new middle position.
        '''
        def findInitialPosition():

            #set the default Middle position = 0
            self.WriteMiddlePos(Servo_ID, 0, type_degree=False)
            #read the current position of the servo
            middlepos = self.read_servo_data(Servo_ID, SMS_STS_PRESENT_POSITION_L, 2)
            print(middlepos)
            #check servos direction
            if self.servos_info_limitation[Servo_ID]["orientation"] == 1:
                middlepos = 2096 - middlepos

            #set the new middle position
            self.WriteMiddlePos(Servo_ID, middlepos, type_degree=False)
            time.sleep(0.5)
            #enable the torque
            self.torque_state(Servo_ID, True)

        self.torque_state(Servo_ID, False)
        return findInitialPosition

    def defineAllMiddlePositions(self):
        '''
        Set the middle position for all servos
        '''
        for Servo_ID in self.servos_info_limitation.keys():
            self.WriteMiddlePos(Servo_ID, 0, type_degree=False)

    def write_servo_data(self, servo_id, address, length, value):
        if length == 1:
            scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(self.portHandler, servo_id, address, value)
        elif length == 2:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, servo_id, address, value)
        elif length == 4:
            scs_comm_result, scs_error = self.packetHandler.write4ByteTxRx(self.portHandler, servo_id, address, value)
        else:
            raise ValueError("Invalid length specified")

        # Check if the returned values are valid
        # if scs_comm_result != 0 or scs_error != 0:
        #     raise Exception(f"Communication error: {scs_comm_result}, Error: {scs_error}")
        return scs_comm_result, scs_error

    def setProtectiveTorque(self, Servo_ID, torque):
        '''
        Set the protective torque: Dont work!!!!!!!!!!!!!!!!
        '''
        #self.packetHandler.write2ByteTxRx(self.portHandler, Servo_ID, 34, torque)
        print("Protective torque", self.read_servo_data(Servo_ID, self.SMS_STS_PROTECTIVE_TORQUE, 1))
        print("Protective time", self.read_servo_data(Servo_ID, self.SMS_STS_PROTECTIVE_TIME, 1))
        print("Overload torque", self.read_servo_data(Servo_ID, self.SMS_STS_OVERLOAD_TORQUE, 1))

        #write the torque
        self.write_servo_data(Servo_ID, self.SMS_STS_PROTECTIVE_TORQUE, 1, torque)







if __name__=="__main__":
    if os.name == 'nt':
        import msvcrt
        print('nt')
        def getch():
            return msvcrt.getch().decode()

    Controler = ServoControler()
    Controler.ScanServos()
    #Controler.setSingleServoPosSpeedAcc(7, 180, 4000)
    Controler.setSingleServoPosSpeedAcc(3, 180, 4000)
    Controler.run_sync_write_commands()

    print(Controler.read_servo_data(3, SMS_STS_PRESENT_POSITION_L, 2))
    time.sleep(0.5)

    # #move servo 7
    # while True:
    #     servo_data = Controler.get_Sync_ServosInfo()
    #     formatted_data = {}
    #     for servo_id, data in servo_data.items():
    #         formatted_data[servo_id] = {
    #         "Position": round(Controler.servoStack.get_state(servo_id, "Pos"),3),
    #         "Speed": round(Controler.servoStack.get_state(servo_id, "Ppd"),3),
    #         "Load": round(Controler.servoStack.get_state(servo_id, "Load"),3),
    #         "Voltage": round(Controler.servoStack.get_state(servo_id, "Voltage"), 2),
    #         "Current": round(Controler.servoStack.get_state(servo_id, "Current"),3),
    #         "Temperature": Controler.servoStack.get_state(servo_id, "Temperature"),
    #         "TorqueEnable": round(Controler.servoStack.get_state(servo_id, "torque_enable"),3),
    #         "MobileSign": Controler.servoStack.get_state(servo_id, "mobile_sign")
    #         }

    #     # Update the table with new data
    #     table = PrettyTable()
    #     table.field_names = ["Servo ID", "Position", "Speed", "Load", "Voltage", "Current", "Temperature", "TorqueEnable", "MobileSign"]

    #     for servo_id, data in formatted_data.items():
    #         table.add_row([
    #             servo_id,
    #             data["Position"],
    #             data["Speed"],
    #             data["Load"],
    #             data["Voltage"],
    #             data["Current"],
    #             data["Temperature"],
    #             data["TorqueEnable"],
    #             data["MobileSign"]
    #         ])

        # print("\033c", end="")  # Clear the console
        # print(table)
        # time.sleep(0.01)

        # time.sleep(0.01)
    # Controler.setSingleServoPosSpeedAcc(7, 180, 4000)
    # Controler.run_sync_write_commands()




    # while True:

    #     #Controler.setProtectiveTorque(4, 10)
    #     #Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,20,20,0,0,0,0], 4000, 255)
    #     Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [-50,-50,-50,-50,-100,-100,-25,-25], 4000, 255)
    #     # #Controler.print_servo_data_table(Controler.syncRead_2([1,2,3,4,5,6,7,8]))
    #     time.sleep(2)
    #     Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,0,0,0,0,0,0], 4000, 255)
    #     #print(Controler.syncRead_2([1,2,3,4,5,6,7,8]))
    #     time.sleep(2)
    #     # Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [50,50,50,50,50,50,50,50], 4000, 255)

    #     #Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0, 0, 0, -66.75000998629827, -66.75000998629827, 0, 0, -25.000010714391994, -25.000010714391994], 4000, 255)



    #     if os.name == 'nt' and msvcrt.kbhit():
    #         key = getch()
    #         if key.upper() == 'Q':
    #             # servo 4,3
    #             print("Terminating loop and turn of torque control.")
    #             for i in range(1, 9):
    #                 Controler.torque_state(i, False)
    #             break


    # time.sleep(2)
    # Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,90,90,0,0,0,0], 4000, 255)

    # time.sleep(2)
    # Controler.syncRead([1,2,3,4,5,6,7,8])


    # print(Controler.actualize_servo_infos())

    # Controler.torque_state(4, False)
    # print("Torque off")
    # time.sleep(1)
    # Controler.torque_state(4, True)

    #read the current position of the servo
    # print(Controler.read_servo_data(4, SMS_STS_OFS_L, 2))
    # middlepos = Controler.read_servo_data(4, SMS_STS_PRESENT_POSITION_L, 2)
    # print(middlepos)
    # #set the new middle position
    # Controler.WriteMiddlePos(4, 0, type_degree=False)

    # print(Controler.read_servo_data(4, SMS_STS_OFS_L, 2))
    # print(Controler.read_servo_data(4, SMS_STS_PRESENT_POSITION_L, 2))

    # #test sync write
    # Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,0,0,0,0,0,0], 4000, 255)
    # time.sleep(2)
    # print(Controler.read_servo_data(4, SMS_STS_PRESENT_POSITION_L, 2))


    #define the initial position
    # finish_func = Controler.defineInitialPosition(4)
    # print("Move the servo to the initial position")
    # input("Press any key to continue")
    # finish_func()
    #Controler.defineAllMiddlePositions()



    # Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,0,0,0,0,0,0], 4000, 255)
    # time.sleep(2)
    # Controler.setGroupSync_ServoPosSpeedAcc([1,2,3,4,5,6,7,8], [0,0,180,180,0,0,0,0], 4000, 255)



