#!/usr/bin/env python
#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
# Be sure that SCServo(STS/SMS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SCServo SDK library

MOVE = False

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_STS_GOAL_ACC          = 41
ADDR_STS_GOAL_POSITION     = 42
ADDR_STS_GOAL_SPEED        = 46
ADDR_STS_PRESENT_POSITION  = 56

# Default setting
SCS_ID                      = [1, 2, 3, 4,5,6,7,8]           # SCServo#1 ID : 1, SCServo#2 ID : 2 

BAUDRATE                    = 921600            # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 100               # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
SCS_MOVING_SPEED            = 0                 # SCServo moving speed
SCS_MOVING_ACC              = 0                 # SCServo moving acc
protocol_end                = 0                 # SCServo bit end(STS/SMS=0, SCS=1)

index = 0
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_STS_GOAL_POSITION, 2)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_STS_PRESENT_POSITION, 4)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

time.sleep(3.5)     #Wait for ESP32 initialization

def set_servo_acceleration(scs_id, acc_value):
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_STS_GOAL_ACC, acc_value)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

def set_servo_speed(scs_id, speed_value):
    scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_STS_GOAL_SPEED, speed_value)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

def add_servo_param_sync_read(scs_id):
    scs_addparam_result = groupSyncRead.addParam(scs_id)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % scs_id)
        quit()
    else:
        print("[ID:%03d] groupSyncRead addparam success" % scs_id)


# Set acceleration for SCServo#1
for scs_id in SCS_ID:
    set_servo_acceleration(scs_id, SCS_MOVING_ACC)


# Set speed for SCServo#1
for scs_id in SCS_ID:
    set_servo_speed(scs_id, SCS_MOVING_SPEED)

if not MOVE:
    # Disable torque for SCServo#1 and SCServo#2
    for scs_id in SCS_ID:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_SCS_TORQUE_ENABLE, 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error)) 

time.sleep(1)

# Add parameter storage for SCServo present position value
for scs_id in SCS_ID:
    add_servo_param_sync_read(scs_id)







scs_present_position_speed = {}
scs_present_position = {}
scs_present_speed = {}

while 1:
    #print("Press any key to continue! (or press ESC to quit!)")
    #if getch() == chr(0x1b):
    #    break

    # Allocate goal position value into byte array
    param_goal_position = [SCS_LOBYTE(scs_goal_position[index]), SCS_HIBYTE(scs_goal_position[index])]

    if MOVE:
        # Add SCServo#1 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(SCS_ID[0], param_goal_position)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % SCS_ID[0])
            quit()

        # Add SCServo#2 goal position value to the Syncwrite parameter storage
        scs_addparam_result = groupSyncWrite.addParam(SCS2_ID, param_goal_position)
        if scs_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % SCS_ID[0])
            quit()

        # Syncwrite goal position
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        scs_comm_result = groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
       
        # Check if groupsyncread data of SCServo#1 is available
        for scs_id in SCS_ID:
         
            scs_getdata_result = groupSyncRead.isAvailable(scs_id, ADDR_STS_PRESENT_POSITION, 4)
            if scs_getdata_result == True:
                # Get SCServo#1 present position value
                scs_present_position_speed[scs_id] = groupSyncRead.getData(scs_id, ADDR_STS_PRESENT_POSITION, 4)
                scs_present_position[scs_id] = SCS_LOWORD(scs_present_position_speed[scs_id])
                scs_present_speed[scs_id] = SCS_HIWORD(scs_present_position_speed[scs_id])
                #print(f"| {'ID':^5} | {'GoalPos':^10} | {'PresPos':^10} | {'PresSpd':^10} |")
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % scs_id)

        # print all values of servos first servo 1 position and speed then next column servo 2 position and speed etc
        for scs_id in SCS_ID:
            print(f"| {scs_present_position[scs_id]:^4} | {scs_present_speed[scs_id]:^4} |",end="")
        print("")

           #if not (abs(scs_goal_position[index] - scs_present_position) > SCS_MOVING_STATUS_THRESHOLD):
           #     break



    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()

# Disable torque for SCServo#1 and SCServo#2
for scs_id in SCS_ID:
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_SCS_TORQUE_ENABLE, 0)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()