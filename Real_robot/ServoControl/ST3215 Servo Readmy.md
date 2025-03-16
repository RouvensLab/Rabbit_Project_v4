                          Communication Protocol User Manual

                                Table of Contents
1. Communication Protocol
   - 1.1 Instruction Package
   - 1.2 Response Package
   - 1.3 Instruction Types
     - 1.3.1 PING
     - 1.3.2 READ DATA
     - 1.3.3 WRITE DATA
     - 1.3.4 REG WRITE
     - 1.3.5 ACTION
     - 1.3.6 SYNC WRITE
     - 1.3.7 SYNC READ
     - 1.3.8 RESET

-------------------------------------------------------------------------------

1.0 Communication Protocol
The serial bus smart servo communication protocol supports both potentiometer
and magnetic encoder series servos:

- Potentiometer Series: Uses TTL-level single-bus communication (multiplexing
  transmit and receive on one line). Connection: 3 wires (positive, negative,
  signal).
- Magnetic Encoder Series: Uses an ARM 32-bit microcontroller with a 360-degree,
  12-bit precision magnetic sensor. Communication: RS485 (strong anti-interference)
  with asynchronous duplex signaling.

Communication uses a question-and-answer mode:
- Controller sends an instruction package.
- Servo responds with a response package.

Multiple servos can share a bus, each with a unique ID (0-253). Instructions
include the target ID; only the matching servo responds. Frame format:
- 1 start bit, 8 data bits, 1 stop bit, no parity, 10 bits total.

Byte Order:
- Potentiometer Series: High byte first, low byte second.
- Magnetic Encoder Series: Low byte first, high byte second.

Check the servo model's memory table for specific control details.

-------------------------------------------------------------------------------

1.1 Instruction Package
Format:
Header    ID    Data Length    Instruction    Parameters          Checksum
0xFF 0xFF ID    Length         Instruction    Parameter1...N      Checksum

- Header: Two 0xFF bytes mark the packet start.
- ID: Servo ID (0x00-0xFD, 0-253). Broadcast ID: 254 (0xFE).
- Broadcast ID: All servos receive; only PING responds (avoid with multiple servos).
- Data Length: Parameters (N) + 2.
- Instruction: Operation code (see 1.3).
- Parameters: Up to 2 bytes per value; byte order varies by servo type.
- Checksum: ~(ID + Length + Instruction + Parameter1 + ... + ParameterN).
  Take lowest byte if sum > 255; ~ is bitwise negation.

-------------------------------------------------------------------------------

1.2 Response Package
Format:
Header    ID    Data Length    Error    Parameters          Checksum
0xFF 0xFF ID    Length         ERROR    Parameter1...N      Checksum

- Error: Servo status (0 = no error; see memory table).
- Parameters: Returned data for READ DATA.
- Checksum: Same as instruction package calculation.

-------------------------------------------------------------------------------

1.3 Instruction Types
Available instructions:
Instruction       Function                         Value    Parameter Length
PING             Query servo status               0x01     0
READ DATA        Read control table               0x02     2
WRITE DATA       Write control table              0x03     >= 1
REG WRITE        Queue write for later            0x04     >= 2
ACTION           Trigger REG WRITE                0x05     0
SYNC WRITE       Control multiple servos          0x83     >= 2
SYNC READ        Query multiple servos            0x82     >= 3
RESET            Reset to factory values          0x06     0

1.3.1 PING
- Function: Query servo working status
- Length: 0x02
- Instruction: 0x01
- Parameters: None
Example: Query ID 1
- Command: FF FF 01 02 01 FB
- Response: FF FF 01 02 00 FC

1.3.2 READ DATA
- Function: Read from memory control table
- Length: 0x04
- Instruction: 0x02
- Parameters: 1: Start address, 2: Data length
Example: Read position (2 bytes) from ID 1 at 0x38
- Command: FF FF 01 04 02 38 02 BE
- Response: FF FF 01 04 00 18 05 DD
- Result: Position = 0x0518 (decimal 1304)

1.3.3 WRITE DATA
- Function: Write to memory control table
- Length: N + 3 (N = parameters)
- Instruction: 0x03
- Parameters: 1: Start address, 2-N+1: Data bytes
Example 1: Set ID to 1 (address 0x05, broadcast)
- Command: FF FF FE 04 03 05 01 F4
- Note: No response; disable EPROM lock first.
Example 2: Set ID 1 to position 2048, speed 1000 (0x2A)
- Command: FF FF 01 09 03 2A 00 08 00 00 E8 03 D5
- Response: FF FF 01 02 00 FC

1.3.4 REG WRITE
- Function: Queue data for ACTION trigger
- Length: N + 3
- Instruction: 0x04
- Parameters: Same as WRITE DATA
Example: Queue position 2048, speed 1000 for ID 1
- Command: FF FF 01 09 04 2A 00 08 00 00 E8 03 D4
- Response: FF FF 01 02 00 FC

1.3.5 ACTION
- Function: Trigger REG WRITE execution
- Length: 0x02
- Instruction: 0x05
- Parameters: None
Example: Trigger all queued commands (broadcast)
- Command: FF FF FE 02 05 FA
- Note: No response.

1.3.6 SYNC WRITE
- Function: Control multiple servos at once
- ID: 0xFE
- Length: (L + 1) * N + 4 (L = data length, N = servos)
- Instruction: 0x83
- Parameters: 1: Start address, 2: Data length, 3+: ID + data
Example: Set position 2048, speed 1000 for IDs 1-4
- Command: FF FF FE 20 83 2A 06 01 00 08 00 00 E8 03 02 00 08 00 00 E8 03
           03 00 08 00 00 E8 03 04 00 08 00 00 E8 03 58
- Note: No response.

1.3.7 SYNC READ
- Function: Query multiple servos at once
- ID: 0xFE
- Length: N + 4 (N = servos)
- Instruction: 0x82
- Parameters: 1: Start address, 2: Data length, 3+: Servo IDs
Example: Read 8 bytes from 0x38 for IDs 1-2
- Command: FF FF FE 06 82 38 08 01 02 36
- Responses:
  - ID 1: FF FF 01 0A 00 00 08 00 00 00 00 79 1E 55
  - ID 2: FF FF 02 0A 00 FF 07 00 00 00 00 77 23 53

1.3.8 RESET
- Function: Reset control table to factory values
- Length: 0x02
- Instruction: 0x06
- Parameters: None
Example: Reset ID 1
- Command: FF FF 01 02 06 F6
- Response: FF FF 01 02 00 FC

-------------------------------------------------------------------------------



Example Code:
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

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_STS_GOAL_ACC          = 41
ADDR_STS_GOAL_POSITION     = 42
ADDR_STS_GOAL_SPEED        = 46
ADDR_STS_PRESENT_POSITION  = 56

# Default setting
SCS1_ID                     = 1                 # SCServo#1 ID : 1
SCS2_ID                     = 2                 # SCServo#1 ID : 2
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
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

# SCServo#1 acc
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS1_ID, ADDR_STS_GOAL_ACC, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# SCServo#2 acc
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS2_ID, ADDR_STS_GOAL_ACC, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# SCServo#1 speed
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS1_ID, ADDR_STS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# SCServo#2 speed
scs_comm_result, scs_error = packetHandler.write2ByteTxRx(portHandler, SCS2_ID, ADDR_STS_GOAL_SPEED, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Add parameter storage for SCServo#1 present position value
scs_addparam_result = groupSyncRead.addParam(SCS1_ID)
if scs_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % SCS1_ID)
    quit()

# Add parameter storage for SCServo#2 present position value
scs_addparam_result = groupSyncRead.addParam(SCS2_ID)
if scs_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % SCS2_ID)
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Allocate goal position value into byte array
    param_goal_position = [SCS_LOBYTE(scs_goal_position[index]), SCS_HIBYTE(scs_goal_position[index])]

    # Add SCServo#1 goal position value to the Syncwrite parameter storage
    scs_addparam_result = groupSyncWrite.addParam(SCS1_ID, param_goal_position)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % SCS1_ID)
        quit()

    # Add SCServo#2 goal position value to the Syncwrite parameter storage
    scs_addparam_result = groupSyncWrite.addParam(SCS2_ID, param_goal_position)
    if scs_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % SCS2_ID)
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
        scs_getdata_result = groupSyncRead.isAvailable(SCS1_ID, ADDR_STS_PRESENT_POSITION, 4)
        scs1_present_position_speed = 0
        scs2_present_position_speed = 0
        if scs_getdata_result == True:
            # Get SCServo#1 present position value
            scs1_present_position_speed = groupSyncRead.getData(SCS1_ID, ADDR_STS_PRESENT_POSITION, 4)
        else:
            print("[ID:%03d] groupSyncRead getdata failed" % SCS1_ID)

        # Check if groupsyncread data of SCServo#2 is available
        scs_getdata_result = groupSyncRead.isAvailable(SCS2_ID, ADDR_STS_PRESENT_POSITION, 4)
        if scs_getdata_result == True:
            # Get SCServo#2 present position value
            scs2_present_position_speed = groupSyncRead.getData(SCS2_ID, ADDR_STS_PRESENT_POSITION, 4)
        else:
            print("[ID:%03d] groupSyncRead getdata failed" % SCS2_ID)

        scs1_present_position = SCS_LOWORD(scs1_present_position_speed)
        scs1_present_speed = SCS_HIWORD(scs1_present_position_speed)
        scs2_present_position = SCS_LOWORD(scs2_present_position_speed)
        scs2_present_speed = SCS_HIWORD(scs2_present_position_speed)
        print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d\t[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
              % (SCS1_ID, scs_goal_position[index], scs1_present_position, SCS_TOHOST(scs1_present_speed, 15), 
                 SCS2_ID, scs_goal_position[index], scs2_present_position, SCS_TOHOST(scs2_present_speed, 15)))

        if not ((abs(scs_goal_position[index] - scs1_present_position_speed) > SCS_MOVING_STATUS_THRESHOLD) and (abs(scs_goal_position[index] - scs2_present_position_speed) > SCS_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()

# SCServo#1 torque
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS1_ID, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# SCServo#2 torque
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(portHandler, SCS2_ID, ADDR_SCS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()