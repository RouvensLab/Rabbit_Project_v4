#!/usr/bin/env python

import sys
import serial
import time
import os
import threading
import queue
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from queue import Queue, Empty
from PySide6.QtWidgets import (QFileDialog, QSlider, QHBoxLayout, QStyleFactory, QApplication, QMainWindow, QVBoxLayout,
                               QWidget, QComboBox, QPushButton, QToolBar, QLabel, QTableWidget, QTableWidgetItem, QMessageBox, QMenu, QInputDialog, QToolBox)
from PySide6.QtCore import QTimer, Qt, Signal, Slot, QSettings, QSize, QPoint, QObject
from PySide6.QtGui import QColor
import pyqtgraph as pg
import pygame
import numpy as np

COMPORT = "COM5"  # Adjust as needed
BAUDRATE = 921600
REGISTERFILE = 'Modules\\Robot\\Robot_Teststand_register.csv'

#!/usr/bin/env python

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
STS_END = 0

# Instruction for STS Protocol
INST_PING       = 1
INST_READ       = 2
INST_WRITE      = 3
INST_REG_WRITE  = 4
INST_ACTION     = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ  = 130  # 0x82

# Communication Result
COMM_SUCCESS        = 0  # tx or rx packet communication success
COMM_PORT_BUSY      = -1  # Port is busy (in use)
COMM_TX_FAIL        = -2  # Failed transmit instruction packet
COMM_RX_FAIL        = -3  # Failed get status packet
COMM_TX_ERROR       = -4  # Incorrect instruction packet
COMM_RX_WAITING     = -5  # Now recieving status packet
COMM_RX_TIMEOUT     = -6  # There is no status packet
COMM_RX_CORRUPT     = -7  # Incorrect status packet
COMM_NOT_AVAILABLE  = -9  #

# Memory table definition
#-------EPROM(Read-only)--------
STS_MODEL_L             = 3
STS_MODEL_H             = 4

#-------EPROM(Read/Write)--------
STS_ID                  = 5
STS_BAUD_RATE           = 6
STS_RET_DELAY           = 7
STS_RESPONSE_LEVEL      = 8
STS_MIN_ANGLE_LIMIT_L   = 9
STS_MIN_ANGLE_LIMIT_H   = 10
STS_MAX_ANGLE_LIMIT_L   = 11
STS_MAX_ANGLE_LIMIT_H   = 12
STS_MAX_TEMPERATURE     = 13
STS_MAX_VOLTAGE         = 14
STS_MIN_VOLTAGE         = 15
STS_MAX_TORQUE_L        = 16
STS_MAX_TORQUE_H        = 17
STS_PHASE               = 18
STS_UNLOAD              = 19
STS_LED_ALARM           = 20
STS_P_CONTROL           = 21
STS_D_CONTROL           = 22
STS_I_CONTROL           = 23
STS_MIN_START_FORCE_L   = 24
STS_MIN_START_FORCE_H   = 25
STS_CLOCKWISE_SENS      = 26
STS_COUNTERCLOCK_SENS   = 27
STS_PROTECTION_CURRENT_L= 28
STS_PROTECTION_CURRENT_H= 29
STS_ANGULAR_CORRECTION  = 30
STS_OFS_L               = 31
STS_OFS_H               = 32
STS_MODE                = 33
STS_PROT_TORQUE         = 34
STS_PROT_TIME           = 35
STS_OVERLOAD_TORQUE     = 36


STS_CW_DEAD             = 26
STS_CCW_DEAD            = 27



#-------SRAM(Read/Write)--------
STS_TORQUE_ENABLE       = 40
STS_ACC                 = 41
STS_GOAL_POSITION_L     = 42
STS_GOAL_POSITION_H     = 43
STS_GOAL_TIME_L         = 44
STS_GOAL_TIME_H         = 45
STS_GOAL_SPEED_L        = 46
STS_GOAL_SPEED_H        = 47
STS_LOCK                = 55


# Define STServo register addresses
STS_PRESENT_POSITION_L  = 56
STS_PRESENT_POSITION_H  = 57
STS_PRESENT_SPEED_L     = 58
STS_PRESENT_SPEED_H     = 59
STS_PRESENT_LOAD_L      = 60
STS_PRESENT_LOAD_H      = 61
STS_PRESENT_VOLTAGE     = 62
STS_PRESENT_TEMPERATURE = 63
STS_MOVING              = 66
STS_PRESENT_CURRENT_L   = 69
STS_PRESENT_CURRENT_H   = 70


SERVOS = [1,2,3,4]
SERIAL_TIMOUT = 0.03

class Serial_Master(QObject):
    data_signal             = Signal(dict)
    status_signal           = Signal(str)
    update_rate_signal      = Signal(float)
    update_servos_signal    = Signal(list)

    def __init__(self, serial_port, baud_rate):
        super().__init__()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = threading.Event()
        self.running.set()
        self.command_queue = Queue()
        self.write_queue = Queue()
        self.read_queue = Queue()
        self.executor = ThreadPoolExecutor(max_workers=3)  # One for main, one for write, one for read
        self.last_time = time.time()
        self.servo_id = SERVOS[0]
        self.servo_ids = SERVOS  # Default servo IDs, updated by search
        self.exit_flag = False

        # Start threads
        self.executor.submit(self.run)
        self.executor.submit(self.writer_thread)
        self.executor.submit(self.reader_thread)

    def start(self):
        self.running.set()
    

    def stop(self):
        self.running.clear()

    def exit_thread(self):
        self.running.clear()
        self.exit_flag = True
        self.executor.shutdown(wait=True)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def check_head(self):
        head = self.ser.read(2)
        return len(head) == 2 and head[0] == 0xFF and head[1] == 0xFF

    def run(self):
        while not self.exit_flag:
            if self.running.is_set():
                try:
                    if not self.ser or not self.ser.is_open:
                        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=SERIAL_TIMOUT)
                        self.ser.flushInput()
                        self.ser.flushOutput()
                    while self.running.is_set():
                        try:
                            command, *args = self.command_queue.get(timeout=SERIAL_TIMOUT)
                            if command == INST_PING:
                                self.write_queue.put((args[0], 0, []))  # Ping
                            elif command == INST_READ:
                                self.read_queue.put((args[0], args[1], [args[2]]))  # Read
                            elif command == INST_WRITE:
                                self.write_queue.put((args[0], args[1], args[2]))  # Write
                            elif command == INST_REG_WRITE:
                                self.write_queue.put((args[0], args[1], args[2]))  # Reg Write
                            elif command == INST_ACTION:
                                self.write_queue.put((args[0], 0, []))  # Action
                            elif command == INST_SYNC_WRITE:
                                self.write_queue.put((args[0], args[1], args[2]))  # Sync Write
                            elif command == INST_SYNC_READ:
                                self.read_queue.put((args[0], args[1], args[2]))  # Sync Read
                        except queue.Empty:
                            # Read syncRead position , speed, load, voltage, temperature,moving, current
                            #data = self.syncRead(STS_PRESENT_POSITION_L, 14, self.servo_ids)
                            #if data:
                            #    self.process_read_data(data, STS_PRESENT_POSITION_L)
                            pass
                except serial.SerialException as e:
                    self.status_signal.emit(f"Serial error: {e}")
                    time.sleep(0.2)  # Retry after delay
            time.sleep(0.002)

    def writer_thread(self):
        while not self.exit_flag:
            if self.running.is_set():
                try:
                    start_address, data_length, servo_data = self.write_queue.get(timeout=SERIAL_TIMOUT)
                    print(".w.")
                    if self.syncWrite(start_address, data_length, servo_data):
                        self.write_queue.task_done()
                except queue.Empty:
                    print("SyncWrite")
                    continue
                time.sleep(0.1)

    def reader_thread(self):
        while not self.exit_flag:
            if self.running.is_set():
                try:
                    start_address, data_length, servo_ids = self.read_queue.get(timeout=SERIAL_TIMOUT)
                    print(".r.")
                    data = self.syncRead(start_address, data_length, servo_ids)
                    if data:
                        self.process_read_data(data, start_address)
                        self.read_queue.task_done()
                except queue.Empty:
                    # Default periodic read of position for all servos
                    print("SyncRead")
                    data = self.syncRead(STS_PRESENT_POSITION_L, 14, self.servo_ids, timeout=SERIAL_TIMOUT)
                    if data:
                        self.process_read_data(data, STS_PRESENT_POSITION_L)
                time.sleep(0.05)  # Prevent bus overload

    def process_read_data(self, data, start_address):
        #Calculate the time between updates
        current_time = time.time()
        delta_time = (current_time - self.last_time) * 1000  # ms
        self.last_time = current_time
        self.update_rate_signal.emit(delta_time)

        for servo_id, bytes_data in data.items():
            parsed_data = {'servo_id': servo_id, 'timestamp': current_time}

            if start_address == STS_PRESENT_POSITION_L:
                if len(bytes_data) == 2:
                    parsed_data['angle'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
                elif len(bytes_data) == 14:
                    parsed_data.update({
                    'angle': int.from_bytes(bytes_data[0:2], byteorder='little', signed=True),
                    'speed': int.from_bytes(bytes_data[2:4], byteorder='little', signed=True),
                    'load': int.from_bytes(bytes_data[4:6], byteorder='little'),
                    'voltage': bytes_data[6],
                    'temperature': bytes_data[7],
                    'async_write_flag': bytes_data[8],
                    'servo_status': bytes_data[9],
                    'mobile_sign': bytes_data[10],
                    'current': int.from_bytes(bytes_data[12:14], byteorder='little', signed=True)
                    })
            elif start_address == STS_ID:
                parsed_data['id'] = bytes_data[0]
            elif start_address == STS_BAUD_RATE:
                parsed_data['baud_rate'] = bytes_data[0]
        
            elif start_address == STS_MIN_ANGLE_LIMIT_L:
                parsed_data['min_angle_limit'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
            
            elif start_address == STS_PRESENT_SPEED_L:
                parsed_data['speed'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
            elif start_address == STS_PRESENT_LOAD_L:
                parsed_data['load'] = int.from_bytes(bytes_data[0:2], byteorder='little')
            elif start_address == STS_PRESENT_VOLTAGE:
                parsed_data['voltage'] = bytes_data[0]
            elif start_address == STS_PRESENT_TEMPERATURE:
                parsed_data['temperature'] = bytes_data[0]
            
            elif start_address == STS_MOVING:
                parsed_data['moving'] = bytes_data[0]
            elif start_address == STS_PRESENT_CURRENT_L:
                parsed_data['current'] = int.from_bytes(bytes_data[0:2], byteorder='little', signed=True)
            

            
   
       
            
            self.data_signal.emit(parsed_data)

    def syncWrite(self, start_address, data_length, servo_data,timeout = 0.01):
        param_length = len(servo_data) * (1 + data_length)
        total_length = param_length + 4
        packet = [0] * (total_length + 4)

        packet[0] = 0xFF
        packet[1] = 0xFF
        packet[2] = 0xFE  # Broadcast ID
        packet[3] = total_length
        packet[4] = 0x83  # Sync Write
        packet[5] = start_address
        packet[6] = data_length

        idx = 7
        for servo_id, value in servo_data:
            packet[idx] = servo_id
            idx += 1
            for i in range(data_length):
                packet[idx + i] = (value >> (8 * i)) & 0xFF
            idx += data_length

        checksum = 0
        for i in range(2, total_length + 3):
            checksum += packet[i]
        packet[-1] = ~checksum & 0xFF

        try:
            if not self.ser or not self.ser.is_open:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=timeout)
            self.ser.flush()
            self.ser.write(bytes(packet))
            return True
        except serial.SerialException:
            return False

    def syncRead(self, start_address, data_length, servo_ids,timeout = 0.005):
        param_length = len(servo_ids)
        total_length = param_length + 4
        packet = [0] * (total_length + 4)

        packet[0] = 0xFF
        packet[1] = 0xFF
        packet[2] = 0xFE  # Broadcast ID
        packet[3] = total_length
        packet[4] = 0x82  # Sync Read
        packet[5] = start_address
        packet[6] = data_length

        for i, servo_id in enumerate(servo_ids):
            packet[7 + i] = servo_id

        checksum = 0
        for i in range(2, total_length + 3):
            checksum += packet[i]
        packet[-1] = ~checksum & 0xFF

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
                        data = list(response[5:5 + data_length])
                        response_data[servo_id] = data
                    checksum = sum(response[2:5 + data_length]) & 0xFF
                    if (~checksum & 0xFF) != response[-1]:
                        response_data.pop(servo_id, None)

        return response_data if response_data else None

    def construct_instruction(self, servo_id, instruction, parameters):
        packet = bytearray([0xFF, 0xFF, int(servo_id), len(parameters) + 2, instruction])
        packet.extend(parameters)
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)
        return bytes(packet)

    def change_servo_id(self, current_id, new_id):
        self.running.clear()
        time.sleep(2)
        try:
            self.unLockEprom(current_id)
            self.write_byte(current_id, 0x05, new_id)
            self.LockEprom(new_id)
            return True
        except serial.SerialException as e:
            self.status_signal.emit(f"Serial error during ID change: {e}")
            return False

    def unLockEprom(self, servo_id):
        self.write_byte(servo_id, 0x37, 0x00)

    def LockEprom(self, servo_id):
        self.write_byte(servo_id, 0x37, 0x01)

    def write_byte(self, servo_id, address, value):
        parameters = [address, value]
        packet = self.construct_instruction(servo_id, 0x03, parameters)
        if self.ser and self.ser.is_open:
            self.ser.write(packet)
            self.ser.flush()

class ServoControlApp(QMainWindow):
    update_signal = Signal(dict)
    status_update_signal = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Servo Control Interface')
        QApplication.setStyle(QStyleFactory.create("Fusion"))

        self.serial_port = COMPORT
        self.baud_rate = BAUDRATE
        self.serial_master = Serial_Master(self.serial_port, self.baud_rate)
        self.serial_master.data_signal.connect(self.update_data_to_plot)
        self.serial_master.status_signal.connect(self.update_status_to_label)
        self.serial_master.update_rate_signal.connect(self.update_rate_to_label)
      

        self.connected = False
        self.start_time = time.time()
        self.position = 2048
        self.speed = 1000
        self.max_points = 2000

         # Use a single deque per servo to store all data with timestamps
        self.servo_data = {i: deque(maxlen=self.max_points) for i in range(1, 8)}

        self.settings = QSettings("Bunny_Robot", "ServoControlApp")
        self.initUI()
        self.initGamepad()
        self.load_settings()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(200)

        self.autoscale_button.setChecked(True)
        self.toggle_autoscale()
        self.table_changed = False


    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        toolbar = QToolBar("Main Toolbar")
        self.addToolBar(Qt.LeftToolBarArea, toolbar)

        self.connect_button = QPushButton('Connect')
        self.connect_button.setCheckable(True)
        self.connect_button.setStyleSheet("background-color: red")
        self.connect_button.clicked.connect(self.toggle_connection)
        toolbar.addWidget(self.connect_button)

        self.servo_selector = QComboBox()
        self.servo_selector.addItems([f"Servo {i+1}" for i in range(8)])
        self.servo_selector.currentIndexChanged.connect(self.select_servo)
        self.servo_selector.setCurrentText("Servo 2")
        toolbar.addWidget(self.servo_selector)

        self.search_servos_button = QPushButton('Search Servos')
        self.search_servos_button.clicked.connect(self.search_servos)
        toolbar.addWidget(self.search_servos_button)

        self.change_id_button = QPushButton('Change ID')
        self.change_id_button.clicked.connect(self.change_servo_id)
        toolbar.addWidget(self.change_id_button)

        self.angle_label = QLabel("Angle: 0")
        toolbar.addWidget(self.angle_label)

        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setMinimum(0)
        self.angle_slider.setMaximum(4096)
        self.angle_slider.setValue(0)
        self.angle_slider.valueChanged.connect(self.update_angle_position)
        toolbar.addWidget(self.angle_slider)

        self.speed_label = QLabel("Speed: 1000")
        toolbar.addWidget(self.speed_label)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(3400)
        self.speed_slider.setValue(1000)
        self.speed_slider.valueChanged.connect(self.update_speed)
        toolbar.addWidget(self.speed_slider)

        self.engage_button = QPushButton('Engage')
        self.engage_button.setCheckable(True)
        self.engage_button.setStyleSheet("background-color: green")
        self.engage_button.clicked.connect(self.toggle_torque)
        toolbar.addWidget(self.engage_button)

        self.autoscale_button = QPushButton('Autoscale')
        self.autoscale_button.setCheckable(True)
        self.autoscale_button.setStyleSheet("background-color: red")
        self.autoscale_button.clicked.connect(self.toggle_autoscale)
        toolbar.addWidget(self.autoscale_button)

        self.read_mode_button = QPushButton('Read Operation Mode')
        self.read_mode_button.clicked.connect(self.read_operation_mode)
        toolbar.addWidget(self.read_mode_button)

        self.read_params_button = QPushButton('Read Feedbacks On')
        self.read_params_button.setCheckable(True)
        self.read_params_button.setChecked(True)
        self.read_params_button.setStyleSheet("background-color: green")
        self.read_params_button.clicked.connect(self.toggle_read_params)
        toolbar.addWidget(self.read_params_button)

        # Plotting
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)

        self.plot_angle = self.plot_widget.addPlot(title="Angle", row=0, col=0)
        self.plot_load = self.plot_widget.addPlot(title="Load", row=1, col=0)
        self.plot_speed = self.plot_widget.addPlot(title="Speed", row=2, col=0)
        self.plot_current = self.plot_widget.addPlot(title="Current", row=3, col=0)

        self.plot_angle.setLabel('bottom', 'Time', units='s')
        self.plot_angle.setLabel('left', 'Angle', units='[]')
        self.plot_load.setLabel('bottom', 'Time', units='s')
        self.plot_load.setLabel('left', 'Load', units='[]')
        self.plot_speed.setLabel('bottom', 'Time', units='s')
        self.plot_speed.setLabel('left', 'Speed', units='[]')
        self.plot_current.setLabel('bottom', 'Time', units='s')
        self.plot_current.setLabel('left', 'Current', units='mA')

        colors = [QColor(255, 0, 0), QColor(0, 255, 0), QColor(0, 0, 255), QColor(255, 255, 0), QColor(255, 0, 255), QColor(0, 255, 255), QColor(128, 0, 128), QColor(255, 165, 0)]
        self.ax_angle = {i: self.plot_angle.plot(pen=colors[i-1], name=f"Servo {i}") for i in range(1, 9)}
        self.ax_load = {i: self.plot_load.plot(pen=colors[i-1], name=f"Servo {i}") for i in range(1, 9)}
        self.ax_speed = {i: self.plot_speed.plot(pen=colors[i-1], name=f"Servo {i}") for i in range(1, 9)}
        self.ax_current = {i: self.plot_current.plot(pen=colors[i-1], name=f"Servo {i}") for i in range(1, 9)}

        self.plot_angle.addLegend()
        self.plot_load.addLegend()
        self.plot_speed.addLegend()
        self.plot_current.addLegend()

        self.plot_load.setXLink(self.plot_angle)
        self.plot_speed.setXLink(self.plot_angle)
        self.plot_current.setXLink(self.plot_angle)

        self.plot_angle.setYRange(-4091, 4091)
        self.plot_load.setYRange(0, 1000)
        self.plot_speed.setYRange(-1000, 1000)
        self.plot_current.setYRange(-1000, 1000)


        # Status bar
        self.status_bar = self.statusBar()
        self.status_label = QLabel("Update rate: N/A")
        self.status_label.setStyleSheet("border: 1px solid black; background-color: green;")
        self.status_bar.addPermanentWidget(self.status_label)

        # Register table
        self.addToolBar(Qt.RightToolBarArea, self.populate_register_table())

        self.register_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.register_table.customContextMenuRequested.connect(self.show_context_menu)

        # Sequencer
        sequencer_toolbar = QToolBar("Sequencer Toolbar")
        self.addToolBar(Qt.RightToolBarArea, sequencer_toolbar)

        button_layout = QHBoxLayout()
        self.load_sequence_button = QPushButton('Load Sequence')
        self.load_sequence_button.clicked.connect(self.load_sequence)
        button_layout.addWidget(self.load_sequence_button)

        self.save_sequence_button = QPushButton('Save Sequence')
        self.save_sequence_button.clicked.connect(self.save_sequence)
        button_layout.addWidget(self.save_sequence_button)

        self.run_sequence_button = QPushButton('Run Sequence')
        self.run_sequence_button.clicked.connect(self.run_sequence)
        button_layout.addWidget(self.run_sequence_button)

        self.run_line_button = QPushButton('Run Line')
        self.run_line_button.clicked.connect(self.run_line)
        button_layout.addWidget(self.run_line_button)

        toolbar_widget = QWidget()
        toolbar_widget.setLayout(button_layout)
        sequencer_toolbar.addWidget(toolbar_widget)

        self.sequencer_table = QTableWidget()
        self.sequencer_table.setColumnCount(9)
        self.sequencer_table.setHorizontalHeaderLabels(["dTime", "C1_a", "C2_a", "C3_a", "C4_a", "C5_a", "C6_a", "C7_a", "C8_a"])
        sequencer_toolbar.addWidget(self.sequencer_table)

        self.sequencer_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.sequencer_table.customContextMenuRequested.connect(self.show_sequencer_context_menu)


    #region plot
    @Slot(dict)
    def update_data_to_plot(self, data):
        servo_id = data['servo_id']
        timestamp = data['timestamp']

        # Create a data point with all available metrics
        data_point = {'timestamp': timestamp}
        if 'angle' in data:
            data_point['angle'] = data['angle']
        if 'speed' in data:
            data_point['speed'] = data['speed']
        if 'load' in data:
            data_point['load'] = data['load']
        if 'current' in data:
            data_point['current'] = data['current']

        # Append the data point to the servo's deque
        self.servo_data[servo_id].append(data_point)

        # Extract arrays for plotting
        timestamps = np.array([d['timestamp'] for d in self.servo_data[servo_id]])
        if 'angle' in data_point:
            angles = np.array([d.get('angle', np.nan) for d in self.servo_data[servo_id]])
            self.ax_angle[servo_id].setData(timestamps, angles)
        if 'speed' in data_point:
            speeds = np.array([d.get('speed', np.nan) for d in self.servo_data[servo_id]])
            self.ax_speed[servo_id].setData(timestamps, speeds)
        if 'load' in data_point:
            loads = np.array([d.get('load', np.nan) for d in self.servo_data[servo_id]])
            self.ax_load[servo_id].setData(timestamps, loads)
        if 'current' in data_point:
            currents = np.array([d.get('current', np.nan) for d in self.servo_data[servo_id]])
            self.ax_current[servo_id].setData(timestamps, currents)

        # Update X-axis range
        time_window = 10
        current_time_range = (timestamp - time_window, timestamp)
        self.plot_angle.setXRange(*current_time_range, padding=0)
        self.plot_load.setXRange(*current_time_range, padding=0)
        self.plot_speed.setXRange(*current_time_range, padding=0)
        self.plot_current.setXRange(*current_time_range, padding=0)


    @Slot(str)
    def update_status_to_label(self, text):
        self.status_label.setText(text)

    @Slot(float)
    def update_rate_to_label(self, rate):
        self.status_label.setText(f"Update rate: {rate:.2f} ms")

    def show_sequencer_context_menu(self, position):
        context_menu = QMenu(self)
        insert_action = context_menu.addAction("Insert Row")
        delete_action = context_menu.addAction("Delete Row")
        insert_action.triggered.connect(self.insert_sequencer_row)
        delete_action.triggered.connect(self.delete_sequencer_row)
        context_menu.exec(self.sequencer_table.viewport().mapToGlobal(position))

    def insert_sequencer_row(self):
        current_row = self.sequencer_table.currentRow()
        self.sequencer_table.insertRow(current_row + 1)

    def delete_sequencer_row(self):
        current_row = self.sequencer_table.currentRow()
        if current_row >= 0:
            self.sequencer_table.removeRow(current_row)

    def load_sequence(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        seq_dir = os.path.join(current_dir, "Seq")
        options = QFileDialog.Options()
        seq_file_path, _ = QFileDialog.getOpenFileName(self, "Open Sequence File", seq_dir, "CSV Files (*.csv);;All Files (*)", options=options)
        if not seq_file_path:
            self.status_bar.showMessage("No sequence file selected")
            return
        if not os.path.exists(seq_file_path):
            self.status_bar.showMessage("Sequence file not found")
            return
        data = np.genfromtxt(seq_file_path, delimiter='\t', dtype=str)
        self.sequencer_table.setRowCount(len(data))
        self.sequencer_table.setColumnCount(len(data[0]))
        for row in range(len(data)):
            for col in range(len(data[row])):
                self.sequencer_table.setItem(row, col, QTableWidgetItem(data[row][col]))
        self.status_bar.showMessage(f"Sequence loaded from {seq_file_path}")

    def save_sequence(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        seq_dir = os.path.join(current_dir, "Seq")
        if not os.path.exists(seq_dir):
            os.makedirs(seq_dir)
        seq_file_path = os.path.join(seq_dir, "Seq1.csv")
        rows = self.sequencer_table.rowCount()
        cols = self.sequencer_table.columnCount()
        data = []
        for row in range(rows):
            row_data = []
            for col in range(cols):
                item = self.sequencer_table.item(row, col)
                row_data.append(item.text() if item else '')
            data.append(row_data)
        np.savetxt(seq_file_path, np.array(data), delimiter='\t', fmt='%s')
        print(f"Sequence saved to {seq_file_path}")

    def run_line(self):
        current_row = self.sequencer_table.currentRow()
        if current_row < 0:
            self.status_bar.showMessage("No row selected")
            return
        servo_data = []
        for col in range(1, 9):  # Columns 1-8 for servo angles
            item = self.sequencer_table.item(current_row, col)
            if item and item.text():
                servo_data.append([col, int(item.text())])
        if servo_data:
            self.serial_master.write_queue.put((42, 2, servo_data))

    def run_sequence(self):
        for row in range(self.sequencer_table.rowCount()):
            dtime_item = self.sequencer_table.item(row, 0)
            dtime = float(dtime_item.text()) if dtime_item and dtime_item.text() else 0
            servo_data = []
            for col in range(1, 9):
                item = self.sequencer_table.item(row, col)
                if item and item.text():
                    servo_data.append([col, int(item.text())])
            if servo_data:
                self.serial_master.write_queue.put((42, 2, servo_data))
            time.sleep(dtime / 1000)  # dTime in ms

    def show_context_menu(self, position):
        context_menu = QMenu(self)
        insert_action = context_menu.addAction("Insert Line")
        delete_action = context_menu.addAction("Delete Line")
        insert_action.triggered.connect(self.insert_line)
        delete_action.triggered.connect(self.delete_line)
        context_menu.exec(self.register_table.viewport().mapToGlobal(position))

    def insert_line(self):
        current_row = self.register_table.currentRow()
        self.register_table.insertRow(current_row + 1)
        self.table_changed = True

    def delete_line(self):
        current_row = self.register_table.currentRow()
        if current_row >= 0:
            self.register_table.removeRow(current_row)
            self.table_changed = True

    def closeEvent(self, event):
        if self.table_changed:
            reply = QMessageBox.question(self, 'Save Changes', 'The register table has been modified. Do you want to save your changes?',
                                         QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel, QMessageBox.Cancel)
            if reply == QMessageBox.Yes:
                self.save_table_content()
                event.accept()
            elif reply == QMessageBox.No:
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()
        self.settings.setValue("size", self.size())
        self.settings.setValue("pos", self.pos())
        self.serial_master.exit_thread()
        super().closeEvent(event)

    def populate_register_table(self):
        self.register_table = QTableWidget()
        self.register_table.setColumnCount(10)
        self.register_table.setHorizontalHeaderLabels(["DEC", "HEX", "Description", "Size", "Default", "Memory", "Access", "Min", "Max", "Actual"])
        self.register_table.setColumnWidth(0, 30)
        self.register_table.setColumnWidth(1, 40)
        self.register_table.setColumnWidth(2, 200)
        self.register_table.setColumnWidth(3, 30)
        self.register_table.setColumnWidth(4, 50)
        self.register_table.setColumnWidth(5, 100)
        self.register_table.setColumnWidth(6, 100)
        self.register_table.setColumnWidth(7, 50)
        self.register_table.setColumnWidth(8, 50)
        self.register_table.setColumnWidth(9, 50)

        registers = np.genfromtxt(REGISTERFILE, delimiter='\t', dtype=str)
        self.register_table.setRowCount(len(registers))
        for row, register in enumerate(registers):
            for col, value in enumerate(register):
                self.register_table.setItem(row, col, QTableWidgetItem(str(value)))

        self.register_table.itemChanged.connect(self.mark_table_changed)

        register_toolbar = QToolBar("Register Table")
        register_toolbar.setMinimumWidth(750)

        toolbar_layout = QVBoxLayout()
        toolbar_widget = QWidget()
        toolbar_widget.setLayout(toolbar_layout)

        button_layout = QHBoxLayout()
        self.read_all_button = QPushButton('Read ALL')
        self.read_all_button.clicked.connect(self.read_all_registers)
        button_layout.addWidget(self.read_all_button)

        self.write_line_button = QPushButton('Write line')
        self.write_line_button.clicked.connect(self.write_register_line)
        button_layout.addWidget(self.write_line_button)

        self.read_line_button = QPushButton('Read Line')
        self.read_line_button.clicked.connect(self.read_register_line)
        button_layout.addWidget(self.read_line_button)

        self.servo_selector_register = QComboBox()
        self.servo_selector_register.addItems([f"Servo {i+1}" for i in range(8)])
        button_layout.addWidget(self.servo_selector_register)

        toolbar_layout.addLayout(button_layout)
        toolbar_layout.addWidget(self.register_table)

        register_toolbar.addWidget(toolbar_widget)
        return register_toolbar

    def mark_table_changed(self):
        self.table_changed = True

    def toggle_read_params(self):
        if self.read_params_button.isChecked():
            self.read_params_button.setStyleSheet("background-color: green")
            self.read_params_button.setText('Read Feedbacks On')
            self.serial_master.running.set()
            #self.serial_master.read_queue.put((STS_PRESENT_POSITION_L, 14, self.serial_master.servo_ids))
        else:
            self.read_params_button.setStyleSheet("background-color: blue")
            self.read_params_button.setText('Read Feedbacks Off')
            self.serial_master.running.clear()

    def read_all_registers(self):
        selected_servo = self.servo_selector_register.currentIndex() + 1
        self.serial_master.read_queue.put((0, 128, [selected_servo]))  # Assuming full register range 0-127

    def read_register_line(self):
        self.serial_master.running.clear()
        time.sleep(1)
        selected_servo = self.servo_selector_register.currentIndex() + 1
        current_row = self.register_table.currentRow()
        if current_row < 0:
            self.status_bar.showMessage("No row selected")
            return
        hex_register = int(self.register_table.item(current_row, 1).text(), 16)
        size = int(self.register_table.item(current_row, 3).text())
        self.serial_master.command_queue.put((INST_READ, hex_register, size, selected_servo))

        self.serial_master.running.set()

    def write_register_line(self):
        self.serial_master.running.clear()
        time.sleep(1)
        selected_servo = self.servo_selector_register.currentIndex() + 1
        current_row = self.register_table.currentRow()
        if current_row < 0:
            self.status_bar.showMessage("No row selected")
            return
        hex_register = int(self.register_table.item(current_row, 1).text(), 16)
        size = int(self.register_table.item(current_row, 3).text())
        access = self.register_table.item(current_row, 6).text()
        actual_value = int(self.register_table.item(current_row, 9).text())
        if "WRITE" not in access.upper():
            self.status_bar.showMessage("Register is not writable")
            return
        servo_data = [[selected_servo, actual_value]]
        self.serial_master.write_queue.put((hex_register, size, servo_data))
        self.serial_master.running.set()

    def update_servo_dropdowns(self, found_servos):
        self.servo_selector.clear()
        self.servo_selector.addItems([f"Servo {i}" for i in found_servos])
        self.servo_selector_register.clear()
        self.servo_selector_register.addItems([f"Servo {i}" for i in found_servos])

    def ping_servo(self, servo_id):
        found_servos = []
        self.serial_master.command_queue.put((INST_PING, servo_id))
        time.sleep(0.05)  # Wait for response
        if not self.serial_master.command_queue.empty():
            found_servos.append(servo_id)
        return found_servos

    def search_servos(self):
        found_servos = []
        for servo_id in range(1, 9):
            found_servos += self.ping_servo(servo_id)
        if found_servos:
            self.serial_master.servo_ids = found_servos
            self.status_bar.showMessage(f"Found servos: {found_servos}")
        else:
            self.status_bar.showMessage("No servos found")
        self.update_servo_dropdowns(found_servos)
        return found_servos

    def read_operation_mode(self):
        self.serial_master.read_queue.put((0x0B, 1, [self.servo_id]))  # Example address for mode

    def enable_buttons(self, enable):
        self.servo_selector.setEnabled(enable)
        self.angle_slider.setEnabled(enable)
        self.engage_button.setEnabled(enable)
        self.autoscale_button.setEnabled(enable)
        self.read_mode_button.setEnabled(enable)

    def load_settings(self):
        self.resize(self.settings.value("size", QSize(800, 600)))
        self.move(self.settings.value("pos", QPoint(50, 50)))

    def save_table_content(self):
        rows = self.register_table.rowCount()
        cols = self.register_table.columnCount()
        data = []
        for row in range(rows):
            row_data = []
            for col in range(cols):
                item = self.register_table.item(row, col)
                row_data.append(item.text() if item else '')
            data.append(row_data)
        np.savetxt(REGISTERFILE, np.array(data), delimiter='\t', fmt='%s')
        print(f"Table content saved to {REGISTERFILE}")

    def initGamepad(self):
        pygame.init()
        pygame.joystick.init()
        try:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            self.gamepad_timer = QTimer(self)
            self.gamepad_timer.timeout.connect(self.read_gamepad)
            self.gamepad_timer.start(100)
        except:
            print("No Controller found")

    def toggle_connection(self):
        if self.connected:
            self.stop_serial()
        else:
            self.start_serial()

    def start_serial(self):
        self.start_time = time.time()
        self.serial_master.start()
        self.connected = True
        self.connect_button.setStyleSheet("background-color: green")
        self.connect_button.setText('Disconnect')
        self.enable_buttons(True)

    def stop_serial(self):
        self.serial_master.stop()
        self.connected = False
        self.connect_button.setStyleSheet("background-color: red")
        self.connect_button.setText('Connect')
        self.enable_buttons(False)

    def send_move_command(self, direction):
        if direction == 'up':
            self.position += 100
        elif direction == 'down':
            self.position -= 100
        elif direction == 'left':
            self.position -= 50
        elif direction == 'right':
            self.position += 50
        elif direction == 'set':
            pass
        self.position = min(max(self.position, 0), 4096)
        servo_data = [[self.servo_id, self.position]]
        self.serial_master.write_queue.put((42, 2, servo_data))

    def update_angle_position(self, value):
        self.angle_label.setText(f"Angle: {value}")
        self.position = value
        self.send_move_command('set')

    def update_speed(self, value):
        self.speed_label.setText(f"Speed: {value}")
        self.speed = value
        # Speed could be written to register 0x2E if needed
        servo_data = [[self.servo_id, self.speed]]
        self.serial_master.write_queue.put((0x2E, 2, servo_data))

    def toggle_torque(self):
        if self.engage_button.isChecked():
            self.set_engage_all_servos()
        else:
            self.set_desengage_all_servos()

    def set_desengage_all_servos(self):
        self.engage_button.setStyleSheet("background-color: green")
        self.engage_button.setText('Engage')
        self.set_torque_all_servos(0)

    def set_engage_all_servos(self):
        self.engage_button.setStyleSheet("background-color: red")
        self.engage_button.setText('STOP')
        self.set_torque_all_servos(1)

    def set_torque_all_servos(self, torque_state):
        servo_data = [[i, torque_state] for i in self.serial_master.servo_ids]
        self.serial_master.write_queue.put((0x28, 1, servo_data))

    def toggle_autoscale(self):
        if self.autoscale_button.isChecked():
            self.autoscale_button.setStyleSheet("background-color: green")
            self.autoscale_button.setText('Autoscale On')
            self.enable_autoscale(True)
        else:
            self.autoscale_button.setStyleSheet("background-color: red")
            self.autoscale_button.setText('Autoscale Off')
            self.enable_autoscale(False)

    def enable_autoscale(self, enable):
        plots = [self.plot_angle, self.plot_load, self.plot_speed, self.plot_current]
        for plot in plots:
            if enable:
                plot.enableAutoRange(axis=pg.ViewBox.YAxis)
            else:
                plot.disableAutoRange(axis=pg.ViewBox.YAxis)

    def select_servo(self, index):
        self.servo_id = self.serial_master.servo_ids[index] if index < len(self.serial_master.servo_ids) else 1
        self.serial_master.servo_id = self.servo_id

    def change_servo_id(self):
        current_id = self.servo_selector.currentIndex() + 1
        new_id, ok = QInputDialog.getInt(self, "Set New ID", "Enter new Servo ID:", 1, 1, 254)
        if ok:
            success = self.serial_master.change_servo_id(current_id, new_id)
            if success:
                self.status_bar.showMessage(f"Servo ID changed from {current_id} to {new_id}")
                self.search_servos()
            else:
                self.status_bar.showMessage(f"Failed to change Servo ID from {current_id} to {new_id}")

    def update_ui(self):
        pass

    def read_gamepad(self):
        pygame.event.pump()
        hat = self.gamepad.get_hat(0)
        if hat[0] == 1:
            self.send_move_command('right')
        elif hat[0] == -1:
            self.send_move_command('left')
        if hat[1] == 1:
            self.send_move_command('up')
        elif hat[1] == -1:
            self.send_move_command('down')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ServoControlApp()
    window.show()
    sys.exit(app.exec())