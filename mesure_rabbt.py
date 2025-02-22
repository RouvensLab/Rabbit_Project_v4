import queue
import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QMainWindow, QGridLayout, QToolBar, QLineEdit, QScrollArea, QSizePolicy
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtCore import QTimer, QSettings
from PySide6.QtGui import QFont


import pyqtgraph as pg

#for threading
import threading
import sys
import numpy as np
import math

from tools.Trajectory import TrajectoryRecorder

#from Rabbit import Rabbit
from Objects.Rabbit_v3 import Rabbit
#real robot
from Real_robot.real_rabbit import Rabbit_real

def get_measuredRabbit(rabbit_type = Rabbit_real, 
                       state_types_body=["base_position", "head_orientation", "head_acceleration" ], 
                       state_types_servos=["joint_angles", "joint_velocities", "joint_torques"], 
                       trajectory_data_structure= ["base_position", "base_orientation", "base_linear_velocity", "base_angular_velocity", "joint_angles", "joint_torques", "joint_velocities", "joint_action_rate", "joint_action_acceleration"]
                       ):
    
    class MeasuredRabbit(rabbit_type):
        def __init__(self, *args, **kwargs):
            # Set up data storage with a limit to prevent excessive memory usage
            # limit = 1000
            # self.BodyData_line = deque(maxlen=limit)
            # self.ServoData_line = deque(maxlen=limit)
            # self.timeline = deque(maxlen=limit)
            # self.timeline.append(0)  # Start the timeline at 0
            self.queue = queue.Queue()
            self.last_send_time = 0  # Initialize the last send time

            # Use general Style Fusion
            QApplication.setStyle("Fusion")
            

            super().__init__(*args, **kwargs)

            # Define the data types to be collected
            self.state_types_body = state_types_body
            self.state_types_servos = state_types_servos
            self.getBodyData_func = self.create_get_informations(self.state_types_body)# format: ("joint_velocities"#, "joint_torques") ==> ("joint_velocities" separated into every servo, "joint_torques")
            self.getServoData_func = self.create_get_informations(self.state_types_servos)

            self.trajectory_recorder = None
            self.trajectory_data_structure = trajectory_data_structure
            self.getTrajectoryData_func = self.create_get_informations(self.trajectory_data_structure)
            # print("creating get functions succeededy")

        def create_seperate_Window(self):
            self.thread = threading.Thread(target=self.init_Gui)
            self.thread.start()
                

        def init_Gui(self):
            #This initialisation can be
            self.gui_app = QApplication([])  # Create a Qt application

            self.Plotter = self.create_GuiWidget()

            self.gui = SeparateWindow(self.Plotter)  # Launch a separate GUI window
            self.gui.show()  # Show the GUI window
            sys.exit(self.gui_app.exec_())

        def create_GuiWidget(self):
            self.Plotter = PlotterWidget(self.queue, Body_GraphLabels=self.state_types_body, Servos_GraphLabel=self.state_types_servos, n_Servos=8)
            #connect the trajectory recorder
            self.Plotter.TrajRecorder.connect_trajectory_recorder(self.start_recording_trajectory, self.stop_recording_trajectory)
            return self.Plotter
        
        def send_data_to_gui(self, data):
            self.queue.put(data)

        def step(self, *args, **kwargs):
            super().step(*args, **kwargs)
            # Record the data at each simulation step
            print(f"Send Data to GUI ")
            current_time = self.get_lifetime()
            if current_time - self.last_send_time >= 0.010:  # Check if 10ms have passed
                self.last_send_time = current_time
                self.send_data_to_gui([current_time, self.getBodyData_func(), self.getServoData_func()])

                if self.trajectory_recorder is not None:
                    self.trajectory_recorder.add_data(self.getTrajectoryData_func(), current_time)


        def start_recording_trajectory(self, trajectory_name):
            self.trajectory_recorder = TrajectoryRecorder()
            self.trajectory_recorder.new_trajectory(trajectory_name, self.trajectory_data_structure)

        def stop_recording_trajectory(self):
            self.trajectory_recorder.save_trajectory()
            #change the start and stop buttons color to green
            self.trajectory_recorder = None

        
        def reset(self, *args, **kwargs):
            super().reset(*args, **kwargs)
            # Clear stored data when resetting
            self.last_send_time = 0

    return MeasuredRabbit
        


# Separate window for visualizing sensor data
class SeparateWindow(QMainWindow):
    def __init__(self, Plotter):
        super().__init__()
        self.setWindowTitle("Rabbit Sensor Viewer")
        self.resize(600, 400)
        self.setCentralWidget(Plotter)


# Main plotting widget to visualize data
class PlotterWidget(QWidget):
    def __init__(self, queue: queue.Queue, Body_GraphLabels, Servos_GraphLabel, n_Servos):
        super().__init__()
        self.load_window_settings()
        self.queue = queue

        self.Body_GraphLabels = Body_GraphLabels
        self.Servos_GraphLabel = Servos_GraphLabel

        self.main_layout = QVBoxLayout()

        self.TrajRecorder = Recorder()
        self.main_layout.addWidget(self.TrajRecorder)

        

        self.last_time = 0

        # Create a scroll area
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)

        self.i = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(10)  # Check the queue every 100 ms

       


        # Widget for managing servo checkboxes to toggle visibility
        class ServoCheckboxesWidget(QWidget):
            """A widget that shows a list with checkboxes for every Servo
            So wether the ServoData should be visible or not.
            """
            def __init__(self, Servo_Names, Servos_id_change_func):
                super().__init__()
                self.main_layout = QVBoxLayout()
                for index, name in enumerate(Servo_Names):
                    servo_checkbox = QCheckBox(name, self)
                    servo_checkbox.setChecked(True)
                    servo_checkbox.toggled.connect(lambda checked, index=index: Servos_id_change_func(index, checked))
                    self.main_layout.addWidget(servo_checkbox)
                self.setLayout(self.main_layout)

        # ...existing code...

        # Create Graphics Layout Widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.plot_widget.setFixedHeight(2000)  # Set fixed height for the plot widget
        self.plot = list(None for _ in range(len(Body_GraphLabels)+len(Servos_GraphLabel)+10))

        # Auto scale all y axis AUTOSCALE
      

        #print(self.plot)

        # Setup body graphs
        self.body_labels = ["x", "y", "z"]
        self.curves_body, last_plot, last_row = self.create_graphs(Body_GraphLabels, body_labels=self.body_labels)
        # Setup servo graphs
        self.servo_labels = ["Servo 1", "Servo 2", "Servo 3", "Servo 4", "Servo 5", "Servo 6", "Servo 7", "Servo 8"]
        self.curves_servos, last_plot, last_row = self.create_graphs(Servos_GraphLabel, last_plot=last_plot, last_row=last_row, body_labels=self.servo_labels)
        last_plot.getAxis('bottom').setStyle(showValues=True)
        last_plot.setLabel("bottom", "Time [s]")
        
        #self.add_Vert_cursor(last_plot)
       
        #create limited arrays for the data.
        self.time_size_limit = 100
        self.curves_array_body = np.zeros((len(Body_GraphLabels), self.time_size_limit, len(self.body_labels)), dtype=np.float16)
        self.curves_array_servos = np.zeros((len(Servos_GraphLabel), self.time_size_limit, len(self.servo_labels)), dtype=np.float16)
        self.time_array = np.zeros(self.time_size_limit, dtype=np.float16)
        #print(self.curves_array_body, self.curves_array_body.shape)

        self.scroll_area.setWidget(self.scroll_content)
        self.scroll_layout.addWidget(self.plot_widget)

        # Create labels for cursor values
        self.label_a = QLabel("Cursor A: 0.00")
        self.label_b = QLabel("Cursor B: 0.00")
        self.label_a.setFont(QFont("Arial", 10))
        self.label_b.setFont(QFont("Arial", 10))
        self.main_layout.addWidget(self.label_a)
        self.main_layout.addWidget(self.label_b)



        self.main_layout.addWidget(self.scroll_area)
        self.setLayout(self.main_layout)


    def add_Vert_cursor(self, last_plot):
        # Add two vertical cursors so we can read the values of each curve out
        self.vLine = pg.InfiniteLine(angle=90, movable=False)
        #self.hLine = pg.InfiniteLine(angle=0, movable=False)
        last_plot.addItem(self.vLine, ignoreBounds=True)
        #last_plot.addItem(self.hLine, ignoreBounds=True)
        self.label = pg.TextItem(anchor=(-1, 10))
        last_plot.addItem(self.label)

        # Connect the mouse move event to update the cursor position
        self.plot_widget.scene().sigMouseMoved.connect(self.mouse_moved)
        self.proxy = pg.SignalProxy(self.plot_widget.scene().sigMouseMoved, rateLimit=60, slot=self.update_crosshair)
    
    

    def update_labels(self):
        # Get cursor positions
        pos_a = self.cursor_a.value()
        pos_b = self.cursor_b.value()

        # Find nearest data points
        idx_a = np.abs(self.time - pos_a).argmin()
        idx_b = np.abs(self.time - pos_b).argmin()

        # Update labels with cursor positions and values
        value_a = self.data[idx_a]
        value_b = self.data[idx_b]
        
        self.label_a.setText(f"Cursor A (Red): x={pos_a:.2f}, y={value_a:.2f}")
        self.label_b.setText(f"Cursor B (Green): x={pos_b:.2f}, y={value_b:.2f}")

   

       

    def create_graphs(self, GraphLabels, last_plot=None, last_row=0, body_labels=["x", "y", "z"]):
        # Create a dictionary to store curves for each graph
        # Setup body graphs
        colors = [pg.intColor(i, hues=len(body_labels)) for i in range(len(body_labels))]
        # Adjust the layout of the plot widget
        startLen = last_row
        curves = []

        for i, graph_name in enumerate(GraphLabels):
            # Create a plot for every graph
            self.plot[i+1+startLen] = self.plot_widget.addPlot(row=i+1+startLen, col=0)
            legend = self.plot[i+1+startLen].addLegend(loc=(1.0,1.0))  # Add legend to the plot
            #legend.setParentItem(self.plot[i+1+startLen].vb)
            legend.anchor((0, 0), (0, 0), offset=(0, 0))  # Anchor and offset legend to the top left corner
            legend.setColumnCount(8)
            
            # Add Legends on top of each plot
            self.plot_widget.ci.layout.setRowStretchFactor(i+1+startLen, 1)
            self.plot[i+1+startLen].getAxis('bottom').setStyle(showValues=False)
            
            # Set plot title
            self.plot[i+1+startLen].setTitle(graph_name)
            self.plot[i+1+startLen].showGrid(x=True, y=True)
            self.plot[i+1+startLen].getAxis("left").setWidth(70)  # Increase left axis width for space
            self.plot[i+1+startLen].getAxis("right").setWidth(50)
            # Labels
            self.plot[i+1+startLen].setLabel("left", "Value")
           

            for item in legend.items:
                item[1].mouseClickEvent = lambda ev: self.toggle_visibility(curve)

            if last_plot:
                # Link x Axis
                self.plot[i+1+startLen].setXLink(last_plot)
                self.plot_widget.nextRow()
            curves.append([])
            for j, curve in enumerate(body_labels):
                curve = self.plot[i+1+startLen].plot(pen=pg.mkPen(color=colors[j]), name=f"{curve}")
                curves[i].append(curve)
            last_plot = self.plot[i+1+startLen]
            last_row = i+1+startLen
        return curves, last_plot, last_row



    def update_add_Plots(self, time, BodyData, ServoData):
        #print("Update Plots")
        # print(BodyData)
        # print(ServoData)
        if time - self.last_time > 0:
            self.last_time = time
            self.time_array = np.concatenate([self.time_array[1:], np.array([time])])
            #add the body data to the curves_array_body
            #bodyData = [np.array([x, y, z]), np.array([x, y, z]), ...] for every graph
            #self.curves_array_body = np.array([np.array([[x, y, z], [x, y, z], ...for every time step]), np.array([[x, y, z], [x, y, z], ...for every time step]), ...for every graph type])
            #add a new dimension to the array
            BodyData_array = np.array(BodyData)
            # print("Bevore", BodyData_array.shape, BodyData_array)
            BodyData_array = np.expand_dims(BodyData_array, axis=1)
            # print("After expansion", BodyData_array.shape, BodyData_array)
            new_curves_array_body = self.curves_array_body[:, 1:, :]
            # print("wanted for concatenation", new_curves_array_body.shape)
            # Ensure the dimensions match before concatenation
            self.curves_array_body = np.concatenate([new_curves_array_body, BodyData_array], axis=1)
            #for the servo data
            ServoData_array = np.array(ServoData)
            #print("Bevore", ServoData_array, ServoData_array.shape)
            ServoData_array = np.expand_dims(ServoData_array, axis=1)
            #print("After expansion", ServoData_array, ServoData_array.shape)
            new_curves_array_servos = self.curves_array_servos[:, 1:, :]
            #print("wanted for concatenation", new_curves_array_servos.shape)
            self.curves_array_servos = np.concatenate([new_curves_array_servos, ServoData_array], axis=1)

        else:
            self.last_time = time
            self.time_array = np.zeros_like(self.time_array)
            self.curves_array_body = np.zeros((len(self.Body_GraphLabels), self.time_size_limit, len(self.body_labels)), dtype=np.float16)
            self.curves_array_servos = np.zeros((len(self.Servos_GraphLabel), self.time_size_limit, len(self.servo_labels)), dtype=np.float16)


        # print(self.curves_array_body, self.curves_array_body.shape)
        for graph_id, graph_curves in enumerate(self.curves_body):
            for graph_curve_id, curve in enumerate(graph_curves):
                curve_array = self.curves_array_body[graph_id, :, graph_curve_id]
                # print("Curve array: ", curve_array, curve_array.shape)
                # print("Time array: ", self.time_array, self.time_array.shape)
                curve.setData(self.time_array, curve_array)

        for graph_id, graph_curves in enumerate(self.curves_servos):
            for graph_curve_id, curve in enumerate(graph_curves):
                curve_array = self.curves_array_servos[graph_id, :, graph_curve_id]
                curve.setData(self.time_array, curve_array)

    def update_add_ServoPlots(self, time, ServoData):
        if time - self.last_time > 0:
            self.last_time = time
        else:
            self.last_time = time

            for graph_id, dataType_array in enumerate(ServoData):
                self.curves[graph_id].clear()

        for graph_id, dataType_array in enumerate(ServoData):
            self.curves_array_servos[graph_id] = np.concatenate([self.curves_array_servos[graph_id], np.array(dataType_array)], axis=1)
            for servo_id in range(len(dataType_array)):
                self.curves[self.Servos_GraphLabel[graph_id]][servo_id].setData(self.time_array, self.curves_array_servos[graph_id][servo_id])

    # ...existing code...
    
    def run(self):
        # self.i += 1
        # rand_data = lambda: math.sin(self.i/10) + np.random.normal(0, 0.1)
        # self.queue.put([self.i, 
        #             [
        #             [rand_data(), rand_data(), rand_data()], 
        #             [rand_data(), rand_data(), rand_data()], 
        #             [rand_data(), rand_data(), rand_data()]
        #             ], 
        #             [
        #             [rand_data(), rand_data(), rand_data(), rand_data(), rand_data(), rand_data(), rand_data(), rand_data()], 
        #             ]])
        while not self.queue.empty():
        #plotter.update_add_Plots(0, [[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            try:
                data_new = self.queue.get()
                #self.time, self.BodyData, self.ServoData = self.queue.get()
                self.time, self.BodyData, self.ServoData = data_new
                #print(self.time)
                #print(self.BodyData)
                #print(self.ServoData)
                self.update_add_Plots(self.time, self.BodyData, self.ServoData)
                #self.update_add_ServoPlots(self.time, self.ServoData)
            except queue.Empty:
                pass


    
    def toggle_visibility(self, curve):
        curve.setVisible(not curve.isVisible())
        


    def closeEvent(self, event):
        self.save_window_settings()
        super().closeEvent(event)

    def save_window_settings(self):
        settings = QSettings("YourCompany", "RabbitProject")
        settings.setValue("geometry", self.saveGeometry())

    def load_window_settings(self):
        settings = QSettings("YourCompany", "RabbitProject")
        geometry = settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)

class Connection:
    """This class makes connections between functions"""
    def __init__(self, rabbit):
        self.rabbit = rabbit
        self.rabbit.create_seperate_Window()

class Recorder(QToolBar):
    start_recording_signal = Signal()
    stop_recording_signal = Signal()
    def __init__(self):
        super().__init__()
        self.initUI()


    def initUI(self):
        # Toolbar for recording trajectories
        self.record_trajectory_action = self.addAction("Start Recording Trajectory")
        self.record_trajectory_action.triggered.connect(self.start_recording_trajectory)
        #to stop the recording
        self.stop_recording_action = self.addAction("Stop Recording Trajectory")
        self.stop_recording_action.triggered.connect(self.stop_recording_trajectory)
        #input for the name of the trajectory
        self.trajectory_name_input = QLineEdit(text="Trajectory1")
        self.addWidget(self.trajectory_name_input)

    def connect_trajectory_recorder(self, start_recording_action, stop_recording_action):
        self.record_trajectory_func = start_recording_action
        self.stop_recording_func = stop_recording_action
    
    # recording trajectorys
    @Slot()
    def start_recording_trajectory(self):
        self.start_recording_signal.emit()
        if hasattr(self, "record_trajectory_func"):
            self.record_trajectory_func(self.trajectory_name_input.text())
        #change the start and stop buttons color to green
        self.record_trajectory_action.setEnabled(False)
        self.stop_recording_action.setEnabled(True)

    @Slot()
    def stop_recording_trajectory(self):
        self.stop_recording_signal.emit()
        if hasattr(self, "stop_recording_func"):
            self.stop_recording_func()
        #change the start and stop buttons color to green
        self.record_trajectory_action.setEnabled(True)
        self.stop_recording_action.setEnabled(False)

    

    def get_trajectory_name(self):
        return self.trajectory_name_input.text()

if __name__ == "__main__":
    import math
    app = QApplication([])
    queue_ = queue.Queue()
    plotter = PlotterWidget(queue_, Body_GraphLabels=["base_position", "head_orientation", "head_acceleration"], Servos_GraphLabel=["joint_torques"], n_Servos=8)
    
    plotter.show()

    sys.exit(app.exec())


