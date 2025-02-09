import queue
import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QMainWindow, QGridLayout, QToolBar, QLineEdit, QScrollArea, QSizePolicy
)
from PySide6.QtCore import Signal, Slot
from PySide6.QtCore import QTimer
import pyqtgraph as pg

#for threading
import threading

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

            

            super().__init__(*args, **kwargs)

            # Define the data types to be collected
            self.state_types_body = state_types_body
            self.state_types_servos = state_types_servos
            self.getBodyData_func = self.create_get_informations(self.state_types_body)# format: ("joint_velocities"#, "joint_torques") ==> ("joint_velocities" separated into every servo, "joint_torques")
            self.getServoData_func = self.create_get_informations(self.state_types_servos)

            self.trajectory_recorder = None
            self.trajectory_data_structure = trajectory_data_structure
            self.getTrajectoryData_func = self.create_get_informations(self.trajectory_data_structure)
            print("creating get functions succeeded")

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
            if current_time - self.last_send_time >= 0.05:  # Check if 10ms have passed
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
    def __init__(self, queue, Body_GraphLabels, Servos_GraphLabel, n_Servos):
        super().__init__()
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

        # Section for visualizing body data
        self.scroll_layout.addWidget(QLabel("--------Body Data--------"))
        self.BodyGraph_layout = QGridLayout()
        self.scroll_layout.addLayout(self.BodyGraph_layout)

        # Separator between sections
        self.scroll_layout.addWidget(QLabel("--------Servo Data--------"))

        # Section for servo data visualization
        self.ServoSplit_layout = QHBoxLayout()
        self.ServosGraph_layout = QGridLayout()

        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(10)  # Check the queue every 100 ms


        class ServoGraphWidget(QWidget):
            """A Widget that shows all kind of Data of Servos e.g. strength, current, velocity or position 
            Specialties:
            When clicked on the Graph, it opens as a big matplotlib window, that can be edited, saved, ...
            """

            def __init__(self, Graph_title, n_Servos, y_range=None):
                super().__init__()
                self.layout = QVBoxLayout()
                self.title = QLabel(Graph_title)
                self.layout.addWidget(self.title)

                self.selected_servos = [i for i in range(n_Servos)]
                self.data = [[] for _ in range(n_Servos)]
                self.time_line = []
                self.plot_widget = pg.PlotWidget()
                self.layout.addWidget(self.plot_widget)
                self.setLayout(self.layout)

                self.n_Servos = n_Servos
                self.colors = [pg.intColor(i, hues=n_Servos) for i in range(n_Servos)]
                self.plot_widget.showGrid(x=True, y=True)
                self.plot_widget.setLabel('left', 'Value')
                self.plot_widget.setLabel('bottom', 'Time (s)')
                self.plot_widget.setTitle(Graph_title)
                if y_range is not None:
                    self.plot_widget.setYRange(*y_range)

                self.plot_widget.addLegend()
                self.curves = [self.plot_widget.plot([], [], pen=self.colors[i], name=f"Servo {i}") for i in range(n_Servos)] 

                # Set size policy to cover 30% of the main widget size
                self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
                self.setMinimumHeight(200)

            def add_data(self, time, data):
                self.time_line.append(time)
                for servo_id, d in enumerate(data):
                    self.data[servo_id].append(d)
                    self.curves[servo_id].setData(self.time_line, self.data[servo_id])  # Update plot without clearing

            def change_servo_visibility(self, idf):
                if idf in self.selected_servos:
                    self.selected_servos.remove(idf)
                else:
                    self.selected_servos.append(idf)
                self.update_visibility()

            def update_visibility(self):
                for i, curve in enumerate(self.curves):
                    curve.setVisible(i in self.selected_servos)

            def clear(self):
                self.time_line = []
                self.data = [[] for _ in range(self.n_Servos)]
                for curve in self.curves:
                    curve.setData([], [])


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

        # Setup all body graphs
        self.body_graphs = []
        for graph_name in Body_GraphLabels:
            body_graph = ServoGraphWidget(Graph_title=graph_name, n_Servos=3)
            self.body_graphs.append(body_graph)
            self.BodyGraph_layout.addWidget(body_graph)
        

        # Setup all servo graphs and their visibility toggle functions
        self.servo_graphs = []
        self.servo_id_visibility_func = []
        for graph_name in Servos_GraphLabel:
            servo_graph = ServoGraphWidget(Graph_title=graph_name, n_Servos=n_Servos)
            self.servo_graphs.append(servo_graph)
            self.servo_id_visibility_func.append(servo_graph.change_servo_visibility)
            self.ServosGraph_layout.addWidget(servo_graph)

        def change_vis_servo_id(id, checked):
            for func in self.servo_id_visibility_func:
                func(id)

        # Add servo checkboxes for selecting which servos to display
        self.servo_checkboxes_widget = ServoCheckboxesWidget([f"Servo {i+1}" for i in range(n_Servos)], change_vis_servo_id)
        self.ServoSplit_layout.addWidget(self.servo_checkboxes_widget)

        self.ServoSplit_layout.addLayout(self.ServosGraph_layout)

        self.scroll_layout.addLayout(self.ServoSplit_layout)
        self.scroll_area.setWidget(self.scroll_content)

        self.main_layout.addWidget(self.scroll_area)
        self.setLayout(self.main_layout)

    def update_add_BodyPlots(self, time, BodyData):
        if time - self.last_time > 0:
            self.last_time = time
        else:
            self.last_time = time

            for graph_id, dataType_array in enumerate(BodyData):
                self.body_graphs[graph_id].clear()

        for graph_id, dataType_array in enumerate(BodyData):
            self.body_graphs[graph_id].add_data(time, dataType_array)

    def update_add_ServoPlots(self, time, ServoData):
        if time - self.last_time > 0:
            self.last_time = time
        else:
            self.last_time = time

            for graph_id, dataType_array in enumerate(ServoData):
                #print(f"{graph_id} ServoData {dataType_array}")
                self.servo_graphs[graph_id].clear()

        for graph_id, dataType_array in enumerate(ServoData):
            #print(f"{graph_id} ServoData {dataType_array}")
            self.servo_graphs[graph_id].add_data(time, dataType_array)

    
    def run(self):
        while not self.queue.empty():
            try:
                self.time, self.BodyData, self.ServoData = self.queue.get_nowait()
                self.update_add_BodyPlots(self.time, self.BodyData)
                self.update_add_ServoPlots(self.time, self.ServoData)
            except queue.Empty:
                pass
    
    
        


    def closeEvent(self, event):
        # Stop the timer when the window is closed
        self.timer.stop()
        event.accept()

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
    app = QApplication([])
    from Objects.Rabbit_v3 import Rabbit
    rabbit = get_measuredRabbit(Rabbit)()
    app.exec_()
 
    

