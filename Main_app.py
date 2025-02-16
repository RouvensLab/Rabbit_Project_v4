import sys
import time
import ast
import threading
import traceback
import numpy as np
from PySide6.QtWidgets import (
    QApplication,  QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,QStatusBar,
    QPushButton, QHeaderView, QMessageBox, QSlider, QFileDialog, QLabel, QCheckBox, QComboBox, QMainWindow, QTabWidget
)
from PySide6.QtCore import Qt,QSettings

from RL_Agent_Env import RL_Env
from stable_baselines3 import SAC
from tools.Controler import ControlInput


class ExtendedTableWidget(QTableWidget):
    """Table with extendable rows."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setColumnCount(9) # Time + 8 action values
        self.setHorizontalHeaderLabels(['Time', 'Action1', 'Action2', 'Action3', 'Action4', 'Action5', 'Action6', 'Action7', 'Action8'])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def get_dict(self):
        #get the whole data from the table and transform it to the timetable dictionary
        timetable = {}
        for row in range(self.rowCount()):
            time = float(self.item(row, 0).text())
            actions = [float(self.item(row, col).text()) for col in range(1, self.columnCount())]
            timetable[time] = actions
        return timetable
    
    def load_action_timetable(self, timetable):
        self.setRowCount(len(timetable))

        for row, (time, actions) in enumerate(timetable.items()):
            self.setItem(row, 0, QTableWidgetItem(str(time)))
            for col, action_value in enumerate(actions):
                self.setItem(row, col + 1, QTableWidgetItem(str(action_value)))

    def add_new_row(self):
        # Insert a new row at the current selected position, or at the end if no selection
        selected_row = self.currentRow()
        if selected_row == -1:  # If no row selected
            selected_row = None
        self.insert_empty_row(selected_row)
    
    def remove_selected_row(self):
        # Remove the selected row
        selected_row = self.currentRow()
        if selected_row != -1:
            self.removeRow(selected_row)

    def insert_empty_row(self, position=None):
        """Inserts a new empty row at the given position (or at the end if no position is specified)."""
        if position is None:
            # If no position specified, append a new row at the end
            position = self.rowCount()
        
        self.insertRow(position)
        # Insert default empty values (e.g., 0.0 for float) in the new row
        self.setItem(position, 0, QTableWidgetItem(str(0.0)))  # Time
        for col in range(1, self.columnCount()):
            self.setItem(position, col, QTableWidgetItem(str(0.0)))  # Default action values

    def highlight_active_row(self, row):
        # Highlight the active row
        for col in range(self.columnCount()):
            self.item(row, col).setBackground(Qt.green)
        # Unhighlight the other rows
        for other_row in range(self.rowCount()):
            if other_row != row:
                for col in range(self.columnCount()):
                    self.item(other_row, col).setBackground(Qt.white)

class EnvParameterEditor(QWidget):
    def __init__(self, env_param_kwargs, restart_callback):
        super().__init__()
        self.initUI(env_param_kwargs, restart_callback)
    def initUI(self, env_param_kwargs, restart_callback):
        self.restart_callback = restart_callback
        
        # Create a layout for the editor
        layout = QVBoxLayout()
        # shows a table with the current environment parameters
        self.table_layout = QVBoxLayout()
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(['Parameter', 'Value'])
        self.table_layout.addWidget(self.table)
        self.load_env_parameters(env_param_kwargs)
        layout.addLayout(self.table_layout)

        # Add button to save the parameterchanges and restart/reopen the environment
        button_layout = QHBoxLayout()
        open_param_button = QPushButton("Open Environment", self)
        open_param_button.clicked.connect(self.open_Env_param_from_file)
        button_layout.addWidget(open_param_button)
        self.save_button = QPushButton("Save and Restart", self)
        self.save_button.clicked.connect(self.save_and_restart)
        button_layout.addWidget(self.save_button)
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def load_env_parameters(self, env_param_kwargs):
        # Load the environment parameters into the table
        self.table.setRowCount(len(env_param_kwargs))
        for row, (param_name, param_value) in enumerate(env_param_kwargs.items()):
            self.table.setItem(row, 0, QTableWidgetItem(param_name))
            if isinstance(param_value, (str)):
                self.table.setItem(row, 1, QTableWidgetItem(str(f"'{param_value}'")))
            else:
                self.table.setItem(row, 1, QTableWidgetItem(str(param_value)))
    def get_env_parameters(self):
        # Get the environment parameters from the table
        env_param_kwargs = {}
        for row in range(self.table.rowCount()):
            param_name = self.table.item(row, 0).text()
            param_value = self.table.item(row, 1).text()
            # Try to convert the parameter value to a float or int if possible or as a json string
            # print(param_value)
            env_param_kwargs[param_name] = ast.literal_eval(param_value)
        print(env_param_kwargs)
        return env_param_kwargs

    def open_Env_param_from_file(self):
        # Opens a QFileDialog to select new environment parameters
        file_dialog = QFileDialog(self, "Open Environment Parameters", r"Models", "Model Files (info.txt)")
        if file_dialog.exec():
            file_path = file_dialog.selectedFiles()[0]
            # Load the environment parameters from the selected txt file
            with open(file_path, "r") as f:
                parameter_text = f.read()
            #find the line that begins with "Env Parameters: " afterwards is a dictionary with the environment parameters
            env_param_str = parameter_text.split("Env Parameters: ")[1]
            env_param_str = env_param_str.split("\n")[0]
            print(env_param_str)
            #convert the string to a dictionary
            try:
                env_param_kwargs = ast.literal_eval(env_param_str)
            except:
                QMessageBox.warning(self, "Error", "Could not load the environment parameters from the file!")
                return

            self.load_env_parameters(env_param_kwargs)
            QMessageBox.information(self, "Parameters Loaded", "Environment parameters loaded successfully!")
        else:
            QMessageBox.warning(self, "Parameters Not Loaded", "No parameters loaded!")

    def save_and_restart(self):
        self.restart_callback(self.get_env_parameters())
    

class ActionTimetableEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Action Timetable Editor")
        self.resize(600, 400)

        # Initialize QSettings
        self.settings = QSettings("MyCompany", "ActionTimetableEditor")


        # Restore geometry and state
        self.restore_geometry()

        self.def_env_param_kwargs = {
        "ModelType": "SAC",
        "rewards_type": ["Disney_Imitation"],
        "observation_type_stacked": [],
        "observation_type_solo": ["phase_signal"],
        "terrain_type": "flat",
        "recorded_movement_file_path_dic": {"PushSprint_v1": 5}, 
        "restriction_2D": False
    }
        self.env_param_kwargs = self.def_env_param_kwargs
        self.env_startup_parms = {"render_mode": "human", "real_robot": False, "gui": True, "RobotType":"Rabbit_v3_mesured"}
        self.env = RL_Env(**self.env_startup_parms, **self.env_param_kwargs)
        self.env.simulation.hung = True
        self.RabbitMesure_widget = self.env.simulation.rabbit.create_GuiWidget()
        #Implement that the rabbit unpause the simulation, when the recording is started
        self.RabbitMesure_widget.TrajRecorder.start_recording_signal.connect(lambda: self.pause_unpause_simulation(False))
        self.RabbitMesure_widget.TrajRecorder.stop_recording_signal.connect(lambda: self.pause_unpause_simulation(True))

        self.manual_exp = ManualExpert(sim_freq=5)
        self.loaded_model = None

        self.env_param_editor = None

        self.control_input = None


        self.env_pause = False
        self.simulation_thread = None
        self.initUI()
        self.simulation_thread = threading.Thread(target=self.run_simulation_with_error_handling)
        self.simulation_thread.start()
        #self.simulation_thread.join()

    def run_simulation_with_error_handling(self):
        try:
            self.run_simulation()
        except Exception as e:
            print(f"Exception in simulation thread: {e}")
            traceback.print_exc()

    def restore_geometry(self):
        """Restore the window's geometry and state from QSettings."""
        geometry = self.settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)


    def closeEvent(self, event):
        self.settings.setValue("geometry", self.saveGeometry())
        self.end_thread = True
        self.simulation_thread.join()
        if self.env:
            self.env.close()
            self.env = None
        QApplication.instance().quit()
        event.accept()

   
    def initUI(self):        
        #Create a tabwidget with two tabs (Control and Observation)
        tab_widget = QTabWidget()
        self.setCentralWidget(tab_widget)

        # Control Tab
        control_tab = QWidget()
        tab_widget.addTab(control_tab, "Control")

        # Observation Tab
        observation_tab = QWidget()
        tab_widget.addTab(observation_tab, "Observation")


        control_layout = QVBoxLayout()
        layout1 = QHBoxLayout()
        #Bar that shows the active model. Also you can select a model file.
        bar_layout = QHBoxLayout()
        #load environment parameter button
        self.env_param_button = QPushButton("Open Environment Parameter Editor", self)
        self.env_param_button.clicked.connect(self.open_Env_param)
        bar_layout.addWidget(self.env_param_button)

        # Load model button
        self.model_loadButton = QPushButton("No Model Loaded", self)
        self.model_loadButton.clicked.connect(self.open_model_file)
        bar_layout.addWidget(self.model_loadButton)

        # Name of loaded model
        self.model_name_label = QLabel(self)
        self.model_name_label.setText("Model Name: Manual Expert")
        bar_layout.addWidget(self.model_name_label)

        # Activation checkbox for model
        self.button_mod_active = QCheckBox("Model Active", self)
        bar_layout.addWidget(self.button_mod_active)

        # drop down box that shows if you can control the robot with the arrow keys
        self.controlMode_active = QComboBox(self)
        self.controlMode_active.addItems(["Control GoalVector", "Body Control", "Auto"])
        self.controlMode_active.setCurrentIndex(2)
        self.controlMode_active.currentIndexChanged.connect(self.update_status)
        self.controlMode_active.currentIndexChanged.connect(self.changeControlMode)
        bar_layout.addWidget(self.controlMode_active)

        # autoreset checkbox
        self.auto_reset_checkbox = QCheckBox("Auto Reset", self)
        self.auto_reset_checkbox.setChecked(True)
        self.auto_reset_checkbox.toggled.connect(self.update_status)
        bar_layout.addWidget(self.auto_reset_checkbox)

        # Real robot Box
        real_robot_layout = QVBoxLayout()
        # Real robot checkbox
        self.button_real_robot = QCheckBox("Real Robot", self)
        self.button_real_robot.clicked.connect(self.toggle_real_robot)
        real_robot_layout.addWidget(self.button_real_robot)
        # checkbox for savety mode or direct mode
        self.button_real_robot_savety = QCheckBox("Savety Mode", self)
        self.button_real_robot_savety.toggled.connect(self.update_status)
        real_robot_layout.addWidget(self.button_real_robot_savety)
        
        bar_layout.addLayout(real_robot_layout)
        control_layout.addLayout(bar_layout)

        # Table for action timetable
        self.table_layout = QVBoxLayout()
        self.table = ExtendedTableWidget(self)
        self.table.load_action_timetable(self.manual_exp.action_timetable)
        self.table.cellChanged.connect(self.on_table_cell_changed)
        self.table_layout.addWidget(self.table)

        layout1.addLayout(self.table_layout, stretch=3)

        # Add Buttons
        button_layout = QVBoxLayout()

        #add new row button
        self.add_row_button = QPushButton("Add New Row", self)
        self.add_row_button.clicked.connect(self.table.add_new_row)
        button_layout.addWidget(self.add_row_button)
        #remove row button
        self.remove_row_button = QPushButton("Remove Selected Row", self)
        self.remove_row_button.clicked.connect(self.table.remove_selected_row)
        button_layout.addWidget(self.remove_row_button)

        self.copy_button = QPushButton("Copy Timetable to Clipboard", self)
        self.copy_button.clicked.connect(self.copy_to_clipboard)
        button_layout.addWidget(self.copy_button)

        #hanging or not hanging button
        self.hung_button = QPushButton("hung", self)
        self.hung_button.clicked.connect(self.hung_change)
        button_layout.addWidget(self.hung_button)

        #change camera focus of the robot
        self.camera_focus_button = QPushButton("Camera Focus", self)
        self.camera_focus_button.clicked.connect(self.camera_focus_change)
        button_layout.addWidget(self.camera_focus_button)

        # Pause/Unpause button
        self.pause_button = QPushButton("Pause/Unpause Simulation", self)
        self.pause_button.clicked.connect(lambda: self.pause_unpause_simulation())
        button_layout.addWidget(self.pause_button)

        # Restart button
        self.restart_button = QPushButton("Restart Simulation", self)
        self.restart_button.clicked.connect(self.reset_simulation)
        button_layout.addWidget(self.restart_button)

        #add a speed slider
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(10)
        self.speed_slider.setValue(0)
        button_layout.addWidget(self.speed_slider)

        layout1.addLayout(button_layout, stretch=1)
        control_layout.addLayout(layout1, stretch=1)
        
        # Status Bar
        self.status_bar = QStatusBar(self)
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # Set the control_layout as the main layout of the Control tab
        control_tab.setLayout(control_layout)


        # Observation Tab
        #add to the observation tab
        observation_layout = QVBoxLayout()
        observation_layout.addWidget(self.RabbitMesure_widget)
        observation_tab.setLayout(observation_layout)
        

    def update_status(self):
        """Update the status bar with current settings."""
        status_message = (
            f"Model Active: {'Yes' if self.button_mod_active.isChecked() else 'No'}, "
            f"active ControlMode: {self.controlMode_active.currentText()}, "
            f"Auto Reset: {'Enabled' if self.auto_reset_checkbox.isChecked() else 'Disabled'}, "
            f"Savety Mode: {'On' if self.button_real_robot_savety.isChecked() else 'Off'}, "
            f"Speed Slider: {self.speed_slider.value()}"
        )
        self.status_bar.showMessage(status_message)

    def hung_change(self):
        self.env.simulation.hung = not self.env.simulation.hung

    def camera_focus_change(self):
        self.env.simulation.toogle_focus()

    def savety_mode_change(self, checked):
        self.env.robot_precise_mode = checked


    def on_table_cell_changed(self):
        # Update the action timetable when a cell is changed
        #update the timetable
        self.manual_exp.action_timetable = self.table.get_dict()

    def reset_simulation(self):        
        self.env.reset()

        #QMessageBox.information(self, "Restart", "Simulation restarted!")

    def pause_unpause_simulation(self, state=None):
        if state is None:
            self.env_pause = not self.env_pause
        else:
            self.env_pause = state

    def read_controler_vector(self):
        pass

    def changeControlMode(self):
        if self.controlMode_active.currentText() == "Body Control":
            self.control_input = ControlInput(self)
            self.env_param_kwargs = self.def_env_param_kwargs
            print("Make shure that the env_parameters do not contain the 'multi_task_goal' in the rewards_type list!")
        elif self.controlMode_active.currentText() == "Auto":
            self.control_input = None
            self.env_param_kwargs["control_goalPos"] = None
            self.env_param_kwargs["rewards_type"] = ["stability"]
            print("Make shure that the env_parameters do not contain the 'multi_task_goal' in the rewards_type list!")
        else:
            self.control_input = ControlInput(self)
            self.env_param_kwargs["control_goalPos"] = self.control_input.get_left_JoystickVector
            self.env_param_kwargs["rewards_type"] = ["multi_task_goal"]
            print("Make shure that the env_parameters do contain the 'multi_task_goal' in the rewards_type list!")

        self.open_Env(self.env_param_kwargs)

    def open_Env(self, env_param_kwargs):
        self.env_param_kwargs = env_param_kwargs
        #close the current environment and the thread
        self.end_thread = True
        self.simulation_thread.join()
        if self.env:
            self.env.close()
            self.env = None

        #open a new environment
        self.env = RL_Env(**self.env_startup_parms, **self.env_param_kwargs)
        self.env.simulation.hung = False
        self.env.robot_precise_mode = self.button_real_robot_savety.isChecked()
        self.env_pause = True

        #if everything is ok, close the editor window
        if self.env_param_editor:
            self.env_param_editor.close()
            self.env_param_editor = None
            print("env_param_editor closed!")


        #open a new thread
        self.simulation_thread = None
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()
        print("Simulation started!")


        

    
    def open_Env_param(self):
        #open the environment parameter editor
        self.env_param_editor = EnvParameterEditor(self.env_param_kwargs, self.open_Env)
        self.env_param_editor.setWindowTitle("Environment Parameter Editor")
        self.env_param_editor.resize(600, 400)
        self.env_param_editor.show()

    def toggle_real_robot(self, checked):
        # Restart the simulation by wether leading or without loading the robot_env
        self.env_startup_parms["real_robot"] = checked
        self.open_Env(self.env_param_kwargs)


    def open_model_file(self):
        # Open a file dialog to select a model file
        file_dialog = QFileDialog(self, "Open Model File", r"Models", "Model Files (*.zip)")
        if file_dialog.exec():
            file_path = file_dialog.selectedFiles()[0]
            # Load the model from the selected file
            self.loaded_model = SAC.load(file_path)
            self.button_mod_active.setChecked(True)
            #change color of the model button
            self.model_loadButton.setStyleSheet("background-color: green")
            self.model_loadButton.setText("Model Loaded")
            self.model_name_label.setText(f"Model Name: {file_path.split('/')[-1]}")
            self.env_pause = True
            QMessageBox.information(self, "Model Loaded", "Model loaded successfully!")
        else:
            self.loaded_model = None
            self.button_mod_active.setChecked(False)
            self.model_loadButton.setStyleSheet("background-color: red")
            self.model_loadButton.setText("No Model Loaded")
            self.model_name_label.setText(f"Model Name: Manual Expert")
            self.env_pause = True
            QMessageBox.warning(self, "Model Not Loaded", "No model loaded!")

    def run_simulation(self):
        # This thread runs as long as there are no errors.
        # As long as the environments fit the model, everything is fine.
        self.no_error = True
        self.end_thread = False

        while not self.end_thread and self.no_error:
            obs, info = self.env.reset()
            done = False

            while not done and not self.end_thread and self.no_error:
                if not self.env_pause:
                    if self.controlMode_active.currentText() == "Body Control" and self.control_input is not None: # Body Control (Joystick) is controlling the robot
                        action = self.control_input.get_BodyPose()
                        print(action)
                    elif self.loaded_model is not None and self.button_mod_active.isChecked():# A RL-Agent is controlling the robot
                        try:
                            action, pred_info = self.loaded_model.predict(obs)
                        except Exception as e:
                            self.no_error = False
                            print(f"Error in model prediction: {e}")
                    else: #self.controlMode_active.currentText() == "Auto": # Manual Expert is controlling the robot
                        try:
                            action, state = self.manual_exp.think_and_respond(obs, None, done, self.env.simulation.rabbit.lifetime)
                            print(action)
                            # Highlight the active row
                            action_key_index = list(self.manual_exp.action_timetable.keys()).index(self.manual_exp.action_key)
                            self.table.highlight_active_row(action_key_index)
                        except Exception as e:
                            self.no_error = False
                            print(f"Error in manual response: {e}")
                            traceback.print_exc()

                    #print(action)
                    try:
                        obs, reward, terminated, truncated, info = self.env.step(action)
                        print(obs)
                        done = (terminated or truncated) and self.auto_reset_checkbox.isChecked()
                    except Exception as e:
                        self.no_error = False
                        print(f"Error in environment step: {e}")
                    

                    time.sleep(self.speed_slider.value() / 10)
                else:
                    time.sleep(0.5)

        if not self.no_error:
            # Dialog if there is an error
            print("Error", "An error occurred! Simulation stopped! Probably the Environment does not fit the model!")
            # If there is an error, close the environment

            self.env.close()
            self.env = None
            
    def copy_to_clipboard(self):
        timetable_str = str(self.manual_exp.action_timetable)
        clipboard = QApplication.clipboard()
        clipboard.setText(timetable_str)
        QMessageBox.information(self, "Clipboard", "Action timetable copied to clipboard!")

class ManualExpert:
    def __init__(self, sim_freq= 5):
        self.sim_freq = sim_freq
        self.current_action = [0 for i in range(9)]
        self.action_key = 0

        #define an action handling list
        # sprinting v1
        # self.action_timetable = {
        #     0:  [-0, 0.0,   -0.5, 0.3,  -0.5, 0.3,    0.2, 0.2],
        #     0.2:  [1, 0.2,   -0.2, 0.3,   -0.2, 0.3,    -1, -1],
        #     0.5:  [1, 0.2,   -0.2, -0.4,   -0.2, -0.4,    -1, -1],
        #     0.6:  [-1, 0.2,   -0.4, 0.7,   -0.4, 0.7,    0.2, 0.2],
        #     0.7 :  [-1, 0.0,   -0.5, 0.7,   -0.5, 0.7,    0.2, 0.2],
            
        #     0.7: [-1, 0,   -0.4, 0,   -0.4, 0,    0, 0]
        # }

        #balancesprinting
        #self.action_timetable = {0.0: [-1.0, 0.0, -0.3, 0.3, -0.3, 0.3, 0.7, 0.7], 0.1: [-0.7, 0.0, -0.16, 0.6, -0.16, 0.6, -0.4, -0.4], 0.25: [-0.0, 0.0, 0.2, 0.4, 0.2, 0.4, -0.4, -0.4], 0.35: [0.8, 0.0, -0.1, 0.0, -0.1, 0.0, 0.6, 0.6], 0.5: [-1.0, 0.0, -0.3, 0.5, -0.3, 0.5, 0.65, 0.65], 0.6: [-1.0, 0.0, -0.4, -0.6, -0.4, -0.6, 1.0, 1.0]}
        #self.action_timetable = {0.0: [-0.9, 0.0, -0.3, 0.2, -0.3, 0.2, 0.7, 0.7], 0.05: [-0.9, 0.0, -0.4, 0.6, -0.4, 0.6, -0.4, -0.4], 0.1: [0.7, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, -0.4], 0.2: [0.7, 0.0, 0.0, -0.5, 0.0, -0.5, 0.7, 0.7], 0.3: [-0.9, 0.0, -0.1, 0.7, -0.1, 0.7, 0.7, 0.7], 0.5: [-1.0, 0.0, -0.4, -0.6, -0.4, -0.6, 1.0, 1.0]}
        # sprinting v2
        # self.action_timetable = {
        #     0:  [0.2, 0,   -0.4, 0.4,  -0.4, 0.4,    0.3, 0.3],
        #     0.2:  [1, 0,   -0., 0.4,   -0., 0.4,    -0.7, -0.7],
        #     0.4:  [1, 0,   -0., -0.6,   -0., -0.6,    -0.7, -0.7],
        #     0.5:  [-0.8, 0,   -0.1, -0.6,   -0.1, -0.6,    -0.7, -0.7],
        #     0.6:  [-0.8, 0,   -0.4, 0.7,   -0.4, 0.7,    0.4, 0.4],
        #     0.7 :  [0.2, 0,   -0.4, 0.4,  -0.4, 0.4,    0.3, 0.3],
            
        #     0.7: [-1, 0,   -0.4, 0,   -0.4, 0,    0, 0]
        # }

        #self.action_timetable = {0.0: [-0.8, 0.0, -0.4, 0.6, -0.4, 0.6, 1.0, 1.0], 0.1: [0.8, 0.0, -0.4, 0.6, -0.4, 0.6, -0.7, -0.7], 0.2: [0.8, 0.0, 0.0, 0.3, 0.0, 0.3, -0.7, -0.7], 0.3: [-0.8, 0.0, -0.4, 0.6, -0.4, 0.6, 1.0, 1.0], 0.5: [-1.0, 0.0, -0.4, -0.6, -0.4, -0.6, -0.7, 0.8]}
        
        #other best reallive Expert
        self.action_timetable = {0.0: [-0.8, 0.0, -0.0, 0.3, -0.0, 0.3, -1.0, -1.0], 0.2: [-0.3, 0.0, 0.25, 0.3, 0.25, 0.3, -1.0, -1.0], 0.3: [0.2, 0.0, 0.4, -0.6, 0.4, -0.6, 0.4, 0.4], 0.4: [-0.2, 0.0, -0.1, 0.5, -0.1, 0.5, 0.5, 0.5], 0.45: [-0.5, 0.0, -0.1, 0.5, -0.1, 0.5, 0.5, 0.5], 0.6: [-0.5, 0.0, -0.1, 0.3, -0.1, 0.3, 0.5, 0.5], 0.8: [-1.0, 0.0, -0.4, -0.6, -0.4, -0.6, 1.0, 1.0]}
        
        #pusch sprint v1
        #self.action_timetable = {0.0: [-0.5, 0.0, -0.2, 0.4, -0.2, 0.4, -0.5, -0.5], 0.2: [-0.5, 0.0, 0.5, -0.4, 0.5, -0.4, 1.0, 1.0], 0.45: [1.0, 0.0, 0.5, 0.6, 0.5, 0.6, 0.0, 0.0], 0.6: [1.0, 0.0, -0.2, 0.6, -0.2, 0.6, -0.5, -0.5], 0.7: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

        #sitting
        #self.action_timetable = {0:  [-0.3, 0,   -0.5, 0.3,  -0.5, 0.3,    0, 0]}



    def think_and_respond(self, obs_, state, done, current_time=0):
        # Define the action based on the time table
        action_keytimes = list(self.action_timetable.keys())
        #get the action which is at the current time step
        last_time = action_keytimes[-1]
        
        #find the next action key
        if 0 ==last_time:
            time_in_timetable = 0
        else:
            time_in_timetable = current_time % last_time
        #search nearest key
        action_key = min(action_keytimes, key=lambda x:abs(x-time_in_timetable))

        action = self.action_timetable[action_key]
        state = obs_[:-1]
        return np.array(action), state

if __name__=="__main__":
    #env = RL_Env(ModelType="SAC", gui=True, render_mode="human", maxSteps=360*5, terrain_type="flat", rewards_type=["stability"], observation_type=["joint_forces", "joint_angles", "rhythm"], simulation_stepSize=5, restriction_2D=False, real_robot=False)

    
    # #shows an automatic controller
    # expert = ManualExpert(sim_freq=5)
    # for episode in range(10):
        
    #     obs, info = env.reset()
    #     done = False
    #     while not done:
    #         action, state = expert.think_and_respond(obs, None, done)
    #         obs, reward, terminated, truncated, info = env.step(action)
    #         #env.render()
    #         done = terminated or truncated
    # env.close()

    app = QApplication(sys.argv)

    gui = ActionTimetableEditor()
  
    gui.show()

    sys.exit(app.exec())

