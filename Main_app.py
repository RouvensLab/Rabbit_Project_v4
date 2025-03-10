import sys
import time
import os
import json

import threading
import traceback
import numpy as np
from PySide6.QtWidgets import (
    QApplication,  QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,QStatusBar,
    QPushButton, QHeaderView, QMessageBox, QSlider, QFileDialog, QLabel, QCheckBox, QComboBox, QMainWindow, QTabWidget, QLineEdit,QStyleFactory
)
from PySide6.QtGui import QIcon,QPalette, QColor
from PySide6.QtCore import Qt,QSettings, QThread, Signal, Slot

from RL_Agent_Env import RL_Env
from RL_Robot_Env import RL_Robot
from stable_baselines3 import SAC
from tools.Controler import ControlInput
from tools.TimeInterval import TimeInterval
from tools.Trajectory import TrajectoryRecorder

from AppComponents.ExtendedTableWidget import ExtendedTableWidget
from AppComponents.EnvParameterEditor import EnvParameterEditor
from AppComponents.FolderWidget import FolderWidget
    
class ActionTimetableEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Action Timetable Editor")
        self.resize(600, 400)

        # Initialize QSettings
        self.settings = QSettings("MyCompany", "ActionTimetableEditor")


        # Restore geometry and state
        self.restore_geometry()

        #get the default parameters for the RLEnv as a dictionary

        self.def_RlEnv_param_kwargs = {
        "ModelType": "SAC",
        "rewards_type": [],
        "observation_type_stacked": [],
        "observation_type_solo": ["phase_signal"],
        "terrain_type": "flat",
        "Horizon_Length":True,
        "recorded_movement_file_path_dic": {"PushSprint_v1": 5},
        "restriction_2D": False,
        "simulation_Timestep": 0.1
    }
        self.RlEnv_param_kwargs = self.def_RlEnv_param_kwargs
        self.RlEnv_startup_parms = {"render_mode": "fast", "real_robot": False, "gui": True, "RobotType":"Rabbit_v3_mesured"}
        self._init_RlEnv()
        self.RlEnv.simulation.hung = True

        self.manual_exp = ManualExpert(sim_freq=5)
        self.loaded_model = None

        self.env_param_editor = None

        self.control_input = None
        self.action = [0]*8

        # Create the Real Robot
        self.RLRobot_param_kwargs = {
            "ModelType": "SAC",
            "observation_type_stacked": [],
            "observation_type_solo": ["phase_signal"],
            "simulation_Timestep": 0.1,
            "obs_time_space": 1,
        }
        self.RLRobot_starup_parms = {"render_mode": "fast", "gui": True, "RobotType": "Rabbit_mesured"}
        self.RlRobot = None
        self._init_RlRobot()

        self.initUI()
        self.env_pause = False
        self.start_thread()

    def _init_RlEnv(self):
        try:
            if hasattr(self, 'RlEnv') and self.RlEnv is not None:
                self.RlEnv.close()
            #time steps for every action
            self.time_step = self.RlEnv_param_kwargs["simulation_Timestep"]
            #from RL_Agent_Env import RL_Env
            self.RlEnv = RL_Env(**self.RlEnv_startup_parms, **self.RlEnv_param_kwargs)
            self.RabbitMesure_widget = self.RlEnv.simulation.rabbit.create_GuiWidget()
            #self.RabbitMesure_widget.setParent(self)  # Ensure proper parent-child relationship
            self.RlEnv.reset()
            
            #Implement that the rabbit unpause the simulation, when the recording is started
            self.RabbitMesure_widget.TrajRecorder.start_recording_signal.connect(lambda: self.pause_unpause_simulation(False))
            self.RabbitMesure_widget.TrajRecorder.stop_recording_signal.connect(lambda: self.pause_unpause_simulation(True))

            #update the self.SimObservation_tab and show again
            if hasattr(self, "SimObservation_tab"):
                if self.SimObservation_tab.layout():
                    QWidget().setLayout(self.SimObservation_tab.layout())
                layout = QVBoxLayout()
                layout.addWidget(self.RabbitMesure_widget)
                self.SimObservation_tab.setLayout(layout)
            return True
        except Exception as e:
            print(f"Error initializing RLEnv: {e}")
            self.RlEnv = None
            return False

    def _init_RlRobot(self):
        try:
            if hasattr(self, 'RlRobot') and self.RlRobot is not None:
                self.RlRobot.close()
            #time steps for every action
            self.time_step = self.RLRobot_param_kwargs["simulation_Timestep"]

            self.RlRobot = RL_Robot(**self.RLRobot_starup_parms, **self.RLRobot_param_kwargs)
            self.RealRabbitMesure_widget = self.RlRobot.rabbit.create_GuiWidget()
            self.RlRobot.reset()

            #update the self.RealObservation_tab and show again
            #self.RealRabbitMesure_widget.setParent(self)  # Ensure proper parent-child relationship
            self.RealRabbitMesure_widget.TrajRecorder.start_recording_signal.connect(lambda: self.pause_unpause_simulation(False))
            self.RealRabbitMesure_widget.TrajRecorder.stop_recording_signal.connect(lambda: self.pause_unpause_simulation(True))
            
            if hasattr(self, "RealObservation_tab"):
                if self.RealObservation_tab.layout():
                    QWidget().setLayout(self.RealObservation_tab.layout())
                layout = QVBoxLayout()
                layout.addWidget(self.RealRabbitMesure_widget)
                self.RealObservation_tab.setLayout(layout)
            return True
        except Exception as e:
            print(f"Error initializing RLRobot: {e}")
            self.RlRobot = None
            self.RealRabbitMesure_widget = QWidget()
            if hasattr(self, "RealObservation_tab"):
                if self.RealObservation_tab.layout():
                    QWidget().setLayout(self.RealObservation_tab.layout())
                layout = QVBoxLayout()
                layout.addWidget(self.RealRabbitMesure_widget)
                self.RealObservation_tab.setLayout(layout)
            return False

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
        self.cleanup_simulation()
        if self.RlEnv:
            self.RlEnv.close()
            self.RlEnv = None
        QApplication.instance().quit()
        event.accept()

   
    def initUI(self):
        #create a toolbar
        self.toolbar = self.addToolBar("Toolbar")

        # Pause/Unpause button
        class QPauseButton(QPushButton):
            def __init__(self, parent):
                super().__init__(parent)
                self.paused = False
                self.setIcon(QIcon(r"public\start.png"))
                self.clicked.connect(self.toggle_pause)
            def toggle_pause(self):
                self.paused = not self.paused
                if self.paused:
                    self.setIcon(QIcon(r"public\pause.png"))
                else:
                    self.setIcon(QIcon(r"public\start.png"))
        self.pause_button = QPauseButton(self)
        self.pause_button.clicked.connect(lambda: self.pause_unpause_simulation())
        self.toolbar.addWidget(self.pause_button)

        # Restart button
        self.restart_button = QPushButton("Restart Simulation", self)
        self.restart_button.clicked.connect(self.reset_simulation)
        self.toolbar.addWidget(self.restart_button)

        #add a speed slider
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(10)
        self.speed_slider.setValue(0)
        self.toolbar.addWidget(self.speed_slider)


        #dopdown box for what gets controled
        self.isControled_dropdown = QComboBox(self)
        self.isControled_dropdown.addItems(["Only Simulation", "Simulation_Imitation", "Only Real Robot"])
        self.isControled_dropdown.setCurrentIndex(0)
        self.isControled_dropdown.currentIndexChanged.connect(self.update_status)
        self.isControled_dropdown.currentIndexChanged.connect(self.changeWhatIsControlled)
        self.toolbar.addWidget(self.isControled_dropdown)

        #add toolbar to the main window
        self.addToolBar(self.toolbar)


        #Create a tabwidget with two tabs (Control and Observation)
        tab_widget = QTabWidget()
        self.setCentralWidget(tab_widget)

        # Control Tab
        self.control_tab = QWidget()
        tab_widget.addTab(self.control_tab, "Control")

        # Simulation Observation Tab
        self.SimObservation_tab = QWidget()
        tab_widget.addTab(self.SimObservation_tab, "Observation")

        # Real Observation Tab
        self.RealObservation_tab = QWidget()
        tab_widget.addTab(self.RealObservation_tab, "Real Observation")


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
        # checkbox for savety mode or direct mode
        self.button_real_robot_savety = QCheckBox("Savety Mode", self)
        self.button_real_robot_savety.toggled.connect(self.update_status)
        real_robot_layout.addWidget(self.button_real_robot_savety)
        
        bar_layout.addLayout(real_robot_layout)
        control_layout.addLayout(bar_layout)

        # Table for action timetable
        self.table_layout = QVBoxLayout()
        def_timeTable_name = "Action Timetable"
        def_timeTable_folder_path = r"Trajectories\Experts"
        self.timeTable_toolbar = QHBoxLayout()
        
        self.timeTable_folderPath = QLabel(def_timeTable_folder_path)
        self.timeTable_toolbar.addWidget(self.timeTable_folderPath)
        self.timeTable_name_input = QLineEdit(self)
        self.timeTable_name_input.setPlaceholderText("Action Timetable Name")
        self.timeTable_name_input.setText(def_timeTable_name)
        self.timeTable_toolbar.addWidget(self.timeTable_name_input)
        self.timeTable_save_button = QPushButton("Save Timetable", self)
        self.timeTable_save_button.clicked.connect(self.saveTimeTable)
        self.timeTable_toolbar.addWidget(self.timeTable_save_button)
        self.table_layout.addLayout(self.timeTable_toolbar)

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

        #Button to paste current roboter position to the timetable
        self.paste_button = QPushButton("Insert Current Position", self)
        self.paste_button.clicked.connect(lambda: self.table.add_action_row(self.action, append=False))
        button_layout.addWidget(self.paste_button)
        #Button to paste current roboter position to the timetable
        self.paste_button1 = QPushButton("Append Current Position", self)
        self.paste_button1.clicked.connect(lambda: self.table.add_action_row(self.action, append=True))
        button_layout.addWidget(self.paste_button1)


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

        #add a fileexplorer to load a timetable
        self.trajectory_folder_widget = FolderWidget(r"Trajectories")
        self.trajectory_folder_widget.fileClicked.connect(self.load_timetable)
        button_layout.addWidget(self.trajectory_folder_widget)

        layout1.addLayout(button_layout, stretch=1)
        control_layout.addLayout(layout1, stretch=1)
        
        # Status Bar
        self.status_bar = QStatusBar(self)
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready")

        # Set the control_layout as the main layout of the Control tab
        self.control_tab.setLayout(control_layout)


        # Observation Tab
        #add to the observation tab
        observation_layout = QVBoxLayout()
        observation_layout.addWidget(self.RabbitMesure_widget)
        self.SimObservation_tab.setLayout(observation_layout)

        # Real Observation Tab
        #add to the observation tab
        real_observation_layout = QVBoxLayout()
        real_observation_layout.addWidget(self.RealRabbitMesure_widget)
        self.RealObservation_tab.setLayout(real_observation_layout)
        

    def update_status(self):
        """Update the status bar with current settings."""
        status_message = (
            f"Model Active: {'Yes' if self.button_mod_active.isChecked() else 'No'}, "
            f"active ControlMode: {self.controlMode_active.currentText()}, "
            f"Auto Reset: {'Enabled' if self.auto_reset_checkbox.isChecked() else 'Disabled'}, "
            f"Savety Mode: {'On' if self.button_real_robot_savety.isChecked() else 'Off'}, "
            f"Speed Slider: {self.speed_slider.value()}"
            f"What is controled: {self.isControled_dropdown.currentText()}"
            f"Control Mode: {self.controlMode_active.currentText()}"
        )
        self.status_bar.showMessage(status_message)

    def hung_change(self):
        self.RlEnv.simulation.hung = not self.RlEnv.simulation.hung

    def camera_focus_change(self):
        self.RlEnv.simulation.toogle_focus()


    def on_table_cell_changed(self):
        # Update the action timetable when a cell is changed
        #update the timetable
        self.manual_exp.action_timetable = self.table.get_dict()

    def reset_simulation(self):    
        #check if the simulation is recording
        if self.RlEnv.simulation.rabbit.is_recording():
            self.RabbitMesure_widget.TrajRecorder.stop_recording_trajectory()


        try:
            if self.RlEnv and self.isControled_dropdown.currentText() == "Only Simulation":
                return self.RlEnv.reset()
            elif self.RlRobot and self.isControled_dropdown.currentText() == "Only Real Robot":
                return self.RlRobot.reset()
            elif self.RlEnv and self.RlRobot:
                obs, inf = self.RlEnv.reset()
                self.RlRobot.reset()
                return obs, inf
            else:
                print("No Environment or Robot to reset!")
                return None, None
        except Exception as e:
            print(f"--------------------Error in reset_simulation: {e}")
            self.no_error = False
            return None, None

    def pause_unpause_simulation(self, state=None):
        if state is None:
            self.env_pause = not self.env_pause
        else:
            self.env_pause = state

    def read_controler_vector(self):
        pass

    def changeControlMode(self):
        #self.cleanup_simulation()
        self.env_pause = True
        time.sleep(0.5)
        if self.controlMode_active.currentText() == "Auto":
            self.close_control_input()
        elif self.controlMode_active.currentText() == "Body Control":
            self.open_control_input()
            #self.env_param_kwargs = self.def_RlEnv_param_kwargs
            ##self.open_RlEnv(self.RlEnv_param_kwargs)
            #self.start_thread()
        else:
            self.open_control_input()
        
        self.env_pause = False

    def open_control_input(self):
        if self.control_input:
            self.control_input.close()
        self.control_input = ControlInput(self)
        self.control_input.Button_connect("RB", lambda: self.table.add_action_row(self.action, append=True))

    def close_control_input(self):
        if self.control_input:
            self.control_input.close()
            self.control_input = None



    def changeWhatIsControlled(self):
        self.cleanup_simulation()
        
        if self.isControled_dropdown.currentText() == "Only Simulation":
            self.RlEnv_param_kwargs = self.def_RlEnv_param_kwargs
            self.control_input = None
            self.controlMode_active.setCurrentIndex(2)
            self._init_RlEnv()
            if hasattr(self, 'RlRobot'):
                self.RlRobot = None
                self.RealRabbitMesure_widget = None
            
        elif self.isControled_dropdown.currentText() == "Simulation_Imitation":
            self.RlEnv_param_kwargs = self.def_RlEnv_param_kwargs
            
            self._init_RlEnv() and self._init_RlRobot()
            
        elif self.isControled_dropdown.currentText() == "Only Real Robot":
            if hasattr(self, 'RlEnv'):
                self.RlEnv = None
            self._init_RlRobot()
        self.start_thread()

    def open_RlEnv(self, env_param_kwargs):
        self.RlEnv_param_kwargs = env_param_kwargs
        #open a new environment
        self._init_RlEnv()
        self.RlEnv.simulation.hung = True
        self.env_pause = True

        #if everything is ok, close the editor window
        if self.env_param_editor:
            self.env_param_editor.close()
            self.env_param_editor = None
            print("env_param_editor closed!")

    def open_RlRobot(self, RLRobot_param_kwargs):
        self.RLRobot_param_kwargs = RLRobot_param_kwargs
        #open a new environment
        self._init_RlRobot()
        self.env_pause = True

        #if everything is ok, close the editor window
        if self.env_param_editor:
            self.env_param_editor.close()
            self.env_param_editor = None
            print("env_param_editor closed!")

    def cleanup_simulation(self):
        self.end_thread = True
        self.env_pause = True
        if self.simulation_thread:
            self.simulation_thread.join()
            self.simulation_thread = None
        if self.RlEnv:
            self.RlEnv.close()
            self.RlEnv = None
        if self.RlRobot:
            self.RlRobot.close()
            self.RlRobot = None



    def start_thread(self):
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()
        print("Threat started!")


    def restart_Env_with_new_param(self, env_param_kwargs):
        self.cleanup_simulation()
        self.RlEnv_param_kwargs = env_param_kwargs
        self.RLRobot_param_kwargs["simulation_Timestep"] = env_param_kwargs["simulation_Timestep"]
        self._init_RlEnv()
        self._init_RlRobot()
        self.start_thread()
        

    
    def open_Env_param(self):
        #open the environment parameter editor
        self.env_param_editor = EnvParameterEditor(self.RlEnv_param_kwargs, self.RlEnv_startup_parms, lambda x: self.restart_Env_with_new_param(x))
        self.env_param_editor.setWindowTitle("Environment Parameter Editor")
        self.env_param_editor.resize(600, 400)
        self.env_param_editor.show()


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

        print("Model loaded!--------------------------")

    def load_timetable(self, folder_path, file_name: str):
        # Check if the file is a JSON file
        if ".json" in file_name:
            #remove the .json from the file name
            file_name = file_name.replace(".json", "")
        #pause the simulation
        self.env_pause = True

        # Load the timetable from the selected file
        file_path = os.path.join(folder_path, file_name +".json")
        if "Experts" in file_path:
            # The structure of the file corresponds to the manual expert timetable
            try:
                with open(file_path, 'r') as file:
                    data = json.load(file)
                    #self.manual_exp.action_timetable = data
                    self.table.load_action_timetable(data)
                    QMessageBox.information(self, "Timetable Loaded", "Timetable loaded successfully!")
                    self.timeTable_folderPath.setText(folder_path)
                    self.timeTable_name_input.setText(file_name)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load timetable from {file_name}: {e}")
        else:
            try:
                #open the file with Trajecotry
                recTrajectory = TrajectoryRecorder()
                recTrajectory.load_trajectory(file_name, folder_path)
                #get the keys of the trajectory
                keys = recTrajectory.get_keys()
                if "actions" in keys:
                    #load the actions to the timetable
                    actions = recTrajectory.get_keyValues("actions")
                    times = recTrajectory.get_times()
                    #concatenate the times and actions
                    time_table_dic = dict(zip(times, actions))
                    self.table.load_action_timetable(time_table_dic)
                    QMessageBox.information(self, "Timetable Loaded", "Timetable loaded successfully!")
                    self.timeTable_folderPath.setText(folder_path)
                    self.timeTable_name_input.setText(file_name)
                elif "joint_angles" in keys:
                    #load the joint positions to the timetable
                    joint_positions = recTrajectory.get_keyValues("joint_angles")
                    times = recTrajectory.get_times()
                    #concatenate the times and actions
                    time_table_dic = dict(zip(times, joint_positions))
                    #convert the joint positions to actions
                    time_table_dic = {key: self.RlEnv.simulation.rabbit.convert_joint_angles_to_actions(value) for key, value in time_table_dic.items()}
                    self.table.load_action_timetable(time_table_dic)
                    QMessageBox.information(self, "Timetable Loaded", "Timetable loaded successfully!")
                    self.timeTable_folderPath.setText(folder_path)
                    self.timeTable_name_input.setText(file_name)
                else:
                    QMessageBox.warning(self, "Error", "No actions or joint angles found in the trajectory file!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load timetable from {file_name}: {e}")

    def saveTimeTable(self):
        def_path_folder = r"Trajectories\Experts"
        folder_path = def_path_folder
        file_name = self.timeTable_name_input.text()
        if not file_name:
            QMessageBox.warning(self, "Error", "Please enter a name for the timetable!")
            return
        file_path = os.path.join(folder_path, file_name + ".json")
        #check whether the folder and file exists, and if so, ask the user if he wants to overwrite the file
        if os.path.exists(file_path):
            reply = QMessageBox.question(self, "File Exists", f"The file {file_name} already exists! Do you want to overwrite it?", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.No:
                return
        try:
            # Save the timetable to the selected file
            with open(file_path, 'w') as file:
                json.dump(self.table.get_dict(), file, indent=4)
            QMessageBox.information(self, "Timetable Saved", "Timetable saved successfully!")
            self.timeTable_folderPath.setText(folder_path)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save timetable to {file_name}: {e}")



    def run_simulation(self):
        self.no_error = True
        self.end_thread = False
        self.IntTimer = TimeInterval(self.time_step)
        try:
            obs, inf = self.reset_simulation()
            if obs is None:
                print("obs is None Error!!!!!!!!!!!")
                self.no_error = False
                return
                
            while not self.end_thread and self.no_error:
                done = False
                while not done and not self.end_thread and self.no_error:
                    print("Simulation running-------------")
                    if not self.env_pause:
                        if self.controlMode_active.currentText() == "Body Control" and self.control_input is not None: # Body Control (Joystick) is controlling the robot
                            self.action = self.control_input.get_BodyPose()
                            print(self.action)
                        elif self.loaded_model is not None and self.button_mod_active.isChecked():# A RL-Agent is controlling the robot
                            try:
                                self.action, pred_info = self.loaded_model.predict(obs)
                                print("RL-Agent Prediction: ", self.action)
                            except Exception as e:
                                self.no_error = False
                                print(f"Error in model prediction: {e}")
                        else: #self.controlMode_active.currentText() == "Auto": # Manual Expert is controlling the robot
                            try:
                                life_time = self.RlEnv.simulation.rabbit.lifetime if self.isControled_dropdown.currentText() in ["Only Simulation", "Simulation_Imitation"] else self.RlRobot.rabbit.lifetime
                                self.action, state, action_key = self.manual_exp.think_and_respond(obs, None, done, life_time)
                                print(self.action)
                                # Highlight the active row
                                #action_key_index = list(self.manual_exp.action_timetable.keys()).index(self.manual_exp.action_key)
                                self.table.highlight_active_row(action_key)
                            except Exception as e:
                                print(f"Error in manual response: {e}")
                                self.no_error = False
                        #print(action)
                        try:
                            if self.isControled_dropdown.currentText() == "Only Simulation":
                                obs, reward, terminated, truncated, info = self.RlEnv.step(self.action)
                            elif self.isControled_dropdown.currentText() == "Simulation_Imitation":
                                _, _, _, _, _ = self.RlRobot.step(self.action)
                                obs, reward, terminated, truncated, info = self.RlEnv.step(self.action)

                            elif self.isControled_dropdown.currentText() == "Only Real Robot":
                                obs, reward, terminated, truncated, info = self.RlRobot.step(self.action)
                            done = (terminated or truncated) and self.auto_reset_checkbox.isChecked()

                            if done:
                                obs, inf = self.reset_simulation()
                        except Exception as e:
                            self.no_error = False
                            print(f"Error in environment step: {e}")
                        
                        self.IntTimer.wait_for_step((1+self.speed_slider.value())*self.time_step)
                    else:
                        time.sleep(0.5)

        except Exception as e:
            print(f"Error in simulation thread: {e}")
            self.no_error = False
            
        finally:
            if not self.no_error:
                if self.RlEnv:
                    self.RlEnv.close()
                    self.RlEnv = None
                if self.RlRobot:
                    self.RlRobot.close()
                    self.RlRobot = None
                QMessageBox.critical(self, "Error", "Simulation stopped due to an error!")

        if not self.no_error:
            # Dialog if there is an error
            print("Error", "An error occurred! Simulation stopped! Probably the Environment does not fit the model!")
            # If there is an error, close the environment
            if self.RlEnv:
                self.RlEnv.close()
                self.RlEnv = None
            if self.RlRobot:
                self.RlRobot.close()
                self.RlRobot = None
            self.end_thread = True
        time.sleep(0.5)
            
    def copy_to_clipboard(self):
        timetable_str = str(self.manual_exp.action_timetable)
        clipboard = QApplication.clipboard()
        clipboard.setText(timetable_str)
        QMessageBox.information(self, "Clipboard", "Action timetable copied to clipboard!")

    @Slot(str)
    def handle_simulation_error(self, error_msg):
        QMessageBox.critical(self, "Simulation Error", error_msg)
        self.cleanup_simulation()

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
        #self.action_timetable = {0.0: [-0.8, 0.0, -0.0, 0.3, -0.0, 0.3, -1.0, -1.0], 0.2: [-0.3, 0.0, 0.25, 0.3, 0.25, 0.3, -1.0, -1.0], 0.3: [0.2, 0.0, 0.4, -0.6, 0.4, -0.6, 0.4, 0.4], 0.4: [-0.2, 0.0, -0.1, 0.5, -0.1, 0.5, 0.5, 0.5], 0.45: [-0.5, 0.0, -0.1, 0.5, -0.1, 0.5, 0.5, 0.5], 0.6: [-0.5, 0.0, -0.1, 0.3, -0.1, 0.3, 0.5, 0.5], 0.8: [-1.0, 0.0, -0.4, -0.6, -0.4, -0.6, 1.0, 1.0]}
        
        #pusch sprint v1
        #self.action_timetable = {0.0: [-0.5, 0.0, -0.2, 0.4, -0.2, 0.4, -0.5, -0.5], 0.2: [-0.5, 0.0, 0.5, -0.4, 0.5, -0.4, 1.0, 1.0], 0.45: [1.0, 0.0, 0.5, 0.6, 0.5, 0.6, 0.0, 0.0], 0.6: [1.0, 0.0, -0.2, 0.6, -0.2, 0.6, -0.5, -0.5], 0.7: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

        #big jumps
        #self.action_timetable = {0.0: [0.5, 0.0, -0.2, -0.3, -0.2, -0.3, 0.0, 0.0], 0.25: [0.4, 0.0, -0.2, 0.2, -0.2, 0.2, 0.0, 0.0], 0.5: [0.1, 0.0, -0.8, -0.9, -0.8, -0.9, -0.5, -0.5], 0.75: [1.0, 0.0, 0.1, 0.5, 0.1, 0.5, 0.0, 0.0]}
        #fast jump Sim
        #self.action_timetable = {0.0: [0.9, 0.0, -0.3, 0.4, -0.3, 0.4, 0.0, 0.0], 0.25: [0.7, 0.0, -0.5, 0.3, -0.5, 0.3, 0.0, 0.0], 0.3: [0.1, 0.0, -0.6, 0.0, -0.6, 0.0, -0.5, -0.5], 0.8: [1.0, 0.0, 0.3, 0.4, 0.3, 0.4, -0.5, -0.5]}
        #Sim Jumps
        #self.action_timetable = {0.0: [0.8, 0.0, -0.4, 0.2, -0.4, 0.2, 0.0, 0.0], 0.2: [0.1, 0.0, -0.4, 0.5, -0.4, 0.5, -0.7, -0.7], 0.35: [0.1, 0.0, -0.5, -0.2, -0.5, -0.2, -0.7, -0.7], 0.657: [1.0, 0.0, -0.1, 0.5, -0.1, 0.5, 0.0, 0.0]}
        #slow jumps
        self.action_timetable = {0.0: [1.0, 0.0, 0.3, 0.4, 0.3, 0.4, 0.0, 0.0], 1.0: [0.5, 0.0, -0.3, 0.4, -0.3, 0.4, -0.5, -0.5], 1.25: [0.1, 0.0, -0.5, 0.3, -0.5, 0.3, -0.5, -0.5], 1.3: [0.1, 0.0, -0.6, 0.0, -0.6, 0.0, -0.5, -0.5], 1.8: [1.0, 0.0, 0.3, 0.4, 0.3, 0.4, -0.5, -0.5]}
        
        #Männchen
        #self.action_timetable = {0.0: [-0.003936767578125, 0.00390625, 0.00390625, -0.003936767578125, 0.00390625, -0.003936767578125, -1.0, -1.0], 1.0: [-0.003936767578125, 0.00390625, 0.00390625, -0.003936767578125, 0.00390625, -0.003936767578125, -1.0, -1.0], 2.0: [0.999969482421875, 0.00390625, 0.00390625, 0.999969482421875, 0.00390625, 0.999969482421875, -1.0, -1.0], 3.0: [0.999969482421875, 0.00390625, -0.129425048828125, 0.403900146484375, -0.129425048828125, 0.403900146484375, -1.0, -1.0], 4.0: [0.999969482421875, 0.00390625, -0.62353515625, 0.529388427734375, -0.62353515625, 0.529388427734375, -1.0, -1.0], 5.0: [0.999969482421875, 0.00390625, 0.00390625, 0.450958251953125, 0.00390625, 0.450958251953125, -1.0, -1.0], 6.0: [-0.835296630859375, -0.4039306640625, 0.00390625, 0.01959228515625, 0.00390625, 0.01959228515625, -0.13726806640625, -0.160797119140625], 7.0: [-0.207855224609375, -1.0, 0.00390625, -0.003936767578125, 0.00390625, -0.003936767578125, -0.23138427734375, -0.254913330078125], 8.0: [-0.003936767578125, 0.00390625, 0.00390625, -0.003936767578125, 0.00390625, -0.003936767578125, 0.137237548828125, -0.04315185546875], 9.0: [0.192138671875, 0.999969482421875, 0.00390625, -0.003936767578125, 0.00390625, -0.003936767578125, 0.1607666015625, -0.207855224609375]}
        
        #slow jumps
        #self.action_timetable = {0.0: [-0.003936767578125, 0.0, -0.003936767578125, 0.0, -0.003936767578125, 0.0, -0.160797119140625, -0.160797119140625], 1.0: [0.999969482421875, 0.0, 0.00390625, 0.2, 0.00390625, 0.2, -0.160797119140625, -0.160797119140625], 2.0: [0.5, 0.0, -0.4039306640625, 0.3, -0.4039306640625, 0.3, -1.0, -1.0], 3.0: [0.5, 0.0, -0.5, 0.5, -0.5, 0.5, -0.160797119140625, -0.0902099609375]}

    
    
    def think_and_respond(self, obs_, state, done, current_time=0):
        # Define the action based on the time table
        action_keytimes = list(self.action_timetable.keys())
        #get the action which is at the current time step
        last_time = action_keytimes[-1]
        
        #find the next action key
        if (0 ==last_time):
            time_in_timetable = 0
        else:
            time_in_timetable = current_time % last_time
        #search nearest key
        action_key = min(action_keytimes, key=lambda x:abs(x-time_in_timetable))

        action = self.action_timetable[action_key]
        state = obs_[:-1]
        action_index = list(self.action_timetable.keys()).index(action_key)
        print(action_index)
        return np.array(action), state, action_index

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
    # Fusion dark style
   # Set the Fusion style
    # app.setStyle(QStyleFactory.create("Fusion"))

    # Create a custom dark palette
    '''
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.ColorRole.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.ColorRole.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.ToolTipText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.white)
    dark_palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    dark_palette.setColor(QPalette.ColorRole.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)

    # Apply the custom palette to the application
    app.setPalette(dark_palette)
    '''
    gui = ActionTimetableEditor()
  
    gui.show()

    sys.exit(app.exec())

