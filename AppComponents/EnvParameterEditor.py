from PySide6.QtWidgets import (
    QApplication,  QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,QStatusBar,
    QPushButton, QHeaderView, QMessageBox, QSlider, QFileDialog, QLabel, QCheckBox, QComboBox, QMainWindow, QTabWidget
)
from PySide6.QtGui import QIcon
from PySide6.QtCore import Qt,QSettings, QThread, Signal, Slot

import ast


class EnvParameterEditor(QWidget):
    def __init__(self, env_param_kwargs, fixed_env_params, restart_callback):
        super().__init__()
        self.fixed_env_params = fixed_env_params
        self.env_param_kwargs = self.overwrite_with_fixed_env_params(env_param_kwargs)

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

    def overwrite_with_fixed_env_params(self, env_param_kwargs):
        #remove all same keys of fix_env_params from the env_param_kwargs
        for key in self.fixed_env_params.keys():
            if key in env_param_kwargs:
                env_param_kwargs.pop(key)
                #add the fixed env params to the env_param_kwargs
        #env_param_kwargs.update(self.fixed_env_params)
        return env_param_kwargs


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
        print("Env_Parameters:   ", env_param_kwargs)
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

            self.env_param_kwargs = self.overwrite_with_fixed_env_params(env_param_kwargs)

            self.load_env_parameters(self.env_param_kwargs)
            QMessageBox.information(self, "Parameters Loaded", "Environment parameters loaded successfully!")
        else:
            QMessageBox.warning(self, "Parameters Not Loaded", "No parameters loaded!")

    def save_and_restart(self):
        self.restart_callback(self.get_env_parameters())