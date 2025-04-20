from PySide6.QtWidgets import (
    QApplication,  QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,QStatusBar,
    QPushButton, QHeaderView, QMessageBox, QSlider, QFileDialog, QLabel, QCheckBox, QComboBox, QMainWindow, QTabWidget
)
from PySide6.QtGui import QIcon
from PySide6.QtCore import Qt,QSettings, QThread, Signal, Slot
from PySide6.QtCore import QSize
from PySide6.QtGui import QFont
from PySide6.QtCore import QUrl
from PySide6.QtWidgets import QLineEdit, QFormLayout, QGroupBox, QSpinBox, QDoubleSpinBox, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QWidget
from PySide6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView, QSizePolicy, QLineEdit, QFormLayout, QGroupBox, QSpinBox, QDoubleSpinBox

import numpy as np

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
            time_item = self.item(row, 0)
            time = float(time_item.text()) if time_item is not None else 0.0 and print("No time found!")
            actions = [float(self.item(row, col).text()) if self.item(row, col) is not None else 0.0 for col in range(1, self.columnCount())]
            timetable[time] = actions
        print("Timetable: ", timetable)
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
        if (selected_row == -1):  # If no row selected
            selected_row = None
        self.insert_empty_row(selected_row)
    
    def remove_selected_row(self):
        # Remove the selected row
        selected_row = self.currentRow()
        if (selected_row != -1):
            self.removeRow(selected_row)

    def insert_empty_row(self, position=None):
        """Inserts a new empty row at the given position (or at the end if no position is specified)."""
        if (position is None):
            # If no position specified, append a new row at the end
            position = self.rowCount()
        
        self.insertRow(position)
        # Insert default empty values (e.g., 0.0 for float) in the new row
        self.setItem(position, 0, QTableWidgetItem(str(0.0)))  # Time
        for col in range(1, self.columnCount()):
            self.setItem(position, col, QTableWidgetItem(str(0.0)))  # Default action values

    def add_action_row(self, actions, time=None, append=True):
        """Appends a new row with the given time and action values."""
        if append:
            selected_row = self.rowCount()
            if time is None:
                previous_item = self.item(selected_row - 1, 0)
                if previous_item is not None:
                    time = float(previous_item.text()) + 1
                else:
                    print("No previous time found! Action row is not there!")
            self.insert_action_row(time, actions, selected_row)
        else:
            selected_row = self.currentRow()
            if (selected_row == -1):  # If no row selected
                selected_row = self.rowCount()
            #get a time between the last time and the next time
            if time is None:
                time1 = float(self.item(selected_row, 0).text())
                if selected_row+1 == self.rowCount():
                    time = time1 + 1
                else:
                    time2 = float(self.item(selected_row+1, 0).text())
                    time = (time1 + time2) / 2
            self.insert_action_row(time, actions, selected_row + 1)

    def insert_action_row(self, time, actions, position=None):
        """Inserts a new row with the given time and action values at the given position."""
        #actions = actions_func()
        if (position is None):
            # If no position specified, append a new row at the end
            position = self.rowCount()
        self.insertRow(position)
        self.setItem(position, 0, QTableWidgetItem(str(time)))
        for col, action_value in enumerate(actions):
            self.setItem(position, col + 1, QTableWidgetItem(str(action_value)))  # Shift column by 1 for time

    def highlight_active_row(self, row):
        # Highlight the active row
        for col in range(self.columnCount()):
            self.item(row, col).setBackground(Qt.green)
        # Unhighlight the other rows
        for other_row in range(self.rowCount()):
            if (other_row != row):
                for col in range(self.columnCount()):
                    self.item(other_row, col).setBackground(Qt.white)


    def clear_table(self):
        self.setRowCount(0)
        self.setColumnCount(9)
        self.setHorizontalHeaderLabels(['Time', 'Action1', 'Action2', 'Action3', 'Action4', 'Action5', 'Action6', 'Action7', 'Action8'])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.insert_empty_row()


class RL_Input_And_Output_Widget(QWidget):
    """This is a widget that shows the input and output of the RL agent. It can be implemented in every QApplication window.

    It is structured like following:
    +---------------------+
    | RL Input            |
    +---------------------+
    Arrow down
    +---------------------+
    | RL Output/Action    |
    +---------------------+
    Arrow down
    +---------------------+
    | Reward, scoor(inf)  |
    +---------------------+

    RL Input: This is a table that shows the observation of the RL agent.
    RL Output/Action: This is a table that shows the action of the RL agent that will be feed into the environment.
    Reward, scoor(inf): This is a table that shows the reward and score of the RL agent.

    There will be a function for each table to update the data in the table.
    """    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Main layout
        self.main_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        
        # Initialize tables
        self.input_table = QTableWidget()
        self.output_table = QTableWidget()
        self.reward_table = QTableWidget()
        
        # Add arrow labels between tables
        self.arrow1 = QLabel("↓")
        self.arrow2 = QLabel("↓")
        self.arrow1.setAlignment(Qt.AlignCenter)
        self.arrow2.setAlignment(Qt.AlignCenter)
        self.arrow1.setFont(QFont("Arial", 16, QFont.Bold))
        self.arrow2.setFont(QFont("Arial", 16, QFont.Bold))
        
        # Add widgets to layout
        self._setup_table(self.input_table, "RL Input")
        self.main_layout.addWidget(self.input_table)
        self.main_layout.addWidget(self.arrow1)
        self._setup_table(self.output_table, "RL Output/Action")
        self.main_layout.addWidget(self.output_table)
        self.main_layout.addWidget(self.arrow2)
        self._setup_table(self.reward_table, "Reward/Score")
        self.main_layout.addWidget(self.reward_table)
        
        # Set size policy
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
    def _setup_table(self, table, title):
        """Initialize a table with a title header."""
        table.setColumnCount(2)
        table.setRowCount(0)
        table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        table.horizontalHeader().setStretchLastSection(True)
        #make them not editable
        table.setEditTriggers(QTableWidget.NoEditTriggers)


        # Add title as a label above the table
        title_label = QLabel(title)
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(title_label)
        
    def update_input_table(self, observations):
        """Update the RL Input table with observation data.
        
        Args:
            observations: Dict or list of tuples containing parameter-value pairs
        """
        self._update_table(self.input_table, observations)
        
    def update_output_table(self, actions):
        """Update the RL Output/Action table with action data.
        
        Args:
            actions: Dict or list of tuples containing parameter-value pairs
        """
        self._update_table(self.output_table, actions)
        
    def update_reward_table(self, rewards):
        """Update the Reward/Score table with reward data.
        
        Args:
            rewards: Dict or list of tuples containing parameter-value pairs
        """
        self._update_table(self.reward_table, rewards)
        
    def _update_table(self, table, data):
        """Generic method to update table content.
        
        Args:
            table: QTableWidget to update
            data: Dict or list of tuples containing parameter-value pairs
        """
        # Clear existing content
        #table.setRowCount(0)

        if isinstance(data, dict):
            # Convert dict to list of tuples
            headers = list(data.keys())
            data = list(data.values())
            table.setHorizontalHeaderLabels(headers)

        # Set new row count
        table.setRowCount(1)
        table.setColumnCount(len(data))
        
        # Fill table
        for column, value in enumerate(data):
            table.setItem(0, column, QTableWidgetItem(str(value)))            
            
        # Adjust column widths
        #table.resizeColumnsToContents()
        table.resizeRowsToContents()
        
    def clear_tables(self):
        """Clear all tables."""
        self.input_table.setRowCount(0)
        self.output_table.setRowCount(0)
        self.reward_table.setRowCount(0)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout(window)
    
    # Create and add the RL Input and Output widget
    rl_widget = RL_Input_And_Output_Widget()
    layout.addWidget(rl_widget)
    
    # Show the window
    window.setWindowTitle("RL Input and Output Widget")
    window.resize(400, 300)
    window.show()
    
    # Example data to update the tables
    observations =[0, 0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 5]
    actions = [0, 0, 0, 0.2, -0.1, 0.5, 0.3, 0.1, 0.4]
    rewards = {"Reward": 10, "Score": 100}
    
    # Update tables with example data
    rl_widget.update_input_table(observations)
    rl_widget.update_output_table(actions)
    rl_widget.update_reward_table(rewards)
    
    sys.exit(app.exec())

        