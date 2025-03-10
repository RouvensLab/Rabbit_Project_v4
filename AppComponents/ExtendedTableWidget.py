from PySide6.QtWidgets import (
    QApplication,  QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,QStatusBar,
    QPushButton, QHeaderView, QMessageBox, QSlider, QFileDialog, QLabel, QCheckBox, QComboBox, QMainWindow, QTabWidget
)
from PySide6.QtGui import QIcon
from PySide6.QtCore import Qt,QSettings, QThread, Signal, Slot


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
        