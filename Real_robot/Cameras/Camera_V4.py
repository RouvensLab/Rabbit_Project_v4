import sys
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                              QHBoxLayout, QLabel, QPushButton)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
import pyqtgraph as pg

class AccelerometerVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Accelerometer Data Visualizer")
        self.setGeometry(100, 100, 800, 600)

        # Main widget and layout
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.layout = QVBoxLayout(self.main_widget)

        # Setup plot
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle("Accelerometer Readings")
        self.plot_widget.setLabel('left', 'Acceleration', units='m/s²')
        self.plot_widget.setLabel('bottom', 'Time', units='s')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()
        self.plot_widget.setYRange(-5, 5)  # Set initial visible range
        
        # Data storage
        self.max_points = 100
        self.x_data = np.zeros(self.max_points)
        self.y_data = np.zeros(self.max_points)
        self.z_data = np.zeros(self.max_points)
        self.time_data = np.linspace(-10, 0, self.max_points)

        # Plot curves with thicker lines
        pen_width = 2
        self.x_curve = self.plot_widget.plot(self.time_data, self.x_data, 
                                           pen=pg.mkPen('r', width=pen_width), 
                                           name='X-axis')
        self.y_curve = self.plot_widget.plot(self.time_data, self.y_data, 
                                           pen=pg.mkPen('g', width=pen_width), 
                                           name='Y-axis')
        self.z_curve = self.plot_widget.plot(self.time_data, self.z_data, 
                                           pen=pg.mkPen('b', width=pen_width), 
                                           name='Z-axis')

        # Control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        
        # Status labels
        self.x_label = QLabel("X: 0.00 m/s²")
        self.y_label = QLabel("Y: 0.00 m/s²")
        self.z_label = QLabel("Z: 0.00 m/s²")
        
        for label in (self.x_label, self.y_label, self.z_label):
            label.setFont(QFont('Arial', 12))
            control_layout.addWidget(label)

        # Start/Stop button
        self.start_stop_btn = QPushButton("Start")
        self.start_stop_btn.clicked.connect(self.toggle_recording)
        control_layout.addWidget(self.start_stop_btn)

        # Add widgets to main layout
        self.layout.addWidget(self.plot_widget)
        self.layout.addWidget(control_panel)

        # Timer for updating data
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.is_recording = False
        self.time = 0

    def toggle_recording(self):
        if self.is_recording:
            self.timer.stop()
            self.start_stop_btn.setText("Start")
            self.is_recording = False
        else:
            self.timer.start(100)  # Update every 100ms
            self.start_stop_btn.setText("Stop")
            self.is_recording = True

    def update_data(self):
        self.time += 0.1
        # Generate more visible data (larger amplitude)
        new_x = 3 * np.sin(self.time) + np.random.normal(0, 0.2)
        new_y = 3 * np.cos(self.time) + np.random.normal(0, 0.2)
        new_z = 2 * np.sin(self.time * 2) + np.random.normal(0, 0.2)

        # Shift data and append new values
        self.x_data[:-1] = self.x_data[1:]
        self.y_data[:-1] = self.y_data[1:]
        self.z_data[:-1] = self.z_data[1:]
        self.time_data[:-1] = self.time_data[1:]

        self.x_data[-1] = new_x
        self.y_data[-1] = new_y
        self.z_data[-1] = new_z
        self.time_data[-1] = self.time

        # Update plots
        self.x_curve.setData(self.time_data, self.x_data)
        self.y_curve.setData(self.time_data, self.y_data)
        self.z_curve.setData(self.time_data, self.z_data)

        # Update labels
        self.x_label.setText(f"X: {new_x:.2f} m/s²")
        self.y_label.setText(f"Y: {new_y:.2f} m/s²")
        self.z_label.setText(f"Z: {new_z:.2f} m/s²")

        # Optional: Print debug info
        print(f"X: {new_x:.2f}, Y: {new_y:.2f}, Z: {new_z:.2f}")

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    app.setStyleSheet("""
        QWidget {
            font-family: Arial;
            font-size: 12px;
        }
        QPushButton {
            background-color: #4CAF50;
            color: white;
            padding: 5px;
            border-radius: 3px;
        }
        QPushButton:hover {
            background-color: #45a049;
        }
    """)
    
    window = AccelerometerVisualizer()
    window.show()
    sys.exit(app.exec())