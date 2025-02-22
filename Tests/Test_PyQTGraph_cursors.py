import sys
import numpy as np
from PySide6.QtWidgets import QMainWindow, QApplication, QVBoxLayout, QWidget, QLabel
from PySide6.QtCore import QTimer
import pyqtgraph as pg
from PySide6.QtGui import QFont

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-time Sine Wave with Cursors")
        self.setGeometry(100, 100, 800, 600)

        # Create main widget and layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)

        # Create labels for cursor values
        self.label_a = QLabel("Cursor A: 0.00")
        self.label_b = QLabel("Cursor B: 0.00")
        self.label_a.setFont(QFont("Arial", 10))
        self.label_b.setFont(QFont("Arial", 10))
        self.layout.addWidget(self.label_a)
        self.layout.addWidget(self.label_b)

        # Initialize data
        self.time = np.linspace(0, 10, 1000)
        self.phase = 0
        self.data = np.sin(self.time + self.phase)

        # Create plot
        self.curve = self.plot_widget.plot(self.time, self.data, pen='y')
        self.plot_widget.setLabel('left', 'Amplitude')
        self.plot_widget.setLabel('bottom', 'Time')
        self.plot_widget.setYRange(-1.5, 1.5)

        # Add vertical cursors (InfiniteLine)
        self.cursor_a = pg.InfiniteLine(pos=2.0, angle=90, movable=True, pen='r')
        self.cursor_b = pg.InfiniteLine(pos=4.0, angle=90, movable=True, pen='g')
        self.plot_widget.addItem(self.cursor_a)
        self.plot_widget.addItem(self.cursor_b)

        # Connect cursor movement signals
        self.cursor_a.sigPositionChanged.connect(self.update_labels)
        self.cursor_b.sigPositionChanged.connect(self.update_labels)

        # Initial update of labels
        self.update_labels()

        # Setup timer for real-time update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # Update every 50ms

    def update_plot(self):
        # Update phase for animation
        self.phase += 0.05
        self.data = np.sin(self.time + self.phase)
        self.curve.setData(self.time, self.data)
        self.update_labels()

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())