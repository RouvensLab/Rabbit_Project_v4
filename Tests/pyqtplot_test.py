import sys
import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget

class MultiAxisPlot(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQtGraph Multi Y-Axis with Three Subplots")
        self.setGeometry(100, 100, 800, 600)
        
        # Central Widget
        widget = QWidget()
        self.setCentralWidget(widget)
        layout = QVBoxLayout()
        widget.setLayout(layout)
        
        # Create Graphics Layout Widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.plot_widget)
        
        # Generate Sample Data
        x = np.linspace(0, 10, 100)
        y1 = np.sin(x)
        y2 = np.cos(x)
        y3 = np.tan(x) * 0.1  # Scale to avoid extreme values
        
        # Create Subplots
        self.plot1 = self.plot_widget.addPlot(row=1, col=0)
        self.plot_widget.nextRow()
        self.plot2 = self.plot_widget.addPlot(row=3, col=0)
        self.plot_widget.nextRow()
        self.plot3 = self.plot_widget.addPlot(row=5, col=0)
        
        # Adjust all plot sizes equally
        self.plot_widget.ci.layout.setRowStretchFactor(1, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(3, 1)
        self.plot_widget.ci.layout.setRowStretchFactor(5, 1)
        
        # Link X-Axis
        self.plot2.setXLink(self.plot1)
        self.plot3.setXLink(self.plot1)
        
        # Plot Data
        self.curve1 = self.plot1.plot(x, y1, pen='r', name="Sine Wave")
        self.curve2 = self.plot2.plot(x, y2, pen='g', name="Cosine Wave")
        self.curve3 = self.plot3.plot(x, y3, pen='b', name="Tangent Wave")
        
        # Add Legends on top of each plot
        self.legend1 = pg.LegendItem(offset=(0, -30))
        self.plot_widget.addItem(self.legend1, row=0, col=0)
        self.legend1.addItem(self.curve1, "Sine Wave")

        self.legend2 = pg.LegendItem(offset=(0, -30))
        self.plot_widget.addItem(self.legend2, row=2, col=0)
        self.legend2.addItem(self.curve2, "Cosine Wave")

        self.legend3 = pg.LegendItem(offset=(0, -30))
        self.plot_widget.addItem(self.legend3, row=4, col=0)
        self.legend3.addItem(self.curve3, "Tangent Wave")
        
        # Connect legend items to toggle visibility
        for item in self.legend1.items:
            item[1].mouseClickEvent = lambda ev: self.toggle_visibility(self.curve1)
        for item in self.legend2.items:
            item[1].mouseClickEvent = lambda ev: self.toggle_visibility(self.curve2)
        for item in self.legend3.items:
            item[1].mouseClickEvent = lambda ev: self.toggle_visibility(self.curve3)
        
        # Multiple Y-Axes for Plot 1
        self.right_axis1 = pg.AxisItem("right")
        self.plot1.layout.addItem(self.right_axis1, 2, 2)
        self.right_axis1.linkToView(self.plot1.vb)
        self.right_axis1.setLabel("Sine Wave", color="r")
        
        # Multiple Y-Axes for Plot 2
        self.right_axis2 = pg.AxisItem("right")
        self.plot2.layout.addItem(self.right_axis2, 2, 2)
        self.right_axis2.linkToView(self.plot2.vb)
        self.right_axis2.setLabel("Cosine Wave", color="g")
        
        # Multiple Y-Axes for Plot 3
        self.right_axis3 = pg.AxisItem("right")
        self.plot3.layout.addItem(self.right_axis3, 2, 2)
        self.right_axis3.linkToView(self.plot3.vb)
        self.right_axis3.setLabel("Scaled Tangent", color="b")
        
        # Labels
        self.plot1.setLabel("left", "Sine Wave")
        self.plot2.setLabel("left", "Cosine Wave")
        self.plot3.setLabel("left", "Tangent Wave")
        self.plot3.setLabel("bottom", "Time (s)")
        
        # Show Grid
        for plot in [self.plot1, self.plot2, self.plot3]:
            plot.showGrid(x=True, y=True)
        
        # Hide X-Axis for the first two plots
        self.plot1.getAxis('bottom').setStyle(showValues=False)
        self.plot2.getAxis('bottom').setStyle(showValues=False)
        
        # Set fixed width for left and right axes to ensure same width
        left_axis_width = 50
        right_axis_width = 50
        self.plot1.getAxis('left').setWidth(left_axis_width)
        self.plot2.getAxis('left').setWidth(left_axis_width)
        self.plot3.getAxis('left').setWidth(left_axis_width)
        self.right_axis1.setWidth(right_axis_width)
        self.right_axis2.setWidth(right_axis_width)
        self.right_axis3.setWidth(right_axis_width)
        
        # Align right end of the plots
        self.plot1.getAxis('right').setWidth(right_axis_width)
        self.plot2.getAxis('right').setWidth(right_axis_width)
        self.plot3.getAxis('right').setWidth(right_axis_width)
        
    def toggle_visibility(self, curve):
        curve.setVisible(not curve.isVisible())
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MultiAxisPlot()
    window.show()
    sys.exit(app.exec())
