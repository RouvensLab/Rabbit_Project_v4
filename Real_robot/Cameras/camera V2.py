from PySide6.QtCore import QObject, Signal, Slot, QThread
from PySide6.QtGui import QPixmap
from PySide6.QtMultimedia import QMediaCaptureSession, QVideoSink, QVideoFrame, QCamera
from PySide6.QtWidgets import QMainWindow, QLabel
import time


class FrameWorker(QObject):

    pixmapChanged = Signal(QPixmap)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ready = True

    @Slot(QVideoFrame)
    def setVideoFrame(self, frame: QVideoFrame):
        self.ready = False
        time.sleep(0.01)  # represents time-consuming work
        self.pixmapChanged.emit(QPixmap.fromImage(frame.toImage()))
        self.ready = True


class FrameSender(QObject):
    frameChanged = Signal(QVideoFrame)


class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.workerThread = QThread()
        self.captureSession = QMediaCaptureSession()
        self.frameSender = FrameSender()
        self.frameWorker = FrameWorker()
        self.displayLabel = QLabel()

        self.frameWorker.moveToThread(self.workerThread)
        self.workerThread.start()

        self.captureSession.setVideoSink(QVideoSink(self))
        self.captureSession.videoSink().videoFrameChanged.connect(
            self.onFramePassedFromCamera
        )
        self.frameSender.frameChanged.connect(self.frameWorker.setVideoFrame)
        self.frameWorker.pixmapChanged.connect(self.displayLabel.setPixmap)
        self.setCentralWidget(self.displayLabel)

        camera = QCamera(self)
        self.captureSession.setCamera(camera)
        camera.start()

        # Add status bar
        self.statusBar().showMessage("Ready")

        # Add menu bar with File -> Exit
        menubar = self.menuBar()
        fileMenu = menubar.addMenu("File")
        exitAction = fileMenu.addAction("Exit")
        exitAction.triggered.connect(self.close)

        # Scale the camera size to half
        self.frameWorker.pixmapChanged.connect(self.scalePixmap)

    @Slot(QPixmap)
    def scalePixmap(self, pixmap: QPixmap):
        scaled_pixmap = pixmap.scaled(pixmap.width() // 2, pixmap.height() // 2)
        self.displayLabel.setPixmap(scaled_pixmap)

    @Slot(QVideoFrame)
    def onFramePassedFromCamera(self, frame: QVideoFrame):
        if self.frameWorker.ready:
            self.frameSender.frameChanged.emit(frame)

    def closeEvent(self, event):
        self.workerThread.quit()
        self.workerThread.wait()
        super().closeEvent(event)


if __name__ == "__main__":
    from PySide6.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)
    window = Window()
    window.show()
    app.exec()
    app.quit()