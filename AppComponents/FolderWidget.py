from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QMessageBox, QToolBar, QPushButton, QLabel, 
    QFileSystemModel, QTreeView, QAbstractItemView, QSizePolicy,QStyleFactory
)
from PySide6.QtCore import Signal, Qt, QSortFilterProxyModel
import os
import json


class PyFilesFilterProxyModel(QSortFilterProxyModel):
    def __init__(self, searchable_files, parent=None):
        super().__init__(parent)
        self.searchable_files = searchable_files
    def filterAcceptsRow(self, source_row, source_parent):
        index = self.sourceModel().index(source_row, 0, source_parent)
        if not index.isValid():
            return False
        
        file_info = self.sourceModel().fileInfo(index)
        
        # Show all folders
        if file_info.isDir():
            return True
        
        # Show only .json files
        return file_info.suffix() in self.searchable_files  # FIXED: Removed leading dot


class FolderWidget(QWidget):
    fileClicked = Signal(str, str)  # Signal with folder path and filename

    def __init__(self, folder_path, searchable_files=list(["json"]), parent=None):
        super().__init__(parent)
        self.folder_path = folder_path
        self.searchable_files = searchable_files  # Ensure consistency with no dots
        self.initUI()
        self.current_item = None

    def initUI(self):
        self.layout = QVBoxLayout(self)
        self.add_toolbar_file_explorer()  # Fixed: Call function to add toolbar
        self.layout.addWidget(self.toolbar_FileExplorer)  # FIXED: Add to layout

    def add_toolbar_file_explorer(self):
        # Toolbox File Explorer
        file_explorer_widget = QWidget()
        file_explorer_layout = QVBoxLayout(file_explorer_widget)

        # File Explorer Toolbar
        self.toolbar_FileExplorer = QToolBar("File Explorer")
        self.toolbar_FileExplorer.setObjectName("FileExplorerToolbar")
        self.toolbar_FileExplorer.setMinimumSize(400, 500)  
        self.toolbar_FileExplorer.setMovable(True)  
        self.toolbar_FileExplorer.setAllowedAreas(Qt.ToolBarArea.AllToolBarAreas)  
        self.toolbar_FileExplorer.setFloatable(True)  
        self.toolbar_FileExplorer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)  
        self.toolbar_FileExplorer.addWidget(file_explorer_widget)
        self.toolbar_FileExplorer.setHidden(False)  # FIXED: Ensure it's visible

        change_root_button = QPushButton("Select script path")
        file_explorer_layout.addWidget(change_root_button)

        self.file_explorer_path_label = QLabel(self.folder_path)
        self.file_explorer_path_label.setStyleSheet("font-weight: bold;")
        file_explorer_layout.addWidget(self.file_explorer_path_label)

        self.file_model = QFileSystemModel()
        self.file_model.setRootPath(self.folder_path)

        self.filter_model = PyFilesFilterProxyModel(self.searchable_files)  
        self.filter_model.setSourceModel(self.file_model)

        self.file_exp_tree = QTreeView(file_explorer_widget)
        self.file_exp_tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.file_exp_tree.setModel(self.filter_model)
        self.file_exp_tree.setRootIndex(self.filter_model.mapFromSource(self.file_model.index(self.folder_path)))
        self.file_exp_tree.setColumnWidth(0, 160)
        self.file_exp_tree.setColumnWidth(1, 80)
        self.file_exp_tree.setColumnWidth(2, 80)
        self.file_exp_tree.setColumnWidth(3, 80)

        # Enable drag & drop properly
        self.file_exp_tree.setDragEnabled(True)
        self.file_exp_tree.setAcceptDrops(True)
        self.file_exp_tree.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)  # FIXED: Use InternalMove
        self.file_exp_tree.setDefaultDropAction(Qt.DropAction.MoveAction)

        file_explorer_layout.addWidget(self.file_exp_tree)

        self.file_exp_tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.file_exp_tree.doubleClicked.connect(self.on_tree_item_double_clicked)  # Connect double-click signal

    def on_tree_item_double_clicked(self, index):
        file_info = self.file_model.fileInfo(self.filter_model.mapToSource(index))
        if file_info.isFile():
            file_name = file_info.fileName()
            folder_path = os.path.dirname(file_info.filePath())
            self.on_item_clicked(folder_path, file_name)

    def on_item_clicked(self, folder_path, file_name):
        # Find the item in the tree view and set its background color
        index = self.file_model.index(os.path.join(folder_path, file_name))
        tree_index = self.filter_model.mapFromSource(index)
        self.file_exp_tree.setCurrentIndex(tree_index)

        self.fileClicked.emit(folder_path, file_name)  # Emit the signal

        #self.open_trajectory(folder_path, file_name)

    def open_trajectory(self, folder_path, file_name):
        file_path = os.path.join(folder_path, file_name)
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
                print(f"Opened {file_name}: {data}")  # Process JSON
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open {file_name}: {e}")

    def set_folder_path(self, folder_path):
        self.folder_path = folder_path
        self.file_model.setRootPath(self.folder_path)
        self.file_explorer_path_label.setText(self.folder_path)


if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    QApplication.setStyle(QStyleFactory.create("Fusion"))
    folder_widget = FolderWidget(r"Trajectories")
    folder_widget.fileClicked.connect(lambda folder, file: print(f"File clicked: {folder}/{file}"))
    folder_widget.show()
    sys.exit(app.exec())
