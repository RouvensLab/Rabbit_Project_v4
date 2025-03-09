from PySide6.QtWidgets import QWidget, QVBoxLayout, QListWidget, QMessageBox, QToolBar, QPushButton, QLabel, QFileSystemModel, QTreeView, QAbstractItemView, QSizePolicy
from PySide6.QtCore import Signal, Qt, QDir
from PySide6.QtGui import QSortFilterProxyModel
import os
import json


class PyFilesFilterProxyModel(QSortFilterProxyModel):
    def filterAcceptsRow(self, source_row, source_parent):
        index = self.sourceModel().index(source_row, 0, source_parent)
        if not index.isValid():
            return False
        
        file_info = self.sourceModel().fileInfo(index)
        
        # Show all folders
        if file_info.isDir():
            return True
        
        # Show only .py files
        return file_info.suffix()=='.json'

class FolderWidget(QWidget):
    fileClicked = Signal(str)
    def __init__(self, folder_path, searchable_files:list[str] = [".json"], parent=None):
        super().__init__(parent)
        self.folder_path = folder_path
        self.searchable_files = searchable_files
        self.initUI()
        self.current_item = None

    def initUI(self):
        self.layout = QVBoxLayout(self)
        self.file_list = QListWidget(self)
        self.layout.addWidget(self.file_list)

        self.file_list.itemClicked.connect(self.on_item_clicked)
        self.populate_file_list()

        self.add_toolbar_file_explorer()    # New Script Explorer DARFT

    def populate_file_list(self):
        if not os.path.exists(self.folder_path):
            QMessageBox.critical(self, "Error", f"Folder path {self.folder_path} does not exist.")
            return

        for root, dirs, files in os.walk(self.folder_path):
            for file_name in files:
                if any(file_name.endswith(ext) for ext in self.searchable_files):
                    relative_path = os.path.relpath(os.path.join(root, file_name), self.folder_path)
                    self.file_list.addItem(relative_path)

    def add_toolbar_file_explorer(self):
        # Add Toolbox File Explorer on the left
        file_explorer_widget = QWidget()
        file_explorer_layout = QVBoxLayout(file_explorer_widget)

        # Add File Explorer to the toolbar
        self.toolbar_FileExplorer = QToolBar("File Explorer")
        self.toolbar_FileExplorer.setObjectName("FileExplorerToolbar")
        self.toolbar_FileExplorer.setMinimumSize(400, 500)  # Set minimum size
        self.toolbar_FileExplorer.setMovable(True)  # Make toolbar movable
        self.toolbar_FileExplorer.setAllowedAreas(Qt.ToolBarArea.AllToolBarAreas)  # Allow toolbar in all areas
        self.toolbar_FileExplorer.setFloatable(True)  # Allow toolbar to be floatable
        self.toolbar_FileExplorer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)  # Make toolbar resizable
        self.toolbar_FileExplorer.addWidget(file_explorer_widget)
        self.toolbar_FileExplorer.setHidden(True)
        self.addToolBar(Qt.ToolBarArea.LeftToolBarArea, self.toolbar_FileExplorer)
        
        # Add Toolbox File Explorer on the left
        self.file_explorer_root_path = QDir.currentPath()

        change_root_button = QPushButton("Select script path")
        change_root_button.clicked.connect(self.change_script_path)
        file_explorer_layout.addWidget(change_root_button)

        self.file_explorer_path_label = QLabel(f"{self.file_explorer_root_path}")
        self.file_explorer_path_label.setStyleSheet("font-weight: bold;")
        file_explorer_layout.addWidget(self.file_explorer_path_label)

        self.file_model = QFileSystemModel()
        self.file_model.setRootPath(QDir.currentPath())

        self.filter_model = PyFilesFilterProxyModel()  # Filter to show only ".py" files
        self.filter_model.setSourceModel(self.file_model)

        self.file_exp_tree = QTreeView(file_explorer_widget)
        self.file_exp_tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.file_exp_tree.setModel(self.filter_model)
        self.file_exp_tree.setRootIndex(self.filter_model.mapFromSource(self.file_model.index(QDir.currentPath())))
        self.file_exp_tree.setColumnWidth(0, 280)
        self.file_exp_tree.setColumnWidth(1, 200)
        #self.file_exp_tree.clicked.connect(self.on_file_exp_selected)
        
        # Enable drag & drop
        self.file_exp_tree.setDragEnabled(True)
        self.file_exp_tree.setAcceptDrops(True)
        self.file_exp_tree.setDragDropMode(QTreeView.DragDropMode.DragDrop)
        self.file_exp_tree.setDefaultDropAction(Qt.DropAction.MoveAction)
        
        self.file_exp_tree.selectionModel().selectionChanged.connect(self.on_file_exp_selection_changed)  # self.Init_selected_script

        file_explorer_layout.addWidget(self.file_exp_tree)

        # Add context menu for right-click
        self.file_exp_tree.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.file_exp_tree.customContextMenuRequested.connect(self.show_Seq_Result_context_menu)

    def on_item_clicked(self, item):
        file_name = item.text()
        self.open_trajectory(file_name)
        
        # Set the color of the current item to green
        item.setBackground(Qt.green)
        self.current_item = item

        # Reset the color of the previously clicked item
        if self.current_item:
            self.current_item.setBackground(Qt.white)

    def open_trajectory(self, file_name):
        file_path = os.path.join(self.folder_path, file_name)
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
                # Process the JSON data as needed
                print(f"Opened {file_name}: {data}")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to open {file_name}: {e}")


if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    folder_widget = FolderWidget(r"Trajectories")
    folder_widget.show()
    sys.exit(app.exec())