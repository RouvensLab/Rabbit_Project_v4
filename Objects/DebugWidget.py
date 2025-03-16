import pybullet as p
import pybullet_data
import time


class DebugSwitch:
    def __init__(self, name, default_value):
        """This button is used to switch between different states on/off"""
        self.name = name
        self.default_value = default_value
        self.id = p.addUserDebugParameter(self.name, 1, 0, self.default_value)
        self.state = self.default_value

    def get_value(self):
        return p.readUserDebugParameter(self.id)
    
    def check_if_on(self):
        current_value = self.get_value()
        print(current_value)
        if current_value != self.default_value:
            self.default_value = current_value
            self.state = not self.state
        return self.state
    def remove(self):
        p.removeUserDebugItem(self.id)

class DebugSlider:
    def __init__(self, name, default_value, min_value, max_value):
        self.name = name
        self.default_value = default_value
        self.id = p.addUserDebugParameter(self.name, min_value, max_value, self.default_value)

    def get_value(self):
        return p.readUserDebugParameter(self.id)
    
    def check_if_changed(self):
        return self.get_value() != self.default_value
    def remove(self):
        p.removeUserDebugItem(self.id)

class DebugButton:
    def __init__(self, name):
        self.name = name
        self.id = p.addUserDebugParameter(self.name, 1, 0, 0)

    def check_if_clicked(self):
        clicked = False
        # Get the current state of the reset button
        current_button_value = p.readUserDebugParameter(self.id)
        if not hasattr(self, "previous_button_value"):
            self.previous_button_value = current_button_value
        # Check if the button was pressed (value transition from 0 to 1)
        if current_button_value > self.previous_button_value:
            clicked = True
        # Update previous button state for the next check
        self.previous_button_value = current_button_value
        return clicked
    
    def remove(self):
        p.removeUserDebugItem(self.id)
    
    