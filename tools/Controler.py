import pygame
import threading
from pynput import keyboard

class VirtualJoystick:
    def __init__(self):
        self.vector = [0, 0]
        #give the joystick dynamics. Meaning that the value is accelerating towards the desired value and deaccelerating when the desired value is not the same as the current value
        self.dynamics = 0.2

    def update_vector(self, keys_pressed, up_key, down_key, left_key, right_key):
        target_vector = [0, 0]
        if left_key in keys_pressed and not right_key in keys_pressed:
            target_vector[0] = -1
        elif right_key in keys_pressed and not left_key in keys_pressed:
            target_vector[0] = 1
        if up_key in keys_pressed and not down_key in keys_pressed:
            target_vector[1] = -1
        elif down_key in keys_pressed and not up_key in keys_pressed:
            target_vector[1] = 1

        # Apply dynamics to the vector
        self.vector[0] += (target_vector[0] - self.vector[0]) * self.dynamics
        self.vector[1] += (target_vector[1] - self.vector[1]) * self.dynamics

        return self.vector
    
    def close(self):
        pass

class VirtualControler:
    def __init__(self):
        self.left_joystick = VirtualJoystick()
        self.right_joystick = VirtualJoystick()

        # Start a listener for global key events
        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()
        self.keys_pressed = set()
    
    def on_press(self, key):
        try:
            self.keys_pressed.add(key.char)
        except AttributeError:
            self.keys_pressed.add(key)

    def on_release(self, key):
        try:
            self.keys_pressed.discard(key.char)
        except AttributeError:
            self.keys_pressed.discard(key)

    def get_left_JoystickVector(self):
        return self.left_joystick.update_vector(self.keys_pressed, 'w', 's', 'a', 'd')

    def get_right_JoystickVector(self):
        return self.right_joystick.update_vector(self.keys_pressed, 'i', 'k', 'j', 'l')
    
    # def get_Button(self, button):
    #     return button in self.keys_pressed
    
    def close(self):
        self.key_listener.stop()
    
class ControlInput:
    def __init__(self, main_window):
        self.main_window = main_window  # Reference to the MainWindow
        self.joystick = None

        # Joystick setup with pygame
        pygame.init()
        pygame.joystick.init()
        self.check_joystick()

        # Start a background thread to detect joystick connections
        self.running = True
        self.joystick_thread = threading.Thread(target=self.detect_joysticks)
        self.joystick_thread.start()

        # Start a background thread to listen to JoyButton events
        self.joybutton_thread = threading.Thread(target=self.JoyButton_event_listener)
        self.joybutton_thread.start()


        # initialize the virtual controler
        self.virtual_controler = VirtualControler()

        self.Button_connect_func = {
            "a": [],
            "b": [],
            "x": [],
            "y": [],
            "RB": [],
            "LB": [],
        }
        self.Button_translate = {
            "a": 0,
            "b": 1,
            "x": 2,
            "y": 3,
            "RB": 5,
            "LB": 4,
        }




    def check_joystick(self):
        pygame.event.pump()  # Process internal pygame events
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self.joystick = pygame.joystick.Joystick(event.device_index)
                print(f"Joystick connected: {self.joystick.get_id()}")
        self.joystick_num = pygame.joystick.get_count()

    def detect_joysticks(self):
        while self.running:
            self.check_joystick()
            pygame.time.wait(1000)


    def close(self):
        self.running = False
        self.joystick_thread.join()
        self.joybutton_thread.join()
        if self.joystick:
            self.joystick.quit()
        pygame.quit()
        self.virtual_controler.close()

    #event listener for Buttons
    def Button_connect(self, button, func):
        self.Button_connect_func[button].append(func)

    # the keyboard event listener
    def JoyButton_event_listener(self):
        while self.running:
            if self.joystick:
                pygame.event.pump()
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        #print(f"Button {event.button} pressed")
                        for button, funcs in self.Button_connect_func.items():
                            if event.button == self.Button_translate[button]:
                                for func in funcs:
                                    func()
            pygame.time.wait(100)
            

    

    def get_left_JoystickVector(self, normalize=True):
        """Returns the current control input vector as a tuple (x, y) in range -1 to 1."""
        vector = [0, 0]
        pygame.event.pump()  # Process internal pygame events
        if self.joystick:
            vector[0] = self.joystick.get_axis(0)
            vector[1] = self.joystick.get_axis(1)
        else:
            vector = self.virtual_controler.get_left_JoystickVector()

        #print(self.vector)
        if normalize:
            vector_length = (vector[0]**2 + vector[1]**2)**0.5
            if vector_length > 1:
                vector[0] /= vector_length
                vector[1] /= vector_length
        return vector
    
    def get_right_JoystickVector(self, normalize=True):
        """Returns the current control input vector as a tuple (x, y) in range -1 to 1."""
        vector = [0, 0]
        pygame.event.pump()  # Process internal pygame events
        if self.joystick:
            vector[0] = self.joystick.get_axis(2)
            vector[1] = self.joystick.get_axis(3)
        else:
            vector = self.virtual_controler.get_right_JoystickVector()

        if normalize:
            vector_length = (vector[0]**2 + vector[1]**2)**0.5
            if vector_length > 1:
                vector[0] /= vector_length
                vector[1] /= vector_length
        return vector
    
    def get_LRT_ButtonsVector(self, normalize=True):
        """Returns the current control input vector as a tuple (x, y) in range -1 to 1."""
        vector = [0, 0]
        if self.joystick:
            pygame.event.pump()  # Process internal pygame events
            vector[0] = self.joystick.get_axis(4)
            vector[1] = self.joystick.get_axis(5)
        else:
            vector[0] = 0
            vector[1] = 0

        if normalize:
            vector_length = (vector[0]**2 + vector[1]**2)**0.5
            if vector_length > 1:
                vector[0] /= vector_length
                vector[1] /= vector_length
        return vector

    def get_BodyPose(self):
        """This returns only the body pose action
        That contains 8 Servo values
        """
        pose_action = [0 for i in range(8)]
        left_vec = self.get_left_JoystickVector(normalize=False)
        pose_action[2] = left_vec[0]
        pose_action[3] = left_vec[1]
        pose_action[4] = left_vec[0]
        pose_action[5] = left_vec[1]
        pose_action[1], pose_action[0] = self.get_right_JoystickVector(normalize=False)
        pose_action[6], pose_action[7] = self.get_LRT_ButtonsVector(normalize=False)

        return pose_action

if __name__ == "__main__":
    control_input = ControlInput(None)
    control_input.Button_connect("a", lambda: print("Button A pressed"))
    control_input.Button_connect("RB", lambda: print("Button RB pressed"))
    for i in range(100):
        print([round(i, 2) for i in control_input.get_BodyPose()])
        pygame.time.wait(100)
    
    control_input.close()
    control_input = None
    #create
    control_input = ControlInput(None)
    for i in range(100):
        print(control_input.get_BodyPose())
        pygame.time.wait(100)

    control_input.close()
