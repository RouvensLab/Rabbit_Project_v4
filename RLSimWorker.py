from PySide6.QtCore import Signal, QObject
from RL_Agent_Env import RL_Env
from RL_Robot_Env import RL_Robot

from Manual_Expert import ManualExpert
from tools.Controler import ControlInput

from tools.TimeInterval import TimeInterval
import threading

class RobotControler(QObject):
    return_feedback = Signal(int)  # Emit the row number to highlight in the table

    def __init__(self, controling_class):
        super().__init__()
        self.controling_class = controling_class
        self.lock = threading.Lock()  # Lock to ensure thread safety

    def change_controler(self, controler):
        """Change the controler of the robot"""
        with self.lock:  # Acquire the lock to prevent concurrent access
            self.controling_class = controler

    def get_actions(self, obs, life_time, done):
        with self.lock:  # Acquire the lock to ensure thread safety
            if isinstance(self.controling_class, ControlInput):
                self.action = self.controling_class.get_BodyPose()
                #print(self.action)
            elif isinstance(self.controling_class, ManualExpert): # Manual Expert is controlling the robot
                self.action, _, action_key = self.controling_class.think_and_respond(obs, None, done, life_time)
                print(self.action)
                # Highlight the active row
                #action_key_index = list(self.manual_exp.action_timetable.keys()).index(self.manual_exp.action_key)
                self.return_feedback.emit(action_key)  # Emit the row number to highlight in the table

            else: # A RL-Agent is controlling the robot
                self.action, _ = self.controling_class.predict(obs)
                print("RL-Agent Prediction: ", self.action)
            return self.action

    def close(self):
        with self.lock:  # Acquire the lock to ensure thread safety
            if isinstance(self.controling_class, ControlInput):
                self.controling_class.close()
            elif isinstance(self.controling_class, ManualExpert):
                self.controling_class = None
            else:
                self.controling_class = None
        

class SimulationWorker(QObject):
    finished = Signal()
    error = Signal(str)
    observation_trans = Signal(object)  # Emit observations to the main thread
    action_trans = Signal(object)  # Emit actions to the main thread
    env_feedback_trans = Signal(dict)  # Emit feedback to the main thread
    reset_done_trans = Signal()

    def __init__(self, rl_env:RL_Env, rl_robot:RL_Robot, robot_controler:RobotControler, live_time: callable, time_step, control_mode: callable, isControlled: callable, auto_reset: callable, env_pause: callable, speed:callable=lambda: 1.0):
        super().__init__()
        self.RlEnv = rl_env
        self.RlRobot = rl_robot
        self.robot_controler = robot_controler

        self.time_step = time_step
        self.control_mode = control_mode


        self.live_time = live_time
        self.auto_reset = auto_reset
        self.isControlled = isControlled
        self.running = True
        self.env_pause = env_pause
        self.speed = speed

        #general variables
        self.action = None
        self.obs = None

    def reset_simulation(self):    
        #emit reset signal to the main thread
        self.reset_done_trans.emit()

        # Reset the environment and robot
        print("Resetting simulation. . .")        
        if self.isControlled() == "Only Simulation":
            self.obs, inf = self.RlEnv.reset()
        elif self.isControlled() == "Only Real Robot":
            self.obs, inf = self.RlRobot.reset()
        elif self.isControlled() == "Simulation_Imitation":
            self.obs, inf = self.RlEnv.reset()
            self.RlRobot.reset()
        else:
            raise ValueError("Invalid isControlled mode.")
        
        return self.obs, inf

    def run(self):
        try:
            self.no_error = True
            self.end_thread = False
            self.IntTimer = TimeInterval(self.time_step)
            self.obs, _ = self.reset_simulation()
            self.observation_trans.emit(self.obs)
            if self.obs is None:
                self.error.emit("Failed to reset environment.")
                return
            while self.running:
                done = False
                while not done and self.running:
                    print("Simulation running-------------")
                    if not self.env_pause():
                        self.action = self.robot_controler.get_actions(self.obs, self.live_time(), done)
                        self.action_trans.emit(self.action)

                        try:
                            if self.isControlled() == "Only Simulation":
                                self.obs, reward, terminated, truncated, info = self.RlEnv.step(self.action)
                            elif self.isControlled() == "Simulation_Imitation":
                                _, _, _, _, _ = self.RlRobot.step(self.action)
                                self.obs, reward, terminated, truncated, info = self.RlEnv.step(self.action)

                            elif self.isControlled() == "Only Real Robot":
                                self.obs, reward, terminated, truncated, info = self.RlRobot.step(self.action)
                            done = (terminated or truncated) and self.auto_reset()
                            self.observation_trans.emit(self.obs)
                            self.env_feedback_trans.emit({"reward": reward, "terminated": terminated, "truncated": truncated, **info})

                            if done:
                                self.reset_done_trans.emit()
                                self.obs, info = self.reset_simulation()

                        except Exception as e:
                            self.no_error = False
                            print(f"Error in environment step: {e}")
                        
                        self.IntTimer.wait_for_step((1+self.speed())*self.time_step)
        except Exception as e:
            self.error.emit(str(e))
        finally:
            self.finished.emit()

            

    def stop(self):
        self.running = False