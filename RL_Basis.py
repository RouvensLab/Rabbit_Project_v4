from collections import deque
from gymnasium import Env, spaces

import numpy as np
import time

class RL_Base(Env):
    def __init__(self,
                ModelType="SAC",
                render_mode = "human",
                observation_type_stacked=["head_orientation", "head_linear_acceleration", "head_angular_velocity", "joint_torques"],
                observation_type_solo=["phase_signal", "last_action", "User_command"],
                observation_noise = 0,
                simulation_Timestep = 0.1,
                obs_time_space = 1, #in seconds
                n_stack = 5,
                Horizon_Length = True
                ):
        super(RL_Base, self).__init__()
        self.ModelType = ModelType
        self.observation_type_stacked = observation_type_stacked
        self.observation_type_solo = observation_type_solo
        self.observation_noise = observation_noise
        self.simulation_Timestep = simulation_Timestep
        self.obs_time_space = obs_time_space
        self.render_mode = render_mode
        self.n_steps = 0
        self.Horizon_Length = Horizon_Length

        self.action_size = 8
        observation_type_sizes = {
            "base_position": 3,
            "base_orientation": 3,
            "base_linear_velocity": 3,
            "base_angular_velocity": 3,
            "head_orientation": 3,
            "head_linear_acceleration": 3,
            "head_angular_acceleration": 3,
            "head_linear_velocity": 3,
            "head_angular_velocity": 3,
            "joint_angles": 8,
            "joint_velocities": 8,
            "joint_torques": 8,

            "total_current": 3,

            "feet_forces": 4,
            "phase_signal": 1,
            "last_action": self.action_size,
            "User_command": 2,
        }

        self.n_stack = n_stack  # Number of frames to stack
        self.stacked_frames = deque(maxlen=self.n_stack)
        self.obs_time_space = obs_time_space #in seconds
        # Observation space
        self.observation_size_stacked = sum([observation_type_sizes[obs] for obs in self.observation_type_stacked]) 
        self.observation_size_solo = sum([observation_type_sizes[obs] for obs in self.observation_type_solo])


        self.action_low_high = [-1, 1]
        self.observation_low_high = [-1, 1]

        if ModelType == "SAC" or ModelType == "PPO":
            self.action_space = spaces.Box(low=self.action_low_high[0], high=self.action_low_high[1], shape=(self.action_size,), dtype=np.float16)
            self.observation_space = spaces.Box(low=self.observation_low_high[0], high=self.observation_low_high[1], shape=(self.observation_size_stacked*self.n_stack+self.observation_size_solo,), dtype=np.float16)
        print(f"Using {ModelType} model\n Action space: {self.action_size}\n Observation space: {self.observation_size_stacked* self.n_stack+self.observation_size_solo}")



    def get_observation(self, observation_stack, observation_solo):
        
        #Stacked observations

        # add noise to the observation stack
        if self.observation_noise:
            observation_stack = np.clip(observation_stack + np.random.normal(0, self.observation_noise, size=observation_stack.shape), -1, 1)

        #print("stacked_obs", len(observation))
         # Append the new observation to the stack
        if self.n_steps % int(self.obs_time_space/(self.n_stack*self.simulation_Timestep)) == 0:
            #print("New Observation", self.n_steps)
            self.stacked_frames.append(observation_stack)
        else:
            #just replace the last(so the one in -1 possition) observation with the new observation
            self.stacked_frames[-1] = observation_stack
            #print("Replace Observation", self.ROS_Env.simulation_steps)
        #print("stacked_frames", len(self.stacked_frames))
        
        # Concatenate stacked frames
        stacked_obs = np.concatenate(self.stacked_frames, axis=None, dtype=np.float16)

        # Solo observations
        solo_obs = np.array(observation_solo, dtype=np.float16)
        stacked_obs = np.concatenate([stacked_obs, solo_obs], axis=None)

        #check if the observation is the right size
        if len(stacked_obs) != self.observation_size_stacked*self.n_stack+self.observation_size_solo:
            print("Error")
            raise ValueError(f"Observation size is not correct. Got {len(stacked_obs)} but expected {self.observation_size_stacked*self.n_stack+self.observation_size_solo}")
        return stacked_obs
    
    def step(self):
        self.n_steps += 1

    def reset(self):
        # Reset the state of the environment to an initial state
        self.n_steps = 0
        self.current_action = np.zeros(self.action_size)
        self.last_action = np.zeros(self.action_size)
        self.last2_action = np.zeros(self.action_size)

        # Clear the frame stack and add the initial observation
        self.stacked_frames.clear()
        for _ in range(self.n_stack):
            self.stacked_frames.append(np.zeros(self.observation_size_stacked))

    def render(self):
        if self.render_mode == "human":
            if not hasattr(self, "last_time"):
                self.last_time = time.time()
            if self.simulation_Timestep-(time.time()-self.last_time) > 0:
                time.sleep(self.simulation_Timestep-(time.time()-self.last_time))
            self.last_time = time.time()
