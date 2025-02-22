from gymnasium import Env, spaces
import numpy as np
import math
from collections import deque

import time

from Real_robot.real_rabbit import Rabbit_real



class RL_Env(Env):
    def __init__(self,
                ModelType="SAC",
                gui = True,
                render_mode = "human",
                observation_type_stacked=["head_orientation", "head_linear_acceleration", "head_angular_velocity", "joint_torques"],
                observation_type_solo=["phase_signal", "last_action", "User_command"],
                simulation_Timestep = 0.1,
                obs_time_space = 1, #in seconds
                n_stack = 5,
                real_robot = False,

                
                ):
        super(RL_Env, self).__init__()
        self.ModelType = ModelType
        self.observation_type_stacked = observation_type_stacked
        self.observation_type_solo = observation_type_solo
        self.simulation_Timestep = simulation_Timestep
        self.obs_time_space = obs_time_space

        self.gui = gui
        self.render_mode = render_mode
        self.n_steps = 0

        #self.rabbit = Rabbit_real()
        from mesure_rabbt import get_measuredRabbit 
        self.rabbit = get_measuredRabbit(Rabbit_real,
                                        state_types_body=["head_orientation", "head_linear_acceleration", "head_angular_velocity"], 
                                        state_types_servos=["joint_torques", "joint_velocities", "joint_angles"], 
                                        trajectory_data_structure= ["joint_torques"]
                                            )()
        self.rabbit.create_seperate_Window()
        
        self.action_low_high = [-1, 1]
        self.observation_low_high = [-1, 1]

        # Action space
        self.action_size = self.rabbit.numMotors

        observation_type_sizes = {
            "head_orientation": 3,
            "head_linear_acceleration": 3,
            "head_angular_acceleration": 3,
            "base_position": 3,
            "base_orientation": 3,
            "base_linear_velocity": 3,
            "base_angular_velocity": 3,
            "head_linear_velocity": 3,
            "head_angular_velocity": 3,
            "joint_angles": self.rabbit.numMotors,
            "joint_velocities": self.rabbit.numMotors,
            "joint_torques": self.rabbit.numMotors,

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
        # Frame stacking parameters      


        if ModelType == "SAC" or ModelType == "PPO":
            self.action_space = spaces.Box(low=self.action_low_high[0], high=self.action_low_high[1], shape=(self.action_size,), dtype=np.float32)
            self.observation_space = spaces.Box(low=self.observation_low_high[0], high=self.observation_low_high[1], shape=(self.observation_size_stacked*self.n_stack+self.observation_size_solo,), dtype=np.float32)
        print(f"Using {ModelType} model\n Action space: {self.action_size}\n Observation space: {self.observation_size_stacked* self.n_stack+self.observation_size_solo}")

        #to get the Observations
        self.get_rabbit_observation = self.rabbit.create_get_informations(self.observation_type_stacked)


        

    def euclidean_distance(self, list1, list2):
        """Calculate the euclidean distance between two lists.
        return: float"""
        array1, array2 = np.array(list1), np.array(list2)
        return np.linalg.norm(array1 - array2)
    
    def get_User_command_by_Trajectory(self, agent_pos, agent_orient, expert_pos, expert_orient):
        """
        Get the user command by the trajectory.
        This is calculated by the base_position difference between the agent and the expert
        and the base_orientation difference between the agent and the expert.

        agent_pos: [x, y, z]
        agent_orient: [roll, pitch, yaw]
        expert_pos: [x, y, z]
        expert_orient: [roll, pitch, yaw]

        return: [v_x, omega_z], [0, 0, angle_orient]
        """
        #calculate the difference between the agent and the expert
        diff = np.array(expert_pos) - np.array(agent_pos)
        #calculate the angle between the agent and the expert
        angle = math.atan2(diff[1], diff[0])
        #calculate the difference between the agent and the expert orientation
        diff_orient = agent_orient[2] - expert_orient[2]
        #calculate the difference between the agent and the expert orientation
        angle_orient = math.atan2(math.sin(diff_orient), math.cos(diff_orient))/math.pi
        
        #get v_x, v_y, from the perspective of the agent
        v_x = np.clip(np.linalg.norm(diff), -1, 1)
        omega_z = np.clip(angle_orient, -1, 1)
        return [v_x, omega_z], [0, 0, angle]

    
    def get_observation(self):
        
        #Stacked observations
        observation = np.array([])
        for i, obs in enumerate(self.get_rabbit_observation()):
            if self.observation_type_stacked[i] in ["euler_array", "base_orientation", "head_orientation", "joint_angles", "head_angular_velocity",]:
                # Flatten each part of the observation into a 1D array
                _array = np.array(obs) / (2*math.pi)
                observation = np.concatenate([observation, _array])
            
            elif self.observation_type_stacked[i] in ["head_orientation", "head_acceleration",  "joint_torques", "joint_velocities"]:
                _array = np.clip(np.array(obs) / 100, -1, 1)
                observation = np.concatenate([observation, _array])

            else:
                _array = np.clip(obs, -1, 1)
                observation = np.concatenate([observation, _array])

        #print("stacked_obs", len(observation))
         # Append the new observation to the stack
        if self.n_steps % int(self.obs_time_space/self.n_stack/self.simulation_Timestep) == 0:
            #print("New Observation", self.n_steps)
            self.stacked_frames.append(observation)
        else:
            #just replace the last(so the one in -1 possition) observation with the new observation
            self.stacked_frames[-1] = observation
            #print("Replace Observation", self.ROS_Env.simulation_steps)
        #print("stacked_frames", len(self.stacked_frames))
        
        # Concatenate stacked frames
        stacked_obs = np.concatenate(self.stacked_frames, axis=None)
        #print("stacked_obs", len(stacked_obs))

        
        #Single observations
        #add phase_signal
        if "phase_signal" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, [self.phase_generator.update()]])
        if "last_action" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, self.current_action])
        if "User_command" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, self.User_command])

        # 
        #check if the observation is the right size
        if len(stacked_obs) != self.observation_size_stacked* self.n_stack+self.observation_size_solo:
            raise ValueError(f"Observation size is not correct. Got {len(stacked_obs)} but expected {self.observation_size_stacked*self.n_stack+self.observation_size_solo}")
        return stacked_obs

    def step(self, action):
        self.current_action = action        
        # Execute one time step within the environment
        self.rabbit.send_goal_pose(action)
        #simulate the environment
        self.rabbit.step(timeStep_size=self.simulation_Timestep)
        #render the environment
        self.render()
        #get the outputs
        observations = self.get_observation()
        reward = 0
        terminated = False
        truncated = False

        info = {}
        

        self.n_steps += 1
        self.last_action = self.current_action
        self.last2_action = self.last_action
        #print("reward: ", reward, "\n action: ", action ,"\n observations: ", observations)
        
        return observations, reward, terminated, truncated, info

    def reset(self, seed=None):
        # Reset the state of the environment to an initial state
        self.rhythm = 0
        self.n_steps = 0           
        self.current_action = np.zeros(self.action_size)
        self.last_action = np.zeros(self.action_size)
        self.last2_action = np.zeros(self.action_size)

        if "phase_signal" in self.observation_type_solo:
            self.phase_generator = PhaseGenerator(is_periodic=True, duration=1.0, dt=self.simulation_Timestep)
        if "User_command" in self.observation_type_solo:
            self.User_command = list(np.zeros(2))

        # Reset the state of the environment to an initial state
        self.rabbit.reset()
        # Clear the frame stack and add the initial observation
        self.stacked_frames.clear()
        for _ in range(self.n_stack):
            self.stacked_frames.append(np.zeros(self.observation_size_stacked))
        #get the observations
        observations = np.array(self.get_observation())

        # put noise on the observation
        noise_low, noise_high = -0.8, 0.8
        observations = np.clip(observations + np.random.uniform(noise_low, noise_high, size=observations.shape), -1, 1)        
        #print("reset obs",len(observations))

        return observations, {}
    
    def render(self):
        if self.render_mode == "human":
            if not hasattr(self, "last_time"):
                self.last_time = time.time()
            if self.simulation_Timestep-(time.time()-self.last_time) > 0:
                time.sleep(self.simulation_Timestep-(time.time()-self.last_time))
            self.last_time = time.time()

    def close(self):
        pass

    def seed(self, seed=None):
        pass

    def configure(self, *args, **kwargs):
        pass



class PhaseGenerator:
    def __init__(self, is_periodic=True, duration=1.0, dt=0.02):
        """
        Initialize a phase signal generator.

        Args:
            is_periodic (bool): If True, phase loops continuously (for walking).
                                If False, phase stops at 1 (for episodic motions).
            duration (float): Total time for one cycle.
            dt (float): Time step per update.
        """
        self.phase = 0.0
        self.dt = dt
        self.duration = duration
        self.is_periodic = is_periodic

    def update(self):
        """Update the phase signal."""
        self.phase += self.dt / self.duration
        if self.is_periodic:
            self.phase %= 1.0  # Loop phase for walking
        else:
            self.phase = min(self.phase, 1.0)  # Stop at 1 for episodic motions
        return self.phase
    

    
if __name__ == "__main__":
    env = RL_Env()
    env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        print(action)
        obs, rew, done, trunc, info = env.step(action)
        if done:
            env.reset()
    env.close()



