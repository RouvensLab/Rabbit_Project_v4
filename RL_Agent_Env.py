from gymnasium import Env, spaces
from Env import Simulation
from tools.Trajectory import TrajectoryRecorder
from tools.SpaceRepetition import FlashTrajectoryDeck

import numpy as np
import math
from collections import deque

import time



class RL_Env(Env):
    def __init__(self,
                ModelType="SAC",
                RobotType="Rabbit_v3",
                gui = True, 
                render_mode="human",
                rewards_type=["Disney_Imitation"], 
                observation_type_stacked=["head_orientation", "head_acceleration", "head_angular_acceleration", "joint_torques"],
                observation_type_solo=["phase_signal", "last_action", "User_command"],
                Horizon_Length = True,
                simulation_Timestep = 0.1,
                obs_time_space = 1, #in seconds
                recorded_movement_file_path_dic = {r"Trajectory1": 4, 
                                                    },
                terrain_type = "random_terrain",
                
                real_robot = False,

                
                ):
        super(RL_Env, self).__init__()
        self.ModelType = ModelType
        self.RewardsType = rewards_type
        self.observation_type_stacked = observation_type_stacked
        self.observation_type_solo = observation_type_solo
        self.Horizon_Length = Horizon_Length
        self.simulation_Timestep = simulation_Timestep
        self.obs_time_space = obs_time_space
        self.recorded_movement_file_path_dic = recorded_movement_file_path_dic

        self.gui = gui
        self.n_steps = 0
        
        self.expert_trajectory = TrajectoryRecorder()

        self.simulation = Simulation(gui=gui, simulation_speed=render_mode, terrain_type = terrain_type, rabbit_type = RobotType)
        
        self.action_low_high = [-1, 1]
        self.observation_low_high = [-1, 1]

        # Action space
        self.action_size = self.simulation.rabbit.numMotors
        if "rhythm" in self.observation_type_stacked:
            self.action_size += 1
            #rhytm values
            self.rhythm = 0

        observation_type_sizes = {
            "head_orientation": 3,
            "head_acceleration": 3,
            "head_angular_acceleration": 3,
            "base_position": 3,
            "base_orientation": 3,
            "base_linear_velocity": 3,
            "base_angular_velocity": 3,
            "head_linear_velocity": 3,
            "head_angular_velocity": 3,
            "joint_angles": self.simulation.rabbit.numMotors,
            "joint_velocities": self.simulation.rabbit.numMotors,
            "joint_torques": self.simulation.rabbit.numMotors,

            "feet_forces": 4,
            "goal_orientation": 2,
            "rhythm": 1,
            "phase_signal": 1,
            "last_action": self.action_size,
            "User_command": 3,
        }
        self.n_stack = 3  # Number of frames to stack
        self.stacked_frames = deque(maxlen=self.n_stack)
        self.obs_time_space = obs_time_space #in seconds
        # Observation space
        self.observation_size_stacked = sum([observation_type_sizes[obs] for obs in self.observation_type_stacked]) * self.n_stack
        self.observation_size_solo = sum([observation_type_sizes[obs] for obs in self.observation_type_solo])
        # Frame stacking parameters


        


        if ModelType == "SAC" or ModelType == "PPO":
            self.action_space = spaces.Box(low=self.action_low_high[0], high=self.action_low_high[1], shape=(self.action_size,), dtype=np.float16)
            self.observation_space = spaces.Box(low=self.observation_low_high[0], high=self.observation_low_high[1], shape=(self.observation_size_stacked+self.observation_size_solo,), dtype=np.float16)
        print(f"Using {ModelType} model\n Action space: {self.action_size}\n Observation space: {self.observation_size_stacked+self.observation_size_solo}")

        #to get important datas
        self.infos_types = ["base_position", "base_orientation", "base_linear_velocity", "joint_torques", "component_coordinates_world"]
        self.get_rabbit_infos = self.simulation.rabbit.create_get_informations(self.infos_types)
        #to get the Observations
        self.get_rabbit_observation = self.simulation.rabbit.create_get_informations(self.observation_type_stacked)


        

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

        return: [v_x, v_y, omega_z], [0, 0, angle_orient]
        """
        #calculate the difference between the agent and the expert
        diff = np.array(agent_pos) - np.array(expert_pos)
        #calculate the angle between the agent and the expert
        angle = math.atan2(diff[1], diff[0])
        #calculate the difference between the agent and the expert orientation
        diff_orient = agent_orient[2] - expert_orient[2]
        #calculate the difference between the agent and the expert orientation
        angle_orient = math.atan2(math.sin(diff_orient), math.cos(diff_orient))/math.pi
        
        #clip the values between -1 and 1
        v_x = np.clip(diff[0], -1, 1)
        v_y = np.clip(diff[1], -1, 1)
        omega_z = np.clip(angle_orient, -1, 1)
        return [v_x, v_y, omega_z], [0, 0, angle]
    
    def calculate_reward(self): 
        if "Disney_Imitation" in self.RewardsType:            
            # Weights from the provided reward function
            weights = {
                "torso_pos": 1.0,
                "torso_orient": 1.0,
                "linear_vel_xy": 1.0,
                "linear_vel_z": 1.0,
                "angular_vel_xy": 0.5,
                "angular_vel_z": 0.5,
                "LegJoint_pos": 15.0,
                "LegJoint_vel": 1.0*10e-3,
                "Contact": 1.0,

                "Joint_torques": 1.0*10e-3,
                "Joint_acc": 2.5*10e-6,
                "action_rate": 1.5,
                "action_acc": 0.45,

                "survival": 20.0,
            }

            def reverse_exp_reward(base, agent_value, expert_value, weight):
                return np.exp(-base * self.euclidean_distance(agent_value, expert_value) ** 2)*weight
            
            def quad_reward(agent_value, expert_value, weight):
                return -(self.euclidean_distance(agent_value, expert_value) ** 2)*weight
            
            r_imitation = 0
            r_regularization = 0
            r_survival = 0



            self.simulation.show_Points([self.expert_states[0]], color=[0, 1, 0])
            
            # print("Expert States:", self.expert_states, len(self.expert_states))
            # print("Rabbit States:", self.get_rabbit_states(), len(self.get_rabbit_states()))
            #data_structure = ["base_position", "base_orientation", "base_linear_velocity", "base_angular_velocity", "joint_angles", "joint_torques", "joint_velocities", "joint_action_rate", "joint_action_acceleration"]
            base_position, base_orientation, base_linear_velocity, base_angular_velocity, joint_angles, joint_torques, joint_velocities, joint_action_rate, joint_action_acceleration = self.get_rabbit_states()
            expert_position, expert_orientation, expert_linear_velocity, expert_angular_velocity, expert_joint_angles, expert_joint_torques, expert_joint_velocities, expert_joint_action_rate, expert_joint_action_acceleration = self.expert_states
            # Imitation

            r_imitation += reverse_exp_reward(200, base_position[:2], expert_position[:2], weights["torso_pos"])
            r_imitation += reverse_exp_reward(20, base_orientation, expert_orientation, weights["torso_orient"])
            r_imitation += reverse_exp_reward(8, base_linear_velocity[:2], expert_linear_velocity[:2], weights["linear_vel_xy"])
            r_imitation += reverse_exp_reward(8, base_linear_velocity[2], expert_linear_velocity[2], weights["linear_vel_z"])
            r_imitation += reverse_exp_reward(2, base_angular_velocity[:2], expert_angular_velocity[:2], weights["angular_vel_xy"])
            r_imitation += reverse_exp_reward(2, base_angular_velocity[2], expert_angular_velocity[2], weights["angular_vel_z"])
            r_imitation += quad_reward(joint_angles, expert_joint_angles, weights["LegJoint_pos"])
            r_imitation += quad_reward(joint_velocities, expert_joint_velocities, weights["LegJoint_vel"])
            #contact !!!

            # Regularization
            r_regularization += -(self.euclidean_distance(joint_torques, np.zeros_like(joint_torques))**2) * weights["Joint_torques"]
            #joint acceleration
            # r_regularization += -(self.euclidean_distance(joint_action_rate, np.zeros_like(joint_action_rate)) **2)* weights["action_rate"]
            # r_regularization += -(self.euclidean_distance(joint_action_acceleration, np.zeros_like(joint_action_acceleration)) **2)* weights["action_acc"]
            r_regularization += -1.5 * np.linalg.norm(self.current_action - self.last_action)**2

            # Survival
            r_survival += weights["survival"]

            tot_reward = r_imitation + r_regularization + r_survival

            #give feedback to the trajectory deck
            max_score = sum(weights.values())
            score = (tot_reward) / max_score
            #gives a score between 0 and 5 in integer
            self.TrajectroyDeck.give_feedback(self.expert_trajectory.trajectory_path, int(score*5))


            return tot_reward
        

        else:
            return 0

            
    def check_terminated(self):
        #get the get_rabbit_infos
        base_position, base_orientation, base_linear_velocity, joint_torques, component_coordinates_world = self.get_rabbit_infos()
        #check if the robot is terminated
        #checks how many steps where simulated, stops the simulation when the animation is over
        #if (self.ROS_Env.simulation_steps*10*self.speed)+self.start_recording_time > self.end_recording_time:
        if self.Horizon_Length:
            if abs(base_orientation[0]) > math.pi or abs(base_orientation[1]) > math.pi  or base_position[2] < 0.005:
                terminated = True
            elif self.simulation.rabbit.check_delicate_self_collision():
                terminated = True
            else:
                terminated = False
        # elif self.Horizon_Length and self.ROS_Env.simulation_steps >= self.maxSteps:
        #     terminated = True
        else:
            terminated = False
        return terminated
    
    def check_truncated(self):
        """Check if the episode was truncated. That means when the limit of maximal steps in a episode is reached."""
        if self.Horizon_Length:
            if hasattr(self.expert_trajectory, "trajectory_time") and self.n_steps*self.simulation_Timestep >= self.expert_trajectory.trajectory_time[-1]:
                return True
            elif not hasattr(self.expert_trajectory, "trajectory_time") and self.n_steps*self.simulation_Timestep >= 10:
                return True
            else:
                return False
        else:
            return False
    
    def get_observation(self):
        
        #Stacked observations
        observation = np.array([])
        for i, obs in enumerate(self.get_rabbit_observation()):
            if self.observation_type_stacked[i] in ["euler_array", "base_orientation", "head_orientation", "joint_angles"]:
                # Flatten each part of the observation into a 1D array
                _array = np.array(obs) / (2*math.pi)
                observation = np.concatenate([observation, _array])
            
            elif self.observation_type_stacked[i] in ["base_position", "base_linear_velocity", "base_angular_velocity", "head_acceleration", "head_angular_acceleration", "joint_torques", "joint_velocities"]:
                _array = np.clip(np.array(obs) / 100, -1, 1)
                observation = np.concatenate([observation, _array])

            else:
                _array = np.clip(obs, -1, 1)
                observation = np.concatenate([observation, _array])
            #print(len(_array))

         # Append the new observation to the stack
        if self.n_steps % int(self.obs_time_space/self.n_stack/self.simulation_Timestep) == 0:
            #print("New Observation", self.n_steps)
            self.stacked_frames.append(observation)
        else:
            #just replace the last(so the one in -1 possition) observation with the new observation
            self.stacked_frames[-1] = observation
            #print("Replace Observation", self.ROS_Env.simulation_steps)
        
        # Concatenate stacked frames
        stacked_obs = np.concatenate(self.stacked_frames, axis=None)

        
        #Single observations
        #add phase_signal
        if "phase_signal" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, [self.phase_generator.update()]])
        if "last_action" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, self.current_action])
        if "User_command" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, self.User_command])
        if "rhythm" in self.observation_type_solo:
            stacked_obs = np.concatenate([stacked_obs, [self.rhythm]])
        #print("get_observation",observation, len(observation))       
        #  
        return stacked_obs

    def step(self, action):
        self.current_action = action        
        # Execute one time step within the environment
        self.simulation.rabbit.send_goal_pose(action)
        #simulate the environment
        self.simulation.Sim_step()

        if "Disney_Imitation" in self.RewardsType:
            #find the experts nearest action/data to the current time
            self.expert_states = self.expert_trajectory.get_near_data(self.n_steps*self.simulation_Timestep)

        if "User_command" in self.observation_type_solo:
            #get the automated User command
            agent_pos, agent_orient = self.get_rabbit_infos()[:2]
            expert_pos, expert_orient = self.expert_states[:2]
            self.User_command, User_orient = self.get_User_command_by_Trajectory(agent_pos, agent_orient, expert_pos, expert_orient)


        

        #get the outputs
        observations = self.get_observation()
        reward = self.calculate_reward()
        terminated = self.check_terminated()
        truncated = self.check_truncated()
        info = {}
        

        self.n_steps += 1
        
        return observations, reward, terminated, truncated, info

    def reset(self, seed=None):
        # Reset the state of the environment to an initial state
        self.rhythm = 0
        self.n_steps = 0

        if "Disney_Imitation" in self.RewardsType:

            self.TrajectroyDeck = FlashTrajectoryDeck(self.recorded_movement_file_path_dic)
            #Open the recorded movement files
            self.expert_trajectory.load_trajectory(self.TrajectroyDeck.get_trajectory_path())
            #to get the expert data_types
            self.get_rabbit_states = self.simulation.rabbit.create_get_informations(self.expert_trajectory.data_structure)

        self.current_action = np.zeros(self.action_size)
        self.last_action = np.zeros(self.action_size)

        if "phase_signal" in self.observation_type_solo:
            self.phase_generator = PhaseGenerator(is_periodic=True, duration=1.0, dt=self.simulation_Timestep)
        if "User_command" in self.observation_type_solo:
            self.User_command = [0, 0, 0]

        # Reset the state of the environment to an initial state
        self.simulation.rabbit.reset()
        # Clear the frame stack and add the initial observation
        self.stacked_frames.clear()
        for _ in range(self.n_stack):
            self.stacked_frames.append(np.zeros(self.observation_size_stacked))
        #get the observations
        observations = np.array(self.get_observation())

        # put noise on the observation
        noise_low, noise_high = -0.8, 0.8
        observations = np.clip(observations + np.random.uniform(noise_low, noise_high, size=observations.shape), -1, 1)
        
        if "rhythm" in self.observation_type_stacked:
            observations[-1] = self.rhythm #because we don't want the rhythm to be changed by the noise
        
        #print("reset obs",len(observations))

        return observations, {}

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
    

    




