from Env import Simulation
from tools.Trajectory import TrajectoryRecorder
from tools.SpaceRepetition import FlashTrajectoryDeck
from tools.Phase_Generator import PhaseGenerator

import numpy as np
import math
from collections import deque

import time

from RL_Basis import RL_Base



class RL_Env(RL_Base):
    def __init__(self,
                RobotType="Rabbit_v3",
                gui = True,
                rewards_type=["Disney_Imitation"],
                recorded_movement_file_path_dic = {"Bewegung2": 4, 
                                                    },
                terrain_type = "random_terrain",
                different_terrain = False,
                different_gravity = False,
                restriction_2D = False,
                
                real_robot = False,

                reward_weights = {
                # Imitation
                "torso_pos": 1.5,
                "torso_orient": 2.0,
                "linear_vel_xy": 0.5,
                "linear_vel_z": 0.5,
                "angular_vel_xy": 0.5,
                "angular_vel_z": 0.5,
                "LegJoint_pos": 3.0,
                "LegJoint_vel": 0.25,
                "component_coordinates_world": 2.0,
                #"Contact": 1.0,

                # Regularization
                "Joint_torques": 0.15,
                "Joint_acc": 0.15,
                "action_rate": 1.0,
                "action_acc": 0.05,

                # Survival
                "survival": 1.5,
            },
                **kwargs
                ):
        super(RL_Env, self).__init__()
        super().__init__(**kwargs)
        
        self.RewardsType = rewards_type
        self.recorded_movement_file_path_dic = recorded_movement_file_path_dic

        self.terrain_type = terrain_type
        self.restriction_2D = restriction_2D
        self.different_terrain = different_terrain
        self.different_gravity = different_gravity

        self.reward_weights = reward_weights

        self.gui = gui
        
        self.expert_trajectory = TrajectoryRecorder()

        self.simulation = Simulation(gui=gui, simulation_speed=self.render_mode, simulation_Timestep=self.simulation_Timestep, terrain_type = terrain_type, rabbit_type = RobotType)

        if "Disney_Imitation" in self.RewardsType:
            self.TrajectroyDeck = FlashTrajectoryDeck(self.recorded_movement_file_path_dic, max_revisions=10, random_interval=12)
        #to get important datas
        self.infos_types = ["base_position", "base_orientation", "base_linear_velocity", "joint_torques", "component_coordinates_world", "joint_acceleration"]
        self.get_rabbit_infos = self.simulation.rabbit.create_get_informations(self.infos_types)
        #to get the Observations
        self.get_rabbit_observation = self.simulation.rabbit.create_get_informations(self.observation_type_stacked)


        

    def euclidean_distance(self, list1, list2, norm=1):
        """Calculate the euclidean distance between two lists.
        return: float"""
        array1, array2 = np.array(list1)/norm, np.array(list2)/norm
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
    
    def calculate_reward(self): 
        if "Disney_Imitation" in self.RewardsType:            
            # Weights from the provided reward function

            def reverse_exp_reward(base, agent_value, expert_value, weight, normalize_value = 1):
                distance = self.euclidean_distance(agent_value, expert_value, norm=normalize_value)
                return np.exp(-base * distance** 2)*weight
            
            def exp_sum_reward(base, agent_value, expert_value, weight, normalize_value = 1):
                distance = np.sum(self.euclidean_distance(agent_value[j], expert_value[j], norm=normalize_value)**2 for j in range(len(agent_value)))
                return np.exp(-base * distance)*weight
            
            def quad_reward(agent_value, expert_value, weight):
                return -(self.euclidean_distance(agent_value, expert_value) ** 2)*weight
            
            r_imitation = 0
            r_regularization = 0
            r_survival = 0



            self.simulation.show_Points([self.expert_states[0]], color=[0, 1, 0])
            
            # print("Expert States:", self.expert_states, len(self.expert_states))
            # print("Rabbit States:", self.get_rabbit_states(), len(self.get_rabbit_states()))
            #data_structure = ["base_position", "base_orientation", "base_linear_velocity", "base_angular_velocity", "joint_angles", "joint_torques", "joint_velocities"]
            base_position, base_orientation, base_linear_velocity, base_angular_velocity, joint_angles, joint_torques, joint_velocities, component_coordinates_world = self.get_rabbit_states()
            expert_position, expert_orientation, expert_linear_velocity, expert_angular_velocity, expert_joint_angles, expert_joint_torques, expert_joint_velocities, expert_component_coordinates_world = self.expert_states
            # Imitation
            r_imitation += reverse_exp_reward(10, base_position, expert_position, self.reward_weights["torso_pos"])
            r_imitation += reverse_exp_reward(20, base_orientation, expert_orientation, self.reward_weights["torso_orient"], normalize_value=math.pi)
            #for the component cooddinates
            r_imitation += reverse_exp_reward(30, component_coordinates_world, expert_component_coordinates_world, self.reward_weights["component_coordinates_world"])
            r_imitation += reverse_exp_reward(0.5, base_linear_velocity[:2], expert_linear_velocity[:2], self.reward_weights["linear_vel_xy"])
            r_imitation += reverse_exp_reward(5, base_linear_velocity[2], expert_linear_velocity[2], self.reward_weights["linear_vel_z"])
            r_imitation += reverse_exp_reward(0.5, base_angular_velocity[:2], expert_angular_velocity[:2], self.reward_weights["angular_vel_xy"], normalize_value=math.pi)
            r_imitation += reverse_exp_reward(5, base_angular_velocity[2], expert_angular_velocity[2], self.reward_weights["angular_vel_z"], normalize_value=math.pi)
            r_imitation += reverse_exp_reward(20, joint_angles, expert_joint_angles, self.reward_weights["LegJoint_pos"], normalize_value=math.pi)
            r_imitation += reverse_exp_reward(0.5, joint_velocities, expert_joint_velocities, self.reward_weights["LegJoint_vel"], normalize_value=math.pi)
            #contact !!!

            base_position, base_orientation, base_linear_velocity, joint_torques, component_coordinates_world, joint_acceleration = self.get_rabbit_infos()
            # Regularization
            r_regularization += -np.clip((np.linalg.norm(joint_torques)**2) * self.reward_weights["Joint_torques"], 0, self.reward_weights["Joint_torques"])
            r_regularization += -np.clip((np.linalg.norm(joint_acceleration/(8*math.pi**3)) **2)* self.reward_weights["Joint_acc"], 0, self.reward_weights["Joint_acc"])
            r_regularization += -np.clip((self.euclidean_distance(self.current_action, np.array(self.last_action))**2)* self.reward_weights["action_rate"], 0, self.reward_weights["action_rate"])
            r_regularization += -np.clip((np.linalg.norm(np.array(self.current_action)-2*np.array(self.last_action)+np.array(self.last2_action))**2)* self.reward_weights["action_acc"], 0, self.reward_weights["action_acc"])

            # Survival
            r_survival += self.reward_weights["survival"]

            tot_reward = r_imitation + r_regularization + r_survival

            #give feedback to the trajectory deck
            max_score = sum(self.reward_weights.values())
            score = (tot_reward) / max_score
            #gives a score between 0 and 5 in integer
            print("score", score)
            self.TrajectroyDeck.collect_episode_feedback(score)


            return tot_reward
        

        else:
            return 0

            
    def check_terminated(self):
        #get the get_rabbit_infos
        base_position, base_orientation, base_linear_velocity, joint_torques, component_coordinates_world, joint_acceleration = self.get_rabbit_infos()
        #check if the robot is terminated
        #checks how many steps where simulated, stops the simulation when the animation is over
        #if (self.ROS_Env.simulation_steps*10*self.speed)+self.start_recording_time > self.end_recording_time:
        if self.Horizon_Length:
            if abs(base_orientation[0]) > math.pi/2 or abs(base_orientation[1]) > math.pi/2  or base_position[2] < 0.075:
                terminated = True
            elif self.simulation.rabbit.check_delicate_collision(self.simulation.ground.id):
                terminated = True
            elif "User_command" in self.observation_type_solo and self.User_command[0] >= 0.3:
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
            if hasattr(self.expert_trajectory, "trajectory_time") and self.n_steps*self.simulation_Timestep >= self.expert_trajectory.trajectory_times[-1]:
                return True
            elif not hasattr(self.expert_trajectory, "trajectory_time") and self.n_steps*self.simulation_Timestep >= 10:
                return True
            else:
                return False
        else:
            return False
    
    def get_observation(self):
        
        #Stacked observations
        stacked_obs = np.array([])
        for i, obs in enumerate(self.get_rabbit_observation()):
            if self.observation_type_stacked[i] in ["euler_array", "base_orientation", "head_orientation", "joint_angles", "head_angular_velocity",]:
                # Flatten each part of the observation into a 1D array
                _array = np.array(obs) / (2*math.pi)
                stacked_obs = np.concatenate([stacked_obs, _array])
            
            elif self.observation_type_stacked[i] in ["base_position", "base_linear_velocity", "base_angular_velocity", "head_linear_acceleration",  "joint_torques", "joint_velocities"]:
                _array = np.clip(np.array(obs) / 100, -1, 1)
                stacked_obs = np.concatenate([stacked_obs, _array])

            else:
                _array = np.clip(obs, -1, 1)
                stacked_obs = np.concatenate([stacked_obs, _array])
        
        #Single observations
        #add phase_signal
        observation_solo = np.array([])
        if "phase_signal" in self.observation_type_solo:
            observation_solo = np.concatenate([observation_solo, [self.phase_generator.update()]])
        if "last_action" in self.observation_type_solo:
            observation_solo = np.concatenate([observation_solo, self.current_action])
        if "User_command" in self.observation_type_solo:
            observation_solo = np.concatenate([observation_solo, self.User_command])
        return super().get_observation(stacked_obs, observation_solo)

    def step(self, action):
        self.current_action = action        
        # Execute one time step within the environment
        self.simulation.rabbit.send_goal_pose(action)
        #simulate the environment
        self.simulation.Sim_step()

        if self.restriction_2D:
            self.simulation.rabbit.restrict_2D()

        if "Disney_Imitation" in self.RewardsType:
            #find the experts nearest action/data to the current time
            self.expert_states = self.expert_trajectory.get_near_data(self.n_steps*self.simulation_Timestep)
            #print("expert_states", self.expert_states)

        if "User_command" in self.observation_type_solo:
            #get the automated User command
            agent_pos, agent_orient = self.get_rabbit_infos()[:2]
            expert_pos, expert_orient, _ = self.expert_states[:3]
            self.User_command, User_orient = self.get_User_command_by_Trajectory(agent_pos, agent_orient, expert_pos, expert_orient)


        

        #get the outputs
        observations = self.get_observation()
        reward = self.calculate_reward()
        terminated = self.check_terminated()
        truncated = self.check_truncated()
        info = {}
        super().step()
        self.last_action = self.current_action
        self.last2_action = self.last_action
        #print("reward: ", reward, "action: ", action)# ,"\n observations: ", observations)
        
        return observations, reward, terminated, truncated, info

    def reset(self, seed=None):
        super().reset()

        if "Disney_Imitation" in self.RewardsType:
            if hasattr(self.expert_trajectory, "trajectory_name"):
                self.TrajectroyDeck.give_feedback(self.expert_trajectory.trajectory_name, self.TrajectroyDeck.return_episode_feedback())       
            #Open the recorded movement files
            self.expert_trajectory.load_trajectory(self.TrajectroyDeck.get_trajectory_path())
            #to get the expert data_types
            self.get_rabbit_states = self.simulation.rabbit.create_get_informations(self.expert_trajectory.data_structure)

        if "phase_signal" in self.observation_type_solo:
            self.phase_generator = PhaseGenerator(is_periodic=True, duration=1.0, dt=self.simulation_Timestep)
        if "User_command" in self.observation_type_solo:
            self.User_command = list(np.zeros(2))

        # Reset the state of the environment to an initial state
        if hasattr(self, "num_reset"):
            self.num_reset += 1
        else:
            self.num_reset = 1
        print("reset number: ", self.num_reset)
        #reset the simulation
        self.simulation.reset(seed=seed, reset_gravity_direction=self.num_reset % 25 == 0 and self.different_gravity, reset_terrain=self.num_reset % 25 == 0 and self.different_terrain)#resets the robot and the terrain every 25 resets

        #get the observations
        observations = np.array(self.get_observation())

        # put noise on the observation
        noise_low, noise_high = -0.1, 0.1
        observations = np.clip(observations + np.random.uniform(noise_low, noise_high, size=observations.shape), -1, 1)   
        #print("reset obs",len(observations))

        return observations, {}

    def close(self):
        self.simulation.close()

    def seed(self, seed=None):
        pass

    def configure(self, *args, **kwargs):
        pass




    

    
if __name__ == "__main__":
    env_param_kwargs = {
        "ModelType": "SAC",
        "render_mode": "human",
        "RobotType": "Rabbit_v3",
        "rewards_type": ["Disney_Imitation"],
        "observation_type_stacked": ["head_orientation", "joint_torques"],
        "observation_type_solo": ["phase_signal", "last_action", "User_command"],
        "Horizon_Length": True,
        "obs_time_space": 2,
        "simulation_Timestep": 0.25,
        "terrain_type": "random_terrain",
        "recorded_movement_file_path_dic": {
                                             r"Bewegung1": 5,
                                             },
    }
    env = RL_Env(**env_param_kwargs)

    # env = RL_Env(
    #     RobotType="Rabbit_v3_mesured",
    # )
    # env.simulation.rabbit.create_seperate_Window()
    env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        obs, rew, done, trunc, info = env.step(action)
        done = done or trunc
        if obs.shape[0] != env.observation_size_stacked* env.n_stack+env.observation_size_solo:
            print("Error")
            break
        if done:
            env.reset()
        time.sleep(0.01)

    #testing the env
    env.close()



