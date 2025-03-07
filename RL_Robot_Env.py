from RL_Basis import RL_Base
import numpy as np
import math
from collections import deque
from tools.Phase_Generator import PhaseGenerator
import time

# from Real_robot.real_rabbit import Rabbit_real



class RL_Robot(RL_Base):
    def __init__(self,
                RobotType="Rabbit_real",
                gui = True,
                **kwargs
                
                ):
        super(RL_Robot, self).__init__()
        super().__init__(**kwargs)

        self.gui = gui

        if RobotType == "Rabbit_real":
            from Real_robot.real_rabbit import Rabbit_real
            self.rabbit = Rabbit_real(simulation_Timestep=self.simulation_Timestep)
        elif RobotType == "Rabbit_mesured":
            from Real_robot.real_rabbit import Rabbit_real
            from mesure_rabbt import get_measuredRabbit
            self.rabbit = get_measuredRabbit(Rabbit_real,
                                            state_types_body=["head_orientation"],
                                            state_types_servos=["joint_angles", "joint_velocities"], 
                                            trajectory_data_structure= ["joint_angles", "joint_velocities"]
                                                )(simulation_Timestep=self.simulation_Timestep)
            #self.rabbit.create_seperate_Window()
        else:
            raise ValueError("Unknown Robot type")
        
        
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
        stacked_obs = np.array([])
        for i, obs in enumerate(self.get_rabbit_observation()):
            if self.observation_type_stacked[i] in ["euler_array", "base_orientation", "head_orientation", "joint_angles", "head_angular_velocity",]:
                # Flatten each part of the observation into a 1D array
                _array = np.array(obs) / (2*math.pi)
                stacked_obs = np.concatenate([stacked_obs, _array])
            
            elif self.observation_type_stacked[i] in ["head_orientation", "head_acceleration",  "joint_torques", "joint_velocities"]:
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
        
        super().step()
        self.last_action = self.current_action
        self.last2_action = self.last_action
        #print("reward: ", reward, "\n action: ", action ,"\n observations: ", observations)
        
        return observations, reward, terminated, truncated, info

    def reset(self, seed=None):
        super().reset()


        if "phase_signal" in self.observation_type_solo:
            self.phase_generator = PhaseGenerator(is_periodic=True, duration=1.0, dt=self.simulation_Timestep)
        if "User_command" in self.observation_type_solo:
            self.User_command = list(np.zeros(2))

        # Reset the state of the environment to an initial state
        self.rabbit.reset()
        #get the observations
        observations = np.array(self.get_observation())

        # put noise on the observation
        noise_low, noise_high = -0.8, 0.8
        observations = np.clip(observations + np.random.uniform(noise_low, noise_high, size=observations.shape), -1, 1)        
        #print("reset obs",len(observations))

        return observations, {}
    


    def close(self):
        self.rabbit.close()

    def seed(self, seed=None):
        pass

    def configure(self, *args, **kwargs):
        pass
    

    
if __name__ == "__main__":
    env = RL_Robot(RobotType="Rabbit_mesured")
    env.rabbit.create_seperate_Window()
    env.reset()
    for _ in range(100000):
        action = [0]*8#env.action_space.sample()
        print(action)
        obs, rew, done, trunc, info = env.step(action)
        if done:
            env.reset()
    env.close()



