#from Rabbit import Rabbit

from Objects.Env_surroundings import Terrain

import pybullet as p
import pybullet_data

import time
import numpy as np

from tools.TimeInterval import TimeInterval

class Simulation:
    def __init__(self,
                 gui=True,
                 simulation_speed="human",
                 simulation_Timestep = 0.05,
                 terrain_type = "random_terrain",
                 rabbit_type = "Rabbit_v3"
    ):
        super().__init__()
        self.GUI = gui
        self.connection_id = None
        self.initialize_physics_server()

        self.time_interval = TimeInterval(time_per_step=simulation_Timestep)


        
        #the ground of the simulation needs to be created first. Afterwards the rabbit can be created
        self.ground = Terrain(terrain_type)

        if rabbit_type == "Rabbit_v3":
            from Objects.Rabbit_v3 import Rabbit
            self.rabbit = Rabbit([0, 0, 0.15])
            #self.rabbit.simplify_collision(self.ground._id)

        elif rabbit_type == "Rabbit_v3_mesured":
            from Objects.Rabbit_v3 import Rabbit
            from mesure_rabbt import get_measuredRabbit
            self.rabbit = get_measuredRabbit(Rabbit,
                                            state_types_body=["head_orientation", "head_angular_velocity", "head_linear_acceleration" ], 
                                            state_types_servos=["joint_angles", "joint_velocities", "joint_torques"], 
                                            trajectory_data_structure= ["base_position", "base_orientation", "base_linear_velocity", "base_angular_velocity", "joint_angles", "joint_torques", "joint_velocities", "component_coordinates_world", "action"]
                                             )([0, 0, 0.15])
        elif rabbit_type == "Rabbit_real_mesured":
            from Real_robot.real_rabbit import Rabbit_real
            from mesure_rabbt import get_measuredRabbit
            self.rabbit = get_measuredRabbit(Rabbit_real,
                                            state_types_body=["head_orientation", "head_angular_velocity", "head_linear_acceleration" ], 
                                            state_types_servos=["joint_currents", "joint_velocities", "joint_torques", "joint_angles"], 
                                            trajectory_data_structure= []
                                             )([0, 0, 0.15])
            #self.rabbit.init_Gui()
        else:
            raise ValueError("Unknown Rabbit type")
        self.hung = False
        
        p.setGravity(0, 0, -9.81)
        self.simulation_Timestep = simulation_Timestep
        self.simulationSpeed = simulation_speed
        
        #p.saveWorld("rabit.py")
        #self.rabbit.get_link_infos()

        #time.sleep(1000)
        
        #self.rabbit.send_motor_commands([0, 0, 0, 0,  0, 0,0,   0, 0,0,   0, 0])
        #self.rabbit.add_UserControlPanel(all_joints=False)
        p.stepSimulation()

    def initialize_physics_server(self):
        if self.connection_id is not None:
            try:
                p.disconnect(self.connection_id)
            except:
                pass
                
        try:
            if self.GUI:
                self.connection_id = p.connect(p.GUI)
            else:
                self.connection_id = p.connect(p.DIRECT)
        except Exception as e:
            print(f"Failed to connect to physics server: {e}")
            self.connection_id = None
            raise



    def show_world_coordinate_system(self):
        #show the world coordinate system
        p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 1)
        p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 1)
        p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 1)
        #show labled axes of the coordinate system
        p.addUserDebugText("X", [1, 0, 0], [1, 0, 0], 1)
        p.addUserDebugText("Y", [0, 1, 0], [0, 1, 0], 1)
        p.addUserDebugText("Z", [0, 0, 1], [0, 0, 1], 1)

        #make dots in distances of 0.1 meters on all axes
        for i in range(1, 11):
            p.addUserDebugText(str(i), [i*0.1, 0, 0], [1, 0, 0], 1)
            p.addUserDebugText(str(i), [0, i*0.1, 0], [0, 1, 0], 1)
            p.addUserDebugText(str(i), [0, 0, i*0.1], [0, 0, 1], 1)

    def show_linked_vectors(self, vector_list):
        vector_chain = np.array(vector_list[0])
        for i in range(1, len(vector_list)):
            #print(vector_chain, vector_list[i])
            p.addUserDebugLine(vector_chain, vector_chain+vector_list[i], [1, 0, 0], 2, lifeTime=self.simulation_Timestep)
            vector_chain += np.array(vector_list[i])

    def show_Points(self, point_list, color=[0, 0, 1]):
        p.addUserDebugPoints(point_list, pointSize=5, pointColorsRGB=[color for i in range(len(point_list))], lifeTime=self.simulation_Timestep)
    
    def show_TextPoint(self, point_coord, text):
        #show the text at a specific point
        p.addUserDebugText(text, point_coord, [0, 0, 1], 1.2, lifeTime=self.simulation_Timestep)

    def remove_all_debug_objects(self):
        #remove all debug lines
        p.removeAllUserDebugItems()


    def Sim_step(self):
        self.rabbit.step(self.simulation_Timestep)
        
        for _ in range(int(self.simulation_Timestep/p.getPhysicsEngineParameters()["fixedTimeStep"])):
            if self.hung:
                p.resetBasePositionAndOrientation(self.rabbit.id, [0,0, 0.5], [0, 0, 0, 1])
            self.rabbit.render()
            p.stepSimulation()
        
        if self.simulationSpeed == "human":
            self.time_interval.wait_for_step(self.simulation_Timestep)

    def reset(self, seed, reset_gravity_direction=False, reset_terrain=False, rabbit_orientation=0):
        if reset_terrain:
            #self.ground.reset()
            pass
        if reset_gravity_direction:
            # give a random gravity vector whichs roll and pitch is in range of [-30, 30]. The length is 1*9.81
            maxAngle = 5
            pitch, roll = np.random.uniform(-np.pi/180*maxAngle, np.pi/180*maxAngle, 2)
            normal_vec = np.array([0,0,-1])
            rotation_matrix_x = np.array([[1, 0, 0],
                                        [0, np.cos(pitch), -np.sin(pitch)],
                                        [0, np.sin(pitch), np.cos(pitch)]])
            
            rotation_matrix_y = np.array([[np.cos(roll), 0, np.sin(roll)],
                                        [0, 1, 0],
                                        [-np.sin(roll), 0, np.cos(roll)]])
            gravity_direction = rotation_matrix_x @ normal_vec
            gravity_direction = rotation_matrix_y @ gravity_direction
            gravity_direction = gravity_direction*9.81
            gravity_direction = gravity_direction.tolist()
            p.setGravity(*gravity_direction)

        if self.rabbit is not None:
            self.rabbit.reset(rabbit_orientation)

    def close(self):
        if self.connection_id is not None:
            try:
                p.disconnect(self.connection_id)
                self.connection_id = None
            except:
                pass

    # def __del__(self):#called when the simulation is closed
    #     # Disconnect from the physics server if connected
    #     if self.connection_id is not None:
    #         p.disconnect(self.connection_id)

if __name__ == "__main__":
    env = Simulation(gui=True, simulation_speed="human", rabbit_type="Rabbit_v3_mesured")
    env.rabbit.add_DebugPanel()
    angel_func = env.rabbit.create_get_informations(["joint_angles"])
    action_func = lambda: env.rabbit.convert_joint_angles_to_actions(angel_func()[0])

    #env.rabbit.set_new_mass()
    #env.rabbit.create_seperate_Window()
    env.show_world_coordinate_system()
    #env.rabbit.add_UserControlPanel(all_joints=False)
    env.rabbit.get_link_infos()
    #env.rabbit.show_center_of_mass()
    time.sleep(10000)
    # funktion = env.rabbit.create_get_informations(["joint_angles"])
    # env.rabbit.servoControler.set_torque_state(7, True)
    # env.rabbit.send_goal_pose([0, 0, 0, 0, 0, 0, 0, 0])
    random_action = np.random.uniform(-1, 1, 8)
    while True:
        print("step start")
        time.sleep(0.1)
        env.Sim_step()
        # print("Joint angles", funktion())
        env.rabbit.send_goal_pose(random_action)

        print(np.array(action_func())-np.array(random_action))
        print("Real action", action_func())
        print("Inputaction", random_action)
    # env1 = Simulation(gui=True, simulation_speed="human", rabbit_type="Rabbit_v3_mesured")
    # env1.show_world_coordinate_system()

    # # delete env1
    # env1.close()

    # # create a new env2
    # env2 = Simulation(gui=True, simulation_speed="human", rabbit_type="Rabbit_v3")
    # env2.show_world_coordinate_system()
    # time.sleep(10000)
    # env2.close()