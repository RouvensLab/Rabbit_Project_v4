from Rabbit import Rabbit

import pybullet as p
import pybullet_data

import time

class Environement:
    def __init__(self,
                 gui=True,
                 simulation_speed="human",
                 status_types=["position", "orientation", "linear_velocity", "angular_velocity", "joint_angles", "joint_torques", "joint_velocitys", "foot_contacts", "component_coordinates_world", "component_coordinates_local"],
                 terrain_type = "random_terrain",
    ):
        super().__init__()
        self.GUI = gui

        #initialization of pybullet
        if self.GUI:
            connection_id = p.connect(p.GUI)            #RL 2024
        else:
            connection_id = p.connect(p.DIRECT)         #RL 2024

        #the frequency of the simulation
        print("PhysicsEngineParameter: ",p.getPhysicsEngineParameters())

        self.rabbit = Rabbit(r"Roh_Rabbit_v4_description\urdf\Roh_Rabbit_v4.xacro", [0, 0, 0.5])
        
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(p.isConnected())
        self.ground = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.81)
        #set the grounds lateral_friction to 1
        p.changeDynamics(self.ground, -1, lateralFriction=1)


        #set the robots lateral_friction to 1
        print("lateralFriction of Robot: ", p.getDynamicsInfo(self.rabbit.id, -1)[1])
        p.changeDynamics(self.rabbit.id, -1, 
                         lateralFriction=1,
                         jointDamping=0.1  # Damping for compliance
                         )
        #p.saveWorld("rabit.py")
        self.rabbit.get_link_infos()
        #self.rabbit.send_motor_commands([0, 0, 0, 0,  0, 0,   0, 0,   0, 0])
        self.rabbit.add_UserControlPanel(all_joints=False)
        p.stepSimulation()
        #time.sleep(1000)


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

    def Sim_step(self):
        self.rabbit.step_simulation()
        p.stepSimulation()
        

if __name__ == "__main__":
    env = Environement(gui=True)
    while True:
        
        env.Sim_step()
        time.sleep(0.05)

        # for i in range(100):
        #     # Send new command to the motors
        #     env.rabbit.send_motor_commands([2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
        #     time.sleep(0.2)
            
        # for i in range(100):
        #     # Send new command to the motors
        #     env.rabbit.send_motor_commands([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        #     env.Sim_step()
        #     time.sleep(0.2)