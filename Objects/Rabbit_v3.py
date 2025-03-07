import numpy as np
from collections import deque
import math
import pybullet as p
from PySide6.QtGui import QGuiApplication

class Rabbit:
    """A class to represent a rabbit in the simulation

    Infos/Checklist:
    - [x] Input and output are identical
    - [x] The motors are in the right orientation
    - [x] The motors are in the right range

    """
    
    def __init__(self, init_pos= [0, 0, 0]):
        self.URDF_path = r"URDFs\URDF_description\urdf\URDF.xacro"
        self._id = p.loadURDF(self.URDF_path, init_pos[0], init_pos[1], init_pos[2])
        self.init_pos = init_pos

        self.focus_Robot = False
        self.focus_pos = deque(maxlen=30)#to get a smooth focus on the robot
        self.focus_orn = deque(maxlen=30)#to get a smooth focus on the robot

        #UserControlPanel_added
        self.UserControlPanel_added = False
        self.UserSlider = []

        #Install the camera with the acceleration and orientation sensors at the head of the rabbit
        self.Camera_Link_id = 4
        self.worldLinkLinearVelocity_last = [0,0,0]


        #get the names of the joints
        self.Joints_index = [9,10,  0, 1,    15, 16, 17,   12, 13, 14,    5, 6] #this are all the motors of the rabbit that can be controlled and read out. In the preferred order. !!!!
        [14, 17, 6, 5]
        self.numMotors = 8
        self.current_action = [0]*self.numMotors

        #when there are other physical properties for the motors, they can be set here
        self.MAXFORCE = 2.9  #2.9#2.941995#5#in Newton   3 N/m
        self.MAXVELOCITY = (2*math.pi)/(0.222*6)#in rad/s
        self.ACCELERATION = 8.79/180*math.pi#8.79 #(degrees/ s^2) by 7.4 v
        self.Motors_strength = [self.MAXFORCE for i in range(self.numMotors)]
        self.Motors_velocity = [self.MAXVELOCITY for i in range(self.numMotors)]
        
        #get the range of the motors
        #self.joint_ranges = [p.getJointInfo(self._id, i)[8:10] for i in self.Motors_index]
        #print("joint_ranges:", self.joint_ranges)
        self.numJoints = p.getNumJoints(self._id)

        self.Motors_range = [p.getJointInfo(self._id, i)[8:10] for i in self.Joints_index]
        print("Motors_range:", self.Motors_range)

        #enableJointForceTorqueSensor
        for joint in self.Joints_index:
            p.enableJointForceTorqueSensor(self._id, joint, enableSensor=True)

        #set the robots lateral_friction to 1
        #print("lateralFriction of Robot: ", p.getDynamicsInfo(self.rabbit.id, -1)[1])
        p.changeDynamics(self._id, -1, 
                         lateralFriction=1,
                         jointDamping=0.1  # Damping for compliance
                         )
        
        #set the self collision of the robot
        #self.set_self_collision()
        self.lifetime = 0
        # Create a deque to hold (actions, time) tuples; maxlen=3 lets us compute acceleration.
        self.action_history = deque(maxlen=3)

        self.linkWorldOrientation, self.worldLinkLinearAcceleration, self.worldLinkAngularVelocity, self.worldLinkLinearVelocity = [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]

        #reset the robot to the initial position, so that the constraints can be set
        self.reset()


    def set_new_mass(self):
        """Set new masses for every Bodypart (link) of the robot
        """
        

        
    def set_self_collision(self, collistionPartners = [(11, 14), (11, 13), (7, 3), (7, 4)], enable=True):
        """Set the self collision of the robot
        Which Body parts of the rabbit should collide with each other.
        """
        allowed_collision_between_links = collistionPartners
        for link1, link2 in allowed_collision_between_links:
            p.setCollisionFilterPair(self._id, self._id, link1, link2, enable)
            #p.setCollisionFilterGroupMask(self._id, -1, 0, 0, enableCollision=enable)

    def simplify_collision(self, ground_id):
        """This removes the collision detection between the Robots links and the ground"""
        links_true = [14, 17, 6, 5]
        #the rest should be false
        links_false = [i for i in range(p.getNumJoints(self._id)) if i not in links_true]
        for l in links_false:
            p.setCollisionFilterPair (self._id, ground_id, l, -1, 0)


    def check_delicate_self_collision(self):
        """Check if the robot collides with itself
        """
        return False



    def get_link_infos(self):
        """Get the information of all links at the coresponding koorinates of the robot.

        The robot gets a bit invisible, because the dots are in the center of the links.
        """
        self.change_Visibility(0.6)

        link_infos = [p.getLinkState(self._id, i) for i in range(p.getNumJoints(self._id))]
        #get all the names of the links
        link_names = [p.getJointInfo(self._id, i)[12].decode("utf-8") for i in range(p.getNumJoints(self._id))]
        #get all the weights of the links
        link_masses = [p.getDynamicsInfo(self._id, i)[0] for i in range(p.getNumJoints(self._id))]
        #get the total mass of the robot
        total_mass = sum(link_masses)
        print("total_mass:", total_mass, "\n")

        print("link_masses:", link_masses, "\n")

        print("link_names:", link_names, "\n")

        print("link_infos:", link_infos, "\n")

        #_show_weights_in_UI
        link_positions = [link_info[4] for link_info in link_infos]
        p.addUserDebugPoints(link_positions, [[0, 1, 0] for i in range(len(link_positions))], 5)


        for i, link_mass in enumerate(link_masses):
            #make a dot at the position of the center mass of the link
            p.addUserDebugText(f"Nr:{i}, {link_names[i]} m={str(round(link_mass, 3))}", link_infos[i][4], [1, 0, 0], 1)
        
        # #show total mass at the top of the robot
        p.addUserDebugText(str(round(total_mass, 3)), [0, 0, 0.2], [1, 0, 0], 1)

    def change_Visibility(self, visibility):
        """Change the visibility of the whole robot
        visibility: 0 = invisible, 1 = visible
        """
        for i in range(-1, p.getNumJoints(self._id)):
            p.changeVisualShape(self._id, i, rgbaColor=[1, 1, 1, visibility])

    def convert_12_to_8_motors(self, motors_12_value):
        """
        Converts the 12 Joints values to 8 Joints values (to compare it with the real robot who has only 8 Joints)
        motors_12_value: list with the 12 motor values (in order of self.Joints_index)
        return: list with the 8 motor values (in order of self.Joints_index)
        """
        # Ensure that motors_10_value contains numerical values

        if len(motors_12_value) != 12:
            raise ValueError("The list with the motor values has to have 12 values, but has ", len(motors_12_value))
        
        if any(isinstance(value, float) or isinstance(value, int) for value in motors_12_value):
            return [(motors_12_value[0] - motors_12_value[3]) / 2, 
                    (motors_12_value[1] - motors_12_value[2]) / 2,


                    motors_12_value[4], 
                    motors_12_value[5],

                    motors_12_value[7],
                    motors_12_value[8],


                    motors_12_value[10], 
                    motors_12_value[11]
                    
                    ]
        #if the values are tuples convert them to np.arrays and calculate the mean
        elif any(isinstance(value, tuple) for value in motors_12_value):
            return [(np.array(motors_12_value[0]) - np.array(motors_12_value[3])) / 2, 
                    (np.array(motors_12_value[1]) - np.array(motors_12_value[2])) / 2,

                    motors_12_value[4], 
                    motors_12_value[5],

                    motors_12_value[7],
                    motors_12_value[8],


                    motors_12_value[10], 
                    motors_12_value[11]
                    ]


            
        else:
            raise ValueError(f"The list with the motor values has to have float not {type(motors_12_value[0])}")
                
    
    def convert_8_to_12_motors(self, motors_8_value):
        """Convert the 8 motor values to 10 motor values"""
        if len(motors_8_value) != 8:
            raise ValueError("The list with the motor values has to have 8 values. But has ", len(motors_8_value))
        return [motors_8_value[0], 
                motors_8_value[1], 
                -motors_8_value[0], 
                -motors_8_value[1],

                motors_8_value[2],
                motors_8_value[3],
                0,

                motors_8_value[4],
                motors_8_value[5],
                0,

                motors_8_value[6],
                motors_8_value[7]

                
                ]

    
    def convert_12_to_right_orientation(self, motors_12_value):
        """Convert the 12 motor values to the right orientation
        Doesn't make changes
        From symetrical to asymetrical. (Because the servos are mounted in a different orientation)
        """
        new_orn_values = [0]*12
        orientation_data = [0, 0, 0, 0,   0, 0, 0,   0, 0, 0,  0, 0]
        for i, value in enumerate(motors_12_value):
            if orientation_data[i] == 1:
                if isinstance(value, tuple) or isinstance(value, list) or isinstance(value, np.ndarray):
                    print("Test:", value)
                    new_orn_values[i] = -np.array([value[1], value[0]])
                elif isinstance(value, float):
                    new_orn_values[i] = -value
            else:
                new_orn_values[i] = value
        return new_orn_values
        

        


    def create_get_informations(self, status_types):
        """Create a function that returns the preferred states of the robot
        status_types: list of strings, the types of the status that should be returned
        return: function

        This allows us to save resources, like RAM, because we can only get the informations we need.
        """
        #create a list of all the status types
        lambda_list = []
        #["base_position", "base_orientation", "base_linear_velocity", "base_angular_velocity", "joint_angles", "joint_torques", "joint_velocities", "joint_action_rate", "joint_action_acceleration"]
        for state_type in status_types:
            if "base_position"== state_type:
                lambda_list.append(lambda: p.getBasePositionAndOrientation(self._id)[0])
            elif "base_orientation"== state_type:
                lambda_list.append(lambda: p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self._id)[1]))
            elif "base_linear_velocity"== state_type:
                lambda_list.append(lambda: p.getBaseVelocity(self._id)[0])
            elif "base_angular_velocity"== state_type:
                lambda_list.append(lambda: p.getBaseVelocity(self._id)[1])

            elif "head_orientation"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[0])
            elif "head_angular_velocity"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[1])
            elif "head_linear_velocity"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[2])
            elif "head_linear_acceleration"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[3])

            elif "joint_angles" == state_type:
                lambda_list.append(lambda:  self.convert_12_to_8_motors([p.getJointState(self._id, i)[0] for i in self.Joints_index]))
            elif "joint_torques" == state_type:
                lambda_list.append(lambda: self.convert_12_to_8_motors([p.getJointState(self._id, i)[3] for i in self.Joints_index]))
            elif "joint_velocities"== state_type:
                lambda_list.append(lambda:  self.convert_12_to_8_motors([p.getJointState(self._id, i)[1] for i in self.Joints_index]))
            elif "joint_action_rate"== state_type:
                lambda_list.append(lambda: self.get_action_rate())
            elif "joint_acceleration"== state_type:
                lambda_list.append(lambda: self.get_action_acceleration())
            elif "action" == state_type:
                lambda_list.append(lambda: self.current_action)
            
            elif "component_coordinates_world"== state_type:
                lambda_list.append(lambda: [p.getLinkState(self._id, i)[0] for i in range(p.getNumJoints(self._id))])
            elif "component_coordinates_local"== state_type:
                lambda_list.append(lambda: [p.getLinkState(self._id, i)[2] for i in range(p.getNumJoints(self._id))])

            elif "vision"== state_type:
                #get the camera image from the camera at the head of the rabbit
                lambda_list.append(lambda: self.get_camera_image())

        def get_informations():
            return [func() for func in lambda_list]

        return get_informations
    
    def get_lifetime(self):
        """Get the lifetime of the robot
        """
        return self.lifetime
    
    def get_head_sensors(self):
        """Get the orientation and velocity of the robot
        return: linkWorldOrientation, #worldLinkAngularVelocity, worldLinkLinearAcceleration
        """
        return self.linkWorldOrientation, self.worldLinkAngularVelocity, self.worldLinkLinearVelocity, self.worldLinkLinearAcceleration
    
    def update_head_sensors(self):
        """Get the orientation and velocity of the robot
        return: linkWorldOrientation, #worldLinkAngularVelocity, worldLinkLinearAcceleration
        """
        _, linkWorldOrientation, _, _, _, _, worldLinkLinearVelocity, worldLinkAngularVelocity = p.getLinkState(self._id, self.Camera_Link_id, computeLinkVelocity=True)
        #transform the orientation to euler angles
        self.linkWorldOrientation = p.getEulerFromQuaternion(linkWorldOrientation)
        #calculate the acceleration
        if not hasattr(self, "worldLinkLinearVelocity_last"):
            self.worldLinkLinearVelocity_last = [worldLinkLinearVelocity, self.lifetime]
            self.worldLinkLinearAcceleration = [0, 0, 0]
        else:
            #calculate the acceleration
            if self.lifetime - self.worldLinkLinearVelocity_last[1] == 0:
                self.worldLinkLinearAcceleration = [0, 0, 0]
            else:
                self.worldLinkLinearAcceleration = (np.array(worldLinkLinearVelocity)-np.array(self.worldLinkLinearVelocity_last[0]))/(self.lifetime - self.worldLinkLinearVelocity_last[1])
        self.worldLinkLinearVelocity_last = [worldLinkLinearVelocity, self.lifetime]

        #calculate the angular acceleration
        # if not hasattr(self, "worldLinkAngularVelocity_last"):
        #     self.worldLinkAngularVelocity_last = [worldLinkAngularVelocity, self.lifetime]
        #     self.worldLinkAngularVelocity = [0, 0, 0]
        # else:
        #     #calculate the acceleration
        #     if self.lifetime - self.worldLinkAngularVelocity_last[1] == 0:
        #         self.worldLinkAngularVelocity = [0, 0, 0]
        #     else:
        #         self.worldLinkAngularVelocity = (np.array(worldLinkAngularVelocity)-np.array(self.worldLinkAngularVelocity_last[0]))/(self.lifetime - self.worldLinkAngularVelocity_last[1])
        # self.worldLinkAngularVelocity_last = [worldLinkAngularVelocity, self.lifetime]
        self.worldLinkAngularVelocity = worldLinkAngularVelocity
        self.worldLinkLinearVelocity = worldLinkLinearVelocity

    def update_action_history(self, actions):
        """
        Store the current actions and the current time (self.lifetime)
        """
        self.action_history.append((np.array(actions), self.lifetime))
    
    def get_action_rate(self):
        """
        Compute the first derivative (action rate) from the last two history entries.
        Returns zeros if not enough history is available.
        """
        if len(self.action_history) < 2:
            return np.zeros_like(self.action_history[-1][0]) if self.action_history else None

        # Get the two most recent entries.
        (actions_prev, time_prev), (actions_curr, time_curr) = list(self.action_history)[-2:]
        dt = time_curr - time_prev
        if dt == 0:
            return np.zeros_like(actions_curr)
        action_rate = (actions_curr - actions_prev) / dt
        return action_rate
    
    def get_action_acceleration(self):
        """
        Compute the second derivative (action acceleration) using the last three history entries.
        Returns zeros if not enough history is available.
        """
        if len(self.action_history) < 3:
            # Not enough data to compute acceleration: return zeros of the same shape as actions.
            return np.zeros_like(self.action_history[-1][0]) if self.action_history else None

        # Unpack the last three history points.
        (actions_old, time_old), (actions_prev, time_prev), (actions_curr, time_curr) = list(self.action_history)
        
        # Compute the rate between old and prev, and between prev and curr:
        dt1 = time_prev - time_old
        dt2 = time_curr - time_prev
        if dt1 == 0 or dt2 == 0:
            return np.zeros_like(actions_curr)
        
        rate1 = (actions_prev - actions_old) / dt1
        rate2 = (actions_curr - actions_prev) / dt2
        
        # To get acceleration, we can compute the change in rate over the total time between rate1 and rate2.
        dt_rate = time_curr - time_old
        if dt_rate == 0:
            return np.zeros_like(actions_curr)
        
        action_acceleration = (rate2 - rate1) / dt_rate
        return action_acceleration
    
    def send_goal_pose(self, pose_positions, range=[-1, 1]):
        """
        This sends the goal pose to the robot. The default range is from -1 to 1
        pose_positions: list with the goal positions of the servos, with the used range. e.g. [0,0,   0,0,   0,0,   0,0]
        """
        #map the servo_positions to the range of the servos
        # print("pose_positions:", pose_positions)
        # print("Motors_range:", self.Motors_range)
        self.current_action = pose_positions
        servo_positions = self.convert_8_to_12_motors(pose_positions)
        #print("after servopositions:", servo_positions)

        servo_positions = [self._map(pos, range[0], range[1], self.Motors_range[i][0], self.Motors_range[i][1]) for i, pos in enumerate(servo_positions)]
        #servo_positions = [self.Motors_range[i][0] + (self.Motors_range[i][1] - self.Motors_range[i][0]) * (pos + 1) / 2 for i, pos in enumerate(servo_positions)]
        #add the foot_angles
        servo_positions[6] = self.calculate_foot_position(servo_positions[4], servo_positions[5])
        servo_positions[9] = self.calculate_foot_position(servo_positions[7], servo_positions[8])

        self.send_motor_commands(servo_positions)

        

    def send_motor_commands(self, motor_commands):
        """Send motor commands to the robot
        motor_commands: list with the motor commands in the order of the motors
        
        """
        # print("motor_commands 12 Dimensions:", motor_commands)
        # print("motor_commands 8 Dimensions:", self.convert_12_to_8_motors(motor_commands))
        for i, position in zip(self.Joints_index, motor_commands):
            p.setJointMotorControl2(self._id, i, p.POSITION_CONTROL, targetPosition=position, force=self.MAXFORCE, maxVelocity=self.MAXVELOCITY)

    def set_other_joints_to_passive(self):
        """Set all the other joints to passive
        """
        for i in range(self.numJoints):
            if i not in self.Joints_index:
                p.setJointMotorControl2(self._id, i, p.POSITION_CONTROL, targetPosition=0, force=0)

    def add_UserControlPanel(self, all_joints=False):
        """This adds sliders that can be used to control the main motors and a button to reset position
        """
        if all_joints:
            for i in self.Joints_index:
                self.UserSlider.append([p.addUserDebugParameter(f"Joint_{i}", -1, 1, 0)])
            self.UserControlPanel_added = True
        else:
            for i in range(8):
                self.UserSlider.append([p.addUserDebugParameter(f"Joint_{i}", -1, 1, 0)])
                # Add a button to reset the position
            self.UserControlPanel_added = True
        self.reset_button = p.addUserDebugParameter("Reset Position", 1, 0, 1)

        



    def check_UserOverride(self):
        """
        Check if the user overrides the motor commands.
        Means all autonomous cammands are ignored and the user commands are executed
        """            
            
        if self.UserControlPanel_added:
            """Check if the reset button is pressed and reset the position if it is"""
            # Get the current state of the reset button
            current_button_value = p.readUserDebugParameter(self.reset_button)
            if not hasattr(self, "previous_button_value"):
                self.previous_button_value = current_button_value
            # Check if the button was pressed (value transition from 0 to 1)
            if current_button_value > self.previous_button_value:
                self.reset()  # Perform the reset action
                print("Resetting position")
            # Update previous button state for the next check
            self.previous_button_value = current_button_value

            """Execute the user commands"""
            if len(self.UserSlider) == 8:
                self.send_goal_pose([p.readUserDebugParameter(i[0]) for i in self.UserSlider])
            elif len(self.UserSlider) == 12:
                self.send_motor_commands([p.readUserDebugParameter(i[0]) for i in self.UserSlider])
            else:
                #Rais Error
                raise ValueError("The number of sliders is not 8 or 12")


            return True
        else:
            return False
        
    def toogle_focus(self):
        self.focus_Robot = not self.focus_Robot
        return self.focus_Robot
        
    def focus_on_robot(self):
        # Get the robot's current position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot)
        self.focus_pos.append(pos)
        self.focus_orn.append(p.getEulerFromQuaternion(orn))
        # Set the camera to look at the robot
        # Dynamically update the camera to follow the robot
        camera_distance = 0.5
        camera_yaw = (np.mean([orn[2] for orn in self.focus_orn]))*180/math.pi
        camera_pitch = -10
        robot_pos = np.array(self.focus_pos).mean(axis=0)
        # print("camera_yaw:", camera_yaw)
        # print("robot_pos:", robot_pos)

        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=robot_pos,
        )
        # print("Camera focused on robot")

            

    def remove_UserControlPanel(self):
        """Remove the User Control Panel
        """
        for i in self.UserSlider:
            p.removeUserDebugItem(i[0])
        self.UserSlider = []
        self.UserControlPanel_added = False

    
    def step(self, step_time):
        """Step the simulation
        step_time: the time that has passed since the last step (ms)
        """
        #update the lifetime
        self.lifetime += step_time

        self.update_head_sensors()

        self.update_action_history(self.convert_12_to_8_motors([p.getJointState(self._id, i)[0] for i in self.Joints_index]))

        if self.focus_Robot:
            self.focus_on_robot()

        #check if the user overrides the motor commands
        if self.check_UserOverride():
            return
        


        #remove the UserDebugTexts
        #p.removeAllUserDebugItems()



    def reset(self):
        """Reset the robot to the initial position
        """
        self.lifetime = 0
        p.resetBasePositionAndOrientation(self._id, self.init_pos, [0, 0, 0, 1])

        for i in range(self.numJoints):
            p.resetJointState(self._id, i, 0, 0)

        #self.send_motor_commands([0, 0, 0, 0,  0, 0,0,  0, 0,0,  0, 0])
        for i in range(self.numJoints):
            p.resetJointState(self._id, i, targetValue=0, targetVelocity=0)
        self.send_goal_pose([0, 0,  0, 0,  0, 0,  0, 0])
        #reset the head sensors
        self.update_head_sensors()



    def calculate_foot_position(self, Motor1, knee_angle):
        """Calculate the position of the foot
        Motor1: the position of the motor 1 (rad)
        knee_angle: the position of the knee joint (rad)
        return: the position of the foot

        PS: This is not precise. But in the near

        """
        foot_angle = -knee_angle
        return foot_angle


    def get_kneeJoint_inverse_kinematics(self, Motor1_angel, Motor2_angle):
        """Calculate the inverse kinematics for the knee joint
        Motor1: the position of the motor 1
        Motor2: the position of the motor 2
        return: the position of the knee joints
        """
        
        #return kneeJoint_angle

    def _map(self, x, in_min, in_max, out_min, out_max):
        """Map a value from one range to another
        """
        return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min
    
    
    def get_Motor2_inverse_kinematics(self, Motor1_angle, Knee_angle, degree=False):
        """Calculate the inverse kinematics for the motors 1
        Motor2: the position of the motor 2
        Knee_angle: the position of the knee joint
        return: the position of the motor 1
        """
        pi = math.pi
        #Motor 1 Position
        Motor1_coo = np.array([0, 0])

        #set some constants
        S1 = 0.07#FrontHip to Knee
        S4 = 0.030#Knee to MiddleUnderLeg
        t2 = np.array([0.0275, 0])#Distance between the FrontHip and the BackHip
        S2 = 0.04#BackHip to Knee2
        S3 = 0.07#Knee2 to MiddleUnderLeg

        S5 = 0.04#MiddleUnderLeg to Foot
        Motor1_angle = -Motor1_angle
        Knee_angle = -Knee_angle
        if degree:
            Motor1_angle = Motor1_angle/180*math.pi
            Knee_angle = Knee_angle/180*math.pi
        
        a1 = self._map(Motor1_angle, -math.pi, math.pi, 0, 2*math.pi)
        a2 = 0.5*math.pi-Knee_angle

        print("a1:", a1/pi*180, "a2:", a2/pi*180, "Knee_angle:", Knee_angle/pi*180, "FrontHip_angle:", Motor1_angle/pi*180)


        #calculate the angle of the BackHip
        # m1 = np.array([math.cos(-a1+pi), math.sin(-a1+pi)])*S1
        # m4 = np.array([math.cos(-a1+pi*1.5-a2), math.sin(-a2+pi*1.5-a2)])*S4
        # m5 = np.array([math.cos(-a1+pi*1.5-a2), math.sin(-a2+pi*1.5-a2)])*S5
        m1 = np.array([math.cos(a1), math.sin(a1)])*S1
        m4 = np.array([math.cos(a1+a2), math.sin(a1+a2)])*S4
        m5 = np.array([math.cos(a1+a2), math.sin(a1+a2)])*S5


        #show_arms(m1, (0,0), (0, 0), m4, m5, t2)
        P = m1+m4+m5
        T = m1+m4
        t1_vec =  T-t2
        t1 = np.linalg.norm(t1_vec)

        z = (S2**2-S3**2+t1**2)/(2*S2*t1)
        # shoulden't be greater than 1
        if z > 1:
            z = 1
            print("z is greater than 1")
        elif z < -1:
            z = -1
            print("z is smaller than -1")

        phi1 = math.acos(z)
        
        #atan doesn't give angels above 90 degrees, so I have to calculate with sin and cos
        #phi2 = math.asin(t1_vec[1]/(t1_vec[0]**2+t1_vec[1]**2)**0.5)
        if t1_vec[0] > 0:
            phi2 = math.atan(t1_vec[1]/t1_vec[0])
            phi2 = pi+phi2
        elif t1_vec[0] < 0:
            phi2 = math.atan(t1_vec[1]/t1_vec[0])
        else:
            phi2 = pi/2

        #phi2 = math.atan2(t1_vec[1], t1_vec[0])

        
        b1 = phi1+phi2
        m2 = np.array([math.cos(b1+pi)*S2, math.sin(b1+pi)*S2])
        m3 = t1_vec-m2
        print("phi1: ", phi1, "phi2: ", phi2, t1_vec, "b1", b1/pi*180)

        Motor2_angle = b1
        knee_linkage_angle = (math.atan2(m3[1], m3[0])+b1+pi)
        return Motor2_angle, knee_linkage_angle

    def getTotalCurrent(self):
        return 0
    

    #getter for _robot
    @property
    def id(self):
        return self._id
    
    def __int__(self):
        return self._id


