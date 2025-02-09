import numpy as np
from collections import deque
import math
import pybullet as p

class Rabbit:
    """A class to represent a rabbit in the simulation
    """
    
    def __init__(self, init_pos= [0, 0, 0]):
        self.URDF_path = r"URDFs\Roh_Rabbit_v4_description\urdf\Roh_Rabbit_v4.xacro"
        self._id = p.loadURDF(self.URDF_path, init_pos[0], init_pos[1], init_pos[2])
        self.init_pos = init_pos

        #UserControlPanel_added
        self.UserControlPanel_added = True
        self.UserSlider = []
        self.hung = False

        #Install the camera with the acceleration and orientation sensors at the head of the rabbit
        self.Camera_Link_id = 19
        self.worldLinkLinearVelocity_last = [0,0,0]


        #get the names of the joints
        self.Motors_index = [0, 1,  15, 16,    9, 13,   5, 3,    21, 20] #this are all the motors of the rabbit that can be controlled and read out. In the preferred order. !!!!
        #self.Joints_index = [0, 1,  15, 16,     9, 13, 12, 13, 16, 14,     6, 3, 7, 8, 4, 10,    23, 22]
        
        self.numMotors = len(self.Motors_index)

        #when there are other physical properties for the motors, they can be set here
        self.MAXFORCE = 10  #2.9#2.941995#5#in Newton   3 N/m
        self.MAXVELOCITY = 4.65#(2*math.pi)/(0.222*6)#in rad/s
        self.Motors_strength = [self.MAXFORCE for i in range(self.numMotors)]
        self.Motors_velocity = [self.MAXVELOCITY for i in range(self.numMotors)]
        
        #get the range of the motors
        #self.joint_ranges = [p.getJointInfo(self._id, i)[8:10] for i in self.Motors_index]
        #print("joint_ranges:", self.joint_ranges)
        self.numJoints = p.getNumJoints(self._id)

        self.Motors_range = [p.getJointInfo(self._id, i)[8:10] for i in self.Motors_index]

        #enableJointForceTorqueSensor
        for joint in self.Motors_index:
            p.enableJointForceTorqueSensor(self._id, joint, enableSensor=True)

        #set the robots lateral_friction to 1
        #print("lateralFriction of Robot: ", p.getDynamicsInfo(self.rabbit.id, -1)[1])
        p.changeDynamics(self._id, -1, 
                         lateralFriction=1,
                         jointDamping=0.1  # Damping for compliance
                         )
        
        #allow collision between the links of the robot
        allowed_collision_between_links = [
            (11, 14),
            (11, 13),
            (7, 3),
            (7, 4),
        ]
        for link1, link2 in allowed_collision_between_links:
            p.setCollisionFilterPair(self._id, self._id, link1, link2, 1)

        #reset the robot to the initial position, so that the constraints can be set
        self.reset()

        
        



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

    def close_kinematic_loop_at_legs(self):
        """Close the kinematic loop of the legs
        """
        #list with the Points that are connected
        need_Point2Point_constraint = [
            #["parent_id", "chield_id", (0, 0, 0), (0, 0, 0)],
            [14, 10, (0.069958, -0.011, 0.002443), (0.0, 0.0, -0.03), (0, -1, 0), 100], #Umdrehung 84, 85
            [4, 6, (0.069958, -0.011, 0.002443), (0.0, 0.0, -0.03), (0, -1, 0), 100], #Umdrehung 86, 87
            [12, 11, (-0.00853, 0.0, -0.069478), (-0.024905, -0.0135, 0.002179), (0, -1, 0), 150],
            [8, 7, (-0.00853, 0.0, -0.069478), (-0.024905, 0.0, 0.002179), (0, -1, 0), 150]
        ]

        #create the Point2Point_constraints
        for constraint in need_Point2Point_constraint:
            p.changeVisualShape(self._id, constraint[0], rgbaColor=[0.5, 1, 0.5, 0.6])
            p.changeVisualShape(self._id, constraint[1], rgbaColor=[0.5, 1, 0.5, 0.6])

            #constrains go from the linkWorldPosition (cartesian position of cernter of mass) to the local joint position.
            #urdflinks do with the worldLinkFramePosition
            link1_diff = np.round(np.array(p.getLinkState(self._id, constraint[0])[4]) - np.array(p.getLinkState(self._id, constraint[0])[0]), 5)
            link2_diff = np.round(np.array(p.getLinkState(self._id, constraint[1])[4]) - np.array(p.getLinkState(self._id, constraint[1])[0]), 5)

            link1_pos = np.array(constraint[2])+link1_diff
            link2_pos = np.array(constraint[3])+link2_diff

            #show the local link points that are connected
            link1_COM_coor = np.array(p.getLinkState(self._id, constraint[0])[0])
            link2_COM_coor = np.array(p.getLinkState(self._id, constraint[1])[0])
            p.addUserDebugLine(link1_COM_coor+link1_pos, link2_COM_coor+link2_pos, [1, 0, 0], 1)

            cid = p.createConstraint(self._id, constraint[0], self._id, constraint[1], p.JOINT_POINT2POINT, constraint[4], link1_pos, link2_pos)
            p.changeConstraint(cid, maxForce=constraint[5], erp=0.9)

    def convert_10_to_8_motors(self, motors_10_value):
        # Ensure that motors_10_value contains numerical values

        motors_10_value = [value[0] if isinstance(value, tuple) else value for value in motors_10_value]
        return [(motors_10_value[0] - motors_10_value[2]) / 2, 
                (motors_10_value[1] - motors_10_value[3]) / 2, 
                motors_10_value[4], 
                motors_10_value[5], 
                motors_10_value[6], 
                motors_10_value[7], 
                motors_10_value[8], 
                motors_10_value[9]]
        #return motors_10_value
    
    def convert_8_to_10_motors(self, motors_8_value):
        """Convert the 8 motor values to 10 motor values"""
        if len(motors_8_value) != 8:
            raise ValueError("The list with the motor values has to have 8 values")
        return [motors_8_value[0], motors_8_value[1], -motors_8_value[0], -motors_8_value[1]]+motors_8_value[2:]


    def create_get_informations(self, status_types):
        """Create a function that returns the preferred states of the robot
        status_types: list of strings, the types of the status that should be returned
        return: function

        This allows us to save resources, like RAM, because we can only get the informations we need.
        """
        #create a list of all the status types
        lambda_list = []
        for state_type in status_types:
            if "position"== state_type:
                lambda_list.append(lambda: p.getBasePositionAndOrientation(self._id)[0])
            elif "head_orientation"== state_type:
                lambda_list.append(lambda: self.get_head_sensors())
            elif "orientation"== state_type:
                lambda_list.append(lambda: p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self._id)[1]))
            elif "linear_velocity"== state_type:
                lambda_list.append(lambda: p.getBaseVelocity(self._id)[0])
            elif "angular_velocity"== state_type:
                lambda_list.append(lambda: p.getBaseVelocity(self._id)[1])

            elif "joint_angles" == state_type:
                lambda_list.append(lambda:  self.convert_10_to_8_motors([p.getJointState(self._id, i)[0] for i in self.Motors_index]))
            elif "joint_torques" == state_type:
                lambda_list.append(lambda: self.convert_10_to_8_motors([p.getJointState(self._id, i)[2] for i in self.Motors_index]))
            elif "joint_velocities"== state_type:
                lambda_list.append(lambda:  self.convert_10_to_8_motors([p.getJointState(self._id, i)[1] for i in self.Motors_index]))
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

    def get_current_Motor_states(self):
        """Get the current states of all Motors in order
        return: list with all Motor and their states. States are: [angle, velocity, reactionForces, appliedTorque]
        """
        motor_state = [p.getJointState(self._id, i) for i in self.Motors_index]
        return motor_state
    
    def get_head_sensors(self):
        """Get the orientation and velocity of the robot
        return: linkWorldOrientation, #worldLinkAngularVelocity, worldLinkLinearAcceleration
        """
        _, linkWorldOrientation, _, _, _, _, worldLinkLinearVelocity, worldLinkAngularVelocity = p.getLinkState(self._id, self.Camera_Link_id, computeLinkVelocity=True)
        worldLinkLinearAcceleration = np.array(worldLinkLinearVelocity)-np.array(self.worldLinkLinearVelocity_last)/p.getPhysicsEngineParameters()["fixedTimeStep"]
        self.worldLinkLinearVelocity_last = worldLinkLinearVelocity
        return linkWorldOrientation, worldLinkLinearAcceleration
    
    # def get_camera_image(self, width=320, height=240):
    #     """Get the camera image
    #     """
    #     view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0], distance=1, yaw=90, pitch=-90, roll=0, upAxisIndex=2)
    #     projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=width/height, nearVal=0.1, farVal=100)
    #     image = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=projection_matrix)
    #     return image
    
    def send_goal_pose(self, pose_positions, range=[-1, 1]):
        """
        This sends the goal pose to the robot. The default range is from -1 to 1
        pose_positions: list with the goal positions of the servos, with the used range. e.g. [0,0,   0,0,   0,0,   0,0]
        """
        #map the servo_positions to the range of the servos
        servo_positions = self.convert_8_to_10_motors(pose_positions)
        servo_positions = [self._map(pos, range[0], range[1], self.Motors_range[i][0], self.Motors_range[i][1]) for i, pos in enumerate(servo_positions)]
        self.send_motor_commands(servo_positions)
        

    def send_motor_commands(self, motor_commands):
        """Send motor commands to the robot
        motor_commands: list with the motor commands in the order of the motors
        
        """
        #the motors that control the lower legs need inverse kinematics

        #...............................................

        for i, position in zip(self.Motors_index, motor_commands):
            p.setJointMotorControl2(self._id, i, p.POSITION_CONTROL, targetPosition=position, force=self.MAXFORCE, maxVelocity=self.MAXVELOCITY)
    
    def send_motor_commands_to_all_joints(self, motor_commands):
        """Send motor commands to all joints
        """
        for i, position in zip(self.Joints_index, motor_commands):
            p.setJointMotorControl2(self._id, i, p.POSITION_CONTROL, targetPosition=position, force=self.MAXFORCE, maxVelocity=self.MAXVELOCITY)

    def set_other_joints_to_passive(self):
        """Set all the other joints to passive
        """
        for i in range(self.numJoints):
            if i not in self.Motors_index:
                p.setJointMotorControl2(self._id, i, p.POSITION_CONTROL, targetPosition=0, force=0)

    def add_UserControlPanel(self, all_joints=False):
        """This adds sliders that can be used to control the main motors and a button to reset position
        """
        if all_joints: #doesn't work yet!!!!!!!!!!!!!
            for i in self.Joints_index:
                self.UserSlider.append([p.addUserDebugParameter(f"Joint_{i}", -1, 1, 0), i])
            self.UserControlPanel_added = True
        else:
            for i in range(8):
                self.UserSlider.append([p.addUserDebugParameter(f"Joint_{i}", -1, 1, 0), i])
                # Add a button to reset the position
            self.reset_button = p.addUserDebugParameter("Reset Position", 1, 0, 1)
            self.UserControlPanel_added = True

        

    def check_reset_button(self):
        """Check if the reset button is pressed and reset the position if it is"""
        if self.UserControlPanel_added:
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
            

    def remove_UserControlPanel(self):
        """Remove the User Control Panel
        """
        for i in self.UserSlider:
            p.removeUserDebugItem(i[0])
        self.UserSlider = []
        self.UserControlPanel_added = False

    
    def step(self):
        """Step the simulation
        """
        if self.UserControlPanel_added:
            # for i in self.UserSlider:
            #     p.setJointMotorControl2(self._id, i[1], p.POSITION_CONTROL, targetPosition=p.readUserDebugParameter(i[0]), force=self.MAXFORCE, maxVelocity=self.MAXVELOCITY)
            self.send_goal_pose([p.readUserDebugParameter(i[0]) for i in self.UserSlider])
            self.check_reset_button()

        if self.hung:
           p.resetBasePositionAndOrientation(self._id, self.init_pos, [0, 0, 0, 1])


        #remove the UserDebugTexts
        #p.removeAllUserDebugItems()

        #update the lifetime!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.lifetime += p.getPhysicsEngineParameters()["fixedTimeStep"]


    def reset(self):
        """Reset the robot to the initial position
        """
        self.lifetime = 0
        p.resetBasePositionAndOrientation(self._id, self.init_pos, [0, 0, 0, 1])
        # #reset all the motors
        #repostion the joints
        # motor1, knee_linkage_angle = self.get_Motor2_inverse_kinematics(0, 0)
        # print("motor1:", motor1/math.pi*180, "knee_linkage_angle:", knee_linkage_angle/math.pi*180)
        # #knee_linkage_angle = 0
        # initial_joint_positions = [0, 0, 0, 0,    0, motor1, 0, 0, -knee_linkage_angle, 0,      0, motor1, 0, 0, knee_linkage_angle, 0,   0,0]

        # for index, pos in enumerate(initial_joint_positions):
        #     print("joint:", self.Joints_index[index], "pos:", pos)
        #     p.resetJointState(self._id, self.Joints_index[index], pos, 100)

        for i in range(self.numJoints):
            p.resetJointState(self._id, i, 0, 0)

        self.send_motor_commands([0, 0, 0, 0,  0, 0,  0, 0,  0, 0])
        for i in range(self.numJoints):
            p.resetJointState(self._id, i, targetValue=0, targetVelocity=0)
        #p.stepSimulation()

        #close the kinematic loop at the legs
        self.close_kinematic_loop_at_legs()
        p.stepSimulation()

        # #set the other joints to passive
        self.set_other_joints_to_passive()
        #set the motors to the initial position
        self.send_motor_commands([0, 0, 0, 0,  0, 0,  0, 0,  0, 0])






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
    
    def get_Foot_angle(self):
        """Get the angle of the foot
        return: the angle of the foot and the knee joint(for the parallel link)
        """
        #get the current states of all motors
        motor_states = self.get_current_Motor_states()
        #get the angles of the motors
        Motor1 = motor_states[0][0]
        Motor2 = motor_states[1][0]
        Motor3 = self.get_kneeJoint_inverse_kinematics(Motor1, Motor2)
        #return foot_angle, knee_angle

    def getTotalCurrent(self):
        return 0
    

    #getter for _robot
    @property
    def id(self):
        return self._id
    
    def __int__(self):
        return self._id
    
    
