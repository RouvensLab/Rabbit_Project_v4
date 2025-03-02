import numpy as np
from collections import deque
import math
import sys
import os
import pybullet as p

# Assuming Bunny_Project_v2 is the project root directory
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(project_root)

from Real_robot.ServoSupervisor_v1 import ServoSupervisor
from Real_robot.Cameras.Camera_v3 import Camera
from Real_robot.Oscilloscope.RigolOszRead import OscilloscopeReader

class Rabbit_real:
    """A class to represent a rabbit in the simulation
    """
    
    def __init__(self, *args, **kwargs):
        """Initialize the rabbit"""
        #get the names of the joints
        self.Joints_index = [1, 2,   3, 5,   4, 6,   7, 8]
        self.Joint_correction = [0, -0.1,   0, 0,   0.1, 0.3,   0, -0.15]
        self.numMotors = 8

        self.servoControler = ServoSupervisor(servo_id_order=self.Joints_index)
        self.servoControler.ScanServos()
        self.servoControler.start_run()


        
        #camera and accelerometer
        self.cam = Camera()

        #create the RigolOszReader
        self.oscilloscope = OscilloscopeReader()
        self.oscilloscope.connect()
        self.oscilloscope.start_reading()


        #UserControlPanel_added
        self.UserControlPanel_added = False
        self.UserSlider = []

        #Install the camera with the acceleration and orientation sensors at the head of the rabbit




        self.Motors_range = [(-0.610865, 0.785398), (-0.523599, 0.523599), (-0.785398, 0.261799), (-0.523599, 0.523599), (-2.268928, 0.174533), (-1.047198, 0.785398), (-0.785398, 0.785398), (-2.268928, 0.174533), (-1.047198, 0.785398), (-0.785398, 0.785398), (-1.570796, 1.047198), (-1.570796, 1.047198)]
        self.Motors_range = self.convert_12_to_8_motors(self.Motors_range)
        self.Savety_range = [(-1, 1), (-1, 1),   (-1, 1), (-1, 1),    (-1, 1), (-1, 1),    (-1, 1), (-1, 1)]
        print("Motors_range", self.Motors_range)
        #set the self collision of the robot
        #self.set_self_collision()
        self.lifetime = 0
        self.timeStep_size = 0.01
        # Create a deque to hold (actions, time) tuples; maxlen=3 lets us compute acceleration.
        self.action_history = deque(maxlen=3)

        #reset the robot to the initial position, so that the constraints can be set
        self.reset()


    def convert_12_to_8_motors(self, motors_12_value):
        """
        Converts the 12 Joints values to 8 Joints values (to compare it with the real robot who has only 8 Joints)
        motors_12_value: list with the 12 motor values (in order of self.Joints_index)
        return: list with the 8 motor values (in order of self.Joints_index)
        """
        # Ensure that motors_10_value contains numerical values

        if len(motors_12_value) != 12:
            raise ValueError("The list with the motor values has to have 12 values")
        
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
        


    def create_get_informations(self, status_types):
        """Create a function that returns the preferred states of the robot
        status_types: list of strings, the types of the status that should be returned
        return: function

        This allows us to save resources, like RAM, because we can only get the informations we need.
        """
        self.servoControler.stop_run()
        #create a list of all the status types
        lambda_list = []
        left_joint_types = []
        joint_types = ["joint_angles", "joint_torques", "joint_velocities", "joint_currents", "joint_voltages", "joint_temperatures"]
        trans_types = ["Position", "Load", "Velocity", "Current", "Voltage", "Temperature"]
        for state_type in status_types:
            if "head_orientation"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[0])
            elif "head_angular_velocity"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[1])
            elif "head_linear_acceleration"== state_type:
                lambda_list.append(lambda: self.get_head_sensors()[2])
            elif state_type in joint_types:
                transformed_state_type = trans_types[joint_types.index(state_type)]
                left_joint_types.append(transformed_state_type)
                lambda_list.append(self.servoControler.create_get_inf(transformed_state_type))
            elif "total_current" in state_type:
                lambda_list.append(lambda: [self.get_total_current(), 0, 0])
            elif "vision"== state_type:
                #get the camera image from the camera at the head of the rabbit
                lambda_list.append(lambda: self.get_camera_image())
            else:
                raise ValueError(f"Unknown state type: {state_type}")
        
        #create a getfunction
        self.servoControler.continiu_run()
        def get_informations():
            return [func() for func in lambda_list]

        return get_informations
    
    def get_lifetime(self):
        """Get the lifetime of the robot
        """
        return self.lifetime

    def get_camera_image(self):
        """Get the camera image
        """
        return self.cam.get_frame()
    
    def get_total_current(self):
        """Get the total current of the robot
        """
        value = self.oscilloscope.get_channel1_value()
        return self.oscilloscope.calc_current(value)
    
    def get_head_sensors(self):
        """Get the orientation and velocity of the robot
        return: 
            tuple: (orientation (rad), angular_velocity (rad/s), linear_acceleration (m/s²))
        """
        orientation, angular_velocity, linear_acceleration = self.cam.get_accelerometer_data(dt=self.timeStep_size)
        if orientation is None:
            orientation = [0, 0, 0]
            angular_velocity = [0, 0, 0]
            linear_acceleration = [0, 0, 0]
        return orientation, angular_velocity, linear_acceleration
    
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
        pose_positions: list with the goal positions of the servos, with the used range (-1, 1). e.g. [0,0,   0,0,   0,0,   0,0]
        """
        #map the servo_positions to the range of the servos
        pose_positions = np.array(pose_positions)+np.array(self.Joint_correction)

        servo_positions = [self._map(pos, range[0], range[1], self.Motors_range[i][0], self.Motors_range[i][1]) for i, pos in enumerate(pose_positions)]

        #add the knee joint angles ==> hip joint angles
        servo_positions[3] = math.pi - self.InverseKinematics(servo_positions[3], servo_positions[2])
        servo_positions[5] = math.pi - self.InverseKinematics(servo_positions[5], servo_positions[4])

        #transform the spine joint angles
        servo_positions[0] = np.clip(pose_positions[1]*45+pose_positions[0]*-55, -55, 10)/180*math.pi
        servo_positions[1] = np.clip(pose_positions[1]*-45+pose_positions[0]*-55, -50, 10)/180*math.pi

        self.send_motor_commands(servo_positions)

        

    def send_motor_commands(self, motor_commands):
        """Send motor commands to the robot in radians
        motor_commands: list with the motor commands in the order of the motors
        """
        #convert radians to 0 -4094
        print("motor_commands", motor_commands)
        self.servoControler.action_queue.put({"positions": motor_commands, "speeds": [4094]*8})


    
    def step(self, timeStep_size):
        """Step the simulation
        step_time: the time that has passed since the last step (ms)
        """
        self.timeStep_size = timeStep_size
        #update the lifetime
        self.lifetime += timeStep_size

        #self.servoControler.get_Sync_ServosInfo()#updates the servo information

        #update action rate
        self.update_action_history(self.servoControler.get_ServoStates("Position"))# not quiet right. Because servo 1 and 2 are not the same as the one in the simulation



    def reset(self):
        """Reset the robot to the initial position
        """
        self.lifetime = 0

        self.send_goal_pose([0, 0,    0, 0,  0, 0,    0, 0])






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
    
    def InverseKinematics(self, Knee_angle, FrontHip_angle, degree=False):
        """
        give the angle of the Knee and FrontHip.
        Returns: the angle of the BackHip = b1. b1 goes counted anticlockwise from 0 to 2*pi
        """
        pi = math.pi
        #Motor 1 Position
        Motor1_coo = np.array([0, 0])

        #set some constants
        S1 = 0.07#FrontHip to Knee
        S4 = 0.03#Knee to MiddleUnderLeg
        t2 = np.array([0.03, 0])#Distance between the FrontHip and the BackHip
        S2 = 0.04#BackHip to Knee2
        S3 = 0.07#Knee2 to MiddleUnderLeg

        S5 = 0.03#MiddleUnderLeg to Foot
        FrontHip_angle = -FrontHip_angle
        Knee_angle = -Knee_angle
        if degree:
            FrontHip_angle = FrontHip_angle/180*math.pi
            Knee_angle = Knee_angle/180*math.pi
        
        a1 = self._map(FrontHip_angle, -math.pi, math.pi, 0, 2*math.pi)
        a2 = 0.5*math.pi-Knee_angle

        print("a1:", a1/pi*180, "a2:", a2/pi*180, "Knee_angle:", Knee_angle/pi*180, "FrontHip_angle:", FrontHip_angle/pi*180)


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


        return b1
    
    def get_Motor2_inverse_kinematics(self, FrontHip_angle, Knee_angle, degree=False):
        """
        give the angle of the Knee and FrontHip.
        Returns: the angle of the BackHip = b1. b1 goes counted anticlockwise from 0 to 2*pi
        """
        pi = math.pi
        #Motor 1 Position
        Motor1_coo = np.array([0, 0])

        #set some constants
        S1 = 0.07#FrontHip to Knee
        S4 = 0.03#Knee to MiddleUnderLeg
        t2 = np.array([0.03, 0])#Distance between the FrontHip and the BackHip
        S2 = 0.04#BackHip to Knee2
        S3 = 0.07#Knee2 to MiddleUnderLeg

        S5 = 0.03#MiddleUnderLeg to Foot
        FrontHip_angle = -FrontHip_angle
        Knee_angle = -Knee_angle
        if degree:
            FrontHip_angle = FrontHip_angle/180*math.pi
            Knee_angle = Knee_angle/180*math.pi
        
        a1 = self._map(FrontHip_angle, -math.pi, math.pi, 0, 2*math.pi)
        a2 = 0.5*math.pi-Knee_angle

        #print("a1:", a1/pi*180, "a2:", a2/pi*180, "Knee_angle:", Knee_angle/pi*180, "FrontHip_angle:", FrontHip_angle/pi*180)


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
        #print("phi1: ", phi1, "phi2: ", phi2, t1_vec, "b1", b1/pi*180)


        return b1

    def close(self):
        """Close the robot
        """
        self.servoControler.stop_run()
        self.servoControler.close()
        self.cam.close()
        #p.disconnect()

        
if __name__=="__main__":
    import time
    real_rabbit = Rabbit_real()
    while True:
        real_rabbit.step(0.01)
        real_rabbit.send_goal_pose([0, 0,    0, 0,  0, 0,    0, 0])
        get_func = real_rabbit.create_get_informations(["head_orientation", "head_angular_velocity", "head_linear_acceleration", "joint_angles", "joint_torques", "joint_velocities", "total_current"])
        print(get_func())
        time.sleep(1)
        real_rabbit.send_goal_pose([0.1, 0.1,    0.1, 0.1,  0.1, 0.1,    0.1, 0.1])
        time.sleep(1)

    
