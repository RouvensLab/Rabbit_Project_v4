import pybullet as p
import pybullet_data
import time
import numpy as np
# Connect to PyBullet
p.connect(p.GUI)

# Set the search path to find URDF files
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the plane URDF
plane_id = p.loadURDF("plane.urdf")

# Load your custom URDF file (replace 'your_robot.urdf' with your actual URDF file name)
robot_id = p.loadURDF(r"TestURDF_description\urdf\TestURDF.xacro")

#set the Motors force to zero
for i in range(p.getNumJoints(robot_id)):
    p.setJointMotorControl2(robot_id, i, p.VELOCITY_CONTROL, force=0)

# close the kinematik loop with constraints
#p.createConstraint(robot_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
#list with the Points that are connected
need_Point2Point_constraint = [
    #["parent_id", "chield_id", (0, 0, 0), (0, 0, 0)],
    [2, 1, (0.0, 0.0, 0.1), (-0.1, 0.0, 0.0), (0, 0, 0)], #Umdrehung 84, 85
    #[4, 7, (0, 0, 0), (0, 0, 0)],

    # [15, 13, (0, 0, 0), (0, 0, 0)],
    # [10, 8, (0, 0, 0), (0, 0, 0)],
]



#create the Point2Point_constraints
for constraint in need_Point2Point_constraint:
    p.changeVisualShape(robot_id, constraint[0], rgbaColor=[0.5, 1, 0.5, 0.6])
    p.changeVisualShape(robot_id, constraint[1], rgbaColor=[0.5, 1, 0.5, 0.6])
    #show the local link points that are connected
    link1_COM_coor = np.array(p.getLinkState(robot_id, constraint[0])[4])
    link2_COM_coor = np.array(p.getLinkState(robot_id, constraint[1])[4])
    p.addUserDebugLine(link1_COM_coor+np.array(constraint[2]), link2_COM_coor+np.array(constraint[3]), [1, 0, 0], 1)

    #constrains go from the linkWorldPosition (cartesian position of cernter of mass) to the local joint position.
    #urdflinks do with the worldLinkFramePosition
    link1_diff = np.round(np.array(p.getLinkState(robot_id, constraint[0])[4]) - np.array(p.getLinkState(robot_id, constraint[0])[0]), 2)
    link2_diff = np.round(np.array(p.getLinkState(robot_id, constraint[1])[4]) - np.array(p.getLinkState(robot_id, constraint[1])[0]), 2)

    link1_pos = np.array(constraint[2])+link1_diff
    link2_pos = np.array(constraint[3])+link2_diff
    print("link1_pos: ", link1_pos)
    print("link2_pos: ", link2_pos)

    print(link1_pos, "   ", link2_pos)

    cid = p.createConstraint(robot_id, constraint[0], robot_id, constraint[1], p.JOINT_POINT2POINT, constraint[4], link1_pos, link2_pos)
    p.changeConstraint(cid, maxForce=100)





# Set gravity to earth gravity (9.8 m/s^2)
p.setGravity(0, 0, -9.8)

# Enable real-time simulation
p.setRealTimeSimulation(1)

# Run the simulation for a while
try:
    while True:
        time.sleep(1./240.)  # Sleep to keep the simulation running
except KeyboardInterrupt:
    pass

# Disconnect from PyBullet
p.disconnect()