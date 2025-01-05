import pybullet as p
import pybullet_data
import time
import math


#show the constraint
connection_id = p.connect(p.GUI)            #RL 2024

# Assume the simulation is already started and your model is loaded
robot_id = p.loadURDF(r"C:\Users\kevin\3D Objects\Projects\Rabbit\Test2\TestURDF_description\urdf\TestURDF.xacro")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
ground = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)
p.changeDynamics(ground, -1, lateralFriction=1)

# Debug joint information
for i in range(p.getNumJoints(robot_id)):
    print(p.getJointInfo(robot_id, i))

#show lines at the given coordinates
koordinates = [(0.07, 0.005, 0.0), (0.0, 0.005, 0.0), (0.005022, 0.005, 0.06982), (0.051, 0.1, 0.067), (0.056, 0.017, 0.081)]
for i in range(len(koordinates) - 1):
    p.addUserDebugLine(koordinates[i], koordinates[i + 1], [0, 1, 0], 5)
    

middle_point_link = (0, 0, 0)
middle_point_link_2 = (0, 0, 0)

# Fix the base link to the ground
base_constraint_id = p.createConstraint(
    parentBodyUniqueId=robot_id,
    parentLinkIndex=-1,  # Base link index
    childBodyUniqueId=ground,
    childLinkIndex=-1,  # Ground link index
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0]
)
p.changeConstraint(base_constraint_id, maxForce=100000)
middle_point_link_2 = (0, 0, 0)

# Add the constraint
# constraint_id = p.createConstraint(
#     parentBodyUniqueId=robot_id,
#     parentLinkIndex=2,  # Replace with actual parent link index
#     childBodyUniqueId=robot_id,
#     childLinkIndex=0,  # Replace with actual child link index
#     jointType=p.JOINT_POINT2POINT,
#     jointAxis=[0, 0, 0],
#     parentFramePosition=middle_point_link,
#     childFramePosition=middle_point_link_2,
# )
# p.changeConstraint(constraint_id, maxForce=1000)
p.stepSimulation()
time.sleep(1)
# Simulation loop
while True:
    # Get the current position of the joint
    joint_position = p.getJointState(robot_id, 0)[0]
    
    # Define the target position
    target_position = 0.5 * math.sin(time.time())
    
    # Set the joint motor control to move the joint to the target position
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        force=500
    )
    
    p.stepSimulation()
    time.sleep(0.01)

