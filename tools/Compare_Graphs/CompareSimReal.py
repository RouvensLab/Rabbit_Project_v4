#get access to the tools folder
import sys
sys.path.append(r"tools")

from Trajectory import TrajectoryRecorder

import matplotlib.pyplot as plt
import numpy as np

from ExploreSimilarities import explore_similarity


# Trajectory_Base = TrajectoryRecorder()
# Trajectory_Base.load_trajectory("RealJump_current_v1")
# keys1 = Trajectory_Base.get_keys()
# values1 = Trajectory_Base.get_values()
# times1 = Trajectory_Base.get_times()
# joint_angles1 = np.array(Trajectory_Base.get_keyValues("joint_angles"))

# Trajectory_Base2 = TrajectoryRecorder()
# Trajectory_Base2.load_trajectory("SimJump_current_v1")
# keys2 = Trajectory_Base2.get_keys()
# values2 = Trajectory_Base2.get_values()
# times2 = Trajectory_Base2.get_times()
# joint_angles2 = np.array(Trajectory_Base2.get_keyValues("joint_angles"))

# #shorten everything to the same length
# min_len = min(len(joint_angles1), len(joint_angles2))
# joint_angles1 = joint_angles1[:min_len]
# joint_angles2 = joint_angles2[:min_len]
# times1 = times1[:min_len]
# times2 = times2[:min_len]

# #difference
# joint_angles_diff = joint_angles1 - joint_angles2

# show_diff(times1, joint_angles1, joint_angles2, "Joint angles of the oscilloscope and the servos", "Time in s", "Joint angles in rad", "osz_joint_angles", "servo_joint_angles")

#simulation
servo_id = 2
Trajectory_Base = TrajectoryRecorder()
Trajectory_Base.load_trajectory("Bew2")
keys1 = Trajectory_Base.get_keys()
values1 = Trajectory_Base.get_values()
times1 = Trajectory_Base.get_times()
# total_current = np.array(Trajectory_Base.get_keyValues("total_current"))[:, 0]
joint_angels = np.array(Trajectory_Base.get_keyValues("joint_angles"), dtype=float)[:, servo_id]
joint_velocities = np.array(Trajectory_Base.get_keyValues("joint_velocities"), dtype=float)[:, servo_id]
joint_torque1 = np.array(Trajectory_Base.get_keyValues("joint_torques"))[:, servo_id]
joint_power1 =  np.absolute(joint_torque1)

#real robot
Trajectory_Base2 = TrajectoryRecorder()
Trajectory_Base2.load_trajectory("Bew3")
keys2 = Trajectory_Base2.get_keys()
values2 = Trajectory_Base2.get_values()
times2 = Trajectory_Base2.get_times()
joint_angels2 = np.array(Trajectory_Base2.get_keyValues("joint_angles"), dtype=float)[:, servo_id]
joint_vel2 = np.array(Trajectory_Base2.get_keyValues("joint_velocities"), dtype=float)[:, servo_id]
joint_torque2 = np.array(Trajectory_Base2.get_keyValues("joint_torques"))[:, servo_id]
joint_power2 =  np.absolute(joint_torque2)




#explore_similarity(power1, power2, np.intersect1d(times1, times2))


explore_similarity(joint_angels, joint_angels2, np.intersect1d(times1, times2), title="Joint angles of the Simulation and the Real Robot", labels=["Simulation", "Real Robot"], y_label="Joint angles in rad")
explore_similarity(joint_velocities, joint_vel2, np.intersect1d(times1, times2), title="Joint velocities of the Simulation and the Real Robot", labels=["Simulation", "Real Robot"], y_label="Joint velocities in rad/s")
explore_similarity(joint_power1, joint_power2, np.intersect1d(times1, times2), title="Joint power of the Simulation and the Real Robot", labels=["Simulation", "Real Robot"], y_label="Joint power in Nm")









