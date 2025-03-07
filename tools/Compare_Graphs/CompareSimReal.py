#get access to the tools folder
import sys
sys.path.append(r"tools")

from Trajectory import TrajectoryRecorder

import matplotlib.pyplot as plt
import numpy as np

Trajectory_Base = TrajectoryRecorder()
Trajectory_Base.load_trajectory("Trajectory2")
keys1 = Trajectory_Base.get_keys()
values1 = Trajectory_Base.get_values()
times1 = Trajectory_Base.get_times()
joint_angles1 = np.array(Trajectory_Base.get_keyValues("joint_angles"))[:, 0]

Trajectory_Base2 = TrajectoryRecorder()
Trajectory_Base2.load_trajectory("Trajectory3")
keys2 = Trajectory_Base2.get_keys()
values2 = Trajectory_Base2.get_values()
times2 = Trajectory_Base2.get_times()
joint_angles2 = np.array(Trajectory_Base2.get_keyValues("joint_angles"))[:, 0]

#shorten everything to the same length
min_len = min(len(joint_angles1), len(joint_angles2))
joint_angles1 = joint_angles1[:min_len]
joint_angles2 = joint_angles2[:min_len]
times1 = times1[:min_len]
times2 = times2[:min_len]

#difference
joint_angles_diff = joint_angles1 - joint_angles2

plt.plot(times1, joint_angles1, label="Real")
plt.plot(times1, joint_angles2, label="Sim")
plt.plot(times1, joint_angles_diff, label="difference")
plt.xlabel("Time in s")
plt.ylabel("position in rad")
plt.title("Joint angles of the real and the simulated robot")
plt.legend()
plt.show()




