from Trajectory import TrajectoryRecorder

import matplotlib.pyplot as plt
import numpy as np

Trajectory_Base = TrajectoryRecorder()
Trajectory_Base.load_trajectory("Trajectory1")
keys = Trajectory_Base.get_keys()
values = Trajectory_Base.get_values()
times = Trajectory_Base.get_times()

total_current = np.array(Trajectory_Base.get_keyValues("total_current"))[:, 0]
joints_currents = Trajectory_Base.get_keyValues("joint_currents")#list with list of joint currents
total_current2 = np.array(joints_currents).sum(axis=1)

#normalize
# total_current = total_current / np.average(total_current)
# total_current2 = total_current2 / np.average(total_current2)
#smooth the total current total_current2 and total_current
total_current = np.convolve(total_current, np.ones(5)/5, mode="valid")
total_current2 = np.convolve(total_current2, np.ones(5)/5, mode="valid")


cross_correlation = np.correlate(total_current, total_current2, mode="same")
best_time_diff = cross_correlation.argmin() - (len(total_current2) - 1)
print("best_time_diff", best_time_diff)

#put the total_current2 as far back so it overlaps perfectly with total_current
total_current2 = np.roll(total_current2, best_time_diff)

together = total_current-total_current2
#get highest and lowest value of every curve
print("total_current:" , max(total_current), min(total_current))
print("total_current2:" , max(total_current2), min(total_current2))
print("together:" , max(together), min(together))

#make time as long as the total_current
times = times[:len(total_current)]
print("len(times)", len(times))

plt.plot(times, total_current, label="osz_total_current")
plt.plot(times, total_current2, label="servo_total_current")
plt.plot(times, together, label="difference")
plt.xlabel("Time in s")
plt.ylabel("Current in A")
plt.title("Total current of the oscilloscope and the servos")
plt.legend()
plt.show()




