#import necessary packages
import sys
sys.path.append(r"tools")

from Trajectory import TrajectoryRecorder

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

def show_diff(times1, times2, power1, power2, title, xlabel, ylabel, label1, label2):
    # Determine the overlapping time interval
    start_time = max(times1[0], times2[0])
    end_time = min(times1[-1], times2[-1])
    
    # Use the smaller number of samples to set the resolution for the common grid
    num_points = min(len(times1), len(times2))
    common_times = np.linspace(start_time, end_time, num_points)
    
    # Create interpolation functions for both power series
    interp_func1 = interp1d(times1, power1, kind='linear', fill_value="extrapolate")
    interp_func2 = interp1d(times2, power2, kind='linear', fill_value="extrapolate")
    
    # Evaluate the power data on the common time grid
    power1_interp = interp_func1(common_times)
    power2_interp = interp_func2(common_times)
    
    # Calculate the difference
    power_diff = power1_interp - power2_interp

    # Create two subplots: one for the raw data and one for the difference
    fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # First subplot: original and interpolated power curves
    axs[0].plot(times1, power1, label=label1, alpha=0.5, marker='o', linestyle='-', markersize=3)
    axs[0].plot(times2, power2, label=label2, alpha=0.5, marker='o', linestyle='-', markersize=3)
    axs[0].plot(common_times, power1_interp, label=f'{label1} (interp)', linestyle='--', color='blue')
    axs[0].plot(common_times, power2_interp, label=f'{label2} (interp)', linestyle='--', color='orange')
    axs[0].set_ylabel(ylabel)
    axs[0].set_title(title)
    axs[0].legend()
    
    # Second subplot: difference between the two power curves
    axs[1].plot(common_times, power_diff, label="Difference", color='red', linestyle='-', marker='.')
    axs[1].set_xlabel(xlabel)
    axs[1].set_ylabel("Difference (W)")
    axs[1].legend()
    
    plt.tight_layout()
    plt.show()

# Load and process trajectory data from the oscilloscope (RealJump)
Trajectory_Base = TrajectoryRecorder()
Trajectory_Base.load_trajectory("RealJump_current_v1")
times1 = Trajectory_Base.get_times()
total_current = np.array(Trajectory_Base.get_keyValues("total_current"))[:, 0]
power1 = total_current * 12.2  # Calculate power using a constant voltage

# Load and process trajectory data from the servos (SimJump)
Trajectory_Base2 = TrajectoryRecorder()
Trajectory_Base2.load_trajectory("SimJump_current_v1")
times2 = Trajectory_Base2.get_times()
joint_vel2 = np.array(Trajectory_Base2.get_keyValues("joint_velocities"))
joint_torque2 = np.array(Trajectory_Base2.get_keyValues("joint_torques"))
joint_power2 = np.absolute(joint_vel2 * joint_torque2)
power2 = joint_power2.sum(axis=1)

# Visualize the comparison
show_diff(times1, times2, power1, power2,
          title="Power Oscilloscope vs. Power Simulation",
          xlabel="Time (s)",
          ylabel="Power (W)",
          label1="osz_power",
          label2="sim_power")
