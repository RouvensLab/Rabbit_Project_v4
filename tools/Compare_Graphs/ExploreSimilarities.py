import numpy as np
import matplotlib.pyplot as plt

# Helper functions for similarity metrics
def correlation_coefficient(series1, series2):
    """Compute the Pearson correlation coefficient between two series."""
    return np.corrcoef(series1, series2)[0, 1]

def mean_squared_error(series1, series2):
    """Calculate the mean squared error between two series."""
    return np.mean((series1 - series2) ** 2)

def simple_dtw(series1, series2):
    """
    Compute a simplified Dynamic Time Warping distance using NumPy.
    Returns the DTW distance (lower means more similar).
    """
    n, m = len(series1), len(series2)
    dtw_matrix = np.zeros((n + 1, m + 1))
    dtw_matrix[0, 1:] = np.inf
    dtw_matrix[1:, 0] = np.inf

    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = abs(series1[i - 1] - series2[j - 1])
            dtw_matrix[i, j] = cost + min(
                dtw_matrix[i - 1, j],    # Insertion
                dtw_matrix[i, j - 1],    # Deletion
                dtw_matrix[i - 1, j - 1] # Match
            )
    return dtw_matrix[n, m]

def derivative_correlation(series1, series2, times):
    """Compute the correlation of the derivatives (rates of change) of two series."""
    deriv1 = np.gradient(series1, times)
    deriv2 = np.gradient(series2, times)
    return np.corrcoef(deriv1, deriv2)[0, 1]

def normalize_series(series):
    """Normalize a series to focus on shape, scaling by its maximum absolute value."""
    return series / np.max(np.abs(series))

# Main function to explore similarities and plot results
def explore_similarity(total_current, total_current2, times, title="Total Currents Comparison", labels=("osz_total_current", "servo_total_current"), x_label="Time (s)", y_label="Current (A)"):
    """
    Explore similarities between two time series using multiple metrics and visualize them.
    
    Parameters:
    - total_current: First time series (e.g., oscilloscope total current)
    - total_current2: Second time series (e.g., servo total current)
    - times: Time array corresponding to both series
    - title: Title for the plots
    - labels: Tuple containing labels for the two series
    - x_label: Label for the x-axis
    - y_label: Label for the y-axis
    """
    # Ensure all arrays are the same length
    min_len = min(len(total_current), len(total_current2), len(times))
    total_current = total_current[:min_len]
    total_current2 = total_current2[:min_len]
    times = times[:min_len]

    # Normalize the series to compare shapes
    total_current_norm = normalize_series(total_current)
    total_current2_norm = normalize_series(total_current2)

    # Compute similarity metrics
    corr_coef = correlation_coefficient(total_current, total_current2)
    mse_value = mean_squared_error(total_current, total_current2)
    dtw_dist = simple_dtw(total_current, total_current2)
    deriv_corr = derivative_correlation(total_current, total_current2, times)
    corr_coef_norm = correlation_coefficient(total_current_norm, total_current2_norm)
    mse_value_norm = mean_squared_error(total_current_norm, total_current2_norm)

    # Display results
    print("Similarity Metrics:")
    print(f"- Correlation Coefficient (original): {corr_coef:.4f}")
    print(f"- Mean Squared Error (original): {mse_value:.4f}")
    print(f"- DTW Distance (simplified): {dtw_dist:.4f}")
    print(f"- Derivative Correlation: {deriv_corr:.4f}")
    print(f"- Correlation Coefficient (normalized): {corr_coef_norm:.4f}")
    print(f"- Mean Squared Error (normalized): {mse_value_norm:.4f}")

    # Plotting
    plt.figure(figsize=(12, 8))

    # Subplot 1: Original series
    plt.subplot(2, 1, 1)
    plt.plot(times, total_current, label=labels[0])
    plt.plot(times, total_current2, label=labels[1])
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(f"Original {title}")
    plt.legend()
    plt.grid(True)

    # Subplot 2: Normalized series
    plt.subplot(2, 1, 2)
    plt.plot(times, total_current_norm, label=f"{labels[0]} (normalized)")
    plt.plot(times, total_current2_norm, label=f"{labels[1]} (normalized)")
    plt.xlabel(x_label)
    plt.ylabel("Normalized Current")
    plt.title(f"Normalized {title}")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # Additional plot for derivatives
    deriv1 = np.gradient(total_current, times)
    deriv2 = np.gradient(total_current2, times)
    plt.figure(figsize=(8, 4))
    plt.plot(times, deriv1, label=f"d({labels[0]})/dt")
    plt.plot(times, deriv2, label=f"d({labels[1]})/dt")
    plt.xlabel(x_label)
    plt.ylabel("Rate of Change")
    plt.title(f"Derivatives of {title}")
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage with your original data
if __name__ == "__main__":
    # Assuming you have your data from your original script
    import sys
    sys.path.append(r"tools")
    from Trajectory import TrajectoryRecorder

    Trajectory_Base = TrajectoryRecorder()
    Trajectory_Base.load_trajectory("Bew3")
    keys = Trajectory_Base.get_keys()
    values = Trajectory_Base.get_values()
    times1 = Trajectory_Base.get_times()

    Trajectory_Base2 = TrajectoryRecorder()
    Trajectory_Base2.load_trajectory("Trajectory1")
    times2 = Trajectory_Base2.get_times()
    total_current = np.array(Trajectory_Base2.get_keyValues("total_current"))[:, 0]
    # joints_currents = np.array(Trajectory_Base.get_keyValues("joint_torques"))  # list with list of joint currents
    # total_current2 = np.absolute(np.sum(joints_currents, axis=1))/12  # sum of all joint currents
    joints_currents = np.array(Trajectory_Base2.get_keyValues("joint_currents"))  # list with list of joint currents
    wechsel = 1024*0.0065
    new_joints_currents = np.where(joints_currents > wechsel, -(joints_currents-wechsel), joints_currents)
    total_current2 = np.absolute(np.sum(new_joints_currents, axis=1))
    
    print(total_current2)
    print(total_current)
    print(times1)
    print(times2)
    #make both time arrays the same length
    times1 = times1[:len(total_current2)]
    times2 = times2[:len(total_current2)]
    # Call the function
    explore_similarity(total_current, total_current2, times2)