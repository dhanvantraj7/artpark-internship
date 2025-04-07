import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pywt

def read_imu_data(filename):
    df = pd.read_excel(filename, engine='openpyxl')
    time = df.iloc[:, 0].values  
    accel_data = df.iloc[:, 1:4].values
    gyro_data = df.iloc[:, 4:7].values  
    return time, accel_data, gyro_data

def low_pass_filter(data, alpha):
    filtered_data = np.zeros_like(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
    return filtered_data

def differentiate_data(data, time):
    dt = np.diff(time)
    differentiated_data = np.diff(data) / dt
    differentiated_data = np.append(differentiated_data, differentiated_data[-1])  # Extend to match the length
    return differentiated_data

def integrate_data(time, data):
    dt = np.diff(time)
    integral = np.zeros(len(data))
    integral[0] = data[0] * dt[0]  # Initialize the first element
    for i in range(1, len(data)):
        integral[i] = integral[i-1] + data[i] * dt[i-1]
    return integral

def double_integrate_data(time, data):
    first_integral = integrate_data(time, data)
    second_integral = integrate_data(time, first_integral)
    return second_integral

def plot_data(time, data_series, labels, title):
    colors = ['cyan', 'blue', 'purple', 'green', 'red', 'black', 'orange']
    fig, axs = plt.subplots(3, 1, figsize=(10, 18))
    for i in range(3):
        for j, series in enumerate(data_series):
            axs[i].plot(time[:len(series[:, i])], series[:, i], label=labels[j], color=colors[j])
        axs[i].set_title(f'{title} {["X", "Y", "Z"][i]}-axis')
        axs[i].set_xlabel('Time (seconds)')
        axs[i].set_ylabel('Measurement')
        axs[i].legend()
    plt.tight_layout()
    plt.show()

def main():
    filename = 'C:\\Users\\admin\\Downloads\\ARTpark\\Kalman_filter\\particle filter\\noise_removal\\Dhanvanthraj.xlsx'
    time, accel_data, gyro_data = read_imu_data(filename)

    # Accelerometer processing
    lp_accel = np.array([low_pass_filter(accel_data[:, i], 0.1) for i in range(3)]).T
    diff_accel = np.array([differentiate_data(lp_accel[:, i], time) for i in range(3)]).T
    int_accel = np.array([integrate_data(time, lp_accel[:, i]) for i in range(3)]).T
    double_int_accel = np.array([double_integrate_data(time, int_accel[:, i]) for i in range(3)]).T

    # Gyroscope processing
    lp_gyro = np.array([low_pass_filter(gyro_data[:, i], 0.1) for i in range(3)]).T
    diff_gyro = np.array([differentiate_data(lp_gyro[:, i], time) for i in range(3)]).T
    int_gyro = np.array([integrate_data(time, lp_gyro[:, i]) for i in range(3)]).T
    double_int_gyro = np.array([double_integrate_data(time, int_gyro[:, i]) for i in range(3)]).T

    # Plotting
    plot_data(time, [accel_data, lp_accel, diff_accel, int_accel, double_int_accel], 
              ['Raw Data', 'Low-Pass Filtered', 'Differentiated', 'Integrated', 'Double Integrated'], 'Accelerometer')
    plot_data(time, [gyro_data, lp_gyro, diff_gyro, int_gyro, double_int_gyro], 
              ['Raw Data', 'Low-Pass Filtered', 'Differentiated', 'Integrated', 'Double Integrated'], 'Gyroscope')

if __name__ == "__main__":
    main()
