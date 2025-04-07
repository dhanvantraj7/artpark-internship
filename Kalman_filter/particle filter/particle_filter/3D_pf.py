import numpy as np
import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D

def read_sensor_data(filename):
    timestamps = []
    accelerometer_data = []
    gyroscope_data = []
    gps_data = []

    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row

        for row in reader:
            timestamps.append(float(row[0]))
            accelerometer_data.append([float(v) for v in row[1:4]])
            gyroscope_data.append([float(v) for v in row[4:7]])
            gps_data.append([float(v) for v in row[7:10]])

    # Convert lists of lists into 2D NumPy arrays
    accelerometer_data = np.array(accelerometer_data)
    gyroscope_data = np.array(gyroscope_data)
    gps_data = np.array(gps_data)

    return timestamps, accelerometer_data, gyroscope_data, gps_data

def estimate_position_change_from_accel(accel_data):
    # Placeholder: actual implementation would involve integrating acceleration twice
    return np.random.normal(0, 0.1, 3)

def particle_filter(timestamps, accelerometer_data, gps_data, N=100, V=2):
    if gps_data.size == 0:
        return np.array([])

    initial_position = gps_data[0]
    particles = np.random.multivariate_normal(initial_position, np.diag([V, V, V]), N)
    estimated_positions = [initial_position]

    for i in range(1, len(timestamps)):
        position_changes = estimate_position_change_from_accel(accelerometer_data[i])
        particles += position_changes

        if i < len(gps_data):
            measurement_noise = np.random.normal(0, 1, (N, 3))
            gps_measurement = gps_data[i] + measurement_noise
            particles = (particles + gps_measurement) / 2

        new_estimated_position = np.mean(particles, axis=0)
        estimated_positions.append(new_estimated_position)

    return np.array(estimated_positions)

filename = "C:\\Users\\admin\\Downloads\\ARTpark\\Kalman_filter\\particle filter\\2014-04-23-GPS-IMU-Data1.csv"
timestamps, accelerometer_data, gyroscope_data, gps_data = read_sensor_data(filename)
estimated_positions = particle_filter(timestamps, accelerometer_data, gps_data)

# 2D Plotting for better visualization
fig, axs = plt.subplots(3, 1, figsize=(15, 10))

# X positions over time
axs[0].plot(timestamps, gps_data[:, 0], 'g-', label='GPS X Position')
axs[0].plot(timestamps[:len(estimated_positions)], estimated_positions[:, 0], 'r--', label='Estimated X Position')
axs[0].set_title('X Position Over Time')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('X Position')
axs[0].legend()

# Y positions over time
axs[1].plot(timestamps, gps_data[:, 1], 'g-', label='GPS Y Position')
axs[1].plot(timestamps[:len(estimated_positions)], estimated_positions[:, 1], 'r--', label='Estimated Y Position')
axs[1].set_title('Y Position Over Time')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Y Position')
axs[1].legend()

# Z positions over time (e.g., altitude)
axs[2].plot(timestamps, gps_data[:, 2], 'g-', label='GPS Z Position')
axs[2].plot(timestamps[:len(estimated_positions)], estimated_positions[:, 2], 'r--', label='Estimated Z Position')
axs[2].set_title('Z Position Over Time')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Z Position')
axs[2].legend()

plt.tight_layout()
plt.show()

def plot_sensor_data(timestamps, accelerometer_data, gyroscope_data, gps_data):
    # Assuming gps_data contains [latitude, longitude, altitude]
    gps_data = np.array(gps_data)
    accelerometer_data = np.array(accelerometer_data)
    gyroscope_data = np.array(gyroscope_data)
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))  # Create 5 subplots
    
    # GPS Data
    axs[0].plot(timestamps, gps_data[:, 0], label='Latitude')
    axs[0].plot(timestamps, gps_data[:, 1], label='Longitude')
    axs[0].plot(timestamps, gps_data[:, 2], label='Altitude')
    axs[0].set_ylabel('GPS Data')
    axs[0].legend()
    
    # Accelerometer Data
    axs[1].plot(timestamps, accelerometer_data[:, 0], label='X')
    axs[1].plot(timestamps, accelerometer_data[:, 1], label='Y')
    axs[1].plot(timestamps, accelerometer_data[:, 2], label='Z')
    axs[1].set_ylabel('Accelerometer')
    axs[1].legend()
    
    # Gyroscope Data
    axs[2].plot(timestamps, gyroscope_data[:, 0], label='X')
    axs[2].plot(timestamps, gyroscope_data[:, 1], label='Y')
    axs[2].plot(timestamps, gyroscope_data[:, 2], label='Z')
    axs[2].set_ylabel('Gyroscope')
    axs[2].legend()
    
    for ax in axs:
        ax.set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()

plot_sensor_data(timestamps, accelerometer_data, gyroscope_data, gps_data)