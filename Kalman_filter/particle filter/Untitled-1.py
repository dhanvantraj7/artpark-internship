import numpy as np
import matplotlib.pyplot as plt
import csv

class ParticleFilter:
    def __init__(self, num_particles, dt, process_noise_std, gps_noise_std):
        self.num_particles = num_particles
        self.dt = dt
        self.process_noise_std = process_noise_std
        self.gps_noise_std = gps_noise_std

        # State vector: (position, velocity)
        self.state_dim = 6

        # Initialize particles
        self.particles = np.random.rand(self.num_particles, self.state_dim)
        self.particles[:, :3] *= 10  # Random position within 10 meters
        self.particles[:, 3:6] *= 2  # Random velocity within -2 to 2 m/s
        self.weights = np.ones(self.num_particles) / self.num_particles

    def normalize_weights(self):
        total_weight = np.sum(self.weights)
        if total_weight != 0:
            self.weights /= total_weight
        else:
            self.weights = np.ones(self.num_particles) / self.num_particles

    def predict(self, imu_data):
        # Extract IMU data
        accel = imu_data[:3]

        # Process noise for position and velocity
        process_noise_pos = np.random.normal(scale=self.process_noise_std, size=(self.num_particles, 3))
        process_noise_vel = np.random.normal(scale=self.process_noise_std, size=(self.num_particles, 3))

        # Update position and velocity
        self.particles[:, :3] += self.particles[:, 3:6] * self.dt + 0.5 * accel * self.dt**2 + process_noise_pos
        self.particles[:, 3:6] += accel * self.dt + process_noise_vel

    def update(self, gps_data):
        # Observation function (extract position from GPS)
        z_pred = self.particles[:, :3]

        # Innovation (difference between GPS and predicted position)
        innovation = gps_data - z_pred

        # Calculate weights using Gaussian PDF (assuming GPS noise is Gaussian)
        self.weights = np.exp(-0.5 * np.sum(innovation**2 / (self.gps_noise_std**2), axis=1))
        self.normalize_weights()

    def resample(self):
        # Implement stratified or residual resampling here (optional)
        pass

    def estimate(self):
        # Reshape particles to match weights (optional)
        particles_reshaped = self.particles[:, :3]  # Select only position components
        return np.average(particles_reshaped, weights=self.weights, axis=0)

# class KalmanFilter:
#     def __init__(self, F, B, H, Q, R, initial_state, initial_covariance):
#         self.F = F  # State transition matrix
#         self.B = B  # Control input matrix
#         self.H = H  # Measurement matrix
#         self.Q = Q  # Process noise covariance
#         self.R = R  # Measurement noise covariance

#         self.state = initial_state  # Initial state estimate
#         self.covariance = initial_covariance  # Initial covariance estimate

#     def predict(self, control_input=None):
#         # Predict state and covariance
#         self.state = np.dot(self.F, self.state)
#         if control_input is not None:
#             self.state += np.dot(self.B, control_input)
#         self.covariance = np.dot(self.F, np.dot(self.covariance, self.F.T)) + self.Q

#     def update(self, measurement):
#         # Calculate Kalman gain
#         K = np.dot(self.covariance, np.dot(self.H.T, np.linalg.inv(np.dot(self.H, np.dot(self.covariance, self.H.T)) + self.R)))

#         # Update state estimate
#         self.state += np.dot(K, measurement - np.dot(self.H, self.state))

#         # Update covariance estimate
#         self.covariance = np.dot((np.eye(self.covariance.shape[0]) - np.dot(K, self.H)), self.covariance)

def read_sensor_data(filename):
    timestamps = []
    accelerometer_data = []
    gyroscope_data = []
    gps_data = []  

    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  

        for row in reader:
            timestamps.append(float(row[0]))
            accelerometer_data.append(np.array([float(v) for v in row[2:5]]))
            gyroscope_data.append(np.array([float(v) for v in row[5:8]]))
            gps_data.append(np.array([float(v) for v in row[13:16]]))

    return timestamps, accelerometer_data, gyroscope_data, gps_data

# def kalman_filter(accelerometer_data, gyroscope_data, gps_data):
#     # Define Kalman Filter matrices and initial estimates
#     F = np.eye(3)  # State transition matrix
#     B = np.zeros((3, 3))  # Control input matrix
#     H = np.eye(3)  # Measurement matrix
#     Q = np.eye(3) * 0.01  # Process noise covariance
#     R = np.eye(3) * 0.1  # Measurement noise covariance
#     initial_state = np.zeros(3)  # Initial state estimate
#     initial_covariance = np.eye(3) * 0.1  # Initial covariance estimate

#     # Create Kalman Filter
#     kf = KalmanFilter(F, B, H, Q, R, initial_state, initial_covariance)

#     estimated_state = np.zeros((len(gps_data), 3))  # Initialize estimated state

#     for i in range(len(gps_data)):
#         # Prediction step
#         kf.predict(control_input=gyroscope_data[i])

#         # Update step
#         kf.update(gps_data[i])

#         # Store the estimated state
#         estimated_state[i] = kf.state

#     return estimated_state

filename = "C:\\Users\\admin\\Downloads\\ARTpark\\Kalman_filter\\particle filter\\2014-04-23-GPS-IMU-Data.csv"
timestamps, accelerometer_data, gyroscope_data, gps_data = read_sensor_data(filename)

# # Kalman Filter to estimate position
# estimated_position = kalman_filter(accelerometer_data, gyroscope_data, gps_data)

# Original GPS position
original_position = np.array(gps_data)

# Particle Filter to estimate position
pf = ParticleFilter(num_particles=1000, dt=0.01, process_noise_std=0.1, gps_noise_std=0.1)
estimated_position_pf = np.zeros((len(gps_data), 3))  # Initialize estimated position for particle filter

for i in range(len(gps_data)):
    # Prediction step
    pf.predict(accelerometer_data[i])

    # Update step
    pf.update(gps_data[i])

    # Estimate position
    estimated_position_pf[i] = pf.estimate()

# Plotting the filtered and original positions for Kalman and Particle filters
plt.figure(figsize=(10, 6))
# plt.plot(timestamps, estimated_position[:, 0], label='Kalman Filtered X Position', color='blue')
# plt.plot(timestamps, estimated_position[:, 1], label='Kalman Filtered Y Position', color='green')
# plt.plot(timestamps, estimated_position[:, 2], label='Kalman Filtered Z Position', color='red')
plt.plot(timestamps, accelerometer_data, label='accel', color='red')
plt.plot(timestamps, gyroscope_data, label='gyro', color='blue')
plt.plot(timestamps, gps_data, label='gps', color='green')
plt.plot(timestamps, estimated_position_pf[:, 0], label='Particle Filtered X Position', linestyle='--', color='cyan')
plt.plot(timestamps, estimated_position_pf[:, 1], label='Particle Filtered Y Position', linestyle='--', color='lime')
plt.plot(timestamps, estimated_position_pf[:, 2], label='Particle Filtered Z Position', linestyle='--', color='magenta')
plt.plot(timestamps, original_position[:, 0], linestyle='-.', label='Original X Position', color='blue')
plt.plot(timestamps, original_position[:, 1], linestyle='-.', label='Original Y Position', color='green')
plt.plot(timestamps, original_position[:, 2], linestyle='-.', label='Original Z Position', color='red')
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Comparison of Kalman and Particle Filtered Position Estimates')
plt.legend()
plt.grid(True)
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
# # Plotting the filtered and original positions
# plt.figure(figsize=(10, 6))
# plt.plot(timestamps, estimated_position[:, 0], label='Filtered X Position', color='blue')
# plt.plot(timestamps, estimated_position[:, 1], label='Filtered Y Position', color='green')
# plt.plot(timestamps, estimated_position[:, 2], label='Filtered Z Position', color='red')
# plt.plot(timestamps, original_position[:, 0], linestyle='--', label='Original X Position', color='blue')
# plt.plot(timestamps, original_position[:, 1], linestyle='--', label='Original Y Position', color='green')
# plt.plot(timestamps, original_position[:, 2], linestyle='--', label='Original Z Position', color='red')
# plt.xlabel('Time')
# plt.ylabel('Position')
# plt.title('Filtered and Original Position')
# plt.legend()
# plt.grid(True)
# plt.show()
 
# # Generate random GPS and IMU measurements
# num_measurements = 100
# gps_measurements = np.random.rand(num_measurements) * 10  # Random GPS measurements
# imu_measurements = np.random.randn(num_measurements) * 2   # Random IMU measurements

# # Initialize particles randomly
# def initialize_particles():
#     return np.random.rand(num_measurements)

# # Predict particles' positions based on motion model (IMU)
# def predict_motion_model(particles, imu_measurement):
#     return particles + imu_measurement  # Simple motion model, adding IMU measurement to particles

# # Update particle weights based on sensor measurements (GPS)
# def update_particle_weights(particles, gps_measurement):
#     # Dummy weight update, comparing GPS measurement with particles
#     return np.abs(particles - gps_measurement)

# # Resample particles based on their weights
# def resample_particles(particles, weights):
#     # Simple resampling by randomly selecting particles based on weights
#     indices = np.random.choice(range(len(particles)), size=len(particles), p=weights / np.sum(weights))
#     return particles[indices]

# # Fuse GPS and IMU measurements using Particle Filter
# def fuse_gps_imu_with_pf(gps_measurements, imu_measurements):
#     particles = initialize_particles()
#     fused_measurements = []
#     for imu_measurement, gps_measurement in zip(imu_measurements, gps_measurements):
#         particles = predict_motion_model(particles, imu_measurement)
#         weights = update_particle_weights(particles, gps_measurement)
#         particles = resample_particles(particles, weights)
#         fused_measurement = np.mean(particles)  # Fuse particles to get output measurement
#         fused_measurements.append(fused_measurement)
#     return fused_measurements

# # Fuse measurements
# fused_measurements_pf = fuse_gps_imu_with_pf(gps_measurements, imu_measurements)

# # Plot input (GPS and IMU) and output (fused measurements)
# plt.figure(figsize=(10, 6))
# plt.plot(gps_measurements, label='GPS Measurements')
# plt.plot(imu_measurements, label='IMU Measurements')
# plt.plot(fused_measurements_pf, label='Fused Measurements (Particle Filter)')
# plt.xlabel('Time')
# plt.ylabel('Measurement')
# plt.title('Sensor Fusion of GPS and IMU Measurements with Particle Filter')
# plt.legend()
# plt.grid(True)
# plt.show()

