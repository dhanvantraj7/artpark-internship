import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

class IMUParticleFilter:
    def __init__(self, num_particles, state_dim, resample_threshold=0.5):
        """
        Initialize the IMU Particle Filter.
        
        Args:
        num_particles (int): Number of particles to use in the filter.
        state_dim (int): Dimensionality of the state space.
        resample_threshold (float): Threshold for resampling to avoid particle degeneracy.
        """
        self.num_particles = num_particles
        self.state_dim = state_dim
        self.resample_threshold = resample_threshold
        self.particles = np.random.randn(num_particles, state_dim)
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, motion_model, dt, motion_noise):
        """
        Predict the next state of the particles using the motion model.
        
        Args:
        motion_model (function): Function to model the motion of the particles.
        dt (float): Time step for prediction.
        motion_noise (np.ndarray): Motion noise to be added to the prediction.
        """
        noise = np.random.randn(self.num_particles, self.state_dim) * motion_noise
        self.particles = motion_model(self.particles, dt) + noise

    def update(self, observation, observation_model, observation_noise):
        """
        Update the particle weights based on the observation.
        
        Args:
        observation (np.ndarray): The observed state.
        observation_model (function): Function to model the observation.
        observation_noise (np.ndarray): Observation noise.
        """
        for i in range(self.num_particles):
            predicted_observation = observation_model(self.particles[i])
            self.weights[i] = self._gaussian_likelihood(observation, predicted_observation, observation_noise)
        
        # Normalize the weights
        self.weights += 1.e-300  # Prevent division by zero
        self.weights /= np.sum(self.weights)

        # Resample if necessary
        if self._effective_sample_size() < self.resample_threshold * self.num_particles:
            self._resample()

    def _gaussian_likelihood(self, observation, prediction, observation_noise):
        """
        Compute the Gaussian likelihood of an observation.
        
        Args:
        observation (np.ndarray): The observed state.
        prediction (np.ndarray): The predicted state.
        observation_noise (np.ndarray): Observation noise.
        
        Returns:
        float: The likelihood of the observation given the prediction.
        """
        return np.exp(-0.5 * np.sum(((observation - prediction) / observation_noise)**2))

    def _effective_sample_size(self):
        """
        Compute the effective sample size of the particle weights.
        
        Returns:
        float: The effective sample size.
        """
        return 1.0 / np.sum(self.weights**2)

    def _resample(self):
        """
        Resample the particles to avoid degeneracy.
        """
        indices = np.random.choice(np.arange(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def estimate(self):
        """
        Estimate the state as the weighted mean of the particles.
        
        Returns:
        np.ndarray: The estimated state.
        """
        return np.average(self.particles, weights=self.weights, axis=0)

def imu_motion_model(particles, dt):
    """
    Motion model for the IMU.
    
    Args:
    particles (np.ndarray): Current state of the particles.
    dt (float): Time step for prediction.
    
    Returns:
    np.ndarray: Predicted state of the particles.
    """
    return particles  # Modify this based on your specific motion model

def imu_observation_model(particle):
    """
    Observation model for the IMU.
    
    Args:
    particle (np.ndarray): State of a single particle.
    
    Returns:
    np.ndarray: Observed state.
    """
    return particle

# Parameters
num_particles = 1000
state_dim = 6  # 3 for accelerometer + 3 for gyroscope
motion_noise = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
observation_noise = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])

# Load IMU data from CSV file
file_path = 'C:/Users/admin/Documents/particle filter/IMU_readings.csv'
imu_data_df = pd.read_csv(file_path)

# Extract time and sensor data
time_data = imu_data_df.iloc[:, 0].values  # Time column
imu_data = imu_data_df.iloc[:, 1:7].values  # Accel x, y, z and Gyro x, y, z

# Calculate time differences
dt = np.diff(time_data, prepend=time_data[0])

# Initialize IMU Particle Filter
imu_pf = IMUParticleFilter(num_particles, state_dim)

# Array to hold denoised data
denoised_data = []

# Process each observation
for i, observation in enumerate(imu_data):
    # Predict step
    imu_pf.predict(imu_motion_model, dt[i], motion_noise)
    
    # Update step
    imu_pf.update(observation, imu_observation_model, observation_noise)
    
    # Estimate the state
    state_estimate = imu_pf.estimate()
    denoised_data.append(state_estimate)

# Convert denoised data to DataFrame
denoised_df = pd.DataFrame(denoised_data, columns=['Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
denoised_df['Time'] = time_data

# Plot raw and denoised data for comparison
fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)

axs[0].plot(time_data, imu_data[:, 0], label='Raw Accel_X')
axs[0].plot(time_data, denoised_df['Accel_X'], label='Denoised Accel_X')
axs[0].legend()

axs[1].plot(time_data, imu_data[:, 1], label='Raw Accel_Y')
axs[1].plot(time_data, denoised_df['Accel_Y'], label='Denoised Accel_Y')
axs[1].legend()

axs[2].plot(time_data, imu_data[:, 2], label='Raw Accel_Z')
axs[2].plot(time_data, denoised_df['Accel_Z'], label='Denoised Accel_Z')
axs[2].legend()

axs[3].plot(time_data, imu_data[:, 3], label='Raw Gyro_X')
axs[3].plot(time_data, denoised_df['Gyro_X'], label='Denoised Gyro_X')
axs[3].legend()

axs[4].plot(time_data, imu_data[:, 4], label='Raw Gyro_Y')
axs[4].plot(time_data, denoised_df['Gyro_Y'], label='Denoised Gyro_Y')
axs[4].legend()

axs[5].plot(time_data, imu_data[:, 5], label='Raw Gyro_Z')
axs[5].plot(time_data, denoised_df['Gyro_Z'], label='Denoised Gyro_Z')
axs[5].legend()

plt.xlabel('Time')
plt.show()
