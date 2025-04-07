import numpy as np
import matplotlib.pyplot as plt

# Initialize variables
x_gps = 0  # Initial GPS reading
x_imu = 0  # Initial IMU reading
x_gps_R = 1  # GPS noise covariance
x_imu_R = 1  # IMU noise covariance
T = 100  # Duration
N = 10  # Number of particles

# Generate initial particles around initial GPS reading
V = 2  # Variance of initial estimate
x_P = np.random.normal(x_gps, np.sqrt(V), N)

# Generate initial observations
z_gps_out = [x_gps + np.random.normal(0, np.sqrt(x_gps_R))]
z_imu_out = [x_imu + np.random.normal(0, np.sqrt(x_imu_R))]
x_out = [x_gps]
x_est = [x_gps]
x_est_out = [x_est[-1]]

# Update GPS reading with noise
def update_gps(x_gps, x_gps_R):
    return x_gps + np.random.normal(0, np.sqrt(x_gps_R))

# Update IMU reading with noise
def update_imu(x_imu, x_imu_R):
    return x_imu + np.random.normal(0, np.sqrt(x_imu_R))

# Update particle based on GPS and IMU readings
def update_particle(x_P, x_gps, x_imu, x_gps_R, x_imu_R):
    return (x_P + x_gps + x_imu) / 3  # Simple average fusion

# Calculate likelihood of observation given true value and noise
def gaussian_likelihood(z_out, z_update, x_R):
    return 1 / np.sqrt(2 * np.pi * x_R) * np.exp(-(z_out - z_update) ** 2 / (2 * x_R))


for t in range(1, T + 1):
    # Update true state and generate observations
    x_gps = update_gps(x_gps, x_gps_R)
    x_imu = update_imu(x_imu, x_imu_R)
    
    # Perform sensor fusion using particle filter
    x_P_update = np.array([update_particle(p, x_gps, x_imu, x_gps_R, x_imu_R) for p in x_P])
    
    # Generate observations for each particle
    z_gps_update = x_P_update + np.random.normal(0, np.sqrt(x_gps_R), N)
    z_imu_update = x_P_update + np.random.normal(0, np.sqrt(x_imu_R), N)
    
    # Calculate weights based on observation likelihood
    P_w_gps = gaussian_likelihood(z_gps_out[-1], z_gps_update, x_gps_R)
    P_w_imu = gaussian_likelihood(z_imu_out[-1], z_imu_update, x_imu_R)
    
    # Combine weights from GPS and IMU
    P_w = P_w_gps * P_w_imu
    
    # Normalize weights
    P_w /= np.sum(P_w)
    
    # Resample particles
    indices = np.random.choice(np.arange(N), size=N, p=P_w)
    x_P = x_P_update[indices]
    
    # Calculate final estimate
    x_est = np.mean(x_P)
    x_est_out.append(x_est)
    
    # Save data for plotting
    x_out.append(x_gps)
    z_gps_out.append(x_gps)
    z_imu_out.append(x_imu)

# Plot results
t = np.arange(T + 1)
plt.plot(t, x_out, '.-b', label='True state')
plt.plot(t, x_est_out, '-.r', label='Fused sensor estimate')
plt.xlabel('Time step')
plt.ylabel('Fused sensor estimate')
plt.legend()
plt.show()