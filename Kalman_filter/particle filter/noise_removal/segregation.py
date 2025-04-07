import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def read_imu_data(filename):
    df = pd.read_excel(filename, engine='openpyxl')
    time = df.iloc[:, 0].values
    accel_data = df.iloc[:, 1:4].values
    gyro_data = df.iloc[:, 4:7].values
    return time, accel_data, gyro_data

def apply_fft_to_data(data):
    fft_data = np.fft.fft(data, axis=0)
    return fft_data

def plot_fft_results(fft_data, frequency, label):
    plt.figure(figsize=(10, 6))
    plt.plot(frequency, np.abs(fft_data), label=label)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('FFT Results')
    plt.grid(True)
    plt.legend()
    plt.show()

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    if low <= 0 or high >= 1:
        raise ValueError(f"Frequency bounds for the filter must be between 0 and {nyquist} Hz (half the sampling frequency), but got {lowcut} Hz and {highcut} Hz.")
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    filtered_data = filtfilt(b, a, data, axis=0)
    return filtered_data

def integrate_data(data, sampling_interval):
    velocity = np.cumsum(data * sampling_interval, axis=0)
    # position = np.cumsum(velocity * sampling_interval, axis=0)
    return velocity

def plot_position_data(position_data, time, label):
    plt.figure(figsize=(10, 6))
    plt.plot(time, position_data, label=label)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (units)')
    plt.title('Integrated Position Data')
    plt.grid(True)
    plt.legend()
    plt.show()

filename = 'C:\\Users\\Camras\\Documents\\VS code\\python\\noise_removal\\Dhanvanthraj.xlsx'
time, accel_data, gyro_data = read_imu_data(filename)

sampling_interval = time[1] - time[0]
sampling_frequency = 1 / sampling_interval
nyquist_frequency = 0.5 * sampling_frequency

# Adjusted highcut frequencies...
filtered_acceleration_data = apply_bandpass_filter(accel_data, 0.1, min(20, nyquist_frequency - 1), sampling_frequency)

# Integration of filtered acceleration data...
position_data = integrate_data(filtered_acceleration_data, sampling_interval)

# Plotting position data...
plot_position_data(position_data, time, 'Integrated Position (Acceleration)')

# Adjust highcut frequencies based on sampling frequency
vibration_highcut = min(100, nyquist_frequency - 1)  # Safe margin from Nyquist frequency
acceleration_highcut = min(20, nyquist_frequency - 1)

try:
    filtered_vibration_data = apply_bandpass_filter(accel_data, 0.5, vibration_highcut, sampling_frequency)
    filtered_acceleration_data = apply_bandpass_filter(accel_data, 0.1, acceleration_highcut, sampling_frequency)
except ValueError as e:
    print(f"Error: {e}")
    filtered_vibration_data = None
    filtered_acceleration_data = None

if filtered_vibration_data is not None:
    fft_filtered_vibration_data = apply_fft_to_data(filtered_vibration_data)
    plot_fft_results(fft_filtered_vibration_data, frequency=np.fft.fftfreq(len(time), d=sampling_interval), label='Filtered Vibration (Accelerometer)')

if filtered_acceleration_data is not None:
    fft_filtered_acceleration_data = apply_fft_to_data(filtered_acceleration_data)
    plot_fft_results(fft_filtered_acceleration_data, frequency=np.fft.fftfreq(len(time), d=sampling_interval), label='Filtered Linear Acceleration (Accelerometer)')
