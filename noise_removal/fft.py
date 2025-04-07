import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.fft import fft, fftfreq

def read_imu_data(filename):
    df = pd.read_excel(filename, engine='openpyxl')
    time = df.iloc[:, 0].values
    accel_data = df.iloc[:, 1:4].values
    gyro_data = df.iloc[:, 4:7].values
    return time, accel_data, gyro_data

def apply_fft_to_data(data, sampling_interval):
    n = len(data)
    freq = fftfreq(n, d=sampling_interval)
    fft_data = fft(data, axis=0)
    return freq, np.abs(fft_data)

def plot_fft_results(freq, fft_data, label):
    plt.figure(figsize=(10, 6))
    plt.plot(freq, fft_data, label=label)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('FFT Results')
    plt.grid(True)
    plt.legend()
    plt.show()

filename = 'C:\\Users\\Camras\\Documents\\VS code\\python\\noise_removal\\Dhanvanthraj.xlsx'
time, accel_data, gyro_data = read_imu_data(filename)

sampling_interval = time[1] - time[0]

# Compute FFT for accelerometer data
freq, fft_accel_data = apply_fft_to_data(accel_data, sampling_interval)
plot_fft_results(freq, fft_accel_data, 'FFT of Accelerometer Data')

# Compute FFT for gyroscope data
freq, fft_gyro_data = apply_fft_to_data(gyro_data, sampling_interval)
plot_fft_results(freq, fft_gyro_data, 'FFT of Gyroscope Data')
