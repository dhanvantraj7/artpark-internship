import numpy as np
import matplotlib.pyplot as plt

# Generate time points
t = np.linspace(0, 10, 400)
# Generate sine and cosine waves
sin_wave = np.sin(t)
cos_wave = np.cos(t)

# Convolve sine and cosine waves
resultant_wave = np.convolve(sin_wave, cos_wave, mode='same')

# Create subplots
plt.figure(figsize=(10, 8))

# Plotting Sine Wave (Input 1)
plt.subplot(3, 1, 1)  # 3 rows, 1 column, 1st subplot
plt.plot(t, sin_wave, label='Sine Wave')
plt.title('Sine Wave')
plt.ylabel('Amplitude')
plt.legend()

# Plotting Cosine Wave (Input 2)
plt.subplot(3, 1, 2)  # 3 rows, 1 column, 2nd subplot
plt.plot(t, cos_wave, label='Cosine Wave')
plt.title('Cosine Wave')
plt.ylabel('Amplitude')
plt.legend()

# Plotting Convolution Result (Output)
plt.subplot(3, 1, 3)  # 3 rows, 1 column, 3rd subplot
plt.plot(t, resultant_wave, label='Convolution Result', linestyle='--')
plt.title('Convolution Result')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend()

plt.tight_layout()
plt.show()
