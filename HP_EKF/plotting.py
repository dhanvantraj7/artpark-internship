import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file
data = pd.read_csv('C:\\Users\\Camras\\Documents\\VS code\\HP_EKF\\data\\Estimated_roll.csv')

# Plot graph
data.plot()
plt.show()
