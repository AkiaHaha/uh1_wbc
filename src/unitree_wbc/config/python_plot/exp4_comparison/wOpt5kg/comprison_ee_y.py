import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import scienceplots

# scienceplots style
plt.style.use('science')

# Set font as DejaVu Sans
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 14  
plt.rcParams['font.weight'] = 'bold'  # Bold font

# Read data from txt file
file_path = "y.txt"  
data = []

# Analysis txt
with open(file_path, 'r') as file:
    for line in file:
        # Skip comments and empty lines
        if line.startswith("#") or not line.strip():
            continue
        try:
            # Try to parse the line as a list of floats
            values = [float(x) for x in line.strip().split(",")]
            data.append(values)
        except ValueError:
            print(f"Skipping invalid line: {line.strip()}")  # Print the line that caused the error

# Check if there is any valid data
if len(data) == 0:
    raise ValueError("No valid data found in the file!")

# Transform data to numpy array
data = np.array(data)

# Sort data by the first column
num_motors = data.shape[1] // 2
time_data = data[:, 0::2]
torque_data = data[:, 1::2]

# Define labels for each motor
labels = ['Err Y']

# Define interpolation function
time_interp = np.linspace(time_data.min(), time_data.max(), 500)  # Interpolate torque data

# Draw interpolated torque data
plt.figure(figsize=(10, 3))
for motor_idx in range(num_motors):
    interp_func = interp1d(time_data[:, motor_idx], torque_data[:, motor_idx], kind='cubic')
    torque_interp = interp_func(time_interp)
    plt.plot(time_interp, torque_interp, label=labels[motor_idx])

# Add title, labels, and legend
plt.title("Err Y")
plt.xlabel("Time (s)")
plt.ylabel("Err (Nm)")
plt.legend()
plt.grid(True)

# Save and show the plot
plt.savefig("exp4-y-wopt.png", dpi=1000)
plt.show()
