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

# Read data from txt files
file_paths = ["xyz_wOpt_15kg.txt", "xyz_nOpt_15kg.txt"]  # Replace with the paths to your two files
data_list = []

# Function to read and process data from a file
def read_data(file_path):
    data = []
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
    return np.array(data)

# Read both data files
for file_path in file_paths:
    data_list.append(read_data(file_path))

# Check if there is any valid data
if len(data_list[0]) == 0 or len(data_list[1]) == 0:
    raise ValueError("No valid data found in one or both of the files!")

# Process data from both files
data1, data2 = data_list
num_motors = data1.shape[1] // 2
time_data1 = data1[:, 0::2]
torque_data1 = data1[:, 1::2]
time_data2 = data2[:, 0::2]
torque_data2 = data2[:, 1::2]

# Define labels for each motor
labels = ['ErrX', 'ErrY', 'ErrZ']

# Define interpolation function
time_interp = np.linspace(min(time_data1.min(), time_data2.min()), max(time_data1.max(), time_data2.max()), 1000)

# Draw interpolated torque data
plt.figure(figsize=(10, 5))

# Colors for the plots
colors = ['blue', 'green', 'orange']

# Plot data from the first file (with optimal)
for motor_idx in range(num_motors):
    interp_func1 = interp1d(time_data1[:, motor_idx], torque_data1[:, motor_idx], kind='cubic', fill_value="extrapolate")
    torque_interp1 = interp_func1(time_interp)
    plt.plot(time_interp, torque_interp1, label=f"{labels[motor_idx]} (w/)", linestyle='-', color=colors[motor_idx])

# Plot data from the second file (without optimal)
for motor_idx in range(num_motors):
    interp_func2 = interp1d(time_data2[:, motor_idx], torque_data2[:, motor_idx], kind='cubic', fill_value="extrapolate")
    torque_interp2 = interp_func2(time_interp)
    plt.plot(time_interp, torque_interp2, label=f"{labels[motor_idx]} (w/o)", linestyle='--', color=colors[motor_idx])

# Add title, labels, and legend
plt.title("Err XYZ Comparison (Payload=15kg)", pad=0)  # Move the title down a little
plt.xlabel("Time (s)")
plt.ylabel("Err (m)")

# Move legend below the title and set it to one row
plt.legend(bbox_to_anchor=(0.5, 0.7), loc='upper center', fontsize=12, ncol=3, handlelength=2.5)

# Adjust layout to avoid overlap between title, legend, and plot area
plt.subplots_adjust(top=0.75)  # Adjust the top margin to make room for both title and legend

# Add grid
plt.grid(True)

# Save and show the plot
plt.tight_layout()  # Adjust layout to fit the legend and title properly
plt.savefig("exp4-xyz-compare-15kg.png", dpi=1000)
plt.show()
