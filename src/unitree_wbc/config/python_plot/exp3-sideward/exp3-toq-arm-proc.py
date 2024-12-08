import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import scienceplots
from matplotlib.lines import Line2D  # 导入 Line2D 用于调整图例线条

# scienceplots style
plt.style.use('science')

# Set font as DejaVu Sans
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 14  
plt.rcParams['font.weight'] = 'bold'  # Bold font

# Read data from txt file
file_path = "exp3-toq-arm.txt"  
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
labels = ['SP', 'SR', 'SY', 'E']

# Define interpolation function
time_interp = np.linspace(time_data.min(), time_data.max(), 500)  # Interpolate torque data

# Draw interpolated torque data
plt.figure(figsize=(10, 3))
for motor_idx in range(num_motors):
    interp_func = interp1d(time_data[:, motor_idx], torque_data[:, motor_idx], kind='cubic')
    torque_interp = interp_func(time_interp)
    plt.plot(time_interp, torque_interp, label=labels[motor_idx])

# 自定义图例的线条粗细和颜色
legend_handles = [
    Line2D([0], [0], color='blue', lw=2),  # 第一根线，颜色为蓝色，粗细为2
    Line2D([0], [0], color='green', lw=2),  # 第二根线，颜色为绿色，粗细为2
    Line2D([0], [0], color='orange', lw=2), # 第三根线，颜色为橙色，粗细为2
    Line2D([0], [0], color='red', lw=2),    # 第四根线，颜色为红色，粗细为2
]

# Add title, labels, and legend
plt.title("Arm Motor Torques")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend(handles=legend_handles, labels=labels, ncol=1, loc='best', fontsize=12)
plt.grid(True)

# Save and show the plot
plt.savefig("exp3-toq-arm.png", dpi=1000)
plt.show()
