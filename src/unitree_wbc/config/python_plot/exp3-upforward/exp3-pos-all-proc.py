import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import scienceplots

# 使用 scienceplots 风格
plt.style.use('science')

# 设置字体为 DejaVu Sans，并加大字号，加粗
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 14  # 可以调整为需要的字号
plt.rcParams['font.weight'] = 'bold'  # 设置字体加粗

# 读取数据文件
file_path = "exp3-pos-all.txt"  # 替换为你的实际文件路径
data = []

# 解析txt文件数据
with open(file_path, 'r') as file:
    for line in file:
        # 跳过注释行
        if line.startswith("#") or not line.strip():
            continue
        try:
            # 尝试解析数据行
            values = [float(x) for x in line.strip().split(",")]
            data.append(values)
        except ValueError:
            print(f"Skipping invalid line: {line.strip()}")  # 输出错误行方便调试

# 检查是否有有效数据
if len(data) == 0:
    raise ValueError("No valid data found in the file!")

# 转换为numpy数组，方便处理
data = np.array(data)

# 提取时间和力矩数据
num_motors = data.shape[1] // 2
time_data = data[:, 0::2]
torque_data = data[:, 1::2]

# 定义标签列表
labels = ['HIP-YAW', 'HIP-PITCH', 'HIP-ROLL', 'KNEE', 'ANKLE', 'SHOULDER-PITCH', 'SHOULDER-ROLL', 'SHOULDER-YAW', 'ELBOW']

# 定义插值后的时间轴
time_interp = np.linspace(time_data.min(), time_data.max(), 500)  # 更高密度的时间点

# 绘制力矩数据
plt.figure(figsize=(10, 6))
for motor_idx in range(num_motors):
    # 创建插值函数
    interp_func = interp1d(time_data[:, motor_idx], torque_data[:, motor_idx], kind='cubic')
    # 计算插值后的数据
    torque_interp = interp_func(time_interp)
    # 绘制插值后的曲线
    plt.plot(time_interp, torque_interp, label=labels[motor_idx])

# 添加图例、标题和标签
plt.title("Arm Motor Position vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (Rad)")
plt.legend()
plt.grid(True)

# 保存和展示
plt.savefig("exp3-pos-all.png", dpi=1000)
plt.show()
