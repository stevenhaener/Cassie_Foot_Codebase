import serial
import time
import csv
import matplotlib.pyplot as plt

# 串口配置
serial_port = "COM6"   # 修改为你的串口
baud_rate = 115200
output_file = "C:/Users/17986/Desktop/summer/data/analysis/1.csv"

num_tactile = 32
num_imu = 3
num_position = 1
num_audio = 1
total_channels = num_tactile + num_imu + num_position + num_audio  # 37

end_time = 15  # 采集时长（秒）

# 数据存储
timestamps = []
tactile_data = [[] for _ in range(num_tactile)]
imu_data = [[] for _ in range(num_imu)]
position_data = []
audio_data = []

def fix_tactile_err(data):
    """修复 tactile 数据中的 ERR，返回 float 列表"""
    fixed = []
    for i in range(num_tactile):
        val = data[i]
        if val.upper() != "ERR":
            fixed.append(float(val))
        else:
            pos_in_group = i % 8
            if pos_in_group == 0:  # 组内第1个 → 用后一位
                replacement = float(data[i+1]) if data[i+1].upper() != "ERR" else 0.0
            elif pos_in_group == 7:  # 组内第8个 → 用前一位
                replacement = float(data[i-1]) if data[i-1].upper() != "ERR" else 0.0
            else:  # 中间 → 前后平均
                left  = float(data[i-1]) if data[i-1].upper() != "ERR" else 0.0
                right = float(data[i+1]) if data[i+1].upper() != "ERR" else 0.0
                replacement = (left + right) / 2.0
            fixed.append(replacement)
    return fixed

def safe_float(x):
    """非 tactile 数据的安全转换"""
    try:
        return float(x)
    except:
        return 0.0

try:
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser, open(output_file, "w", newline='') as file:
        csv_writer = csv.writer(file)
        header = ["Time (s)"] + [f"Tactile{i+1}" for i in range(num_tactile)] \
                 + ["IMU_X", "IMU_Y", "IMU_Z"] + ["Position"] + ["Auditory"]
        csv_writer.writerow(header)

        print("Listening for data...")
        start_time = time.time()

        while time.time() - start_time < end_time:
            line = ser.readline().decode('utf-8').strip()
            if line:
                data = line.split(",")
                if len(data) == total_channels:
                    current_time = time.time() - start_time

                    # 修复 tactile
                    tactile_fixed = fix_tactile_err(data)

                    # 其他部分
                    imu_fixed = [safe_float(x) for x in data[num_tactile:num_tactile+num_imu]]
                    pos_fixed = safe_float(data[num_tactile+num_imu])
                    audio_fixed = safe_float(data[-1])

                    numeric_data = tactile_fixed + imu_fixed + [pos_fixed] + [audio_fixed]

                    # 写入 CSV
                    csv_writer.writerow([current_time] + numeric_data)

                    # 在控制台打印这一行
                    print([current_time] + numeric_data)

                    # 存储
                    timestamps.append(current_time)
                    for i in range(num_tactile):
                        tactile_data[i].append(tactile_fixed[i])
                    for i in range(num_imu):
                        imu_data[i].append(imu_fixed[i])
                    position_data.append(pos_fixed)
                    audio_data.append(audio_fixed)
                else:
                    print(f"Incomplete row ({len(data)} values): {line}")

        print(f"Data saved to {output_file}")

except serial.SerialException as e:
    print(f"Error: Could not open serial port {serial_port}. {e}")
except IOError as e:
    print(f"Error: Could not write to file {output_file}. {e}")

# ================== 绘图 ==================

if len(timestamps) > 0:
    # 1. 四块 tactile array 平均值曲线
    plt.figure(figsize=(10,6))
    bank_titles = ["Front Left", "Front Right", "Back Left", "Back Right"]
    for b in range(4):
        start = b * 8
        end = start + 8
        bank_avg = [sum(vals)/len(vals) for vals in zip(*tactile_data[start:end])]
        plt.plot(timestamps, bank_avg, label=bank_titles[b])
    plt.xlabel("Time (s)")
    plt.ylabel("Pressure (Pa)")
    plt.title("Tactile Array Average (8 units each)")
    plt.legend()
    plt.grid(True)
    plt.show()

    # 2. IMU 三轴加速度曲线
    plt.figure(figsize=(10,6))
    labels = ["Acc_X", "Acc_Y", "Acc_Z"]
    for i in range(num_imu):
        plt.plot(timestamps, imu_data[i], label=labels[i])
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("IMU 3-Axis Acceleration")
    plt.legend()
    plt.grid(True)
    plt.show()

    # 3. Auditory 曲线
    plt.figure(figsize=(10,6))
    plt.plot(timestamps, audio_data, label="Auditory", color="purple")
    plt.xlabel("Time (s)")
    plt.ylabel("Level")
    plt.title("Auditory Signal")
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print("No valid data to plot.")
