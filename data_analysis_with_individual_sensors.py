import serial
import time
import csv
import matplotlib.pyplot as plt

# ================== 串口配置 ==================
serial_port = "COM6"   # 修改为你的串口
baud_rate = 115200
output_file = "C:/Users/12179/OneDrive/Desktop/GT Grad/CapStone Cassie/Data Collection/Force Testing For Relationship/Force_Testing_Random_3_3_2026_1.csv"

num_tactile = 32
num_imu = 3
num_position = 1
num_audio = 1
total_channels = num_tactile + num_imu + num_position + num_audio  # 37

end_time = 10 # 采集时长（秒）

# ================== 数据存储 ==================
timestamps = []
tactile_data = [[] for _ in range(num_tactile)]
imu_data = [[] for _ in range(num_imu)]
position_data = []
audio_data = []

# ================== 基线（零力）标定参数 ==================
BASELINE_COUNT = 30  # 用多少帧来做零力基线平均
tactile_baseline = [0.0] * num_tactile
baseline_counter = 0
baseline_collected = False


def fix_tactile_err(tactile_strings):
    """
    修复 tactile 数据中的 ERR，输入为长度为 num_tactile 的字符串列表，输出为 float 列表
    规则：
      - 非 ERR：直接转 float
      - ERR：
         * 组内第1个（index % 8 == 0）：用后一位（如果后面也是 ERR 就 0）
         * 组内第8个（index % 8 == 7）：用前一位（如果前面也是 ERR 就 0）
         * 其他：前后平均（ERR 的邻居按 0 处理）
    """
    fixed = []

    # 如果长度不够，先补 0，避免越界
    if len(tactile_strings) < num_tactile:
        tactile_strings = tactile_strings + ["0"] * (num_tactile - len(tactile_strings))

    for i in range(num_tactile):
        val = tactile_strings[i]
        if val.upper() != "ERR":
            # 正常数值
            try:
                fixed.append(float(val))
            except ValueError:
                fixed.append(0.0)
        else:
            # 需要替换
            pos_in_group = i % 8
            if pos_in_group == 0:  # 组内第1个 → 用后一位
                if i + 1 < num_tactile and tactile_strings[i + 1].upper() != "ERR":
                    replacement = float(tactile_strings[i + 1])
                else:
                    replacement = 0.0
            elif pos_in_group == 7:  # 组内第8个 → 用前一位
                if i - 1 >= 0 and tactile_strings[i - 1].upper() != "ERR":
                    replacement = float(tactile_strings[i - 1])
                else:
                    replacement = 0.0
            else:  # 中间 → 前后平均
                left = 0.0
                right = 0.0

                if i - 1 >= 0 and tactile_strings[i - 1].upper() != "ERR":
                    left = float(tactile_strings[i - 1])
                if i + 1 < num_tactile and tactile_strings[i + 1].upper() != "ERR":
                    right = float(tactile_strings[i + 1])

                replacement = (left + right) / 2.0

            fixed.append(replacement)

    return fixed


def safe_float(x):
    """非 tactile 数据的安全转换"""
    try:
        return float(x)
    except Exception:
        return 0.0


# ================== 串口读取与存储 ==================
try:
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser, open(output_file, "w", newline='') as file:
        csv_writer = csv.writer(file)
        header = ["Time (s)"] + [f"Tactile{i + 1}" for i in range(num_tactile)] \
                 + ["IMU_X", "IMU_Y", "IMU_Z"] + ["Position"] + ["Auditory"]
        csv_writer.writerow(header)

        print("Listening for data...")
        print(f"Collecting {BASELINE_COUNT} baseline samples, please keep footpad unloaded...")
        start_time = time.time()

        while time.time() - start_time < end_time:
            # 注意 decode 时忽略非法字符，避免偶发串口乱码导致崩溃
            line_bytes = ser.readline()
            try:
                line = line_bytes.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if line:
                data = line.split(",")
                if len(data) == total_channels:
                    current_time = time.time() - start_time

                    # 拆分各部分
                    tactile_raw = data[:num_tactile]
                    imu_raw = data[num_tactile:num_tactile + num_imu]
                    pos_raw = data[num_tactile + num_imu]
                    audio_raw = data[-1]

                    # 修复 tactile 中的 ERR
                    tactile_fixed = fix_tactile_err(tactile_raw)

                    # ============ 先做基线标定（零力） ============
                    if not baseline_collected:
                        # 累加用于之后平均
                        for i in range(num_tactile):
                            tactile_baseline[i] += tactile_fixed[i]
                        baseline_counter += 1

                        if baseline_counter >= BASELINE_COUNT:
                            tactile_baseline = [v / baseline_counter for v in tactile_baseline]
                            baseline_collected = True
                            print("Baseline collected (per sensor):")
                            print(tactile_baseline)
                            print("Now starting normalized data logging...")

                        # 在基线阶段不记录数据到 CSV / 内存
                        continue

                    # ============ 用基线进行归一化 ============
                    tactile_norm = [tactile_fixed[i] - tactile_baseline[i] for i in range(num_tactile)]

                    # 其他部分
                    imu_fixed = [safe_float(x) for x in imu_raw]
                    pos_fixed = safe_float(pos_raw)
                    audio_fixed = safe_float(audio_raw)

                    numeric_data = tactile_norm + imu_fixed + [pos_fixed] + [audio_fixed]

                    # 写入 CSV
                    csv_writer.writerow([current_time] + numeric_data)

                    # 打印这一行（可选）
                    print([current_time] + numeric_data)

                    # 存储（用于画图）
                    timestamps.append(current_time)
                    for i in range(num_tactile):
                        tactile_data[i].append(tactile_norm[i])
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
    bank_titles = ["Front Left", "Front Right", "Back Left", "Back Right"]

    # ---- 触觉阵列，每组 8 个 ----
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes = axes.ravel()

    for b in range(4):
        ax = axes[b]
        start = b * 8
        end = start + 8
        for s in range(start, end):
            ax.plot(timestamps, tactile_data[s], label=f"Sensor {s - start + 1}")
        ax.set_title(bank_titles[b])
        ax.set_ylabel("Normalized Pressure (Pa)")
        ax.grid(True)
        ax.legend(ncol=4, fontsize=8, loc="upper left",
                  bbox_to_anchor=(0, 1.02), frameon=False)

    axes[2].set_xlabel("Time (s)")
    axes[3].set_xlabel("Time (s)")
    fig.suptitle("Tactile Sensors per Array (Baseline-Normalized, 8 sensors each)", y=0.98)
    fig.tight_layout()
    plt.show()

    # ---- IMU 三轴加速度曲线 ----
    plt.figure(figsize=(10, 6))
    labels = ["Acc_X", "Acc_Y", "Acc_Z"]
    for i in range(num_imu):
        plt.plot(timestamps, imu_data[i], label=labels[i])
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("IMU 3-Axis Acceleration")
    plt.legend()
    plt.grid(True)
    plt.show()

    # ---- Position 线性位移曲线（新增） ----
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, position_data, label="Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (raw units)")  # 如果你确定是 mm，可以改成 "Position (mm)"
    plt.title("Linear Position Sensor Output")
    plt.legend()
    plt.grid(True)
    plt.show()

    # ---- Auditory 曲线 ----
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, audio_data, label="Auditory")
    plt.xlabel("Time (s)")
    plt.ylabel("Level")
    plt.title("Auditory Signal (EMA of Peak)")
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print("No valid data to plot.")