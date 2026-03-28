import serial
import time
import csv
import matplotlib.pyplot as plt

# ================== Serial config ==================
serial_port = "COM6"   # change to your COM port
baud_rate = 115200
output_file = "C:/Users/12179/OneDrive/Desktop/GT Grad/CapStone Cassie/Data Collection/Force Testing For Relationship/Force_Testing_Back_Arrays_Relationship_120N.csv"

# ===== Only using back two tactile arrays now =====
num_tactile = 16      # 2 arrays × 8 sensors each
num_imu = 3
num_position = 1
num_audio = 1
total_channels = num_tactile + num_imu + num_position + num_audio  # 21

end_time = 40  # collection time in seconds

# ================== Data storage ==================
timestamps = []
tactile_data = [[] for _ in range(num_tactile)]
imu_data = [[] for _ in range(num_imu)]
position_data = []
audio_data = []

# ================== Baseline calibration ==================
BASELINE_COUNT = 30
tactile_baseline = [0.0] * num_tactile
baseline_counter = 0
baseline_collected = False


def fix_tactile_err(tactile_strings):
    """
    Fix ERR in tactile data for 16 sensors (2 groups of 8).
    Rules:
      - non-ERR: convert directly to float
      - ERR:
         * first in group (index % 8 == 0): use next value
         * last in group (index % 8 == 7): use previous value
         * otherwise: average previous and next
    """
    fixed = []

    if len(tactile_strings) < num_tactile:
        tactile_strings = tactile_strings + ["0"] * (num_tactile - len(tactile_strings))

    for i in range(num_tactile):
        val = tactile_strings[i]
        if val.upper() != "ERR":
            try:
                fixed.append(float(val))
            except ValueError:
                fixed.append(0.0)
        else:
            pos_in_group = i % 8

            if pos_in_group == 0:
                if i + 1 < num_tactile and tactile_strings[i + 1].upper() != "ERR":
                    replacement = float(tactile_strings[i + 1])
                else:
                    replacement = 0.0

            elif pos_in_group == 7:
                if i - 1 >= 0 and tactile_strings[i - 1].upper() != "ERR":
                    replacement = float(tactile_strings[i - 1])
                else:
                    replacement = 0.0

            else:
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
    try:
        return float(x)
    except Exception:
        return 0.0


# ================== Serial read and save ==================
try:
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser, open(output_file, "w", newline='') as file:
        csv_writer = csv.writer(file)

        header = (
            ["Time (s)"]
            + [f"Tactile{i + 1}" for i in range(num_tactile)]
            + ["IMU_X", "IMU_Y", "IMU_Z", "Position", "Auditory"]
        )
        csv_writer.writerow(header)

        print("Listening for data...")
        print(f"Collecting {BASELINE_COUNT} baseline samples, please keep footpad unloaded...")
        start_time = time.time()

        while time.time() - start_time < end_time:
            line_bytes = ser.readline()
            try:
                line = line_bytes.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if line:
                data = line.split(",")

                if len(data) == total_channels:
                    current_time = time.time() - start_time

                    tactile_raw = data[:num_tactile]
                    imu_raw = data[num_tactile:num_tactile + num_imu]
                    pos_raw = data[num_tactile + num_imu]
                    audio_raw = data[-1]

                    tactile_fixed = fix_tactile_err(tactile_raw)

                    # ----- baseline collection -----
                    if not baseline_collected:
                        for i in range(num_tactile):
                            tactile_baseline[i] += tactile_fixed[i]
                        baseline_counter += 1

                        if baseline_counter >= BASELINE_COUNT:
                            tactile_baseline = [v / baseline_counter for v in tactile_baseline]
                            baseline_collected = True
                            print("Baseline collected (per sensor):")
                            print(tactile_baseline)
                            print("Now starting normalized data logging...")

                        continue

                    # ----- normalize tactile -----
                    tactile_norm = [tactile_fixed[i] - tactile_baseline[i] for i in range(num_tactile)]

                    imu_fixed = [safe_float(x) for x in imu_raw]
                    pos_fixed = safe_float(pos_raw)
                    audio_fixed = safe_float(audio_raw)

                    numeric_data = tactile_norm + imu_fixed + [pos_fixed] + [audio_fixed]

                    csv_writer.writerow([current_time] + numeric_data)
                    print([current_time] + numeric_data)

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

# ================== Plotting ==================
if len(timestamps) > 0:
    bank_titles = ["Back Left", "Back Right"]

    # ---- tactile arrays: 2 groups of 8 ----
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    for b in range(2):
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

    axes[1].set_xlabel("Time (s)")
    fig.suptitle("Back Tactile Sensors per Array (Baseline-Normalized, 8 sensors each)", y=0.98)
    fig.tight_layout()
    plt.show()

    # ---- IMU ----
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

    # ---- Position ----
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, position_data, label="Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (raw units)")
    plt.title("Linear Position Sensor Output")
    plt.legend()
    plt.grid(True)
    plt.show()

    # ---- Audio ----
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, audio_data, label="Auditory")
    plt.xlabel("Time (s)")
    plt.ylabel("Level")
    plt.title("Auditory Signal")
    plt.legend()
    plt.grid(True)
    plt.show()

else:
    print("No valid data to plot.")