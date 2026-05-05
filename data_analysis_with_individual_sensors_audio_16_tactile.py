import serial
import csv
import time
import struct

# ================== Config ==================
serial_port = "COM6"
baud_rate = 2000000

record_time = 12  # seconds

audio_csv = "C:/Users/12179/OneDrive/Desktop/GT Grad/CapStone Cassie/Data Collection/Audio01.csv"
sensor_csv = "C:/Users/12179/OneDrive/Desktop/GT Grad/CapStone Cassie/Data Collection/Sensor01.csv"

# ================== Frame protocol ==================
FRAME_H1 = 0xA5
FRAME_H2 = 0x5A
TYPE_AUDIO = 0x01
TYPE_SENSOR = 0x02

AUDIO_SAMPLE_RATE = 44100


def read_exact(ser, n):
    data = bytearray()

    while len(data) < n:
        chunk = ser.read(n - len(data))

        if not chunk:
            return None

        data.extend(chunk)

    return bytes(data)


try:
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser, \
            open(audio_csv, "w", newline="", buffering=1024 * 1024) as f_audio, \
            open(sensor_csv, "w", newline="", buffering=1024 * 1024) as f_sensor:

        audio_writer = csv.writer(f_audio)
        sensor_writer = csv.writer(f_sensor)

        audio_writer.writerow(["audio_time_s", "audio_sample"])

        sensor_writer.writerow(
            ["time_s"]
            + [f"Tactile{i + 1}" for i in range(16)]
            + ["IMU_X", "IMU_Y", "IMU_Z", "Position", "Audio_Peak"]
        )

        print("Recording started...")
        print("Audio CSV:", audio_csv)
        print("Sensor CSV:", sensor_csv)

        ser.reset_input_buffer()

        start_time = time.time()
        sensor_start_ms = None

        audio_sample_index = 0
        audio_frames = 0
        sensor_frames = 0
        bad_frames = 0

        while time.time() - start_time < record_time:

            # ========== Find frame header ==========
            b1 = ser.read(1)
            if not b1:
                continue

            if b1[0] != FRAME_H1:
                continue

            b2 = ser.read(1)
            if not b2 or b2[0] != FRAME_H2:
                continue

            # ========== Read type + length ==========
            header = read_exact(ser, 3)
            if header is None:
                bad_frames += 1
                continue

            frame_type = header[0]
            payload_len = header[1] | (header[2] << 8)

            payload = read_exact(ser, payload_len)
            if payload is None:
                bad_frames += 1
                continue

            # ========== AUDIO FRAME ==========
            if frame_type == TYPE_AUDIO:
                sample_count = payload_len // 2

                try:
                    samples = struct.unpack("<" + "h" * sample_count, payload)
                except struct.error:
                    bad_frames += 1
                    continue

                rows = []

                for sample in samples:
                    audio_time = audio_sample_index / AUDIO_SAMPLE_RATE
                    rows.append([audio_time, sample])
                    audio_sample_index += 1

                audio_writer.writerows(rows)
                audio_frames += 1

            # ========== SENSOR FRAME ==========
            elif frame_type == TYPE_SENSOR:
                try:
                    line = payload.decode("utf-8", errors="ignore").strip()
                except Exception:
                    bad_frames += 1
                    continue

                if not line:
                    continue

                parts = line.split(",")

                # Skip controller header row if received
                if parts[0] == "time_ms":
                    continue

                # Expected controller columns:
                # 0  time_ms
                # 1-16 tactile sensors
                # 17 acc_x
                # 18 acc_y
                # 19 acc_z
                # 20 distance
                # 21 audio_peak
                # 22 audio_rms
                # 23 sensor_packet_hz
                # 24 estimated_audio_sample_rate

                if len(parts) < 22:
                    bad_frames += 1
                    continue

                try:
                    teensy_time_ms = float(parts[0])
                except ValueError:
                    bad_frames += 1
                    continue

                if sensor_start_ms is None:
                    sensor_start_ms = teensy_time_ms

                time_s = (teensy_time_ms - sensor_start_ms) / 1000.0

                selected_row = (
                    [time_s]
                    + parts[1:17]      # 16 tactile sensors
                    + parts[17:20]     # IMU X, Y, Z
                    + [parts[20]]      # position distance
                    + [parts[21]]      # audio peak
                )

                sensor_writer.writerow(selected_row)
                sensor_frames += 1

            else:
                bad_frames += 1

            # ========== Status print ==========
            if sensor_frames > 0 and sensor_frames % 100 == 0:
                elapsed = time.time() - start_time
                audio_duration = audio_sample_index / AUDIO_SAMPLE_RATE

                print(
                    f"Elapsed: {elapsed:.1f}s | "
                    f"Audio samples: {audio_sample_index} | "
                    f"Audio duration: {audio_duration:.2f}s | "
                    f"Sensor frames: {sensor_frames} | "
                    f"Sensor rate: {sensor_frames / elapsed:.2f} Hz | "
                    f"Bad frames: {bad_frames}"
                )

        print("Recording finished.")
        print(f"Audio samples saved: {audio_sample_index}")
        print(f"Audio duration saved: {audio_sample_index / AUDIO_SAMPLE_RATE:.2f} s")
        print(f"Audio frames received: {audio_frames}")
        print(f"Sensor frames received: {sensor_frames}")
        print(f"Bad frames: {bad_frames}")

except serial.SerialException as e:
    print(f"Serial error: {e}")

except IOError as e:
    print(f"File error: {e}")