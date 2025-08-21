import serial
import time
import csv
import matplotlib.pyplot as plt

# Configure serial port
serial_port = "COM3"  # Replace with your serial port
baud_rate = 115200
output_file = "C:/Users/steve/Downloads/pressure_data_2.csv"

num_sensors = 8
end_time = 20

# Initialize data storage for real-time plotting
sensor_data = [[] for _ in range(num_sensors)]  # num of sensors, each with its own list
timestamps = []  # Store timestamps for plotting


try:
    # Open the serial port
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser, open(output_file, "w", newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(["Time (s)"] + [f"Sensor{i+1}" for i in range(num_sensors)])  # Write CSV header
        
        print("Listening for data...")
        start_time = time.time()

        while time.time() - start_time < end_time:  # Collect data for 20 seconds
            line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
            if line:  # If a non-empty line is received
                print(line)  # Print to console for debugging
                data = line.split(",")
                
                # Ensure we always write a full row of 8 columns
                if len(data) == num_sensors:
                    current_time = time.time() - start_time  # Time elapsed in seconds
                    csv_writer.writerow([current_time] + data)  # Write timestamp and data to CSV
                    
                    # Update real-time storage
                    timestamps.append(current_time)
                    for i in range(num_sensors):
                        try:
                            sensor_data[i].append(float(data[i]))
                        except ValueError:
                            sensor_data[i].append(None)  # Handle non-numeric data
                else:
                    print("Incomplete data row received. Skipping.")
        
        print(f"Data saved to {output_file}")

except serial.SerialException as e:
    print(f"Error: Could not open serial port {serial_port}. {e}")
except IOError as e:
    print(f"Error: Could not write to file {output_file}. {e}")

# Plot the data if any valid data exists
if any(len(sensor) > 0 for sensor in sensor_data):
    plt.figure(figsize=(10, 6))
    for i in range(num_sensors):
        sensor_data[i] = [x - sensor_data[i][0] for x in sensor_data[i]]
        plt.plot(timestamps, sensor_data[i], label=f"Sensor {i+1}")

    plt.xlabel("Time (s)")
    plt.ylabel("Pressure (Pa)")
    plt.title("Pressure Data from BMP585 Sensors")
    plt.legend()
    plt.grid(True)
    plt.show()
    
    bank_titles = ["Front Left", "Front Right", "Back Left", "Back Right"]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True, sharey=True)
    axes = axes.flatten()

    for idx, ax in enumerate(axes):
        start = idx * 8
        end   = start + 8
        for i in range(start, end):
            normalized = [x - sensor_data[i][0] for x in sensor_data[i]]
            ax.plot(timestamps, normalized, label=f"S{i+1}")
        ax.set_title(bank_titles[idx])
        ax.grid(True)
        if idx in (2, 3):
            ax.set_xlabel("Time (s)")
        if idx in (0, 2):
            ax.set_ylabel("Pressure (Pa)")
        ax.legend(ncol=2, fontsize="x-small")

    fig.suptitle("Pressure Data by Quadrant", y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
else:
    print("No valid data to plot.")


   