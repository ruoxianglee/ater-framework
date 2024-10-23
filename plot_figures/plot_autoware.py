import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def read_data(file_name):
    int_values = []
    with open(file_name, 'r') as file:
        # Read each line in the file
        for line in file:
            # Remove any leading/trailing whitespaces and convert the line to an integer
            value = float(line.strip())
            # Append the integer to the list
            int_values.append(value)
    return int_values

lidar_centerpoint = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/autoware/lidar_centerpoint_ms.txt')
ring_outlier_filter = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/autoware/ring_outlier_filter_ms.txt')
tensorrt_yolov = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/autoware/tensorrt_yolov_ms.txt')
traffic_light_classifier = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/autoware/traffic_light_CNN_classifier_ms.txt')
behivour_velocity_planner = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/autoware/behivour_velocity_planner_ms.txt')

# Ensure both lists are of the same length
min_length = min(len(lidar_centerpoint), len(ring_outlier_filter), len(tensorrt_yolov), len(traffic_light_classifier), len(behivour_velocity_planner))

lidar_centerpoint = lidar_centerpoint[:min_length]
ring_outlier_filter = ring_outlier_filter[:min_length]
tensorrt_yolov = tensorrt_yolov[:min_length]
traffic_light_classifier = traffic_light_classifier[:min_length]
behivour_velocity_planner = behivour_velocity_planner[:min_length]

# Sample data generation - replace this with your actual data loading
# Assuming data is a DataFrame with each column representing a group of results
data = pd.DataFrame({
    'Lidar Centerpoint': lidar_centerpoint,
    'Ring Outlier Filter': ring_outlier_filter,
    'TensorRT YOLO': tensorrt_yolov,
    'Traffic Light Classifier': traffic_light_classifier,
    'Behivour Velocity Planner': behivour_velocity_planner
})

# Time axis
time = np.arange(data.shape[0])

# Plotting
plt.figure(figsize=(12, 6))

for column in data.columns:
    plt.plot(time, data[column], label=column)

# Adding labels and title
plt.xlabel('Samples over Time', fontsize=25)
plt.ylabel('Task Execution Time (ms)', fontsize=25)
# plt.title('Dynamic Variability of Latency in Experimental Results')
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)
plt.legend(fontsize=18)
plt.grid(True)

plt.tight_layout()
plt.savefig('autoware_dynamic_execution_time.pdf')
plt.show()