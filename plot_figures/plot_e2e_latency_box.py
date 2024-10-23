import matplotlib.pyplot as plt
import os
import re
import pandas as pd
import seaborn as sns

# Function to read and process data from a file
def read_data(file_path)->list:
    with open(file_path, 'r') as file:
        lines = file.readlines()
        latencies = []
        for line in lines:
            match = re.search(r"hot path latency:\s+([\d.]+)ms", line)
            if match:
                hot_path_latency = float(match.group(1))
                latencies.append(hot_path_latency)
    return latencies

# Read and process data
baseline_latencies = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/10_23/baseline-1-latency.txt')
ater_latencies = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/10_23/ATER-1-latency.txt')

# First 100 samples should be removed, they are got before regulation
ater_latencies = ater_latencies[100:]

# Ensure both lists are of the same length
min_length = min(len(baseline_latencies), len(ater_latencies))
baseline_latencies = baseline_latencies[:min_length]
ater_latencies = ater_latencies[:min_length]

# Combine data into a DataFrame for plotting
data = pd.DataFrame({
    'Baseline': baseline_latencies,
    'ATER': ater_latencies
})

# Plotting the box plot
plt.figure(figsize=(6, 6))
sns.boxplot(data=data)

# Adding labels and title
plt.ylabel('End-to-End Latencies (ms)', fontsize=25)
# plt.xlabel('Experiment Groups', fontsize=25)
# plt.title('Comparison of End-to-End Latencies', fontsize=16)
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

# Save plot
plt.tight_layout()
plt.savefig('setting_4_latency-ATER-1.pdf')

plt.show()
