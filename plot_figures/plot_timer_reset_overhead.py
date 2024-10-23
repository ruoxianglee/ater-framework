import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt
import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import statistics


# Function to read and process data from a file
def read_data(file_path)->list:
    with open(file_path, 'r') as file:
        lines = file.readlines()
        overheads_us = []
        for line in lines:
            match = re.search(r"Execution time:\s+(\d+) ns", line)
            if match:
                time_ns = int(match.group(1))
                overheads_us.append(time_ns/1000)
    return overheads_us

# Read and process data
overheads_us_ater_1 = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-7-latency.txt')
overheads_us_ater_2 = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-8-latency.txt')
overheads_us_ater_3 = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-9-latency.txt')
overheads_us = overheads_us_ater_1 + overheads_us_ater_2 + overheads_us_ater_3

plt.figure(figsize=(6, 3))

box = plt.boxplot(overheads_us, vert=False, patch_artist=True)

colors = ['lightblue']
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

# Customizing whiskers and medians
for whisker in box['whiskers']:
    whisker.set(color='black', linewidth=1.5)
for cap in box['caps']:
    cap.set(color='black', linewidth=1.5)
for median in box['medians']:
    median.set(color='red', linewidth=2)
for flier in box['fliers']:
    flier.set(marker='o', color='green', alpha=0.5)

# Removing y-axis labels
plt.yticks([])

# Scatter plot for individual data points
plt.scatter(overheads_us, [1] * len(overheads_us), color='grey', alpha=0.6)

# Labels and title
plt.xticks(fontsize=20)
plt.xlabel('Overheads ($\mu$s)', fontsize=25)
# plt.title('Box and Whisker Plot of Execution Times', fontsize=16)
plt.grid(True)

# Adjust layout to prevent cutoff
plt.tight_layout()
plt.savefig('timer_reset_overhead-ATER-7-8-9.pdf')
# Display the plot
plt.show()



