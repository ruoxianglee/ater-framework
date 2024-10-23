import matplotlib.pyplot as plt

# Open the file in read mode
def read_data(file_name):
    int_values = []
    with open(file_name, 'r') as file:
        # Read each line in the file
        for line in file:
            # Remove any leading/trailing whitespaces and convert the line to an integer
            value = int(line.strip())
            # Append the integer to the list
            int_values.append(value)
    return int_values

baseline = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/baseline-3-drops.txt')
ater = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-9-drops.txt')

# Epochs
epochs = list(range(1, len(baseline) + 1))

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(epochs, baseline, label='Baseline', marker='o', color=(167/255, 65/255, 60/255), linewidth=3)
plt.plot(epochs, ater, label='ATER', marker='s', color=(51/255, 76/255, 129/255), linewidth=3)

# Labels and Title
plt.xlabel('Sampling Periods', fontsize=25)
plt.ylabel('Number of Message Drops', fontsize=25)
# plt.title('Comparison of Drop Messages Between Baseline and ATER', fontsize=16)
plt.legend(fontsize=25)
plt.grid(True)

# Increase text size for x-ticks and y-ticks
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

# Add vertical lines
######### Date: 6_29
# setting 1 ATER-7
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=28, color='r', linestyle='--', label='Increase', linewidth=2)

# setting 1 ATER-10
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=2, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=7, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=23, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=27, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=29, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=30, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=21, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=28, color='r', linestyle='--', linewidth=2)

# setting 3 ATER-11
# plt.axvline(x=5, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=13, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=27, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=28, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=2, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=4, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=12, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=17, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=24, color='r', linestyle='--', linewidth=2)

# setting 2 ATER-9
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=4, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=5, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=12, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=13, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=19, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=29, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=10, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=11, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=16, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=28, color='r', linestyle='--', linewidth=2)

###### Date: 6_30
# Setting 1: ATER-4
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=4, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=5, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=7, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=8, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=12, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=18, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=21, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=23, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=27, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=6, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=11, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=15, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=24, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=29, color='r', linestyle='--', linewidth=2)

# Setting 2: ATER-5
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=5, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=12, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=13, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=27, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=8, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=18, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=24, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=30, color='r', linestyle='--', linewidth=2)

# Setting 3: ATER-6
# plt.axvline(x=5, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=12, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=13, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=27, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=2, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=4, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=8, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=10, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=16, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=24, color='r', linestyle='--', linewidth=2)

# Setting 1: ATER-7
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=7, color='r', linestyle='--', label='Increase', linewidth=2)

# Setting 2: ATER-8
# plt.axvline(x=1, color='g', linestyle='--', label='Decrease', linewidth=2)
# plt.axvline(x=5, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=13, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=14, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=28, color='g', linestyle='--', linewidth=2)
# plt.axvline(x=11, color='r', linestyle='--', label='Increase', linewidth=2)
# plt.axvline(x=17, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=23, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=24, color='r', linestyle='--', linewidth=2)
# plt.axvline(x=25, color='r', linestyle='--', linewidth=2)

# Setting 3: ATER-9
plt.axvline(x=5, color='g', linestyle='--', label='Decrease', linewidth=2)
plt.axvline(x=6, color='g', linestyle='--', linewidth=2)
plt.axvline(x=12, color='g', linestyle='--', linewidth=2)
plt.axvline(x=20, color='g', linestyle='--', linewidth=2)
plt.axvline(x=23, color='g', linestyle='--', linewidth=2)
plt.axvline(x=28, color='g', linestyle='--', linewidth=2)
plt.axvline(x=2, color='r', linestyle='--', label='Increase', linewidth=2)
plt.axvline(x=3, color='r', linestyle='--', linewidth=2)
plt.axvline(x=9, color='r', linestyle='--', linewidth=2)
plt.axvline(x=18, color='r', linestyle='--', linewidth=2)
plt.axvline(x=22, color='r', linestyle='--', linewidth=2)
plt.axvline(x=24, color='r', linestyle='--', linewidth=2)

# Ensure the legend doesn't duplicate the label for the second 'Increasing' line
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys(), fontsize=22, loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4)

# Display the plot
plt.tight_layout()
plt.savefig('setting_3_msg_drops-ATER-9.pdf')
plt.show()
