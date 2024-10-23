import matplotlib.pyplot as plt

# Open the file in read mode
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

baseline = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/10_23/baseline-1-throughput.txt')
ater = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/10_23/ATER-1-throughput.txt')

# Epochs
epochs = list(range(1, len(baseline) + 1))

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(epochs, baseline, label='Baseline', marker='o', color=(167/255, 65/255, 60/255), linewidth=3)
plt.plot(epochs, ater, label='ATER', marker='s', color=(51/255, 76/255, 129/255), linewidth=3)

# Labels and Title
plt.xlabel('Sampling Periods', fontsize=25)
plt.ylabel('Throughput', fontsize=25)
# plt.title('Comparison of Drop Messages Between Baseline and ATER', fontsize=16)
plt.legend(fontsize=25)
plt.grid(True)

# Increase text size for x-ticks and y-ticks
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

# Setting 4: ATER-1
plt.axvline(x=2, color='r', linestyle='--', label='Increase', linewidth=2)
plt.axvline(x=3, color='r', linestyle='--', linewidth=2)
plt.axvline(x=12, color='r', linestyle='--', linewidth=2)
plt.axvline(x=22, color='r', linestyle='--', linewidth=2)
plt.axvline(x=29, color='r', linestyle='--', linewidth=2)
plt.axvline(x=30, color='r', linestyle='--', linewidth=2)

# Ensure the legend doesn't duplicate the label for the second 'Increasing' line
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys(), fontsize=22, loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4)
# Add this line before plt.show() to set y-axis starting at 90
plt.ylim(90, max(max(baseline), max(ater)) + 5)


# Display the plot
plt.tight_layout()
plt.savefig('throughput-ATER-1.pdf')
plt.show()
