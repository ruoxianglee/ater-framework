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

baseline = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/baseline-1-cputime.txt')
ater = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-7-cputime.txt')
saved_cputime_setting_1 = [a-b for a,b in zip(baseline,ater)]

baseline = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/baseline-2-cputime.txt')
ater = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-8-cputime.txt')
saved_cputime_setting_2 = [a-b for a,b in zip(baseline,ater)]

baseline = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/baseline-3-cputime.txt')
ater = read_data('/Users/chunting/Documents/UpUp/Fuzzing/fuzzer/results/6_30/ATER-9-cputime.txt')
saved_cputime_setting_3 = [a-b for a,b in zip(baseline,ater)]

# Epochs
epochs = list(range(1, len(baseline) + 1))

# Plotting
plt.figure(figsize=(12, 6))
plt.plot(epochs, saved_cputime_setting_1, label='Setting 1', marker='o', color=(167/255, 65/255, 60/255), linewidth=3)
plt.plot(epochs, saved_cputime_setting_2, label='Setting 2', marker='s', color=(51/255, 76/255, 129/255), linewidth=3)
plt.plot(epochs, saved_cputime_setting_3, label='Setting 3', marker='v', color=(242/255, 153/255, 65/255), linewidth=3)

# Labels and Title
plt.xlabel('Sampling Periods', fontsize=25)
plt.ylabel('CPU Time Saving (ms)', fontsize=25)
# plt.title('Comparison of Drop Messages Between Baseline and ATER', fontsize=16)
plt.legend(fontsize=25)
plt.grid(True)

# Increase text size for x-ticks and y-ticks
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

# Ensure the legend doesn't duplicate the label for the second 'Increasing' line
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys(), fontsize=22, loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=4)

# Display the plot
plt.tight_layout()
plt.savefig('cputime-ATER-7-8-9.pdf')
plt.show()
