import matplotlib.pyplot as plt
import numpy as np

# Define the boundaries and lines
B = 1
x = np.linspace(0, 4*B, 400)
y1 = x - B
y2 = 0.5 * x

plt.figure(figsize=(6, 6))

# Create the plot
fig, ax = plt.subplots()

# Fill the regions correctly
ax.fill_between(x, y1, y2, where=(y2 <= y1) & (x >= B), facecolor='blue', alpha=0.5)
ax.fill_between(x, 0, y1, where=(x >= B) & (x <= 2*B), facecolor='green', interpolate=True, alpha=0.5)
ax.fill_between(x, 0, y2, where=(x >= 2*B) & (x <= 4*B), facecolor='green', interpolate=True, alpha=0.5)
# Plot the lines
ax.plot(x, y1, label=r'$N_{i,j}^d = (N_{i,j}^P + N_{i,j-1}^b) - B_i$', color='red')
ax.plot(x, y2, label=r'$N_{i,j}^d = \theta_i (N_{i,j}^P + N_{i,j-1}^b)$, $\theta_i=0.5$', color='blue')


# Plot points P1, P2, P3
P1 = (3.5*B, 2*B)
P2 = (3.5*B, 1.0*B)
# P3 = (3.5*B, 1.0*B)

ax.plot(*P1, 'ko')
ax.plot(*P2, 'ko')
# ax.plot(*P3, 'ko')

ax.text(P1[0], P1[1], 'P1', fontsize=16, verticalalignment='bottom', horizontalalignment='left')
ax.text(P2[0], P2[1], 'P2', fontsize=16, verticalalignment='bottom', horizontalalignment='left')
# ax.text(P3[0], P3[1], 'P3', fontsize=14, verticalalignment='bottom', horizontalalignment='left')

# Set axis limits
ax.set_xlim(0, 4*B)
ax.set_ylim(0, 4*B)

# Remove axis ticks and labels for specific values
ax.set_xticks([B, 2*B, 3*B, 4*B])
ax.set_yticks([B, 2*B, 3*B, 4*B])
ax.set_xticklabels([r'$B_i$', r'$2B_i$', r'$3B_i$', r'$4B_i$'], fontsize=20)
ax.set_yticklabels([r'$B_i$', r'$2B_i$', r'$3B_i$', r'$4B_i$'], fontsize=20)

ax.set_xlabel(r'$N_{i,j}^P + N_{i,j-1}^b$', fontsize=20)
ax.set_ylabel(r'$N_{i,j}^d$', fontsize=20)

# Save the figure as a PDF file
# plt.subplots_adjust(bottom=0.2,left=0.3)
# plt.savefig('figure.pdf', bbox_inches='tight')

# Display the plot
plt.legend(fontsize=16)
plt.grid(True)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.tight_layout()
plt.savefig('imdi_indicator.pdf')
plt.show()
