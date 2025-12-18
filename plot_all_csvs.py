import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

# Set the folder path containing CSV files
folder_path = "csvs"  # Replace with your folder path

# Get all CSV files in the folder
csv_files = glob.glob(os.path.join(folder_path, "*.csv"))

# Read all CSV files and store both the DataFrame and filename
file_data = []
for file in csv_files:
    df = pd.read_csv(file)
    filename = os.path.basename(file).replace('.csv', '') # Get filename without extension
    file_data.append((df, filename))

# Create subplots
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.flatten()  # Flatten the 2x2 array to 1D for easier indexing

# Plot 1: df[0] vs df[5]
for df, filename in file_data:
    axes[0].plot(df.iloc[:, 0], df.iloc[:, 5], label=filename)
axes[0].set_title('Pendulum Angle vs Time')
axes[0].axhline(y=0.1, color='r', linestyle='--', label='0.1 rad Goal')
axes[0].axhline(y=-0.1, color='r', linestyle='--')
axes[0].locator_params(axis='x', nbins=20)
axes[0].set_ylabel('Pendulum Angle (rad)')
axes[0].set_xlabel('Time (s)')
axes[0].legend(loc='lower right')

# Plot 2: df[0] vs df[3]
for df, filename in file_data:
    axes[1].plot(df.iloc[:, 0], df.iloc[:, 3], label=filename)
axes[1].set_title('Flywheel Angle vs Time')
axes[1].axhline(y=0.1, color='r', linestyle='--', label='0.1 rad Goal')
axes[1].axhline(y=-0.1, color='r', linestyle='--')
axes[1].set_ylabel('Flywheel Angle (rad)')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylim([-1,1])
axes[1].legend(loc='lower left')

# Plot 3: df[0] vs df[7]
for df, filename in file_data:
    axes[2].plot(df.iloc[:, 0], df.iloc[:, 7], label=filename)
axes[2].set_title('Flywheel Force vs Time')
axes[2].set_ylabel('Force (N)')
axes[2].set_xlabel('Time (s)')
axes[2].set_ylim([-20, 20])
axes[2].legend(loc='lower right')

# Plot 4: df[0] vs df[2]
for df, filename in file_data:
    axes[3].plot(df.iloc[:, 0], df.iloc[:, 2], label=filename)
axes[3].set_title('Input Kosten tegen Time')
axes[3].set_ylabel('Input Cost')
axes[3].set_xlabel('Time (s)')
axes[3].legend()

# Adjust layout and display
plt.tight_layout()
plt.show()