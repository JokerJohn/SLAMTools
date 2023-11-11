import rosbag
from geographiclib.geodesic import Geodesic
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable
import numpy as np
from matplotlib.font_manager import FontProperties


# Set global font to Times New Roman
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 10

# Read the provided TUM format files and extract required data for plotting
def read_tum_with_variance(file_path):
    timestamps = []
    variances_x = []
    variances_y = []
    variances_z = []
    with open(file_path, 'r') as file:
        for line in file:
            elements = line.strip().split()
            timestamps.append(float(elements[0]))
            variances_x.append(float(elements[8]))
            variances_y.append(float(elements[9]))
            variances_z.append(float(elements[10]))
    return timestamps, variances_x, variances_y, variances_z

# Function to read TUM format file with variance and return required data for plotting
def read_tum_with_variance_for_visualization(file_path):
    timestamps = []
    x_coords = []
    y_coords = []
    variances_x = []
    variances_y = []
    variances_z = []
    with open(file_path, 'r') as file:
        for line in file:
            elements = line.strip().split()
            timestamps.append(float(elements[0]))
            x_coords.append(float(elements[1]))
            y_coords.append(float(elements[2]))
            variances_x.append(float(elements[8]))
            variances_y.append(float(elements[9]))
            variances_z.append(float(elements[10]))
    return timestamps, x_coords, y_coords, variances_x, variances_y, variances_z



def calculate_threshold_for_coverage(variances, coverage=0.8):
    """
    Calculate the variance threshold that covers a certain percentage of data points.
    """
    sorted_variances = sorted(variances)
    index = int(coverage * len(sorted_variances))
    return sorted_variances[index]

# Plotting the data
def plot_data(title, data):
    timestamps, x_coords, y_coords, variances_x, variances_y, variances_z = data
    fig, axs = plt.subplots(2, 1, figsize=(10, 12), sharex=True)
    # XY trajectory plot
    axs[0].plot(x_coords, y_coords, label='XY Trajectory', color='purple')
    axs[0].set_title(f'XY Trajectory for {title}')
    axs[0].set_ylabel('Y Coordinate')
    axs[0].legend()
    axs[0].grid(True)
    # Accuacy over time plot
    axs[1].plot(timestamps, variances_x, label='X Accuacy', color='red')
    axs[1].plot(timestamps, variances_y, label='Y Accuacy', color='green')
    axs[1].plot(timestamps, variances_z, label='Z Accuacy', color='blue')
    axs[1].set_title(f'Accuacy over time for {title}')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Accuacy')
    axs[1].legend()
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f"{title}_visualization.pdf")
    plt.show()

def plot_data_updated(title, data):
    timestamps, x_coords, y_coords, variances_x, variances_y, variances_z = data
    fig, axs = plt.subplots(2, 1, figsize=(10, 12), gridspec_kw={'height_ratios': [1, 2]})
    # XY trajectory plot
    axs[0].plot(x_coords, y_coords, label='XY Trajectory', color='purple')
    axs[0].set_title(f'XY Trajectory for {title}')
    axs[0].set_xlabel('X Coordinate')
    axs[0].set_ylabel('Y Coordinate')
    axs[0].legend()
    axs[0].grid(True)
    # Accuacy over time plot
    axs[1].plot(timestamps, variances_x, label='X Accuacy', color='red')
    axs[1].plot(timestamps, variances_y, label='Y Accuacy', color='green')
    axs[1].plot(timestamps, variances_z, label='Z Accuacy', color='blue')
    axs[1].set_title(f'Accuacy over time for {title}')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Accuacy')
    axs[1].legend()
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f"{title}_visualization_updated.pdf")
    plt.show()


def plot_data_with_legend_colorbar_adjusted(title, data):
    timestamps, x_coords, y_coords, variances_x, variances_y, variances_z = data
    # Get the average variance for coloring
    average_variances = [(vx + vy) / 2 for vx, vy in zip(variances_x, variances_y)]
    fig, axs = plt.subplots(2, 1, figsize=(14, 14), gridspec_kw={'height_ratios': [1, 2]})
    # XY trajectory plot with color based on variance
    sc = axs[0].scatter(x_coords, y_coords, c=average_variances, cmap='jet', s=10)
    # Add colorbar
    divider = make_axes_locatable(axs[0])
    cax = divider.append_axes("right", size="5%", pad=0.2)
    fig.colorbar(sc, cax=cax, label='Average Accuacy (X & Y)', orientation='vertical')
    axs[0].set_title(f'XY Trajectory for {title}')
    axs[0].set_xlabel('X [m]')
    axs[0].set_ylabel('Y [m]')
    axs[0].grid(True)
    # Accuacy over time plot
    axs[1].plot(timestamps, variances_x, label='X Accuacy', color='red')
    axs[1].plot(timestamps, variances_y, label='Y Accuacy', color='green')
    axs[1].plot(timestamps, variances_z, label='Z Accuacy', color='blue')
    axs[1].set_title(f'Accuacy over time for {title}')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Accuacy')
    axs[1].legend()
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f"{title}_visualization_with_legend_colorbar_adjusted.pdf")
    plt.show()


def plot_data_with_legend_colorbar_adjusted_all(title, data):
    timestamps, x_coords, y_coords, variances_x, variances_y, variances_z = data
    # Get the average variance for coloring
    average_variances = [(vx + vy) / 2 for vx, vy in zip(variances_x, variances_y)]
    fig, axs = plt.subplots(2, 1, figsize=(14, 14), gridspec_kw={'height_ratios': [1, 2]})
    # XY trajectory plot with color based on variance
    sc = axs[0].scatter(x_coords, y_coords, c=average_variances, cmap='jet', s=10)
    # Add colorbar
    divider = make_axes_locatable(axs[0])
    cax = divider.append_axes("right", size="5%", pad=0.2)
    fig.colorbar(sc, cax=cax, label='Average Accuacy (X & Y)', orientation='vertical')
    axs[0].set_title(f'XY Trajectory for {title}')
    axs[0].set_xlabel('X [m]')
    axs[0].set_ylabel('Y [m]')
    axs[0].grid(True)
    # Accuacy over time plot
    axs[1].plot(timestamps, variances_x, label='X Accuacy', color='red')
    axs[1].plot(timestamps, variances_y, label='Y Accuacy', color='green')
    axs[1].plot(timestamps, variances_z, label='Z Accuacy', color='blue')
    axs[1].set_title(f'Accuacy over time for {title}')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('Accuacy')
    axs[1].legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True)
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f"{title.replace('/', '_')}_visualization_with_legend_colorbar_adjusted.pdf")
    plt.show()




# Extracting data from the provided files
data_gnss1 = read_tum_with_variance_for_visualization("output_gnss1_tum_with_variance.txt")
data_gnss2 = read_tum_with_variance_for_visualization("output_gnss2_tum_with_variance.txt")
data_gnss3 = read_tum_with_variance_for_visualization("output_gnss_sbg_tum_with_variance.txt")


# # Calculate the thresholds for GNSS1
# thresholds_gnss1_x = calculate_threshold_for_coverage(data_gnss1[3])
# thresholds_gnss1_y = calculate_threshold_for_coverage(data_gnss1[4])
# thresholds_gnss1_z = calculate_threshold_for_coverage(data_gnss1[5])
# # Calculate the thresholds for GNSS2
# thresholds_gnss2_x = calculate_threshold_for_coverage(data_gnss2[3])
# thresholds_gnss2_y = calculate_threshold_for_coverage(data_gnss2[4])
# thresholds_gnss2_z = calculate_threshold_for_coverage(data_gnss2[5])
# print("Thresholds for GNSS1:")
# print("X:", thresholds_gnss1_x)
# print("Y:", thresholds_gnss1_y)
# print("Z:", thresholds_gnss1_z)
# print("\nThresholds for GNSS2:")
# print("X:", thresholds_gnss2_x)
# print("Y:", thresholds_gnss2_y)
# print("Z:", thresholds_gnss2_z)


# Visualize the data for both GNSS topics using the updated function
# plot_data_updated('GNSS1', data_gnss1)
# plot_data_updated('GNSS2', data_gnss2)

# Visualize the data for both GNSS topics using the function with adjusted colorbar
# plot_data_with_legend_colorbar_adjusted('GNSS1', data_gnss1)
# plot_data_with_legend_colorbar_adjusted('GNSS2', data_gnss2)


font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(10)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams['axes.labelweight'] = 'normal'
plt.rcParams['font.size'] = 10
plt.rcParams['xtick.labelsize'] = 10
plt.rcParams['ytick.labelsize'] = 10
plt.rcParams['legend.fontsize'] = 10


# Prepare a list of data for multiple GNSS sources (for this example, I just replicate the same data)
all_gnss_data = [('GNSS1', data_gnss1), ('GNSS2', data_gnss2), ('SBG-GNSS', data_gnss3)]
# For each GNSS source, visualize the data and calculate the thresholds
for gnss_name, data in all_gnss_data:
    plot_data_with_legend_colorbar_adjusted_all(gnss_name, data)
    thresholds_x = calculate_threshold_for_coverage(data[3])
    thresholds_y = calculate_threshold_for_coverage(data[4])
    thresholds_z = calculate_threshold_for_coverage(data[5])
    print(f"Thresholds for {gnss_name}:")
    print("X:", thresholds_x)
    print("Y:", thresholds_y)
    print("Z:", thresholds_z)
    print("-" * 50)
