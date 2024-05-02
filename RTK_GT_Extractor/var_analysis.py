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
# def read_tum_with_variance_for_visualization(file_path):
#     timestamps = []
#     x_coords = []
#     y_coords = []
#     variances_x = []
#     variances_y = []
#     variances_z = []
#     with open(file_path, 'r') as file:
#         for line in file:
#             elements = line.strip().split()
#             timestamps.append(float(elements[0]))
#             x_coords.append(float(elements[1]))
#             y_coords.append(float(elements[2]))
#             variances_x.append(float(elements[8]))
#             variances_y.append(float(elements[9]))
#             variances_z.append(float(elements[10]))
#     return timestamps, x_coords, y_coords, variances_x, variances_y, variances_z


def read_tum_with_variance_for_visualization(file_path):
    timestamps = []
    x_coords = []
    y_coords = []
    z_coords = []
    qx = []
    qy = []
    qz = []
    qw = []
    variances_x = []
    variances_y = []
    variances_z = []
    with open(file_path, 'r') as file:
        for line in file:
            elements = line.strip().split()
            timestamps.append(float(elements[0]))
            x_coords.append(float(elements[1]))
            y_coords.append(float(elements[2]))
            z_coords.append(float(elements[3]))
            qx.append(float(elements[4]))
            qy.append(float(elements[5]))
            qz.append(float(elements[6]))
            qw.append(float(elements[7]))
            variances_x.append(float(elements[8]))
            variances_y.append(float(elements[9]))
            variances_z.append(float(elements[10]))
    return timestamps, x_coords, y_coords, z_coords, qx, qy, qz, qw, variances_x, variances_y, variances_z



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



def plot_data_with_legend_colorbar_adjusted_all(title, data, variances_x, variances_y, variances_z):
    timestamps, x_coords, y_coords, z_coords, qx, qy, qz, qw, _, _, _ = data
# def plot_data_with_legend_colorbar_adjusted_all(title, data):
#     timestamps, x_coords, y_coords, variances_x, variances_y, variances_z = data

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



# 提取数据
data_gnss1 = read_tum_with_variance_for_visualization("demo_results/output_gnss1_tum_with_variance.txt")
data_gnss2 = read_tum_with_variance_for_visualization("demo_results/output_gnss2_tum_with_variance.txt")
data_gnss3 = read_tum_with_variance_for_visualization("demo_results/output_gnss_sbg_tum_with_variance.txt")
data_gnss4 = read_tum_with_variance_for_visualization("demo_results/sample_ins_tum_with_variance.txt")
all_gnss_data = [
    ('GNSS1', data_gnss1, data_gnss1[8], data_gnss1[9], data_gnss1[10]),
    ('GNSS2', data_gnss2, data_gnss2[8], data_gnss2[9], data_gnss2[10]),
    ('SBG-GNSS', data_gnss3, data_gnss3[8], data_gnss3[9], data_gnss3[10]),
    ('INS', data_gnss4, data_gnss4[8], data_gnss4[9], data_gnss4[10])
]
for gnss_name, data, variances_x, variances_y, variances_z in all_gnss_data:
    plot_data_with_legend_colorbar_adjusted_all(gnss_name, data, variances_x, variances_y, variances_z)
    # thresholds_x = calculate_threshold_for_coverage(variances_x)
    # thresholds_y = calculate_threshold_for_coverage(variances_y)
    # thresholds_z = calculate_threshold_for_coverage(variances_z)
    # # 其他代码保持不变
    # print(f"Thresholds for {gnss_name}:")
    # print("X:", thresholds_x)
    # print("Y:", thresholds_y)
    # print("Z:", thresholds_z)
    # print("-" * 50)

