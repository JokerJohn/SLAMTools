import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.font_manager import FontProperties
import os

def load_tum_format_trajectory(file_path):
    data = np.loadtxt(file_path, delimiter=' ')
    trajectory = {
        'timestamp': data[:, 0],
        'tx': data[:, 1],
        'ty': data[:, 2],
        'tz': data[:, 3],
        'qx': data[:, 4],
        'qy': data[:, 5],
        'qz': data[:, 6],
        'qw': data[:, 7],
    }
    return trajectory

def quaternion_to_euler(qx, qy, qz, qw):
    rotation = Rotation.from_quat([qx, qy, qz, qw])
    euler = rotation.as_euler('xyz', degrees=True)
    return euler



def extract_euler_from_trajectory(data):
    euler_angles = np.array([quaternion_to_euler(qx, qy, qz, qw) for qx, qy, qz, qw in zip(data['qx'], data['qy'], data['qz'], data['qw'])])
    return {
        'timestamp': data['timestamp'],
        'roll': euler_angles[:, 0],
        'pitch': euler_angles[:, 1],
        'yaw': euler_angles[:, 2]
    }



def load_multiple_tum_trajectories(*file_paths):
    return [load_tum_format_trajectory(fp) for fp in file_paths]

def visualize_multiple_euler_over_time(datasets, labels, font):
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    linewidth = 1.5
    for i, data in enumerate(datasets):
        axs[0].plot(data['timestamp'], data['roll'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[1].plot(data['timestamp'], data['pitch'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[2].plot(data['timestamp'], data['yaw'], linewidth=linewidth, label=labels[i], color=colors[i])
    axs[0].set_title('Roll', fontproperties=font)
    axs[1].set_title('Pitch', fontproperties=font)
    axs[2].set_title('Yaw', fontproperties=font)
    for ax in axs:
        ax.set_ylabel('Degrees (°)', fontproperties=font)
        ax.legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_color('black')
            spine.set_linewidth(1.5)
        ax.spines['right'].set_visible(True)
        ax.spines['top'].set_visible(True)

    axs[2].set_xlabel('Timestamp', fontproperties=font)
    axs[0].set_xlim([min([min(data['timestamp']) for data in datasets]), max([max(data['timestamp']) for data in datasets])])
    axs[1].set_xlim([min([min(data['timestamp']) for data in datasets]), max([max(data['timestamp']) for data in datasets])])
    axs[2].set_xlim([min([min(data['timestamp']) for data in datasets]), max([max(data['timestamp']) for data in datasets])])
    fig.tight_layout()
    fig.savefig("Multiple_Euler_over_time_visualization.pdf", bbox_inches='tight', dpi=300)
    plt.show()


# Adjusted font properties
font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(12)

# Load data from provided paths
# file_paths = ["optimized_poses_tum.txt", "global_icp_tum.txt", "fastlio_localization.txt"]

bag_file = 'sample'  # 请替换为实际路径
filename = os.path.splitext(bag_file)[0]

file_paths = [filename+"_gnss1_tum_format.txt", filename+"_gnss2_tum_format.txt", filename+"_gnss_sbg_tum_format.txt"]


datasets_loaded = load_multiple_tum_trajectories(*file_paths)
# Extract Euler angles for the loaded datasets
euler_datasets_loaded = [extract_euler_from_trajectory(data) for data in datasets_loaded]
euler_labels_loaded = ["GNSS1", "GNSS2", "SBG"]

# Visualize the Euler angles over time with "Times New Roman" font
# visualize_euler_over_time(euler_datasets_loaded, euler_labels_loaded, font)

# Visualize the Euler angles over time for multiple trajectories
visualize_multiple_euler_over_time(euler_datasets_loaded, euler_labels_loaded, font)

