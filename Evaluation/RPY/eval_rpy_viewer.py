import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from matplotlib.font_manager import FontProperties

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


def corrected_euler_angles(euler):
    """Correct euler angles to be within [-180, 180] degrees."""
    corrected = np.array(euler)
    for i in range(3):  # For roll, pitch, yaw
        while corrected[i] > 180:
            corrected[i] -= 360
        while corrected[i] < -180:
            corrected[i] += 360
    return corrected

def extract_corrected_euler_from_trajectory(data):
    euler_angles = np.array([quaternion_to_euler(qx, qy, qz, qw) for qx, qy, qz, qw in zip(data['qx'], data['qy'], data['qz'], data['qw'])])
    corrected_euler_angles_list = [corrected_euler_angles(euler) for euler in euler_angles]
    return {
        'timestamp': data['timestamp'],
        'roll': [e[0] for e in corrected_euler_angles_list],
        'pitch': [e[1] for e in corrected_euler_angles_list],
        'yaw': [e[2] for e in corrected_euler_angles_list]
    }

def extract_euler_from_trajectory(data):
    euler_angles = np.array([quaternion_to_euler(qx, qy, qz, qw) for qx, qy, qz, qw in zip(data['qx'], data['qy'], data['qz'], data['qw'])])
    return {
        'timestamp': data['timestamp'],
        'roll': euler_angles[:, 0],
        'pitch': euler_angles[:, 1],
        'yaw': euler_angles[:, 2]
    }
def find_nearest_timestamp_index(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def compute_rpy_errors(test_datasets, ground_truth, time_threshold=0.02):
    errors = []
    for test_data in test_datasets:
        roll_error = []
        pitch_error = []
        yaw_error = []
        valid_timestamps = []
        for t, r, p, y in zip(test_data['timestamp'], test_data['roll'], test_data['pitch'], test_data['yaw']):
            idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
            if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
                roll_error.append(r - ground_truth['roll'][idx])
                pitch_error.append(p - ground_truth['pitch'][idx])
                yaw_error.append(y - ground_truth['yaw'][idx])
                valid_timestamps.append(t)
        errors.append({
            'timestamp': valid_timestamps,
            'roll_error': roll_error,
            'pitch_error': pitch_error,
            'yaw_error': yaw_error
        })
    return errors

def compute_rpy_errors2(test_datasets, ground_truth, time_threshold=0.02):
    errors = []
    for test_data in test_datasets:
        roll_error = []
        pitch_error = []
        yaw_error = []
        valid_timestamps = []
        for t, r, p, y in zip(test_data['timestamp'], test_data['roll'], test_data['pitch'], test_data['yaw']):
            idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
            if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
                roll_err = r - ground_truth['roll'][idx]
                pitch_err = p - ground_truth['pitch'][idx]
                yaw_err = y - ground_truth['yaw'][idx]
                # Correct the errors to be within [-180, 180] degrees
                roll_err, pitch_err, yaw_err = corrected_euler_angles([roll_err, pitch_err, yaw_err])
                roll_error.append(roll_err)
                pitch_error.append(pitch_err)
                yaw_error.append(yaw_err)
                valid_timestamps.append(t)
        errors.append({
            'timestamp': valid_timestamps,
            'roll_error': roll_error,
            'pitch_error': pitch_error,
            'yaw_error': yaw_error
        })
    return errors


def visualize_rpy_errors_multi(error_datasets, labels, font):
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    colors = ['g', 'r', 'c', 'm', 'y', 'k']
    linewidth = 1.5
    for i, data in enumerate(error_datasets):
        axs[0].plot(data['timestamp'], data['roll_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[1].plot(data['timestamp'], data['pitch_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[2].plot(data['timestamp'], data['yaw_error'], linewidth=linewidth, label=labels[i], color=colors[i])
    axs[0].set_title('Roll Error', fontproperties=font)
    axs[1].set_title('Pitch Error', fontproperties=font)
    axs[2].set_title('Yaw Error', fontproperties=font)
    for ax in axs:
        # ax.set_xlabel('Timestamp', fontproperties=font)
        ax.set_ylabel('Degrees (°)', fontproperties=font)
        ax.legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_color('black')
            spine.set_linewidth(1.5)
        ax.spines['right'].set_visible(True)
        ax.spines['top'].set_visible(True)
    
    axs[2].set_xlabel('Timestamp', fontproperties=font)  # Only bottom plot shows x-axis label
    fig.tight_layout()
    fig.savefig("Multiple_RPY_Errors_over_time_visualization.pdf", bbox_inches='tight', dpi=300)
    plt.show()


# Calculate average RPY errors for each dataset and save to a txt file
def calculate_and_save_avg_errors(error_datasets, labels, filename="average_rpy_errors.txt"):
    with open(filename, 'w') as f:
        f.write("Dataset\tRoll_Error\tPitch_Error\tYaw_Error\n")
        for data, label in zip(error_datasets, labels):
            avg_roll_error = np.mean(np.abs(data['roll_error']))
            avg_pitch_error = np.mean(np.abs(data['pitch_error']))
            avg_yaw_error = np.mean(np.abs(data['yaw_error']))
            f.write(f"{label}\t{avg_roll_error}\t{avg_pitch_error}\t{avg_yaw_error}\n")


# Main Execution
# Adjusted font properties
font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(12)

file_paths = ["optimized_poses_tum.txt", "20220216_garden_day.txt", "fastlio_localization.txt", "global_icp_tum.txt"]
datasets_loaded = [load_tum_format_trajectory(fp) for fp in file_paths]

euler_datasets_loaded = [extract_corrected_euler_from_trajectory(data) for data in datasets_loaded]
ground_truth_euler = euler_datasets_loaded[-1]
test_euler_datasets = euler_datasets_loaded[:-1]
error_datasets = compute_rpy_errors2(test_euler_datasets, ground_truth_euler, 0.02)
test_labels = ["Proposed", "HDL",  "FL2L", "Ground Truth"]
visualize_rpy_errors_multi(error_datasets, test_labels, font)

# Calculate and save average RPY errors
calculate_and_save_avg_errors(error_datasets, test_labels)
