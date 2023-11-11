import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
# from scipy.linalg import umeyama
from matplotlib.font_manager import FontProperties
from mpl_toolkits import mplot3d  # Ensure 3D support is imported
from mpl_toolkits.axes_grid1 import make_axes_locatable

def save_trajectory_to_file(trajectory, file_name):
    """Save the aligned trajectory to a file."""
    with open(file_name, "w") as file:
        for timestamp, x, y, z in zip(trajectory['timestamp'], trajectory['tx'], trajectory['ty'], trajectory['tz']):
            file.write(f"{timestamp} {x} {y} {z}\n")

def save_transformation_matrix_to_file(matrix, file_name):
    """Save the transformation matrix to a file."""
    np.savetxt(file_name, matrix, delimiter=' ', fmt='%f')

def save_errors_and_transformation_to_file(label, error, transformation_matrix, file_name):
    """Save the average XYZ errors and transformation matrix to a file."""
    with open(file_name, "w") as file:
        file.write(f"Dataset: {label}\n")
        file.write(f"X Avg Error (m): {error['x_avg_error']:.4f}\n")
        file.write(f"Y Avg Error (m): {error['y_avg_error']:.4f}\n")
        file.write(f"Z Avg Error (m): {error['z_avg_error']:.4f}\n\n")
        file.write("Transformation Matrix:\n")
        for row in transformation_matrix:
            file.write(" ".join([f"{elem:.6f}" for elem in row]) + "\n")

def umeyama(src, dst, estimate_scale):
    """Estimate N-D similarity transformation with or without scaling.
    Parameters
    ----------
    src : (M, N) array
        Source points.
    dst : (M, N) array
        Destination points.
    estimate_scale : bool
        Whether to estimate scaling factor.
    Returns
    -------
    T : (N + 1, N + 1)
        The homogeneous similarity transformation matrix. The matrix contains
        NaN values only if the problem is not well-conditioned.
    References
    ----------
    .. [1] "Least-squares estimation of transformation parameters between two
            point patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
    """
    num = src.shape[0]
    dim = src.shape[1]
    # Compute mean of src and dst.
    center_src = src.mean(axis=0)
    center_dst = dst.mean(axis=0)
    # Subtract mean from src and dst.
    src_demean = src - center_src
    dst_demean = dst - center_dst
    # Eq. (38).
    A = np.dot(dst_demean.T, src_demean) / num
    # Eq. (39).
    d = np.ones((dim,), dtype=np.double)
    if np.linalg.det(A) < 0:
        d[dim - 1] = -1
    T = np.eye(dim + 1, dtype=np.double)
    U, S, V = np.linalg.svd(A)
    # Eq. (40) and (43).
    rank = np.linalg.matrix_rank(A)
    if rank == 0:
        return np.nan * T
    elif rank == dim - 1:
        if np.linalg.det(U) * np.linalg.det(V) > 0:
            T[:dim, :dim] = np.dot(U, V)
        else:
            s = d[dim - 1]
            d[dim - 1] = -1
            T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V))
            d[dim - 1] = s
    else:
        T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V.T))
    if estimate_scale:
        # Eq. (41) and (42).
        scale = 1.0 / src_demean.var(axis=0).sum() * np.dot(S, d)
    else:
        scale = 1.0
    T[:dim, dim] = center_dst - scale * np.dot(T[:dim, :dim], center_src.T)
    T[dim, dim] = 1.0
    return T


def umeyama(src, dst, estimate_scale, use_6dof=True):
    """Estimate N-D similarity transformation with or without scaling.
    Parameters:
    ----------
    src : (M, N) array
        Source points.
    dst : (M, N) array
        Destination points.
    estimate_scale : bool
        Whether to estimate scaling factor.
    use_6dof : bool
        Whether to use 6 degrees of freedom (translation + rotation).
        If False, only translation is used.
    Returns:
    -------
    T : (N + 1, N + 1)
        The homogeneous similarity transformation matrix. The matrix contains
        NaN values only if the problem is not well-conditioned.
    """
    num = src.shape[0]
    dim = src.shape[1]
    # Compute mean of src and dst.
    center_src = src.mean(axis=0)
    center_dst = dst.mean(axis=0)
    # Subtract mean from src and dst.
    src_demean = src - center_src
    dst_demean = dst - center_dst
    # Eq. (38).
    A = np.dot(dst_demean.T, src_demean) / num
    # Eq. (39).
    d = np.ones((dim,), dtype=np.double)
    if np.linalg.det(A) < 0:
        d[dim - 1] = -1
    T = np.eye(dim + 1, dtype=np.double)
    U, S, V = np.linalg.svd(A)
    # Adjustments for 6DoF
    if use_6dof:
        if np.linalg.det(U) * np.linalg.det(V) < 0:
            S[-1] = -S[-1]
    # Eq. (40) and (43).
    rank = np.linalg.matrix_rank(A)
    if rank == 0:
        return np.nan * T
    elif rank == dim - 1:
        if np.linalg.det(U) * np.linalg.det(V) > 0:
            T[:dim, :dim] = np.dot(U, V)
        else:
            s = d[dim - 1]
            d[dim - 1] = -1
            T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V))
            d[dim - 1] = s
    else:
        T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V.T))
    if estimate_scale:
        # Eq. (41) and (42).
        scale = 1.0 / src_demean.var(axis=0).sum() * np.dot(S, d)
    else:
        scale = 1.0
    T[:dim, dim] = center_dst - scale * np.dot(T[:dim, :dim], center_src.T)
    T[dim, dim] = 1.0
    return T



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

# Add the missing function to find the nearest timestamp index
def find_nearest_timestamp_index(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def compute_xyz_errors(test_datasets, ground_truth, time_threshold=0.01):
    errors = []
    for test_data in test_datasets:
        x_error = []
        y_error = []
        z_error = []
        valid_timestamps = []
        for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
            idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
            if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
                x_error.append(x - ground_truth['tx'][idx])
                y_error.append(y - ground_truth['ty'][idx])
                z_error.append(z - ground_truth['tz'][idx])
                valid_timestamps.append(t)
        errors.append({
            'timestamp': valid_timestamps,
            'x_error': x_error,
            'y_error': y_error,
            'z_error': z_error
        })
    return errors

def visualize_xyz_errors_multi(error_datasets, labels, font):
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    colors = ['g', 'r', 'c', 'm', 'y', 'k']
    linewidth = 1.5
    for i, data in enumerate(error_datasets):
        axs[0].plot(data['timestamp'], data['x_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[1].plot(data['timestamp'], data['y_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[2].plot(data['timestamp'], data['z_error'], linewidth=linewidth, label=labels[i], color=colors[i])
    axs[0].set_title('X Error', fontproperties=font)
    axs[1].set_title('Y Error', fontproperties=font)
    axs[2].set_title('Z Error', fontproperties=font)
    for ax in axs:
        # ax.set_xlabel('Timestamp', fontproperties=font)
        ax.set_ylabel('Meters (m)', fontproperties=font)
        ax.legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_color('black')
            spine.set_linewidth(1.5)
        ax.spines['right'].set_visible(True)
        ax.spines['top'].set_visible(True)
    axs[2].set_xlabel('Timestamp', fontproperties=font)  # Only bottom plot shows x-axis label

    fig.tight_layout()
    plt.show()
    fig.savefig("Multiple_XYZ_Errors_over_time_visualization.pdf", bbox_inches='tight', dpi=300)

def compute_xyz_errors_modified(test_datasets, ground_truth, time_threshold=0.02):
    """
    Compute the XYZ errors between multiple test datasets and a ground truth dataset.
    Parameters:
    - test_datasets: List of datasets (dicts) containing XYZ positions and timestamps to compare.
    - ground_truth: Dataset (dict) that is considered as the ground truth.
    - time_threshold: Maximum time difference to consider two timestamps as matching.
    Returns:
    - List of error datasets (dicts) with XYZ errors and timestamps.
    """
    errors = []
    for test_data in test_datasets:
        x_error = []
        y_error = []
        z_error = []
        valid_timestamps = []
        for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
            idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
            if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
                x_error.append(x - ground_truth['tx'][idx])
                y_error.append(y - ground_truth['ty'][idx])
                z_error.append(z - ground_truth['tz'][idx])
                valid_timestamps.append(t)
        errors.append({
            'timestamp': valid_timestamps,
            'x_error': x_error,
            'y_error': y_error,
            'z_error': z_error
        })
    return errors

def visualize_xyz_errors_multi_modified(error_datasets, labels, font):
    """
    Visualize the XYZ errors over time for multiple trajectories.
    Only the Z error plot will have the X-axis label 'Timestamp'.
    Parameters:
    - error_datasets: List of datasets, each containing XYZ errors and timestamps.
    - labels: List of labels corresponding to each dataset.
    - font: Font properties for text in the plots.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    colors = ['g', 'r', 'c', 'm', 'y', 'k']
    linewidth = 1.5
    # Plotting errors for each dataset
    for i, data in enumerate(error_datasets):
        axs[0].plot(data['timestamp'], data['x_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[1].plot(data['timestamp'], data['y_error'], linewidth=linewidth, label=labels[i], color=colors[i])
        axs[2].plot(data['timestamp'], data['z_error'], linewidth=linewidth, label=labels[i], color=colors[i])
    # Setting titles and labels
    axs[0].set_title('X Error', fontproperties=font)
    axs[1].set_title('Y Error', fontproperties=font)
    axs[2].set_title('Z Error', fontproperties=font)
    # Formatting for each subplot
    for ax in axs[:-1]:  # Excluding the last subplot for X-axis label
        ax.set_xlabel('', fontproperties=font)
        ax.set_ylabel('Error (m)', fontproperties=font)
        ax.legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        # Making the plot borders visible
        for spine in ax.spines.values():
            spine.set_color('black')
            spine.set_linewidth(1.5)
        ax.spines['right'].set_visible(True)
        ax.spines['top'].set_visible(True)
    # Special handling for the last subplot to add the X-axis label
    ax = axs[-1]
    ax.set_xlabel('Timestamp', fontproperties=font)
    ax.set_ylabel('Error (m)', fontproperties=font)
    ax.legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_color('black')
        spine.set_linewidth(1.5)
    ax.spines['right'].set_visible(True)
    ax.spines['top'].set_visible(True)
    # Display the plots
    fig.tight_layout()
    plt.show()


def align_trajectories_umeyama(test_data, ground_truth, time_threshold=0.02, with_scale=False, use_6dof=False):
    """
    Align the test trajectory to the ground truth using the Umeyama method.
    """
    # Find corresponding points based on timestamps and the given threshold
    src_points = []
    tgt_points = []
    for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
        idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
        if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
            src_points.append([x, y, z])
            tgt_points.append([ground_truth['tx'][idx], ground_truth['ty'][idx], ground_truth['tz'][idx]])
    src_points = np.array(src_points)
    tgt_points = np.array(tgt_points)
    if use_6dof:
        transformation_matrix_3d = umeyama(src_points[:, :2], tgt_points[:, :2], with_scale)
        aligned_xyz = np.dot(transformation_matrix_3d, np.vstack((test_data['tx'], test_data['ty'], np.ones(test_data['tx'].shape))))
        test_data['tx'] = aligned_xyz[0]
        test_data['ty'] = aligned_xyz[1]
    else:
        transformation_matrix = umeyama(src_points, tgt_points, with_scale)
        aligned_xyz = np.dot(transformation_matrix, np.vstack((test_data['tx'], test_data['ty'], test_data['tz'], np.ones(test_data['tx'].shape))))
        test_data['tx'] = aligned_xyz[0]
        test_data['ty'] = aligned_xyz[1]
        test_data['tz'] = aligned_xyz[2]
    return test_data

def align_trajectories_umeyama(test_data, ground_truth, time_threshold=0.01, with_scale=False, use_6dof=False, return_matrix=False):
    """
    Align the test trajectory to the ground truth using the Umeyama method.
    Args:
    - test_data (dict): The test trajectory data.
    - ground_truth (dict): The ground truth trajectory data.
    - time_threshold (float): Time threshold to find corresponding points.
    - with_scale (bool): Whether to consider scale in the transformation.
    - use_6dof (bool): Whether to use 6 degrees of freedom (only XY translation).
    - return_matrix (bool): Whether to return the transformation matrix.
    Returns:
    - dict: Aligned test data.
    - (optional) np.array: Transformation matrix.
    """
    # Find corresponding points based on timestamps and the given threshold
    src_points = []
    tgt_points = []
    for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
        idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
        if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
            src_points.append([x, y, z])
            tgt_points.append([ground_truth['tx'][idx], ground_truth['ty'][idx], ground_truth['tz'][idx]])
    src_points = np.array(src_points)
    tgt_points = np.array(tgt_points)

    transformation_matrix = umeyama(src_points, tgt_points, with_scale, use_6dof)
    
    # Transform the test_data using the transformation matrix
    # homogenous_coords = np.ones((len(test_data['tx']), 4))
    # homogenous_coords[:, :3] = np.array([test_data['tx'], test_data['ty'], test_data['tz']]).T
    # transformed_coords = (transformation_matrix @ homogenous_coords.T).T
    # test_data['tx'] = transformed_coords[:, 0]
    # test_data['ty'] = transformed_coords[:, 1]
    # test_data['tz'] = transformed_coords[:, 2]
    
    aligned_xyz = np.dot(transformation_matrix, np.vstack((test_data['tx'], test_data['ty'], test_data['tz'], np.ones(test_data['tx'].shape))))
    test_data['tx'] = aligned_xyz[0]
    test_data['ty'] = aligned_xyz[1]
    test_data['tz'] = aligned_xyz[2]

    if return_matrix:
        return test_data, transformation_matrix
    else:
        return test_data


def compute_xyz_errors_with_alignment(test_datasets, ground_truth, align=False, time_threshold=0.01, use_6dof=False):
    """
    Compute the XYZ errors between multiple test datasets and a ground truth dataset, with optional alignment.
    Parameters:
    - test_datasets: List of datasets (dicts) containing XYZ positions and timestamps to compare.
    - ground_truth: Dataset (dict) that is considered as the ground truth.
    - align: If True, the test datasets will be aligned to the ground truth using Umeyama.
    - time_threshold: Maximum time difference to consider two timestamps as matching.
    - use_6dof: If True and align is True, use 6DOF alignment.
    Returns:
    - List of error datasets (dicts) with XYZ errors and timestamps.
    """
    errors = []
    for test_data in test_datasets:
        if align:
            # test_data = align_trajectories_umeyama(test_data, ground_truth, with_scale=False, use_6dof=use_6dof)
            test_data = align_trajectories_umeyama(test_data, ground_truth, time_threshold=0.02, with_scale=False, use_6dof=use_6dof)
        x_error = []
        y_error = []
        z_error = []
        valid_timestamps = []
        rmse_errors=[]
        ate_rmses=[]
        for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
            idx = find_nearest_timestamp_index(ground_truth['timestamp'], t)
            if abs(ground_truth['timestamp'][idx] - t) <= time_threshold:
                dx = x - ground_truth['tx'][idx]
                dy = y - ground_truth['ty'][idx]
                dz = z - ground_truth['tz'][idx]
                rmse_error = np.sqrt(dx**2 + dy**2 + dz**2)
                x_error.append(dx)
                y_error.append(dy)
                z_error.append(dz)
                rmse_errors.append(rmse_error)
                valid_timestamps.append(t)
            # Compute the ATE RMSE for the entire trajectory
        ate_rmse = np.mean(rmse_errors)
        errors.append({
            'timestamp': valid_timestamps,
            'x_error': x_error,
            'y_error': y_error,
            'z_error': z_error,
            'rmse_errors': rmse_errors
        })
        ate_rmses.append(ate_rmse)
    return errors, ate_rmses

def visualize_and_save_trajectories(test_data, ground_truth, labels, filename):
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot Ground Truth
    ax.plot(ground_truth['tx'], ground_truth['ty'], ground_truth['tz'], 'g-', label="GT", linewidth=2)
    ax.scatter(ground_truth['tx'][0], ground_truth['ty'][0], ground_truth['tz'][0], c='green', marker='o', s=100, label="Start", zorder=5)
    ax.scatter(ground_truth['tx'][-1], ground_truth['ty'][-1], ground_truth['tz'][-1], c='green', marker='<', s=100, label="End", zorder=5)
    
    # Plot Test Data
    for data, label in zip(test_data, labels):
        ax.plot(data['tx'], data['ty'], data['tz'], label=label)
         # Mark start and end points
        # Use same color as trajectory for start and end points
        color = ax.lines[-1].get_color()
        ax.scatter(data['tx'][0], data['ty'][0], data['tz'][0], c=color, marker='o', s=200, zorder=5, label='_nolegend_')
        ax.scatter(data['tx'][-1], data['ty'][-1], data['tz'][-1], c=color, marker='<', s=200, zorder=5,label='_nolegend_')

    # ax.set_title(f'3D Aligned Trajectories')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.grid(True)
    ax.w_xaxis.pane.fill = False
    ax.w_yaxis.pane.fill = False
    ax.w_zaxis.pane.fill = False
    ax.w_xaxis.pane.set_edgecolor('black')
    ax.w_yaxis.pane.set_edgecolor('black')
    ax.w_zaxis.pane.set_edgecolor('black')
    ax.w_xaxis.pane.set_linewidth(1.5)
    ax.w_yaxis.pane.set_linewidth(1.5)
    ax.w_zaxis.pane.set_linewidth(1.5)
    ax.w_xaxis.line.set_color("black")
    ax.w_xaxis.line.set_linewidth(1.5)
    ax.w_yaxis.line.set_color("black")
    ax.w_yaxis.line.set_linewidth(1.5)
    ax.w_zaxis.line.set_color("black")
    ax.w_zaxis.line.set_linewidth(1.5)

   

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend(loc='upper left', prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
    plt.savefig(filename, bbox_inches='tight')
    plt.show()
    plt.close()

def compute_rmse_errors(test_data, ground_truth_data):
    """
    Compute RMSE errors between test data and ground truth.
    Args:
    - test_data (dict): The test trajectory data.
    - ground_truth_data (dict): The ground truth trajectory data.
    Returns:
    - list: RMSE errors for each timestamp.
    - float: ATE RMSE for the entire trajectory.
    """
    rmse_errors = []
    for t, x, y, z in zip(test_data['timestamp'], test_data['tx'], test_data['ty'], test_data['tz']):
        idx = find_nearest_timestamp_index(ground_truth_data['timestamp'], t)
        # if abs(ground_truth_data['timestamp'][idx] - t) <= TIME_THRESHOLD:
        dx = x - ground_truth_data['tx'][idx]
        dy = y - ground_truth_data['ty'][idx]
        dz = z - ground_truth_data['tz'][idx]
        rmse_error = np.sqrt(dx**2 + dy**2 + dz**2)
        rmse_errors.append(rmse_error)
    # Compute the ATE RMSE for the entire trajectory
    ate_rmse = np.mean(rmse_errors)
    return rmse_errors, ate_rmse



def plot_rmse_and_2D_trajectory(title, data):
    timestamps, x_coords, y_coords, rmse, variances_x, variances_y, variances_z = data
    fig, axs = plt.subplots(2, 1, figsize=(14, 10), gridspec_kw={'height_ratios': [2, 1]})
    # XY trajectory plot with color based on RMSE
    sc = axs[0].scatter(x_coords, y_coords, c=rmse, cmap='jet', s=10)
    # Add colorbar
    divider = make_axes_locatable(axs[0])
    cax = divider.append_axes("right", size="5%", pad=0.2)
    fig.colorbar(sc, cax=cax, label='RMSE (m)', orientation='vertical')
    axs[0].set_title(f'XY Trajectory for {title}')
    axs[0].set_xlabel('X [m]')
    axs[0].set_ylabel('Y [m]')
    axs[0].grid(True)
    # RMSE plot
    axs[1].plot(timestamps, rmse, label='RMSE', color='purple')
    axs[1].set_title(f'RMSE over time for {title}')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('RMSE (m)')
    axs[1].grid(True)
    axs[1].legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True)
    plt.tight_layout()
    plt.savefig(f"{title.replace('/', '_')}_RMSE_and_2D_trajectory.pdf")
    plt.show()

def plot_3D_trajectory_with_rmse_color(title, data):
    x_coords, y_coords, z_coords, rmse = data
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(x_coords, y_coords, z_coords, c=rmse, cmap='jet', s=10)
    ax.set_title(f'3D Trajectory for {title}')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    fig.colorbar(sc, ax=ax, label='RMSE (m)', orientation='vertical', pad=0.1, aspect=20)
    plt.tight_layout()
    plt.savefig(f"{title.replace('/', '_')}_3D_trajectory.pdf")
    plt.show()


def plot_rmse_and_2D_trajectory_line(label, data):
    timestamps, x_coords, y_coords, rmse_errors, x_errors, y_errors, z_errors = data
    avg_rmse = np.mean(rmse_errors)
    max_rmse = np.max(rmse_errors)
    min_rmse = np.min(rmse_errors)
    fig, axs = plt.subplots(2, 1, figsize=(14, 14), gridspec_kw={'height_ratios': [1, 2]})
    sc = axs[0].scatter(x_coords, y_coords, c=rmse_errors, cmap='jet', s=50)
    axs[0].set_title(f'2D Trajectory for {label}')
    axs[0].plot(x_coords, y_coords, color='black', linestyle='-', linewidth=1.5, alpha=1.0)
    axs[0].scatter(x_coords[0], y_coords[0], color='red', s=200, label='Start', marker='o')
    axs[0].scatter(x_coords[-1], y_coords[-1], color='green', s=200, label='End', marker='X')
    axs[0].set_xlabel('X [m]')
    axs[0].set_ylabel('Y [m]')
    axs[0].grid(True)
    axs[0].legend(prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.0, frameon=True).get_frame().set_linewidth(1.5)
    divider = make_axes_locatable(axs[0])
    cax = divider.append_axes("right", size="5%", pad=0.2)
    cbar = fig.colorbar(sc, cax=cax, orientation='vertical')
    cbar.set_label('RMSE', rotation=90)

  # Annotate the RMSE values on y-axis
    # axs[1].annotate(f'Average', xy=(1.02, avg_rmse), xycoords=("axes fraction", "data"), textcoords="axes fraction", xytext=(0,0), ha="left", fontsize=9, verticalalignment='center')
    # axs[1].annotate(f'Max', xy=(1.02, max_rmse), xycoords=("axes fraction", "data"), textcoords="axes fraction", xytext=(0,0), ha="left", fontsize=9, verticalalignment='center')
    # axs[1].annotate(f'Min', xy=(1.02, min_rmse), xycoords=("axes fraction", "data"), textcoords="axes fraction", xytext=(0,0), ha="left", fontsize=9, verticalalignment='center')

    # axs[1].text(timestamps[0], avg_rmse, 'Average', verticalalignment='bottom', horizontalalignment='right', color='blue', fontsize=10)
    # axs[1].text(timestamps[0], max_rmse, 'Max', verticalalignment='top', horizontalalignment='right', color='blue', fontsize=10)
    # axs[1].text(timestamps[0], min_rmse, 'Min', verticalalignment='top', horizontalalignment='right', color='blue', fontsize=10)

    axs[1].set_yticks(list(axs[1].get_yticks()) + [avg_rmse, max_rmse, min_rmse])
    axs[1].plot(timestamps, rmse_errors, color='blue', label='RMSE')
    axs[1].axhline(avg_rmse, color='blue', linestyle='--', linewidth=1.5)
    axs[1].text(timestamps[-1] + 0.02, avg_rmse, 'Average', verticalalignment='center', color='lightblue')
    axs[1].axhline(max_rmse, color='blue', linestyle='--', linewidth=1.5)
    axs[1].text(timestamps[-1] + 0.02, max_rmse, 'Max', verticalalignment='center', color='lightblue')
    axs[1].axhline(min_rmse, color='blue', linestyle='--', linewidth=1.5)
    axs[1].text(timestamps[-1] + 0.02, min_rmse, 'Min', verticalalignment='center', color='lightblue')
    axs[1].set_xlabel('Timestamp')
    axs[1].set_ylabel('RMSE')
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f"{label}_RMSE_and_2D_trajectory.pdf")
    plt.show()


def plot_3D_trajectory_with_rmse_color_line(title, data):
    x_coords, y_coords, z_coords, rmse = data
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_coords, y_coords, z_coords, '-', color='black', alpha=1.0, linewidth=1.5)
    sc = ax.scatter(x_coords, y_coords, z_coords, c=rmse, cmap='jet', s=50, label='_nolegend_')
    ax.set_title(f'3D Trajectory for {title}')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.grid(True)
    ax.w_xaxis.pane.fill = False
    ax.w_yaxis.pane.fill = False
    ax.w_zaxis.pane.fill = False
    ax.w_xaxis.pane.set_edgecolor('black')
    ax.w_yaxis.pane.set_edgecolor('black')
    ax.w_zaxis.pane.set_edgecolor('black')
    ax.w_xaxis.pane.set_linewidth(1.5)
    ax.w_yaxis.pane.set_linewidth(1.5)
    ax.w_zaxis.pane.set_linewidth(1.5)
    ax.w_xaxis.line.set_color("black")
    ax.w_xaxis.line.set_linewidth(1.5)
    ax.w_yaxis.line.set_color("black")
    ax.w_yaxis.line.set_linewidth(1.5)
    ax.w_zaxis.line.set_color("black")
    ax.w_zaxis.line.set_linewidth(1.5)

    
    # Mark start and end points
    ax.scatter(x_coords[0], y_coords[0], z_coords[0], c='green', marker='>', s=200, label='Start Point', zorder=5)
    ax.scatter(x_coords[-1], y_coords[-1], z_coords[-1], c='red', marker='<', s=200, label='End Point', zorder=5)
    ax.set_title(f'3D Trajectory for {title}')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend(loc='upper left', prop=font, edgecolor='black', facecolor='none', framealpha=1, markerscale=1.0, frameon=True).get_frame().set_linewidth(1.5)
    fig.colorbar(sc, ax=ax, label='RMSE (m)', orientation='vertical', pad=0.1, aspect=20)
    plt.tight_layout()
    plt.savefig(f"{title.replace('/', '_')}_3D_trajectory.pdf")
    plt.show()




font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_size(12)
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
plt.rcParams['axes.labelweight'] = 'normal'
plt.rcParams['font.size'] = 12
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12
plt.rcParams['legend.fontsize'] = 12

# Main Execution
# file_paths = ["2023-10-24-01-08-59_gnss1_tum_format.txt", "2023-10-24-01-08-59_gnss2_tum_format.txt", "2023-10-24-01-08-59_gnss_sbg_tum_format.txt", "2023-10-24-01-08-59_tum_ins_format.txt"]
file_paths = ["Proposed.txt", "ICP-LOCALIZATION.txt", "global_icp_tum_o3d.txt", "GT.txt"]

datasets_loaded = [load_tum_format_trajectory(fp) for fp in file_paths]
ground_truth_xyz = datasets_loaded[-1]
test_xyz_datasets = datasets_loaded[:-1]
test_labels = ["Proposed", "ICP", "ICP-o3d", "GT"]
# test_labels = ["Proposed", "FL2",  "HDL", "Ground Truth"]
align_flags = [True, True, True, True, False]  # Example list, set to your preference 

visualize_and_save_trajectories(test_xyz_datasets, ground_truth_xyz, test_labels, "original_trajectories.pdf")


TIME_THRESHOLD = 0.01  # Maximum time difference to consider two timestamps as matching
aligned_datasets = []
aligned_matrixs = []
ate_rmse_values=[]
for test_data, align_flag in zip(test_xyz_datasets, align_flags):
    if align_flag:
        aligned_data, transformation_matrix = align_trajectories_umeyama(test_data, ground_truth_xyz, time_threshold=TIME_THRESHOLD, use_6dof=True, with_scale=False, return_matrix=True)
    else:
        aligned_data = test_data  # No alignment performed, use the original data
        transformation_matrix = np.eye(4)  # 
    aligned_datasets.append(aligned_data)
    aligned_matrixs.append(transformation_matrix)


# Visualize aligned and  original trajectories
error_datasets_xyz_aligned, ate_rmse_values = compute_xyz_errors_with_alignment(aligned_datasets, ground_truth_xyz, time_threshold=TIME_THRESHOLD, align=True)
visualize_xyz_errors_multi(error_datasets_xyz_aligned, test_labels, font)
visualize_and_save_trajectories(aligned_datasets, ground_truth_xyz, test_labels, "aligned_trajectories.pdf")

# Calculate RMSE errors and ATE RMSE for each dataset
rmse_errors_datasets = []
# rmse_errors_datasets = [error_data['rmse_errors'] for error_data in error_datasets_xyz_aligned]

# ate_rmse_values = []
for test_data in test_xyz_datasets:
    rmse_errors, ate_rmse = compute_rmse_errors(test_data, ground_truth_xyz)
    rmse_errors_datasets.append(rmse_errors)
    # ate_rmse_values.append(ate_rmse)
# Print the ATE RMSE for each dataset
for label, ate_rmse in zip(test_labels, ate_rmse_values):
    print(f"ATE RMSE for {label}: {ate_rmse:.4f} m")

# Compute the number of trajectory points for each dataset
trajectory_points_counts = [len(test_data['timestamp']) for test_data in test_xyz_datasets]
# The number of matched points is the length of the RMSE errors
matched_points_counts = [len(rmse_errors) for rmse_errors in rmse_errors_datasets]
# Compute the overall RMSE for each dataset
overall_rmse = [np.mean(rmse_errors) for rmse_errors in rmse_errors_datasets]


# Visualize RMSE, 2D trajectory with RMSE color, and 3D trajectory with RMSE color
for label, test_data, rmse_errors, error_data in zip(test_labels, test_xyz_datasets, rmse_errors_datasets, error_datasets_xyz_aligned):
    data_for_plot = (test_data['timestamp'], test_data['tx'], test_data['ty'], rmse_errors, error_data['x_error'], error_data['y_error'], error_data['z_error'])
    # plot_rmse_and_2D_trajectory(label, data_for_plot)
    plot_rmse_and_2D_trajectory_line(label, data_for_plot)
    data_for_3D_plot = (test_data['tx'], test_data['ty'], test_data['tz'], rmse_errors)
    # plot_3D_trajectory_with_rmse_color(label, data_for_3D_plot)
    plot_3D_trajectory_with_rmse_color_line(label, data_for_3D_plot)

# Compute and save XYZ average errors and alignment matrices to a TXT file
with open("Average_XYZ_Errors_and_Transformation_Matrices.txt", "w") as file:
    file.write("Dataset\tX Avg Error (m)\tY Avg Error (m)\tZ Avg Error (m)\tTransformation Matrix\n")
    for label, errors, test_data, align_flag, aligned_matrix in zip(test_labels, error_datasets_xyz_aligned, test_xyz_datasets, align_flags, aligned_matrixs):
        # Get the average XYZ errors from the computed error datasets
        avg_x_error = np.mean(np.abs(errors['x_error']))
        avg_y_error = np.mean(np.abs(errors['y_error']))
        avg_z_error = np.mean(np.abs(errors['z_error']))
        matrix_str = "\n" + "\n".join(["\t".join(map(str, row)) for row in aligned_matrix])
        file.write(f"{label}\t{avg_x_error:.4f}\t{avg_y_error:.4f}\t{avg_z_error:.4f}{matrix_str}\n\n")
    file.write("\n--- RMSE Errors ---\n")
    for label, rmse_errors in zip(test_labels, rmse_errors_datasets):
        avg_rmse = np.mean(rmse_errors)
        file.write(f"{label}\tAverage RMSE: {avg_rmse:.4f}\n")

    file.write("\nDataset\tTotal Points\tMatched Points with GT\tOverall RMSE\n")
    for label, total_points, matched_points, rmse in zip(test_labels, trajectory_points_counts, matched_points_counts, overall_rmse):
        file.write(f"{label}\t{total_points}\t{matched_points}\t{rmse:.4f}\n")
