# import zipfile
# import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.axes_grid1 import make_axes_locatable
# import numpy as np

# def plot_trajectory_and_error_enhanced(zip_file_path, optimized_poses_path, gt_path, output_pdf_path):
#     # Unzipping the file
#     with zipfile.ZipFile(zip_file_path, 'r') as zip_ref:
#         zip_ref.extractall("data")
#     # Load the error array and timestamps
#     error_array_path = 'data/error_array.npy'
#     timestamps_path = 'data/timestamps.npy'
#     error_array = np.load(error_array_path)
#     timestamps = np.load(timestamps_path)

#     # Load the optimized poses and ground truth data
#     optimized_poses = pd.read_csv(optimized_poses_path, sep=" ", header=None)
#     gt = pd.read_csv(gt_path, sep=" ", header=None)
#     # Extract the x, y coordinates from the truncated trajectories
#     opt_x, opt_y = optimized_poses.iloc[:len(error_array), 1], optimized_poses.iloc[:len(error_array), 2]
#     gt_x, gt_y = gt.iloc[:, 1], gt.iloc[:, 2]
#     # Calculate the mean error
#     mean_error = np.mean(error_array)
#     # Prepare the figure with professional styling
#     plt.rcParams.update({'font.family': 'Times New Roman', 'font.size': 12})
#     fig, axes = plt.subplots(2, 1, figsize=(15, 10), gridspec_kw={'height_ratios': [2, 1]})

#     # Plot the 2D trajectory with error color map
#     sc = axes[0].scatter(opt_x, opt_y, c=error_array, cmap='jet')
#     axes[0].scatter(opt_x.iloc[0], opt_y.iloc[0], color='green', marker='^', s=100, label='Start')
#     axes[0].scatter(opt_x.iloc[-1], opt_y.iloc[-1], color='red', marker='^', s=100, label='End')
#     axes[0].legend(loc='upper left', edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)
#     axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)


#     divider = make_axes_locatable(axes[0])
#     cax = divider.append_axes('right', size='3%', pad=0.2)
#     fig.colorbar(sc, cax=cax, orientation='vertical')

#     # Set titles and labels for the trajectory plot
#     axes[0].set_title('2D Error Trajectory')
#     axes[0].set_xlabel('X [m]')
#     axes[0].set_ylabel('Y [m]')

#     # Plot the overall RMSE over time
#     axes[1].plot(timestamps, error_array, label="RMSE", color='red', linewidth=2)
#     axes[1].axhline(mean_error, color='grey', linestyle='--', linewidth=2, label=f'Mean: {mean_error:.2f}')
#     axes[1].legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)
#     axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
#     axes[1].set_ylabel('RMSE [m]')
#     axes[1].set_xlabel('Timestamp [s]')

#     # Draw borders around the plots
#     for ax in axes:
#         for spine in ax.spines.values():
#             spine.set_edgecolor('black')
#     # Adjust layout
#     plt.tight_layout()
#     # Save the plot as a PDF
#     plt.savefig(output_pdf_path, format='pdf')
#     # Show the plot
#     plt.show()


# # Define file paths
# zip_file_path = 'test.zip'
# optimized_poses_path = 'optimized_poses_tum.txt'
# gt_path = 'GT.txt'
# output_pdf_path = 'trajectory_and_error.pdf'
# plot_trajectory_and_error_enhanced(zip_file_path, optimized_poses_path, gt_path, output_pdf_path)

import zipfile
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import numpy as np

def plot_trajectory_and_error_enhanced(rmse_zip_file_path, rpe_zip_file_path, optimized_poses_path, gt_path, output_pdf_path):
    # Unzipping RMSE results
    with zipfile.ZipFile(rmse_zip_file_path, 'r') as zip_ref:
        zip_ref.extractall("data/ate_results")
    # Unzipping RPE results
    with zipfile.ZipFile(rpe_zip_file_path, 'r') as zip_ref:
        zip_ref.extractall("data/rpe_results")
    # Load RMSE data
    rmse_error_array_path = 'data/ate_results/error_array.npy'
    rmse_timestamps_path = 'data/ate_results/timestamps.npy'
    rmse_error_array = np.load(rmse_error_array_path)
    rmse_timestamps = np.load(rmse_timestamps_path)

    # Load RPE data
    rpe_error_array_path = 'data/rpe_results/error_array.npy'
    rpe_timestamps_path = 'data/rpe_results/timestamps.npy'
    rpe_error_array = np.load(rpe_error_array_path)
    rpe_timestamps = np.load(rpe_timestamps_path)

    # Calculate mean errors for RMSE and RPE
    mean_rmse_error = np.mean(rmse_error_array)
    mean_rpe_error = np.mean(rpe_error_array)
    # Load the optimized poses and ground truth data
    optimized_poses = pd.read_csv(optimized_poses_path, sep=" ", header=None)
    gt = pd.read_csv(gt_path, sep=" ", header=None)
    # Extract the x, y coordinates from the trajectories
    opt_x, opt_y = optimized_poses.iloc[:len(rmse_error_array), 1], optimized_poses.iloc[:len(rmse_error_array), 2]
    gt_x, gt_y = gt.iloc[:, 1], gt.iloc[:, 2]

    # Prepare the figure with professional styling
    plt.rcParams.update({'font.family': 'Times New Roman', 'font.size': 12})
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), gridspec_kw={'height_ratios': [3, 2, 2]})

    # Plot the 2D trajectory with error color map
    sc = axes[0].scatter(opt_x, opt_y, c=rmse_error_array, cmap='jet')
    axes[0].scatter(opt_x.iloc[0], opt_y.iloc[0], color='green', marker='^', s=100, label='Start')
    axes[0].scatter(opt_x.iloc[-1], opt_y.iloc[-1], color='red', marker='^', s=100, label='End')
    axes[0].legend(loc='upper left', edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)
    axes[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    divider = make_axes_locatable(axes[0])
    cax = divider.append_axes('right', size='4%', pad=0.3)
    fig.colorbar(sc, cax=cax, orientation='vertical')
    axes[0].set_title('2D Error Trajectory')
    axes[0].set_xlabel('X [m]')
    axes[0].set_ylabel('Y [m]')

    # Plot RMSE over time
    axes[1].plot(rmse_timestamps, rmse_error_array, label="ATE", color='red', linewidth=2)
    axes[1].axhline(mean_rmse_error, color='black', linestyle='--', linewidth=2, label=f'Mean: {mean_rmse_error:.2f}')
    axes[1].legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)
    axes[1].grid(True, which='both', linestyle='--', linewidth=0.5)
    axes[1].set_ylabel('ATE [m]')
    # axes[1].set_xlabel('Timestamp [s]')

    # Plot RPE over time
    axes[2].plot(rpe_timestamps, rpe_error_array, label="RPE", color='green', linewidth=2)
    axes[2].axhline(mean_rpe_error, color='black', linestyle='--', linewidth=2, label=f'Mean: {mean_rpe_error:.2f}')
    axes[2].legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)
    axes[2].grid(True, which='both', linestyle='--', linewidth=0.5)
    axes[2].set_ylabel('RPE [m]')
    axes[2].set_xlabel('Timestamp [s]')

    # Draw borders around the plots
    for ax in axes:
        for spine in ax.spines.values():
            spine.set_edgecolor('black')
    # Adjust layout
    plt.tight_layout()
    # Save the plot as a PDF
    plt.savefig(output_pdf_path, format='pdf', bbox_inches='tight', dpi=300)
    # Show the plot
    plt.show()

# Define file paths
rmse_zip_file_path = 'test.zip'
rpe_zip_file_path = 'test_rpe.zip'
optimized_poses_path = 'optimized_poses_tum.txt'
gt_path = 'GT.txt'
output_pdf_path = 'trajectory_and_error.pdf'
# Call the function to create and save the plot
plot_trajectory_and_error_enhanced(rmse_zip_file_path, rpe_zip_file_path, optimized_poses_path, gt_path, output_pdf_path)