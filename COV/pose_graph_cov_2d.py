import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np
# Set the font to Times New Roman for all text in the plot
plt.rcParams["font.family"] = "Times New Roman"
# Function to read poses and covariances with timestamps from the data file
def read_poses_covariances_with_timestamp(filename):
    poses = []
    covariances = []
    timestamps = []
    with open(filename, 'r') as file:
        for line in file.readlines():
            parts = line.strip().split(' ')
            timestamp = float(parts[0])
            pose = np.array(parts[1:8], dtype=np.float64)
            covariance = np.array(parts[8:], dtype=np.float64).reshape((6, 6))
            poses.append((pose[0], pose[1], pose[3]))  # pose[3] for the yaw
            covariances.append(covariance[3:5, 3:5])  # Using the submatrix for x, y covariance
            timestamps.append(timestamp)
    return timestamps, poses, covariances

# Function to plot the 2D trajectory with covariance ellipses
def plot_trajectory_2d_with_covariances(timestamps, poses, covariances, filename):
    plt.ion()
    fig, ax = plt.subplots()
    # Extract x, y coordinates and yaw angles
    x, y, yaw = zip(*poses)
    # Plot the trajectory with markers for start and end
    ax.plot(x, y, 'r-', label='Trajectory')
    # ax.plot(x[0], y[0], 'k^', markersize=10, label='Start')
    # ax.plot(x[-1], y[-1], 'g^', markersize=10, label='End')
    # Plot the poses as stars
    ax.plot(x, y, 'r*', markersize=4, label='Poses')
    # Plot covariance ellipses for each pose
    for (pose_x, pose_y, pose_yaw), cov in zip(poses, covariances):
        cov_xy = cov[0:2, 0:2]  # Extract the 2x2 covariance matrix for x and y
        eigenvalues, eigenvectors = np.linalg.eig(cov_xy)
        angle = np.degrees(np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0]))
        width, height = 70* np.sqrt(eigenvalues)
        ellipse = Ellipse(xy=(pose_x, pose_y), width=width, height=height,
                          angle=angle, color='blue', fill=False)
        ax.add_patch(ellipse)
    # Add grid with dashed lines
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_aspect('equal', 'box')
    plt.legend(loc='upper left', edgecolor='black', facecolor='none', framealpha=1, markerscale=0.8, frameon=True)
    # Save the plot as a vector PDF file
    input("调整图形后按Enter键保存并退出: ")  # 用户提示
    # Save the adjusted plot
    plt.savefig(filename.replace('.pdf', '_adjusted.pdf'), format='pdf', bbox_inches='tight', dpi=300)

    plt.show()

filename = 'pose_graph_3d_result.txt'
# Replace 'your_data_file.txt' with the path to your data file
timestamps, poses, covariances = read_poses_covariances_with_timestamp(filename)
# Plot the 2D trajectory with covariance ellipses
plot_trajectory_2d_with_covariances(timestamps, poses, covariances, 'trajectory_with_covariances.pdf')
