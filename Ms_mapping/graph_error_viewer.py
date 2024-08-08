import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib.colors as colors
# Set global font to Times New Roman
rcParams['font.family'] = 'Times New Roman'
rcParams['font.size'] = 10
def read_error_file(filename):
    poses = {}
    factors = []
    reading_poses = True
    with open(filename, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            if reading_poses:
                if len(values) == 8:
                    index = int(values[0])
                    poses[index] = {'x': values[1], 'y': values[2], 'z': values[3],
                                    'qx': values[4], 'qy': values[5], 'qz': values[6], 'qw': values[7]}
                else:
                    reading_poses = False
            else:
                if len(values) == 8:  # Prior factor
                    factors.append({'type': 'prior', 'node': int(values[0]), 'error': values[1:7], 'weight': values[7]})
                elif len(values) == 9:  # Between factor
                    factors.append({'type': 'between', 'nodes': (int(values[0]), int(values[1])), 'error': values[2:8], 'weight': values[8]})
    return poses, factors

def plot_pose_graph_2d(poses, factors):
    fig, ax = plt.subplots(figsize=(6, 4), dpi=300)
    # Extract x and y coordinates
    x = [pose['x'] for pose in poses.values()]
    y = [pose['y'] for pose in poses.values()]
    # Calculate the bounds of the trajectory
    x_min, x_max = min(x), max(x)
    y_min, y_max = min(y), max(y)
    # Add a small margin (e.g., 5% of the range)
    margin = 0.05
    x_margin = (x_max - x_min) * margin
    y_margin = (y_max - y_min) * margin
    # Set the axis limits
    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)
    # Plot poses as very small, light gray dots
    ax.scatter(x, y, c='lightgray', s=1, alpha=0.3)
    # Calculate logarithmic errors
    errors = [np.linalg.norm(factor['error']) for factor in factors if factor['type'] == 'between']
    log_errors = np.log10(errors)
    min_log_error, max_log_error = np.min(log_errors), np.max(log_errors)
    # Create a logarithmic color map
    # cmap = plt.get_cmap('viridis')
    cmap = plt.get_cmap('jet')
    
    norm = colors.Normalize(vmin=min_log_error, vmax=max_log_error)
    # Plot factors and color by logarithmic error
    for factor in factors:
        if factor['type'] == 'between':
            start, end = factor['nodes']
            error = np.linalg.norm(factor['error'])
            log_error = np.log10(error)
            color = cmap(norm(log_error))
            ax.plot([poses[start]['x'], poses[end]['x']],
                    [poses[start]['y'], poses[end]['y']],
                    c=color, linewidth=0.5, alpha=0.8)
    # Add grid with solid lines
    ax.grid(True, linestyle='-', alpha=0.3)
    # Colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label('Log10 Error Magnitude', fontsize=8)
    cbar.ax.tick_params(labelsize=8)
    # Labels and title
    ax.set_xlabel('X [m]', fontsize=10)
    ax.set_ylabel('Y [m]', fontsize=10)
    ax.set_title('Pose Graph with Error Visualization', fontsize=8, fontweight='bold')
    # Set aspect ratio to equal
    ax.set_aspect('equal')
    # Adjust tick parameters
    ax.tick_params(axis='both', which='major', labelsize=8)
    plt.tight_layout()
    plt.savefig('pose_graph_visualization_log_error_trimmed.pdf', dpi=300, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    filename = "graph_error.txt"
    poses, factors = read_error_file(filename)
    plot_pose_graph_2d(poses, factors)
