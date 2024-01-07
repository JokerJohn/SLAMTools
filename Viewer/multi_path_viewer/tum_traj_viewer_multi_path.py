import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d  # Ensure 3D support is imported
from matplotlib.font_manager import FontProperties
def load_tum_format_trajectory(filename):
    data = pd.read_csv(filename, sep=' ', header=None, names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    return data
def visualize_final_complete_trajectories_2D(datasets, ax, labels, font):
    markers = ['^', 'o', 's', 'p', '*']
    colors = ['b', 'g', 'r', 'c', 'm']
    linewidth = 1.8
    for i, data in enumerate(datasets):
        ax.plot(data['tx'], data['ty'], color=colors[i % len(colors)], linewidth=linewidth, label=labels[i])
        ax.scatter(data['tx'].iloc[0], data['ty'].iloc[0], c=colors[i % len(colors)], marker=markers[0], s=100)
        ax.scatter(data['tx'].iloc[-1], data['ty'].iloc[-1], c=colors[i % len(colors)], marker=markers[1], s=100)
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_color('black')
        spine.set_linewidth(1.5)
    ax.spines['right'].set_visible(True)
    ax.spines['top'].set_visible(True)
    ax.set_xlabel('X [m]', fontproperties=font)
    ax.set_ylabel('Y [m]', fontproperties=font)
    ax.set_title('2D Trajectories', fontproperties=font)
    ax.legend(handles=ax.lines + [plt.Line2D([0], [0], color=colors[0], marker=markers[0], linestyle='None', markersize=10, label='Start'),
                                  plt.Line2D([0], [0], color=colors[0], marker=markers[1], linestyle='None', markersize=10, label='End')],
              prop=font, loc='upper right', edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
    ax.axis('equal')
def visualize_final_complete_trajectories_3D(datasets, ax, labels, font):
    markers = ['^', 'o', 's', 'p', '*']
    colors = ['b', 'g', 'r', 'c', 'm']
    linewidth = 1.8
    for i, data in enumerate(datasets):
        ax.plot(data['tx'], data['ty'], data['tz'], color=colors[i % len(colors)], linewidth=linewidth, label=labels[i])
        ax.scatter(data['tx'].iloc[0], data['ty'].iloc[0], data['tz'].iloc[0], c=colors[i % len(colors)], marker=markers[0], s=100)
        ax.scatter(data['tx'].iloc[-1], data['ty'].iloc[-1], data['tz'].iloc[-1], c=colors[i % len(colors)], marker=markers[1], s=100)
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
    ax.set_xlabel('X [m]', fontproperties=font)
    ax.set_ylabel('Y [m]', fontproperties=font)
    ax.set_zlabel('Z [m]', fontproperties=font)
    ax.set_title('3D Trajectories', fontproperties=font)
    ax.legend(handles=ax.lines[:len(labels)] + [plt.Line2D([0], [0], color=colors[0], marker=markers[0], linestyle='None', markersize=10, label='Start'),
                                                plt.Line2D([0], [0], color=colors[0], marker=markers[1], linestyle='None', markersize=10, label='End')],
              prop=font, loc='upper left', edgecolor='black', facecolor='none', framealpha=1, markerscale=1.5, frameon=True).get_frame().set_linewidth(1.5)
# Adjusted font properties
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

# Load the data from multiple files
filenames = [ "2023-12-11-10-22-21_ins_tum_format.txt", "2023-12-11-10-27-15_ins_tum_format.txt", "2023-09-19-11-46-20_ins_tum_format.txt", "2023-09-19-11-31-20_ins_tum_format.txt", "2023-09-19-11-23-50_ins_tum_format.txt"]  # Add your file names here
datasets = [load_tum_format_trajectory(f) for f in filenames]
# labels = ["INS " + str(i+1) for i in range(len(datasets))]
labels = [f.split('_ins')[0] for f in filenames]  # Splitting by '.' and taking the first part as label

# 过滤空的数据集并更新标签
datasets, labels = zip(*[(data, label) for data, label in zip(datasets, labels) if not data.empty])
# Plot final complete 2D trajectories
fig_2d, ax_2d = plt.subplots(figsize=(8, 6))
visualize_final_complete_trajectories_2D(datasets, ax_2d, labels, font)
plt.tight_layout()
# Plot final complete 3D trajectories
fig_3d = plt.figure(figsize=(8, 6))
ax_3d = fig_3d.add_subplot(111, projection='3d')
visualize_final_complete_trajectories_3D(datasets, ax_3d, labels, font)
plt.tight_layout()
plt.show()
# Optionally, save the plots
fig_3d.savefig("3D_trajectory_visualization.pdf", bbox_inches='tight', dpi=300)
fig_2d.savefig("2D_trajectory_visualization.pdf", bbox_inches='tight', dpi=300)