import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import TABLEAU_COLORS

def read_tum_pose_file(file_path):
    poses = []
    with open(file_path, 'r') as file:
        for line in file:
            data = line.split()
            timestamp = float(data[0])
            x, y, z = float(data[1]), float(data[2]), float(data[3])
            poses.append((timestamp, x, y))
    return np.array(poses)

def plot_trajectories(trajectories, labels, colors):
    plt.figure(figsize=(12, 12))
    # 设置图表样式
    plt.style.use('seaborn-whitegrid')
    # plt.rcParams['font.family'] = 'Times New Roman'
    # plt.rcParams['axes.edgecolor'] = 'black'
    # plt.rcParams['axes.linewidth'] = 3
    # plt.rcParams['grid.color'] = 'grey'
    # plt.rcParams['grid.alpha'] = 0.5
    # plt.rcParams['grid.linewidth'] = 1

    plt.rcParams.update({
        'font.family': 'Times New Roman',
        'font.size': 10,
        'axes.labelsize': 18,
        'axes.titlesize': 18,
        'xtick.labelsize': 18,
        'ytick.labelsize': 18,
        'legend.fontsize': 16,
        'lines.linewidth': 2,
        'axes.linewidth': 3,
        'axes.edgecolor': 'black'
    })

    for trajectory, label, color in zip(trajectories, labels, colors):
        plt.plot(trajectory[:, 1], trajectory[:, 2], label=label, color=color, linewidth=4)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    # plt.title('Merged Trajectories', pad=10)
    # plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.legend(edgecolor='black', facecolor='white', framealpha=0.8, frameon=True, fontsize=23, ncol=3 , columnspacing=0.3)
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()
    # 显示图形，但不阻塞程序
    plt.show(block=False)
    # 等待用户调整窗口并确认
    # input("调整窗口大小，然后按回车键保存图像...")
    # 获取当前图形的大小
    fig = plt.gcf()
    size = fig.get_size_inches()
    plt.show()
    # 关闭当前图形
    # plt.close()
    
    # 使用相同的大小创建新图形并重新绘制
    plt.figure(figsize=size)
    for trajectory, label, color in zip(trajectories, labels, colors):
        plt.plot(trajectory[:, 1], trajectory[:, 2], label=label, color=color, linewidth=4)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    # plt.title('Merged Trajectories')
    plt.legend(edgecolor='black', facecolor='white', framealpha=0.8, frameon=True, fontsize=23, ncol=3 , columnspacing=0.3)
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()
    # 保存为高质量PDF
    plt.savefig('trajectories_plot.pdf', format='pdf', dpi=600, bbox_inches='tight')
    print(f"Plot saved as trajectories_plot.pdf in {os.getcwd()}")
    plt.close()

def main():
    # 设置位姿文件所在的文件夹地址
    pose_folder = "/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4-NG/merged_results"  # 请替换为实际的文件夹路径

    
    # 定义前8个颜色
    initial_colors = ['#509b09','#ecd617','#BB4D50', '#87BD96','#6A4774', '#966415',  'blue','#3868A6']
    # initial_colors = ['']
# Loaded trajectory from CC1.txt with label CC1
# Loaded trajectory from CP2.txt with label CP2
# Loaded trajectory from CP5.txt with label CP5
# Loaded trajectory from CS1.txt with label CS1
# Loaded trajectory from IA3.txt with label IA3
# Loaded trajectory from IA4.txt with label IA4
# Loaded trajectory from NG.txt with label NG
# Loaded trajectory from PK1.txt with label PK1

    
    # 获取额外的颜色
    extra_colors = list(TABLEAU_COLORS.values())
    
    # 获取所有tum格式的位姿文件
    tum_files = [f for f in os.listdir(pose_folder) if f.endswith('.txt')]
    tum_files.sort()  # 确保文件顺序一致
    
    trajectories = []
    labels = []
    colors = []

    for i, file in enumerate(tum_files):
        file_path = os.path.join(pose_folder, file)
        trajectory = read_tum_pose_file(file_path)
        trajectories.append(trajectory)
        
        # 使用文件名（不包括扩展名）作为标签
        label = os.path.splitext(file)[0]
        labels.append(label)
        
        # 选择颜色
        if i < 8:
            color = initial_colors[i]
        else:
            color = extra_colors[(i - 8) % len(extra_colors)]
        colors.append(color)
        
        print(f"Loaded trajectory from {file} with label {label}")

    plot_trajectories(trajectories, labels, colors)
    print(f"Plot saved as trajectories_plot.pdf in {os.getcwd()}")

if __name__ == "__main__":
    main()
