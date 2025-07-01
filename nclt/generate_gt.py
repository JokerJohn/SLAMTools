#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R

def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角(r,p,h)转换为四元数(qx,qy,qz,qw)"""
    # rot = R.from_euler('xyz', [roll, pitch, yaw])
    R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll, pitch, yaw)

    return rot.as_quat()  # 返回 [qx, qy, qz, qw]

# def euler_to_quaternion(roll, pitch, yaw):
#     """
#     将欧拉角(r,p,h)转换为四元数(qx,qy,qz,qw)
#     使用航空顺序ZYX (yaw-pitch-roll)
#     输入单位为弧度
#     处理万向锁问题
#     """
#     # 检查奇异点
#     if abs(abs(pitch) - np.pi/2) < 1e-6:
#         print(f"Warning: Gimbal lock detected at pitch = {pitch:.6f} rad (near ±90°)")
#         # 在奇异点附近，可以添加小扰动避免严格奇异
#         if pitch > 0:
#             pitch = np.pi/2 - 1e-6
#         else:
#             pitch = -np.pi/2 + 1e-6
#     # 使用正确的旋转顺序'zyx'
#     rot = R.from_euler('zyx', [yaw, pitch, roll])
#     quat = rot.as_quat()  # 返回 [qx, qy, qz, qw]
#     # 确保四元数是归一化的
#     norm = np.sqrt(np.sum(quat**2))
#     if abs(norm - 1.0) > 1e-10:
#         quat = quat / norm
#     return quat


def getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad):
  R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0.0], 
                     [np.sin(yaw_rad), np.cos(yaw_rad), 0.0], 
                     [0.0, 0.0, 1.0]])

  R_pitch = np.array([[np.cos(pitch_rad), 0.0, np.sin(pitch_rad)], 
                     [0.0, 1.0, 0.0], 
                     [-np.sin(pitch_rad), 0.0, np.cos(pitch_rad)]])

  R_roll = np.array([[1.0, 0.0, 0.0], 
                    [0.0, np.cos(roll_rad), -np.sin(roll_rad)], 
                    [0.0, np.sin(roll_rad), np.cos(roll_rad)]])

  R = np.dot(np.dot(R_yaw, R_pitch), R_roll)
  return R



# def getQuaternionFromFromEulerAnglesRollPitchYawRad(roll_rad, pitch_rad, yaw_rad):
#   R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
#   #assert(np.linalg.det(R) == 1.0)
#   q = mk.Quaternion(R)

  return q

def find_nearest_cov_value(timestamp, t_cov, cov_data):
    """为每个时间戳找到最近的协方差值"""
    idx = np.abs(t_cov - timestamp).argmin()
    if cov_data.ndim > 1 and cov_data.shape[1] > 1:
        # 如果是完整协方差矩阵，使用对角线元素之和(迹)
        return np.sum(cov_data[idx, 1:7])  # 假设前6个对角线元素为位置和方向协方差
    else:
        # 如果是单值，直接返回
        return cov_data[idx]
    
def main(args):
    if len(args) < 4:
        print('Usage: python script.py groundtruth.csv covariance.csv output_tum.txt')
        return 1
    gt_file = args[1]
    cov_file = args[2]
    output_file = args[3]
    # 读取数据
    gt = np.loadtxt(gt_file, delimiter=",")
    cov = np.loadtxt(cov_file, delimiter=",")
    # 提取数据
    t_gt = gt[:, 0]  # 地面真值时间戳
    t_cov = cov[:, 0]  # 协方差时间戳
    # 提取协方差数据
    if cov.shape[1] > 1:
        cov_data = cov[:, 1:]
    else:
        # 如果cov文件只有时间戳列，创建一个单位值数组
        cov_data = np.ones(len(t_cov))
    # 创建TUM格式的位姿文件
    with open(output_file, 'w') as f:
        for i in range(len(t_gt)):
            t = t_gt[i] / 1e6  # 时间戳转换为秒
            x, y, z = gt[i, 1:4]  # 位置 (北, 东, 下)
            roll, pitch, yaw = gt[i, 4:7]  # 欧拉角 (r, p, h)
            # 欧拉角转四元数
            # qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
            qx, qy, qz, qw = getQuaternionFromFromEulerAnglesRollPitchYawRad(roll, pitch, yaw)
            # 写入TUM格式: 时间戳 x y z qx qy qz qw
            f.write(f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
    # 设置Matplotlib参数以获得期刊级别的图形
    mpl.rcParams.update({
        'font.family': 'Times New Roman',
        'font.size': 12,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'pdf.fonttype': 42,  # 确保文本可编辑
        'ps.fonttype': 42
    })
    # 创建图形
    fig, ax = plt.subplots(figsize=(8, 6))
    # 使用插值获取每个协方差时间戳处的位姿
    interp = interp1d(t_gt, gt[:, 1:], kind='nearest', axis=0, bounds_error=False, fill_value="extrapolate")
    pose_at_cov = interp(t_cov)
    # 提取位置
    x = pose_at_cov[:, 0]  # 北向
    y = pose_at_cov[:, 1]  # 东向
    # 计算每个点的协方差值
    color_values = np.array([find_nearest_cov_value(t, t_cov, cov_data) for t in t_cov])
    # 确保颜色值有合理范围
    if np.all(color_values == color_values[0]):
        # 如果所有值相同，添加小的随机变化以显示颜色
        color_values = color_values + np.random.randn(len(color_values)) * 0.01
    # 绘制散点图
    scatter = ax.scatter(y, x, c=color_values, cmap='viridis', s=2, alpha=0.8)
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Covariance', fontsize=12)
    # 设置标题和标签
    ax.set_title('Ground Truth Trajectory with Covariance', fontsize=14)
    ax.set_xlabel('East (m)', fontsize=12)
    ax.set_ylabel('North (m)', fontsize=12)
    # 保持等比例
    ax.set_aspect('equal')
    # 添加网格
    ax.grid(True, linestyle='--', alpha=0.7)
    # 为了清晰度，设置边界
    plt.tight_layout()
    # 保存为PDF
    plt.savefig('trajectory_visualization.pdf', bbox_inches='tight')
    print(f"TUM format data saved to {output_file}")
    print("Visualization saved to trajectory_visualization.pdf")
    return 0
if __name__ == '__main__':
    sys.exit(main(sys.argv))