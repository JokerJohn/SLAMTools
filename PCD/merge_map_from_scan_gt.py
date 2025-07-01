import os
import numpy as np
import open3d as o3d
def read_tum_pose_file(file_path):
    poses = []
    with open(file_path, 'r') as file:
        for line in file:
            data = line.split()
            timestamp = float(data[0])
            tx, ty, tz = float(data[1]), float(data[2]), float(data[3])
            qx, qy, qz, qw = float(data[4]), float(data[5]), float(data[6]), float(data[7])
            poses.append((timestamp, tx, ty, tz, qx, qy, qz, qw))
    return poses
def find_closest_pose(timestamp, gt_poses, threshold=0.01):
    """根据时间戳找到最接近的ground truth位姿，时间差需小于阈值"""
    closest_idx = min(range(len(gt_poses)), key=lambda i: abs(gt_poses[i][0] - timestamp))
    if abs(gt_poses[closest_idx][0] - timestamp) > threshold:
        return None
    return gt_poses[closest_idx]
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]
    ])
def pose_to_transformation_matrix(tx, ty, tz, qx, qy, qz, qw):
    """将位姿转换为变换矩阵"""
    transformation = np.eye(4)
    transformation[:3, :3] = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    transformation[:3, 3] = np.array([tx, ty, tz])
    return transformation
def get_imu_to_lidar_transform():
    """获取从IMU坐标系到Lidar坐标系的变换矩阵"""
    # 从Lidar到IMU的外参: extrinsic_T: [0, 0, 0.28]
    # 从IMU到Lidar的变换是Lidar到IMU的逆变换
    extrinsic_T = np.array([0, 0, -0.28])  # 注意符号变化
    extrinsic_R = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])  # 旋转矩阵是单位矩阵，所以逆变换也一样
    transformation = np.eye(4)
    transformation[:3, :3] = extrinsic_R
    transformation[:3, 3] = extrinsic_T
    return transformation
def get_R_gt_l_matrix():
    """获取从ground truth坐标系到lidar坐标系的旋转矩阵
    R_gt_l = [ 0 -1  0
              -1  0  0
               0  0 -1]
    """
    R_gt_l = np.array([
        [0, -1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    return R_gt_l

def main():
    folder = '/home/xchu/data/pose_slam_result/exp05_20120115_0.5/'
    pose_file = folder + 'optimized_odom_tum.txt'
    gt_pose_file = folder + 'groundtruth_2012-01-15-tum.txt'
    pcd_folder = folder + 'key_point_frame'
    output_file = folder + 'merged_map.pcd'
    # 读取两个位姿文件
    poses = read_tum_pose_file(pose_file)
    gt_poses = read_tum_pose_file(gt_pose_file)
    print(f"原始位姿数量: {len(poses)}")
    print(f"Ground truth位姿数量: {len(gt_poses)}")
    # 获取从IMU坐标系到Lidar坐标系的变换矩阵
    imu_to_lidar = get_imu_to_lidar_transform()
    # 获取从ground truth坐标系到lidar坐标系的旋转矩阵
    R_gt_l = get_R_gt_l_matrix()
    merged_pcd = o3d.geometry.PointCloud()
    valid_frames = 0
    skipped_frames = 0
    for i, pose in enumerate(poses):
        # if i > 1000:
        #     break
        pcd_file = os.path.join(pcd_folder, f'{i}.pcd')
        if not os.path.exists(pcd_file):
            print(f'警告: 点云文件 {i}.pcd 不存在. 跳过.')
            continue
        timestamp = pose[0]  # 获取原始位姿的时间戳
        gt_pose = find_closest_pose(timestamp, gt_poses, threshold=0.005)  # 找到最接近的ground truth位姿
        if gt_pose is None:
            print(f'警告: 帧 {i} 没有找到时间差小于0.01秒的ground truth位姿. 跳过.')
            skipped_frames += 1
            continue
        time_diff = abs(gt_pose[0] - timestamp)
        print(f"帧 {i}: 使用时间戳为 {gt_pose[0]} 的ground truth位姿 (原始: {timestamp}, 差值: {time_diff:.6f}秒)")
        # 读取点云并进行降采样（在IMU坐标系下）
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd_down = pcd.voxel_down_sample(voxel_size=0.1)
        # 首先将点云从IMU坐标系转换到Lidar坐标系
        pcd_in_lidar = pcd_down.transform(imu_to_lidar)
        # 从ground truth位姿获取变换矩阵
        _, tx, ty, tz, qx, qy, qz, qw = gt_pose
        gt_transform = pose_to_transformation_matrix(tx, ty, tz, qx, qy, qz, qw)
        # 根据C++代码: transformIn = transformIn * R_gt_l
        # 将ground truth变换矩阵乘以R_gt_l，得到从world到lidar的变换
        transform = np.dot(gt_transform, R_gt_l)
        # 对点云应用变换
        transformed_pcd = pcd_in_lidar.transform(transform)
        # 添加到合并的点云中
        merged_pcd += transformed_pcd
        valid_frames += 1
    print(f"处理了 {valid_frames} 帧有效点云，跳过了 {skipped_frames} 帧")
    # 对合并的点云进行降采样以减小文件大小
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size=0.3)
    # 保存合并的点云
    o3d.io.write_point_cloud(output_file, merged_pcd)
    print(f"合并地图已保存至 {output_file}")
if __name__ == '__main__':
    main()