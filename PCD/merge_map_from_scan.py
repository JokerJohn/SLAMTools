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
def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]
    ])
def main():
    folder = '/home/xchu/data/prior_map/20230710-final/'
    pose_file = folder + 'optimized_odom_tum.txt'
    pcd_folder = folder + 'key_point_frame'
    output_file = folder + 'merged_map.pcd'
    poses = read_tum_pose_file(pose_file)
    merged_pcd = o3d.geometry.PointCloud()
    for i, pose in enumerate(poses):
        pcd_file = os.path.join(pcd_folder, f'{i}.pcd')
        if not os.path.exists(pcd_file):
            print(f'Warning: Point cloud file not found for pose {i}. Skipping.')
            continue
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd_down = pcd.voxel_down_sample(voxel_size=0.2)
        _, tx, ty, tz, qx, qy, qz, qw = pose
        rotation_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
        translation_vector = np.array([tx, ty, tz])
        pcd_down.rotate(rotation_matrix, center=(0, 0, 0))
        pcd_down.translate(translation_vector)
        merged_pcd += pcd_down
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size=0.5)
    o3d.io.write_point_cloud(output_file, merged_pcd)

if __name__ == '__main__':
    main()