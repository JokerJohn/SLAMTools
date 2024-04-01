import os
import numpy as np
import open3d as o3d
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm

# pip install tqdm
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

def process_pcd(i, pose, pcd_folder):
    pcd_file = os.path.join(pcd_folder, f'{i}.pcd')
    if not os.path.exists(pcd_file):
        print(f'Warning: Point cloud file not found for pose {i}. Skipping.')
        return None
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd_down = pcd.voxel_down_sample(voxel_size=0.1)
    _, tx, ty, tz, qx, qy, qz, qw = pose
    rotation_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    translation_vector = np.array([tx, ty, tz])
    pcd_down.rotate(rotation_matrix, center=(0, 0, 0))
    pcd_down.translate(translation_vector)
    return pcd_down

def main():
    folder = '/home/xchu/data/prior_map/20230710-final/'
    pose_file = folder + 'optimized_odom_tum.txt'
    pcd_folder = folder + 'key_point_frame'
    output_file = folder + 'merged_map.pcd'

    num_threads = 8
    print('Reading pose file...')
    poses = read_tum_pose_file(pose_file)
    total_poses = len(poses)
    print(f'Total poses: {total_poses}')
    merged_pcd = o3d.geometry.PointCloud()
    print('Processing point cloud files...')
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = [executor.submit(process_pcd, i, pose, pcd_folder) for i, pose in enumerate(poses)]
        with tqdm(total=total_poses, unit='pcd') as progress_bar:
            for future in as_completed(futures):
                pcd_down = future.result()
                if pcd_down is not None:
                    merged_pcd += pcd_down
                progress_bar.update(1)
    print('Downsampling merged point cloud...')
    merged_pcd = merged_pcd.voxel_down_sample(voxel_size=0.2)
    print(f'Saving merged point cloud to {output_file}...')
    o3d.io.write_point_cloud(output_file, merged_pcd)
    print('Done.')

if __name__ == '__main__':
    main()