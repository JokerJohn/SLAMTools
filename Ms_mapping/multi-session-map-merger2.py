import os
import numpy as np
import open3d as o3d
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm

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
    _, tx, ty, tz, qx, qy, qz, qw = pose
    rotation_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    translation_vector = np.array([tx, ty, tz])
    pcd_file = os.path.join(pcd_folder, 'key_point_frame', f'{i}.pcd')
    if not os.path.exists(pcd_file):
        print(f"Warning: PCD file not found: {pcd_file}")
        return None
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd_down = pcd.voxel_down_sample(voxel_size=0.25)
    pcd_down.rotate(rotation_matrix, center=(0, 0, 0))
    pcd_down.translate(translation_vector)
    return pcd_down

def divide_sessions_by_folders(new_pcd_folder, old_pcd_folders):
    sessions = []
    all_folders = [new_pcd_folder] + old_pcd_folders
    for folder in all_folders:
        pcd_folder = os.path.join(folder, 'key_point_frame')
        if os.path.exists(pcd_folder):
            pcd_files = [f for f in os.listdir(pcd_folder) if f.endswith('.pcd')]
            pcd_indices = sorted([int(f.split('.')[0]) for f in pcd_files])
            if pcd_indices:
                sessions.append((folder, min(pcd_indices), max(pcd_indices)))
    sessions.sort(key=lambda x: x[1])  # Sort sessions by start index
    return sessions

# def process_session(session_info, poses, output_folder, session_num, num_threads):
#     folder, start_idx, end_idx = session_info
#     session_poses = poses[start_idx:end_idx+1]
#     print(f'\nProcessing session {session_num}...')
#     print(f'Folder: {folder}')
#     print(f'Pose range: {start_idx} to {end_idx}')
#     print(f'Number of poses: {len(session_poses)}')
#     merged_pcd = o3d.geometry.PointCloud()
#     with ThreadPoolExecutor(max_workers=num_threads) as executor:
#         futures = [executor.submit(process_pcd, i, pose, folder) for i, pose in enumerate(session_poses, start=start_idx)]
#         with tqdm(total=len(session_poses), unit='pcd') as progress_bar:
#             for future in as_completed(futures):
#                 pcd_down = future.result()
#                 if pcd_down is not None:
#                     merged_pcd += pcd_down
#                 progress_bar.update(1)
#     print(f'Downsampling merged point cloud for session {session_num}...')
#     merged_pcd_down = merged_pcd.voxel_down_sample(voxel_size=0.05)
#     output_file = os.path.join(output_folder, f'merged_map_session_{session_num}.pcd')
#     print(f'Saving merged point cloud for session {session_num} to {output_file}...')
#     o3d.io.write_point_cloud(output_file, merged_pcd_down)

def process_session(session_info, poses, output_folder, session_num, num_threads):
    folder, start_idx, end_idx = session_info
    session_poses = poses[start_idx:end_idx+1]
    print(f'\nProcessing session {session_num}...')
    print(f'Folder: {folder}')
    print(f'Pose range: {start_idx} to {end_idx}')
    print(f'Number of poses: {len(session_poses)}')
    merged_pcd = o3d.geometry.PointCloud()
    processed_count = 0
    with ThreadPoolExecutor(max_workers=num_threads) as executor:
        futures = [executor.submit(process_pcd, i, pose, folder) for i, pose in enumerate(session_poses, start=start_idx)]
        with tqdm(total=len(session_poses), unit='pcd') as progress_bar:
            for future in as_completed(futures):
                pcd_down = future.result()
                if pcd_down is not None:
                    merged_pcd += pcd_down
                    processed_count += 1
                progress_bar.update(1)
    print(f'Successfully processed {processed_count} out of {len(session_poses)} point clouds.')
    if processed_count == 0:
        print(f'Warning: No point clouds were successfully processed for session {session_num}.')
        return None
    print(f'Downsampling merged point cloud for session {session_num}...')
    merged_pcd_down = merged_pcd.voxel_down_sample(voxel_size=0.10)
    output_file = os.path.join(output_folder, f'merged_map_session_{session_num}.pcd')
    print(f'Saving merged point cloud for session {session_num} to {output_file}...')
    o3d.io.write_point_cloud(output_file, merged_pcd_down)
    return merged_pcd_down

def main():
    # Configuration
    new_pcd_folder = '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4-NG'

    old_pcd_folders = [
        '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3-IA4',
        '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1-IA3',
        '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1-PK1',
        '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1-CC1',
        '/media/xchu/UBUNTU 20_0/results_ms/CP05-CP02-CS1',
        '/home/xchu/data/pose_slam_prior_result/CP05-CP02',
        # Add more old folders as needed
    ]

    pose_file = os.path.join(new_pcd_folder, 'optimized_poses_tum.txt')
    output_folder = os.path.join(new_pcd_folder, 'merged_results')
    os.makedirs(output_folder, exist_ok=True)
    num_threads = 8
    print('Reading pose file...')
    poses = read_tum_pose_file(pose_file)
    total_poses = len(poses)
    print(f'Total poses in file: {total_poses}')
    print('Dividing sessions based on folders...')
    sessions = divide_sessions_by_folders(new_pcd_folder, old_pcd_folders)
    print(f'Number of sessions: {len(sessions)}')
    
    for i, session_info in enumerate(sessions):
        print(f"\nSession {i+1}:")
        print(f"  Folder: {session_info[0]}")
        print(f"  Start index: {session_info[1]}")
        print(f"  End index: {session_info[2]}")

    total_map = o3d.geometry.PointCloud()
    for i, session_info in enumerate(sessions):
        session_map = process_session(session_info, poses, output_folder, i+1, num_threads)
        if session_map is not None:
            total_map += session_map
        else:
            print(f"Warning: Session {i+1} did not produce a valid point cloud. Skipping for total map.")
    if total_map.is_empty():
        print("Error: No valid point clouds were processed. Cannot create total map.")
    else:
        print('\nProcessing total map...')
        print('Downsampling total merged point cloud...')
        total_map_down = total_map.voxel_down_sample(voxel_size=0.1)
        total_map_file = os.path.join(output_folder, 'total_merged_map.pcd')
        print(f'Saving total merged point cloud to {total_map_file}...')
        o3d.io.write_point_cloud(total_map_file, total_map_down)
    print('All sessions and total map processed successfully.')


    

if __name__ == '__main__':
    main()