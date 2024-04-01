import laspy
import os
import time
import numpy as np
import open3d as o3d
# from pyntcloud import PyntCloud
# import pandas as pd


# def las_to_pyntcloud(file_path):
#     las = laspy.read(file_path)
#     points = np.vstack((las.x, las.y, las.z)).transpose()
#     # Extracting RGB and scaling values to [0, 1] range
#     colors = np.vstack((las.red, las.green, las.blue)).transpose() / 65535.0
#     # Construct the data for PyntCloud
#     data = {
#         "x": las.x,
#         "y": las.y,
#         "z": las.z,
#         "red": colors[:, 0],
#         "green": colors[:, 1],
#         "blue": colors[:, 2],
#         "intensity": las.intensity if hasattr(las, "intensity") else None
#         # "return_num": las.return_num if hasattr(las, "return_num") else None,
#         # "scan_dir_flag": las.scan_dir_flag if hasattr(las, "scan_dir_flag") else None,
#         # "scan_angle_rank": las.scan_angle_rank if hasattr(las, "scan_angle_rank") else None,
#         # "edge_of_flight_line": las.edge_of_flight_line if hasattr(las, "edge_of_flight_line") else None,
#         # "pt_src_id": las.pt_src_id if hasattr(las, "pt_src_id") else None
#     }
#     cloud = PyntCloud(pd.DataFrame(data))
#     return cloud


def las_to_open3d_pointcloud(file_path, use_alternative=False):
    """Convert LAS file with RGB and other attributes to Open3D PointCloud."""
    if not use_alternative:
        with laspy.open(file_path) as las:
            points = []
            colors = []
            attributes = {}
            for points_chunk, colors_chunk in read_las_chunks(las):
                points.extend(points_chunk)
                colors.extend(colors_chunk)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)  # Adding RGB info to the point cloud
            # Add intensity to the attributes dictionary (if it exists)
            if hasattr(las, "intensity"):
                attributes["intensity"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "intensity")])
            # Add return number to the attributes dictionary (if it exists)
            if hasattr(las, "return_num"):
                attributes["return_num"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "return_num")])
            # Add scan direction to the attributes dictionary (if it exists)
            if hasattr(las, "scan_dir_flag"):
                attributes["scan_dir_flag"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "scan_dir_flag")])
            # Add scan angle to the attributes dictionary (if it exists)
            if hasattr(las, "scan_angle_rank"):
                attributes["scan_angle_rank"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "scan_angle_rank")])
            # Add edge of flight line to the attributes dictionary (if it exists)
            if hasattr(las, "edge_of_flight_line"):
                attributes["edge_of_flight_line"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "edge_of_flight_line")])
            # Add point source ID to the attributes dictionary (if it exists)
            if hasattr(las, "pt_src_id"):
                attributes["pt_src_id"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "pt_src_id")])
    else:
        las = laspy.read(file_path)
        points = np.vstack((las.x, las.y, las.z)).transpose()
        colors = np.vstack((las.red, las.green, las.blue)).transpose() / 65535.0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)  # Adding RGB info to the point cloud
        attributes = {}
        if hasattr(las, "intensity"):
            attributes["intensity"] = np.asarray(las.intensity)
        if hasattr(las, "return_num"):
            attributes["return_num"] = np.asarray(las.return_num)
        if hasattr(las, "scan_dir_flag"):
            attributes["scan_dir_flag"] = np.asarray(las.scan_dir_flag)
        if hasattr(las, "scan_angle_rank"):
            attributes["scan_angle_rank"] = np.asarray(las.scan_angle_rank)
        if hasattr(las, "edge_of_flight_line"):
            attributes["edge_of_flight_line"] = np.asarray(las.edge_of_flight_line)
        if hasattr(las, "pt_src_id"):
            attributes["pt_src_id"] = np.asarray(las.pt_src_id)
    return pcd, attributes

def read_las_chunks(las, chunk_size=1000000):
    for points in las.chunk_iterator(chunk_size):
        x, y, z = points.x.copy(), points.y.copy(), points.z.copy()
        points_chunk = np.vstack((x, y, z)).transpose()
        if hasattr(points, 'red') and hasattr(points, 'green') and hasattr(points, 'blue'):
            colors_chunk = np.vstack((points.red, points.green, points.blue)).transpose() / 65535.0
        else:
            colors_chunk = np.zeros((len(points_chunk), 3))
        yield points_chunk, colors_chunk
def read_las_attribute_chunks(las, attribute_name, chunk_size=1000000):
    for points in las.chunk_iterator(chunk_size):
        if hasattr(points, attribute_name):
            yield getattr(points, attribute_name)
        else:
            yield np.zeros(len(points))

def read_las(file_path):
    """读取LAS文件 using open3d"""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd
def save_pcd(pcd, file_path):
    """Save PCD to a file using open3d."""
    o3d.io.write_point_cloud(file_path, pcd)
def split_pcd(pcd, size_limit):
    """Split a PCD file if it's over a certain size using open3d."""
    # If the size of the PCD is under the limit, no need to split
    if len(pcd.points) * 3 * 4 < size_limit:  # each point has x,y,z as float (4 bytes each)
        return [pcd]
    # Split the PCD based on voxel downsampling
    downsampled = pcd.voxel_down_sample(voxel_size=0.1)
    # If downsampling still doesn't reduce size sufficiently, split the cloud into two
    if len(downsampled.points) * 3 * 4 > size_limit:
        half_size = len(downsampled.points) // 2
        first_half = o3d.geometry.PointCloud()
        second_half = o3d.geometry.PointCloud()
        first_half.points = o3d.utility.Vector3dVector(np.asarray(downsampled.points)[:half_size])
        second_half.points = o3d.utility.Vector3dVector(np.asarray(downsampled.points)[half_size:])
        return [first_half, second_half]
    return [downsampled]
def process_file(las_file_path, output_folder, use_alternative=False):
    print(f"Processing file {las_file_path}")
    pcd, _ = las_to_open3d_pointcloud(las_file_path, use_alternative)
    pcd_file_path = os.path.join(output_folder, os.path.basename(las_file_path).replace('.las', '.pcd').replace('.laz', '.pcd'))
    o3d.io.write_point_cloud(pcd_file_path, pcd)
def main(input_folder, output_folder, use_alternative=False):
    # 确保输入和输出文件夹都存在
    if not os.path.exists(input_folder):
        print(f"Error: Input folder '{input_folder}' does not exist.")
        return
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    las_files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) if f.endswith('.las') or f.endswith('.laz')]
    # Filtering out LAS files that have already been converted to PCD
    las_files = [f for f in las_files if not os.path.exists(os.path.join(output_folder, os.path.basename(f).replace('.las', '.pcd').replace('.laz', '.pcd')))]
    total_files = len(las_files)
    start_time = time.time()
    for idx, las_file in enumerate(las_files):
        process_file(las_file, output_folder, use_alternative)
        # Progress reporting
        elapsed_time = time.time() - start_time
        progress = (idx+1) / total_files * 100
        estimated_time = (elapsed_time / (idx+1)) * total_files
        remaining_time = estimated_time - elapsed_time
        print(f"Progress: {progress:.2f}% - Processed: {idx+1}/{total_files} - Elapsed: {elapsed_time:.2f}s - Estimated: {estimated_time:.2f}s - Remaining: {remaining_time:.2f}s")
    elapsed_time = time.time() - start_time
    print(f"Processed all {total_files} files in {elapsed_time:.2f} seconds.")
if __name__ == '__main__':
    input_folder = '/home/xchu/Downloads/gzpcd'
    main(input_folder, input_folder+'/pcds/', use_alternative=True)