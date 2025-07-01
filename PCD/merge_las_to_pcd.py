import laspy
import os
import time
import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional


def las_to_open3d_pointcloud(file_path: str, use_alternative: bool = False) -> Tuple[o3d.geometry.PointCloud, dict]:
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
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Add attributes if they exist
            if hasattr(las, "intensity"):
                attributes["intensity"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "intensity")])
            if hasattr(las, "return_num"):
                attributes["return_num"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "return_num")])
            if hasattr(las, "scan_dir_flag"):
                attributes["scan_dir_flag"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "scan_dir_flag")])
            if hasattr(las, "scan_angle_rank"):
                attributes["scan_angle_rank"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "scan_angle_rank")])
            if hasattr(las, "edge_of_flight_line"):
                attributes["edge_of_flight_line"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "edge_of_flight_line")])
            if hasattr(las, "pt_src_id"):
                attributes["pt_src_id"] = np.concatenate([chunk for chunk in read_las_attribute_chunks(las, "pt_src_id")])
    else:
        las = laspy.read(file_path)
        points = np.vstack((las.x, las.y, las.z)).transpose()
        colors = np.vstack((las.red, las.green, las.blue)).transpose() / 65535.0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
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


def read_las_chunks(las, chunk_size: int = 1000000):
    """Read LAS file in chunks to handle large files efficiently."""
    for points in las.chunk_iterator(chunk_size):
        x, y, z = points.x.copy(), points.y.copy(), points.z.copy()
        points_chunk = np.vstack((x, y, z)).transpose()
        if hasattr(points, 'red') and hasattr(points, 'green') and hasattr(points, 'blue'):
            colors_chunk = np.vstack((points.red, points.green, points.blue)).transpose() / 65535.0
        else:
            colors_chunk = np.zeros((len(points_chunk), 3))
        yield points_chunk, colors_chunk


def read_las_attribute_chunks(las, attribute_name: str, chunk_size: int = 1000000):
    """Read specific attribute from LAS file in chunks."""
    for points in las.chunk_iterator(chunk_size):
        if hasattr(points, attribute_name):
            yield getattr(points, attribute_name)
        else:
            yield np.zeros(len(points))


def merge_point_clouds(pcds: List[o3d.geometry.PointCloud], voxel_size: Optional[float] = None) -> o3d.geometry.PointCloud:
    """Merge multiple point clouds into one global point cloud.
    
    Args:
        pcds: List of Open3D point clouds to merge
        voxel_size: If specified, downsample the merged cloud with this voxel size
    
    Returns:
        Merged point cloud
    """
    # Create empty point cloud for merging
    merged_pcd = o3d.geometry.PointCloud()
    
    # Collect all points and colors
    all_points = []
    all_colors = []
    
    for pcd in pcds:
        if len(pcd.points) > 0:
            all_points.append(np.asarray(pcd.points))
            if len(pcd.colors) > 0:
                all_colors.append(np.asarray(pcd.colors))
            else:
                # If no colors, add default white colors
                all_colors.append(np.ones((len(pcd.points), 3)))
    
    # Merge all points and colors
    if all_points:
        merged_points = np.vstack(all_points)
        merged_colors = np.vstack(all_colors)
        
        merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
        merged_pcd.colors = o3d.utility.Vector3dVector(merged_colors)
        
        print(f"Merged point cloud contains {len(merged_pcd.points)} points")
        
        # Apply voxel downsampling if specified
        if voxel_size is not None and voxel_size > 0:
            print(f"Applying voxel downsampling with voxel size: {voxel_size}")
            merged_pcd = merged_pcd.voxel_down_sample(voxel_size=voxel_size)
            print(f"After downsampling: {len(merged_pcd.points)} points")
    
    return merged_pcd


def process_and_merge_las_files(input_folder: str, output_file: str, voxel_size: Optional[float] = None, 
                                use_alternative: bool = False, save_individual: bool = False,
                                output_folder: str = None):
    """Process all LAS files in a folder and merge them into a single PCD file.
    
    Args:
        input_folder: Path to folder containing LAS files
        output_file: Path for the output merged PCD file
        voxel_size: Voxel size for downsampling (None = no downsampling)
        use_alternative: Use alternative loading method
        save_individual: Whether to save individual PCD files as well
        output_folder: Folder for individual PCD files (if different from input_folder)
    """
    # Check if input folder exists
    if not os.path.exists(input_folder):
        print(f"Error: Input folder '{input_folder}' does not exist.")
        return
    
    # Get all LAS/LAZ files
    las_files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) 
                 if f.endswith('.las') or f.endswith('.laz')]
    
    if not las_files:
        print("No LAS/LAZ files found in the input folder.")
        return
    
    print(f"Found {len(las_files)} LAS/LAZ files to process")
    
    # Process each file and collect point clouds
    pcds = []
    start_time = time.time()
    
    for idx, las_file in enumerate(las_files):
        print(f"\nProcessing file {idx+1}/{len(las_files)}: {os.path.basename(las_file)}")
        
        try:
            # Convert LAS to Open3D point cloud
            pcd, attributes = las_to_open3d_pointcloud(las_file, use_alternative)
            pcds.append(pcd)
            
            # Optionally save individual PCD
            if save_individual:
                # Use output_folder if specified, otherwise use default
                if output_folder:
                    individual_output = output_folder
                else:
                    individual_output = os.path.join(input_folder, 'pcd')
                
                os.makedirs(individual_output, exist_ok=True)
                individual_file = os.path.join(individual_output, 
                                             os.path.basename(las_file).replace('.las', '.pcd').replace('.laz', '.pcd'))
                o3d.io.write_point_cloud(individual_file, pcd)
                print(f"  Saved individual PCD: {individual_file}")
            
            # Progress reporting
            elapsed_time = time.time() - start_time
            progress = (idx+1) / len(las_files) * 100
            estimated_time = (elapsed_time / (idx+1)) * len(las_files)
            remaining_time = estimated_time - elapsed_time
            print(f"  Progress: {progress:.2f}% - Elapsed: {elapsed_time:.2f}s - Remaining: {remaining_time:.2f}s")
            
        except Exception as e:
            print(f"  Error processing {las_file}: {str(e)}")
            continue
    
    if not pcds:
        print("No point clouds were successfully loaded.")
        return
    
    # Merge all point clouds
    print(f"\nMerging {len(pcds)} point clouds...")
    merged_pcd = merge_point_clouds(pcds, voxel_size)
    
    # Save merged point cloud
    print(f"Saving merged point cloud to: {output_file}")
    o3d.io.write_point_cloud(output_file, merged_pcd)
    
    total_time = time.time() - start_time
    print(f"\nCompleted! Total processing time: {total_time:.2f} seconds")
    print(f"Final merged point cloud contains {len(merged_pcd.points)} points")


def main():
    # Configuration
    input_folder = '/home/xchu/Downloads/helemt/2-AlignedPointCloud-20250701T082150Z-1-001'
    output_folder = os.path.join(input_folder, 'pcd')
    
    # Create output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        print(f"Created output folder: {output_folder}")
    
    # Output file for merged point cloud
    output_file = os.path.join(output_folder, 'merged_global.pcd')
    
    # Voxel size for downsampling (set to None for no downsampling)
    # You can adjust this value as needed
    voxel_size = 0.1  # meters
    
    # Whether to use alternative loading method
    use_alternative = True
    
    # Whether to save individual PCD files as well
    save_individual = True  # Changed to True to save individual files
    
    # Process and merge
    process_and_merge_las_files(
        input_folder=input_folder,
        output_file=output_file,
        voxel_size=voxel_size,
        use_alternative=use_alternative,
        save_individual=save_individual,
        output_folder=output_folder
    )


if __name__ == '__main__':
    main()