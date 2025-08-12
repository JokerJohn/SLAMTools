#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Bag INSPVAX Trajectory Quality Visualization Tool
Combines rosbag data extraction with NCLT-style covariance visualization
"""

import rosbag
from geographiclib.geodesic import Geodesic
import os
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Patch

class INSPVAXTrajectoryVisualizer:
    """INSPVAX trajectory extraction and visualization class"""
    
    def __init__(self, bag_file, topic_name='fixposition/inspvax'):
        self.bag_file = bag_file
        self.topic_name = topic_name
        self.origin = None
        self.trajectory_data = []
        self.covariance_data = []
        
        # Output file paths
        base_name = os.path.splitext(bag_file)[0]
        self.gt_file = base_name + "_gt.csv"
        self.cov_file = base_name + "_cov.csv"
        self.tum_file = base_name + "_tum.txt"
        self.pdf_file = base_name + "_trajectory_quality.pdf"
        
    def convert_lat_lon_alt_to_xyz(self, lat, lon, alt):
        """Convert latitude, longitude, altitude to local ENU coordinates"""
        if self.origin is None:
            raise ValueError("Origin not set")
            
        geod = Geodesic.WGS84
        g = geod.Inverse(self.origin['lat'], self.origin['lon'], lat, lon)
        distance = g['s12']
        azimuth = math.radians(g['azi1'])
        
        # ENU coordinate system: East=x, North=y, Up=z
        x = distance * math.sin(azimuth)  # East
        y = distance * math.cos(azimuth)  # North
        z = alt - self.origin['alt']      # Height difference
        
        return x, y, z
    
    def euler_to_quaternion(self, roll, pitch, azimuth):
        """Convert Euler angles to quaternion"""
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        # azimuth is from north clockwise, convert to standard yaw (from east counter-clockwise)
        yaw_rad = math.radians(90 - azimuth)
        
        rotation = R.from_euler('zyx', [yaw_rad, pitch_rad, roll_rad])
        quat = rotation.as_quat()  # [qx, qy, qz, qw]
        
        return quat
    
    def extract_from_rosbag(self, std_dev_threshold=0.1, valid_pose_types=[50], 
                           filter_mode='both', use_first_as_origin=True,
                           predefined_origin=None):
        """
        Extract INSPVAX data from rosbag
        
        Parameters:
        - std_dev_threshold: position std threshold (meters) for TUM file
        - valid_pose_types: valid pose_type values for TUM file
        - filter_mode: filter mode for TUM file ('std_only', 'pose_type_only', 'both')
        - use_first_as_origin: whether to use first valid point as origin
        - predefined_origin: predefined origin coordinates
        """
        print(f"\n{'='*60}")
        print(f"Extracting data from rosbag: {self.bag_file}")
        print(f"Topic: {self.topic_name}")
        print(f"Filter for TUM file:")
        print(f"  - Mode: {filter_mode}")
        print(f"  - Std threshold: {std_dev_threshold} m")
        print(f"  - Valid pose_types: {valid_pose_types}")
        print(f"Use first point as origin: {use_first_as_origin}")
        print(f"{'='*60}")
        
        # If not using first point as origin, use predefined origin
        if not use_first_as_origin and predefined_origin:
            self.origin = predefined_origin
            print(f"Using predefined origin: lat={self.origin['lat']:.8f}, lon={self.origin['lon']:.8f}, alt={self.origin['alt']:.3f}")
        
        try:
            bag = rosbag.Bag(self.bag_file, 'r')
            
            # Try different topic name formats
            topics_to_try = [
                self.topic_name,
                self.topic_name.lstrip('/'),
                '/' + self.topic_name.lstrip('/')
            ]
            
            message_count = 0
            valid_count = 0
            tum_count = 0
            first_valid_message = True
            
            for try_topic in topics_to_try:
                bag.close()
                bag = rosbag.Bag(self.bag_file, 'r')
                
                for topic, msg, t in bag.read_messages(topics=[try_topic]):
                    message_count += 1
                    
                    if message_count == 1:
                        print(f"Successfully reading messages from topic: {try_topic}")
                    
                    # Get position standard deviations (in meters)
                    lat_std = msg.latitude_deviation
                    lon_std = msg.longitude_deviation
                    height_std = msg.height_deviation
                    
                    # Get attitude standard deviations
                    roll_std = msg.roll_deviation
                    pitch_std = msg.pitch_deviation
                    azimuth_std = msg.azimuth_deviation
                    
                    # Get pose_type
                    pose_type = -1  # default value
                    if hasattr(msg, 'pos_type'):
                        pose_type = msg.pos_type
                    
                    # Get position
                    lat = msg.latitude
                    lon = msg.longitude
                    alt = msg.height
                    
                    # Set origin using first valid message
                    if first_valid_message:
                        if use_first_as_origin:
                            self.origin = {
                                'lat': lat,
                                'lon': lon,
                                'alt': alt
                            }
                            print(f"\nSet origin from first point: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.3f}")
                            print(f"  Origin pose_type: {pose_type}")
                        first_valid_message = False
                        if use_first_as_origin:
                            continue  # Skip the first point as it will be (0,0,0)
                    
                    # Convert to local coordinates
                    x, y, z = self.convert_lat_lon_alt_to_xyz(lat, lon, alt)
                    
                    # Save trajectory data
                    timestamp_us = int(t.to_sec() * 1e6)
                    self.trajectory_data.append([
                        timestamp_us, x, y, z, 
                        msg.roll, msg.pitch, msg.azimuth
                    ])
                    
                    # Save covariance data (ALL points for analysis)
                    # Format: timestamp(us), x, y, z, lat_std, lon_std, height_std, 
                    #         roll(rad), pitch(rad), yaw(rad), pose_type
                    self.covariance_data.append([
                        timestamp_us, x, y, z,
                        lat_std, lon_std, height_std,  # Individual std deviations
                        math.radians(msg.roll),
                        math.radians(msg.pitch),
                        math.radians(msg.azimuth),
                        pose_type
                    ])
                    
                    valid_count += 1
                    
                    # Check if this point qualifies for TUM file
                    # Condition: any std < threshold AND pose_type in valid list
                    min_std = min(lat_std, lon_std, height_std)
                    if filter_mode == 'std_only':
                        qualifies_for_tum = min_std < std_dev_threshold
                    elif filter_mode == 'pose_type_only':
                        qualifies_for_tum = pose_type in valid_pose_types
                    elif filter_mode == 'both':
                        qualifies_for_tum = (min_std < std_dev_threshold) and (pose_type in valid_pose_types)
                    else:
                        qualifies_for_tum = True
                    
                    if qualifies_for_tum:
                        tum_count += 1
                    
                    if valid_count % 100 == 0:
                        print(f"Processed {message_count} messages, saved {valid_count} points")
                        print(f"  Latest: pose_type={pose_type}, min_std={min_std:.3f}m")
                        print(f"  Points qualifying for TUM: {tum_count}")
                
                if message_count > 0:
                    break
                    
            bag.close()
            
            print(f"\nExtraction complete:")
            print(f"  Total messages: {message_count}")
            print(f"  Points saved to cov file: {valid_count}")
            print(f"  Points qualifying for TUM: {tum_count}")
            
            return valid_count > 0
            
        except Exception as e:
            print(f"Error during extraction: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def save_data_files(self):
        """Save extracted data to text files"""
        if not self.trajectory_data:
            print("Warning: No data to save")
            return False
        
        # Save trajectory data (NCLT format groundtruth.csv)
        with open(self.gt_file, 'w') as f:
            for row in self.trajectory_data:
                # Convert angles to radians for saving
                row_rad = row.copy()
                row_rad[4] = math.radians(row[4])  # roll
                row_rad[5] = math.radians(row[5])  # pitch
                row_rad[6] = math.radians(row[6])  # yaw
                f.write(','.join(map(str, row_rad)) + '\n')
        print(f"Trajectory data saved to: {self.gt_file}")
        
        # Save covariance data (includes position and attitude)
        with open(self.cov_file, 'w') as f:
            for row in self.covariance_data:
                f.write(','.join(map(str, row)) + '\n')
        print(f"Covariance data saved to: {self.cov_file}")
        
        return True
    
    def load_data_files(self):
        """Load data from text files (avoid re-reading rosbag)"""
        try:
            # Load trajectory data
            data = np.loadtxt(self.gt_file, delimiter=",")
            self.trajectory_data = data.tolist()
            
            # Load covariance data
            cov = np.loadtxt(self.cov_file, delimiter=",")
            self.covariance_data = cov.tolist()
            
            print(f"Loaded {len(self.trajectory_data)} trajectory points from files")
            return True
            
        except Exception as e:
            print(f"Failed to load data files: {e}")
            return False
    
    def generate_tum_format(self, std_threshold=0.1, valid_pose_types=[50], filter_mode='both'):
        """
        Generate TUM format file with high-quality points only
        
        Parameters:
        - std_threshold: max std deviation for any axis (meters)
        - valid_pose_types: list of valid pose_type values
        - filter_mode: 'std_only', 'pose_type_only', or 'both'
        """
        if not self.covariance_data:
            print("Warning: No data to generate TUM format")
            return
        
        tum_points = []
        
        for cov in self.covariance_data:
            t = cov[0] / 1e6  # Convert to seconds
            x, y, z = cov[1:4]
            lat_std, lon_std, height_std = cov[4:7]
            roll_rad, pitch_rad, yaw_rad = cov[7:10]
            pose_type = cov[10]
            
            # Check if point qualifies for TUM
            min_std = min(lat_std, lon_std, height_std)
            
            if filter_mode == 'std_only':
                qualifies = min_std < std_threshold
            elif filter_mode == 'pose_type_only':
                qualifies = pose_type in valid_pose_types
            elif filter_mode == 'both':
                qualifies = (min_std < std_threshold) and (pose_type in valid_pose_types)
            else:
                qualifies = True
            
            if qualifies:
                # Convert radians back to degrees for quaternion conversion
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
                yaw_deg = math.degrees(yaw_rad)
                
                quat = self.euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
                qx, qy, qz, qw = quat
                
                tum_points.append(f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
        
        # Save TUM file
        with open(self.tum_file, 'w') as f:
            f.write('\n'.join(tum_points))
        
        print(f"TUM format data saved to: {self.tum_file}")
        print(f"  Total points in cov file: {len(self.covariance_data)}")
        print(f"  Points in TUM file (high quality): {len(tum_points)}")
        print(f"  Filter criteria: min_std < {std_threshold}m AND pose_type in {valid_pose_types}")
    
    def print_statistics(self):
        """Print trajectory statistics to terminal"""
        if not self.covariance_data:
            return
        
        cov_array = np.array(self.covariance_data)
        x = cov_array[:, 1]  # East
        y = cov_array[:, 2]  # North
        z = cov_array[:, 3]  # Up
        lat_std = cov_array[:, 4]
        lon_std = cov_array[:, 5]
        height_std = cov_array[:, 6]
        pose_types = cov_array[:, 10]
        
        print(f"\n{'='*60}")
        print("Trajectory Statistics")
        print(f"{'='*60}")
        print(f"Total data points: {len(x)}")
        print(f"Trajectory length: {self._calculate_trajectory_length():.1f} m")
        
        # Position standard deviation statistics
        print(f"\nPosition Standard Deviation Statistics:")
        print(f"  Latitude std:  mean={np.mean(lat_std):.3f}m, max={np.max(lat_std):.3f}m")
        print(f"  Longitude std: mean={np.mean(lon_std):.3f}m, max={np.max(lon_std):.3f}m")
        print(f"  Height std:    mean={np.mean(height_std):.3f}m, max={np.max(height_std):.3f}m")
        
        # Minimum std for each point
        min_stds = np.minimum(np.minimum(lat_std, lon_std), height_std)
        print(f"\nMinimum std (best of 3 axes):")
        print(f"  Mean: {np.mean(min_stds):.3f} m")
        print(f"  Median: {np.median(min_stds):.3f} m")
        print(f"  Min: {np.min(min_stds):.3f} m")
        print(f"  Max: {np.max(min_stds):.3f} m")
        
        # Pose type distribution
        unique_types, counts = np.unique(pose_types, return_counts=True)
        print(f"\nPose Type Distribution:")
        for pt, cnt in zip(unique_types, counts):
            percentage = 100 * cnt / len(pose_types)
            if int(pt) == 50:
                status = "RTK Fixed"
            elif int(pt) == 34:
                status = "RTK Float"
            elif int(pt) == 0:
                status = "Failed"
            else:
                status = f"Type {int(pt)}"
            print(f"  {status}: {cnt} points ({percentage:.1f}%)")
        
        # Quality assessment
        rtk_fixed_ratio = np.sum(pose_types == 50) / len(pose_types) * 100
        print(f"\nRTK Fixed ratio: {rtk_fixed_ratio:.1f}%")
        
        # Points qualifying for TUM
        high_quality = np.sum((min_stds < 0.1) & (pose_types == 50))
        print(f"\nHigh-quality points (min_std<0.1m AND RTK Fixed): {high_quality} ({100*high_quality/len(pose_types):.1f}%)")
        
        if rtk_fixed_ratio > 95:
            print("Overall quality: Excellent")
        elif rtk_fixed_ratio > 80:
            print("Overall quality: Good")
        elif rtk_fixed_ratio > 60:
            print("Overall quality: Fair")
        else:
            print("Overall quality: Poor")
        print(f"{'='*60}\n")
    
    def visualize_trajectory_quality(self):
        """Create trajectory visualization with quality coloring (NCLT style)"""
        if not self.covariance_data:
            print("Warning: No data to visualize")
            return
        
        # Set Matplotlib parameters (journal quality)
        mpl.rcParams.update({
            'font.family': 'Times New Roman',
            'font.size': 12,
            'axes.labelsize': 12,
            'axes.titlesize': 14,
            'xtick.labelsize': 10,
            'ytick.labelsize': 10,
            'figure.dpi': 300,
            'savefig.dpi': 300,
            'pdf.fonttype': 42,
            'ps.fonttype': 42
        })
        
        # Prepare data
        cov_array = np.array(self.covariance_data)
        
        x = cov_array[:, 1]  # East
        y = cov_array[:, 2]  # North
        lat_std = cov_array[:, 4]
        lon_std = cov_array[:, 5]
        height_std = cov_array[:, 6]
        pose_types = cov_array[:, 10]
        
        # Calculate minimum std for each point
        min_stds = np.minimum(np.minimum(lat_std, lon_std), height_std)
        
        # Create figure
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Subplot 1: Color by pose_type (50=green, 34=yellow, 0=red)
        colors = []
        for pt in pose_types:
            if pt == 50:
                colors.append('green')  # RTK Fixed
            elif pt == 34:
                colors.append('gold')   # RTK Float
            else:
                colors.append('red')     # Failed or other
        
        # Plot with different colors for each pose type
        for pt, color, label in [(50, 'green', 'RTK Fixed'), 
                                  (34, 'gold', 'RTK Float'), 
                                  (0, 'red', 'Failed')]:
            mask = pose_types == pt
            if np.any(mask):
                ax1.scatter(x[mask], y[mask], c=color, s=2, alpha=0.8, 
                          edgecolors='none', label=label)
        
        ax1.set_title('Trajectory with INS Status', fontsize=13)
        ax1.set_xlabel('East (m)', fontsize=11)
        ax1.set_ylabel('North (m)', fontsize=11)
        ax1.set_aspect('equal')
        ax1.grid(True, linestyle='--', alpha=0.3)
        ax1.legend(loc='upper right', markerscale=3, fontsize=10)
        
        # Subplot 2: Color by minimum standard deviation
        scatter2 = ax2.scatter(x, y, c=min_stds, cmap='plasma', 
                              s=2, alpha=0.8, edgecolors='none', 
                              vmin=0, vmax=0.2)  # Set range 0-0.2m
        cbar2 = plt.colorbar(scatter2, ax=ax2)
        cbar2.set_label('Min Std Dev (m)', fontsize=11)
        ax2.set_title('Trajectory with Position Uncertainty', fontsize=13)
        ax2.set_xlabel('East (m)', fontsize=11)
        ax2.set_ylabel('North (m)', fontsize=11)
        ax2.set_aspect('equal')
        ax2.grid(True, linestyle='--', alpha=0.3)

        # i want to adjust the space of two subfigures
        plt.subplots_adjust(wspace=0.1)
        
        # Overall title
        fig.suptitle(f'INSPVAX Trajectory Quality Visualization', 
                    fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        
        # Save as PDF
        plt.savefig(self.pdf_file, bbox_inches='tight')
        print(f"Visualization saved to: {self.pdf_file}")
        
        # Show figure (optional)
        plt.show()
    
    def _calculate_trajectory_length(self):
        """Calculate total trajectory length"""
        if len(self.covariance_data) < 2:
            return 0.0
        
        cov_array = np.array(self.covariance_data)
        positions = cov_array[:, 1:4]  # x, y, z
        
        total_length = 0.0
        for i in range(1, len(positions)):
            diff = positions[i] - positions[i-1]
            total_length += np.linalg.norm(diff)
        
        return total_length


def main(args):
    """Main function - using hard-coded parameters"""
    
    # ========== Configuration (hard-coded) ==========
    # bag_file = 'pandar_test.bag'  # rosbag file path
    bag_file = '2.bag'  # rosbag file path
    topic_name = 'fixposition/inspvax'  # INSPVAX message topic
    
    # Origin configuration
    # For 2.bag
    origin = {
        'lon': 114.39956273770704,
        'lat': 22.737523490958555,
        'alt': 44.555445971898735
    }
    
    # For pandar_test.bag
    # origin = {
    #     'lon': 114.05472049034813,
    #     'lat': 22.505656019639506,
    #     'alt': 2.8602906048971484
    # }
    
    # Filter parameters for TUM file
    std_dev_threshold = 0.1  # Position std threshold (meters)
    valid_pose_types = [50]  # Valid pose_type values (50=RTK Fixed)
    
    # Filter mode for TUM file:
    # filter_mode = 'std_only'       # Only use std filter
    # filter_mode = 'pose_type_only' # Only use pose_type filter
    filter_mode = 'both'           # Both conditions must be met (recommended)
    
    use_first_as_origin = False  # Whether to use first valid point as origin
    
    # Run mode selection
    # mode = 'extract'    # Extract from rosbag
    # mode = 'visualize'  # Only visualize existing data
    mode = 'auto'       # Auto mode: ask if data files exist
    # ========================================
    
    # Check if file exists
    if not os.path.exists(bag_file) and mode == 'extract':
        print(f"Error: Cannot find file {bag_file}")
        return 1
    
    print("\n" + "="*60)
    print("INSPVAX Trajectory Quality Visualization Tool")
    print("="*60)
    print(f"Bag file: {bag_file}")
    print(f"Run mode: {mode}")
    print("="*60)
    
    # Create visualizer instance
    visualizer = INSPVAXTrajectoryVisualizer(bag_file, topic_name)
    
    # Decide whether to extract data based on mode
    need_extract = False
    
    if mode == 'extract':
        need_extract = True
    elif mode == 'visualize':
        # Visualization only mode, try to load existing data
        if not (os.path.exists(visualizer.gt_file) and os.path.exists(visualizer.cov_file)):
            print("Error: Data files not found, please run in extract mode first")
            return 1
        need_extract = False
    elif mode == 'auto':
        # Auto mode
        if os.path.exists(visualizer.gt_file) and os.path.exists(visualizer.cov_file):
            print(f"\nFound existing data files:")
            print(f"  - {visualizer.gt_file}")
            print(f"  - {visualizer.cov_file}")
            user_input = input("\nRe-extract data from rosbag? (y/n, default n): ").strip().lower()
            need_extract = (user_input == 'y')
        else:
            print("\nData files not found, extracting from rosbag...")
            need_extract = True
    
    # Execute data extraction or loading
    if need_extract:
        # Check rosbag file
        if not os.path.exists(bag_file):
            print(f"Error: Cannot find rosbag file {bag_file}")
            return 1
            
        print(f"\nExtracting data from rosbag...")
        print(f"Topic: {topic_name}")
        print(f"TUM file filter configuration:")
        print(f"  - Mode: {filter_mode}")
        print(f"  - Std threshold: {std_dev_threshold} m")
        print(f"  - Valid pose_types: {valid_pose_types}")
        print(f"  - Use first point as origin: {use_first_as_origin}")
        if not use_first_as_origin:
            print(f"  - Predefined origin: ({origin['lat']:.6f}, {origin['lon']:.6f}, {origin['alt']:.1f})")
        
        success = visualizer.extract_from_rosbag(
            std_dev_threshold=std_dev_threshold,
            valid_pose_types=valid_pose_types,
            filter_mode=filter_mode,
            use_first_as_origin=use_first_as_origin,
            predefined_origin=origin if not use_first_as_origin else None
        )
        
        if not success:
            print("Data extraction failed!")
            return 1
        
        # Save data files
        visualizer.save_data_files()
    else:
        # Load existing data
        print(f"\nLoading data from files...")
        if not visualizer.load_data_files():
            print("Failed to load data!")
            return 1
    
    # Generate TUM format (with high-quality points only)
    print("\nGenerating TUM format file...")
    visualizer.generate_tum_format(
        std_threshold=std_dev_threshold,
        valid_pose_types=valid_pose_types,
        filter_mode=filter_mode
    )
    
    # Print statistics to terminal
    visualizer.print_statistics()
    
    # Create visualization
    print("Creating visualization PDF...")
    visualizer.visualize_trajectory_quality()
    
    print(f"\n{'='*60}")
    print("Processing complete! Generated files:")
    print(f"  - Trajectory data: {visualizer.gt_file}")
    print(f"  - Covariance data: {visualizer.cov_file} (all points)")
    print(f"  - TUM format: {visualizer.tum_file} (high-quality only)")
    print(f"  - Visualization PDF: {visualizer.pdf_file}")
    print(f"{'='*60}")
    
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))