import rosbag
from geographiclib.geodesic import Geodesic
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

# 将经纬高数据转换为局部坐标
def convert_lat_lon_alt_to_xyz(lat, lon, alt, origin):
    geod = Geodesic.WGS84
    g = geod.Inverse(origin['lat'], origin['lon'], lat, lon)
    distance = g['s12']
    azimuth = math.radians(g['azi1'])
    x = distance * math.cos(azimuth)
    y = distance * math.sin(azimuth)
    z = alt - origin['alt']
    return x, y, z
# 计算两个点之间的yaw角
def compute_yaw(x1, y1, x2, y2):
    delta_x = x2 - x1
    delta_y = y2 - y1
    yaw = math.atan2(delta_y, delta_x)
    return yaw

# Updated function to extract GNSS data from a rosbag, convert it to TUM format, and append x and y variances
def extract_gnss_data_with_xy_variance(bag_file, origin, configurations):
    results = {}
    for config in configurations:
        topic = config['topic']
        tum_file = config['output_file']
        try:
            bag = rosbag.Bag(bag_file, 'r')
            tum_data = []
            variances_x = []
            variances_y = []
            variances_z = []
            timestamps = []
            last_x, last_y = None, None
            for _, msg, t in bag.read_messages(topics=[topic]):
                # Get latitude, longitude, altitude, and variances
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                variance_x = msg.position_covariance[0]
                variance_y = msg.position_covariance[4]  # Extracting y variance
                variance_z = msg.position_covariance[8]
                # Convert to local coordinates
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # Compute the yaw angle
                if last_x is not None and last_y is not None:
                    yaw = compute_yaw(last_x, last_y, x, y)
                    qx, qy, qz, qw = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
                else:
                    qx, qy, qz, qw = (0, 0, 0, 1)
                last_x, last_y = x, y
                # Append to lists
                tum_data.append(f"{t.to_sec()} {x} {y} {z} {qx} {qy} {qz} {qw} {variance_x} {variance_y} {variance_z}")
                variances_x.append(variance_x)
                variances_y.append(variance_y)
                variances_z.append(variance_z)
                timestamps.append(t.to_sec())
            # Save to a TUM format file with appended variances
            with open(tum_file, 'w') as file:
                file.write("\n".join(tum_data))
            results[topic] = {
                "timestamps": timestamps,
                "variances_x": variances_x,
                "variances_y": variances_y,
                "variances_z": variances_z,
            }
        except Exception as e:
            print(f"An error occurred while processing topic {topic}: {e}")
    return results




origin = {'lat': 22.890390399999998, 'lon': 113.4753985, 'alt': -0.252}
# Path to the rosbag file
bag_file = '../RTK_GT_Extractor/sample.bag'  # Replace with your rosbag path
# bag_file = '/media/xchu/e81eaf80-d92c-413a-a503-1c9b35b19963/home/xchu/data/udi_fusion_2023-04-12-17-05-35.bag'  # Replace with your rosbag path

# Configurations for topics, output files, and variance thresholds
configurations = [
    {
        'topic': '/3dm_ins/gnss1/fix',
        'output_file': 'output_gnss1_tum_with_variance.txt',
        'variance_threshold': 10
    },
    {
        'topic': '/3dm_ins/gnss2/fix',
        'output_file': 'output_gnss2_tum_with_variance.txt',
        'variance_threshold': 10
    },
    {
        'topic': '/imu/nav_sat_fix',
        'output_file': 'output_gnss_sbg_tum_with_variance.txt',
        'variance_threshold': 10
    }
]
# Extract data from the rosbag
extracted_data = extract_gnss_data_with_xy_variance(bag_file, origin, configurations)
