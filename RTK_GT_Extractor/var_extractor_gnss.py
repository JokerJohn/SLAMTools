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
        tum_file_with_variance = tum_file.replace('.txt', '_with_variance.txt')
        try:
            bag = rosbag.Bag(bag_file, 'r')
            tum_data = []
            tum_data_with_variance = []
            last_x, last_y = None, None
            for _, msg, t in bag.read_messages(topics=[topic]):
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                variance_x = msg.position_covariance[0]
                variance_y = msg.position_covariance[4]
                variance_z = msg.position_covariance[8]
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # only consider the yaw angle when there are at least two points in the same plane
                if last_x is not None and last_y is not None:
                    yaw = compute_yaw(last_x, last_y, x, y)
                    qx, qy, qz, qw = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
                else:
                    qx, qy, qz, qw = (0, 0, 0, 1)
                last_x, last_y = x, y

                tum_data.append("{} {} {} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(t.to_sec(), x, y, z, qx, qy, qz, qw))
                tum_data_with_variance.append("{} {} {} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(t.to_sec(), x, y, z, qx, qy, qz, qw, variance_x, variance_y, variance_z))
            with open(tum_file, 'w') as file:
                file.write("\n".join(tum_data))
            with open(tum_file_with_variance, 'w') as file:
                file.write("\n".join(tum_data_with_variance))
            print("Data successfully saved to {} and {}".format(tum_file, tum_file_with_variance))
        except Exception as e:
            print("An error occurred while processing topic {}: {}".format(topic, e))
    return results



# 请替换为实际GNSS origin
origin = {'lat': 22.8901710523756, 'lon': 113.47589813609757, 'alt': 0.07678306745241956}  

# Path to the rosbag file
bag_file = 'sample.bag'  # Replace with your rosbag path

# Configurations for topics, output files, and variance thresholds
# only save the gnss points with variance lower than the threshold
# only support NavSatFix message type
configurations = [
    {
        'topic': '/3dm_ins/gnss1/fix',
        'output_file': 'output_gnss1_tum.txt',
        'variance_threshold': 0.8
    },
    {
        'topic': '/3dm_ins/gnss2/fix',
        'output_file': 'output_gnss2_tum.txt',
        'variance_threshold': 0.1
    },
    {
        'topic': '/imu/nav_sat_fix',
        'output_file': 'output_gnss_sbg_tum.txt',
        'variance_threshold': 100
    }
]
# Extract data from the rosbag
# save tum format and tum with cov data to file
extracted_data = extract_gnss_data_with_xy_variance(bag_file, origin, configurations)
