import rosbag
from geographiclib.geodesic import Geodesic
import math
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

# 从rosbag中读取GNSS数据并转换为TUM格式
def extract_gnss_to_tum_format(bag_file, origin, tum_file, variance_threshold):
    try:
        bag = rosbag.Bag(bag_file, 'r')
        tum_data = []
        last_x, last_y = None, None
        for topic, msg, t in bag.read_messages(topics=['/3dm_ins/gnss1/fix']):
            # 获取经纬度和高度
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            # 转换为局部坐标
            x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
            # 计算yaw角
            if last_x is not None and last_y is not None:
                yaw = compute_yaw(last_x, last_y, x, y)
                qx, qy, qz, qw = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
            else:
                qx, qy, qz, qw = (0, 0, 0, 1)
            last_x, last_y = x, y
            # 检查斜方差
            variance = msg.position_covariance[0]
            if variance < variance_threshold:
                tum_data.append(f"{t.to_sec()} {x} {y} {z} {qx} {qy} {qz} {qw}")
        # 保存为TUM格式的文件
        with open(tum_file, 'w') as file:
            file.write("\n".join(tum_data))
        print(f"Data successfully saved to {tum_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()


# 从rosbag中读取GNSS数据并转换为TUM格式
def extract_gnss_to_tum_format(bag_file, origin, topics, output_files, variance_threshold):
    assert len(topics) == len(output_files), "Number of topics should match the number of output files."
    for topic, tum_file in zip(topics, output_files):
        try:
            bag = rosbag.Bag(bag_file, 'r')
            tum_data = []
            last_x, last_y = None, None
            for _, msg, t in bag.read_messages(topics=[topic]):
                # 获取经纬度和高度
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                # 转换为局部坐标
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # 计算yaw角
                if last_x is not None and last_y is not None:
                    yaw = compute_yaw(last_x, last_y, x, y)
                    qx, qy, qz, qw = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
                else:
                    qx, qy, qz, qw = (0, 0, 0, 1)
                last_x, last_y = x, y
                # 检查斜方差
                variance = msg.position_covariance[0]
                if variance < variance_threshold:
                    tum_data.append(f"{t.to_sec()} {x} {y} {z} {qx} {qy} {qz} {qw}")
            # 保存为TUM格式的文件
            with open(tum_file, 'w') as file:
                file.write("\n".join(tum_data))
            print(f"Data for topic {topic} successfully saved to {tum_file}")
        except Exception as e:
            print(f"An error occurred while processing topic {topic}: {e}")

# Extract GNSS data from a rosbag and convert it to TUM format
def extract_gnss_to_tum_format(bag_file, origin, configurations):
    for config in configurations:
        topic = config['topic']
        tum_file = config['output_file']
        variance_threshold = config['variance_threshold']
        try:
            bag = rosbag.Bag(bag_file, 'r')
            tum_data = []
            last_x, last_y = None, None
            for _, msg, t in bag.read_messages(topics=[topic]):
                # Get latitude, longitude, and altitude
                lat = msg.latitude
                lon = msg.longitude
                alt = msg.altitude
                # Convert to local coordinates
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # Compute the yaw angle
                if last_x is not None and last_y is not None:
                    yaw = compute_yaw(last_x, last_y, x, y)
                    qx, qy, qz, qw = (0, 0, math.sin(yaw/2), math.cos(yaw/2))
                else:
                    qx, qy, qz, qw = (0, 0, 0, 1)
                last_x, last_y = x, y
                # Check the variance
                variance = msg.position_covariance[0]
                if variance < variance_threshold:
                    tum_data.append(f"{t.to_sec()} {x} {y} {z} {qx} {qy} {qz} {qw}")
            # Save to a TUM format file
            with open(tum_file, 'w') as file:
                file.write("\n".join(tum_data))
            print(f"Data for topic {topic} successfully saved to {tum_file}")
        except Exception as e:
            print(f"An error occurred while processing topic {topic}: {e}")


# 已知的原点经纬高数据
origin = {'lat': 22.8901710523756, 'lon': 113.47589813609757, 'alt': 0.07678306745241956}  # 请替换为实际值
# Rosbag文件路径
bag_file = 'sample.bag'  # 请替换为实际路径

# Configurations for topics, output files, and variance thresholds
configurations = [
    {
        'topic': '/3dm_ins/gnss1/fix',
        'output_file': 'sample_gnss1_tum_format.txt',
        'variance_threshold': 0.1
    },
    {
        'topic': '/3dm_ins/gnss2/fix',
        'output_file': 'sample_gnss2_tum_format.txt',
        'variance_threshold': 0.2
    },
    {
        'topic': '/imu/nav_sat_fix',
        'output_file': 'sample_gnss_sbg_tum_format.txt',
        'variance_threshold': 1.0
    }
]
extract_gnss_to_tum_format(bag_file, origin, configurations)