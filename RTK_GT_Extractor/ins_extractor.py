import rosbag
from geographiclib.geodesic import Geodesic
import os
import sys
import math

# 函数将经纬度高度转换为局部坐标
def convert_lat_lon_alt_to_xyz(lat, lon, alt, origin):
    geod = Geodesic.WGS84
    g = geod.Inverse(origin['lat'], origin['lon'], lat, lon)
    # 计算地面坐标
    distance = g['s12']
    azimuth = math.radians(g['azi1'])  # 将方位角从度转换为弧度
    x = distance * math.cos(azimuth)
    y = distance * math.sin(azimuth)
    # 计算高度差
    z = alt - origin['alt']
    return x, y, z

# 从rosbag中读取odom数据并转换为TUM格式
def extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold):
    try:
        bag = rosbag.Bag(bag_file, 'r')
        tum_data = []
        for topic, msg, t in bag.read_messages(topics=['/3dm_ins/nav/odom']):
            # 获取位姿的斜方差
            variance = msg.pose.covariance[0]  # 提取所需的方差值，例如 x 的方差
            # 如果方差低于阈值，则处理odom数据
            if variance < variance_threshold:
                # 获取经纬度和高度
                lat = msg.pose.pose.position.y
                lon = msg.pose.pose.position.x
                alt = msg.pose.pose.position.z
                # 获取姿态四元数
                orientation = msg.pose.pose.orientation
                qx = orientation.x
                qy = orientation.y
                qz = orientation.z
                qw = orientation.w
                # 转换为局部坐标
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # 将数据添加到列表
                tum_data.append(f"{t.to_sec()} {x} {y} {z} {qx} {qy} {qz} {qw}")
        # 保存为TUM格式的文件
        with open(tum_file, 'w') as file:
            file.write("\n".join(tum_data))
        print(f"Data successfully saved to {tum_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()
# 已知的原点经纬高数据
origin = {'lat': 22.8901710523756, 'lon': 113.47589813609757, 'alt': 0.07678306745241956}  # 请替换为实际值
# Rosbag文件路径
bag_file = '../RTK_GT_Extractor/sample.bag'  # 请替换为实际路径
# 输出的TUM格式文件路径
tum_file = 'ins_tum_format.txt'  # 请替换为实际路径
# 斜方差阈值
variance_threshold = 0.1
extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold)
