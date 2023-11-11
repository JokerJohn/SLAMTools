import rosbag
from geographiclib.geodesic import Geodesic
import os
import sys
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

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
# def extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold):
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



# 从rosbag中读取odom数据并转换为TUM格式
def extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold, use_ins_relative_orientation=True):
    try:
        bag = rosbag.Bag(bag_file, 'r')
        tum_data = []
        first_message = True
        T0 = np.eye(4)  # 初始位姿矩阵初始化为单位矩阵
        for topic, msg, t in bag.read_messages(topics=['/3dm_ins/nav/odom']):
            variance = msg.pose.covariance[0]
            if variance < variance_threshold:
                lat = msg.pose.pose.position.y
                lon = msg.pose.pose.position.x
                alt = msg.pose.pose.position.z
                if first_message:
                    # 第一条消息用于设置起点但不计算位姿或写入文件
                    origin['lat'] = lat
                    origin['lon'] = lon
                    origin['alt'] = alt
                    first_message = False
                    continue

                orientation = msg.pose.pose.orientation
                qx = orientation.x
                qy = orientation.y
                qz = orientation.z
                qw = orientation.w
                relative_orientation = R.from_quat([qx, qy, qz, qw])

                # 使用经纬度高度转换局部坐标
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)

                if use_ins_relative_orientation:
                    # 使用INS提供的相对位姿
                    if T0[0,0] == 1:  # 如果T0还是单位矩阵，计算初始位姿
                        _, _, init_yaw = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                        T0[:3, :3] = R.from_euler('z', init_yaw, degrees=True).as_matrix()
                    T1 = relative_orientation.as_matrix()
                    current_pose = np.dot(T0[:3, :3], T1[:3, :3])  # 变换当前位姿
                else:
                    # 继续使用位置差分计算绝对位姿
                    _, _, yaw = R.from_matrix(T0[:3, :3]).as_euler('zyx', degrees=True)
                    current_pose = R.from_euler('z', yaw, degrees=True).as_matrix()


                # 提取当前位姿的四元数
                current_quat = R.from_matrix(current_pose).as_quat()
                tum_data.append(f"{t.to_sec()} {x} {y} {z} {current_quat[0]} {current_quat[1]} {current_quat[2]} {current_quat[3]}")

        with open(tum_file, 'w') as file:
            file.write("\n".join(tum_data))
        print(f"Data successfully saved to {tum_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()

#  已知的原点经纬高数据
origin = {'lat': 22.8901710523756, 'lon': 113.47589813609757, 'alt': 0.07678306745241956}  # 请替换为实际值
# Rosbag文件路径
bag_file = 'sample.bag'  # 请替换为实际路径
# 输出的TUM格式文件路径
tum_file = os.path.splitext(bag_file)[0] + "_ins_tum_format.txt"
# 斜方差阈值
variance_threshold = 0.1
extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold, True)
