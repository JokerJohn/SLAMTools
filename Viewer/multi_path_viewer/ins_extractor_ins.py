#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rosbag
from geographiclib.geodesic import Geodesic
import os
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
def smooth_yaw_transition(yaw, last_yaw):
    # 平滑偏航角度变化
    delta_yaw = yaw - last_yaw
    if delta_yaw > 180:
        yaw -= 360
    elif delta_yaw < -180:
        yaw += 360
    return yaw
def convert_lat_lon_alt_to_xyz(lat, lon, alt, origin):
    # 经纬度高度转换为局部坐标
    geod = Geodesic.WGS84
    g = geod.Inverse(origin['lat'], origin['lon'], lat, lon)
    distance = g['s12']
    azimuth = math.radians(g['azi1'])
    y = distance * math.cos(azimuth)
    x = distance * math.sin(azimuth)
    z = alt - origin['alt']
    return x, y, z
def compute_yaw(x1, y1, x2, y2):
    # 计算两点之间的偏航角
    delta_x = x2 - x1
    delta_y = y2 - y1
    yaw = math.atan2(delta_y, delta_x)
    return yaw
# def apply_transform(position, orientation, transform_R, transform_T):
#     # 应用旋转和平移变换
#     transformed_position = np.dot(transform_R, position) + transform_T
#     transformed_orientation = orientation * R.from_matrix(transform_R)
#     return transformed_position, transformed_orientation
def apply_transform(position, orientation, transform_R, transform_T):
    # 应用旋转和平移变换
    transformed_position = np.matmul(transform_R, position) + transform_T
    transformed_orientation_matrix = np.matmul(transform_R, orientation.as_matrix())
    transformed_orientation = R.from_matrix(transformed_orientation_matrix)
    return transformed_position, transformed_orientation.as_quat()  # 返回四元数数组
def extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold):
    try:
        bag = rosbag.Bag(bag_file, 'r')
        tum_data = []
        last_x, last_y, last_yaw = None, None, 0
        R0_initialized = False
        R1_prime = None
        R0 = None
        yaw = 0
        # body-to-ins 外参旋转矩阵和平移向量
        extrinsic_T = np.array([0.111295833698732, -0.034688482930289, -0.087005256137232])
        extrinsic_R = np.array([
            [0.010147525371018, -0.007018396145658, -0.99992388695695],
            [0.007094265575029, 0.999950688609296, -0.006946595941935],
            [0.999923417104674, -0.007023262213128, 0.010196773555814]
        ])
        for topic, msg, t in bag.read_messages(topics=['/3dm_ins/nav/odom']):
            variance = msg.pose.covariance[0]
            if variance < variance_threshold:
                lat = msg.pose.pose.position.y
                lon = msg.pose.pose.position.x
                alt = msg.pose.pose.position.z
                qx = msg.pose.pose.orientation.x
                qy = msg.pose.pose.orientation.y
                qz = msg.pose.pose.orientation.z
                qw = msg.pose.pose.orientation.w
                x, y, z = convert_lat_lon_alt_to_xyz(lat, lon, alt, origin)
                # temp=x
                # x=-z
                # z = -temp  # 反转z轴坐标
                # 转换INS姿态和位置到车体坐标系
                ins_orientation = R.from_quat([qx, qy, qz, qw])
                body_position, quat = apply_transform( extrinsic_T,  R.from_matrix(extrinsic_R), ins_orientation.as_matrix(),  np.array([x, y, z]))
                # body_orientation = R.from_quat(body_quat)
                # if not R0_initialized and last_x is not None and last_y is not None:
                #     yaw = compute_yaw(last_x, last_y, body_position[0], body_position[1])
                #     yaw = smooth_yaw_transition(yaw, last_yaw)
                #     R0 = body_orientation
                #     R1_prime = body_orientation
                #     R0_initialized = True
                # elif R0_initialized:
                #     R2 = body_orientation
                #     relative_orientation = R2 * R1_prime.inv()
                #     final_orientation = relative_orientation * R0
                #     quat = final_orientation.as_quat()
                tum_data.append(f"{t.to_sec()} {body_position[0]} {body_position[1]} {body_position[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}")
                # last_x, last_y, last_yaw = body_position[0], body_position[1], yaw
        with open(tum_file, 'w') as file:
            file.write("\n".join(tum_data))
        print(f"Data successfully saved to {tum_file}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()

# 使用示例
origin = {'lat': 22.6585228324368, 'lon': 113.650278453549, 'alt': 4.6046}
bag_file = '2023-11-26-19-36-49.bag'
tum_file = os.path.splitext(bag_file)[0] + "_ins_tum_format.txt"
variance_threshold = 0.01
extract_odom_to_tum_format(bag_file, origin, tum_file, variance_threshold)
