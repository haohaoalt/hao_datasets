import numpy as np
from collections import defaultdict, namedtuple
import cv2
from scipy.spatial.transform import Rotation as R
import scipy
import math
import os
import json
import copy
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import Image,Imu, PointField, NavSatFix
from geometry_msgs.msg import Vector3
from pypcd import pypcd
import progressbar
from tqdm import tqdm
import csv
from decimal import Decimal
# IMU
# time, acc_x, acc_y, acc_z (m/s^2), gyro_x, gyro_y, gyro_z (deg/s)
def read_imu_csv(dir_path):
        acc_path = dir_path + "/data/raw_data/accel-0.csv"
        gyro_path = dir_path + "/data/raw_data/gyro-0.csv"
        imu_time = dir_path + "/data/raw_data/time.csv"

        with open(imu_time, 'r') as f:
                rows = len(f.readlines())

        imu_data = np.zeros((rows, 7))

        with open (imu_time, 'r') as f1:
            for j in range(rows):
                if j == 0:
                    a_line = f1.readline()
                    continue
                a_line = f1.readline()
                a_line = a_line.split(' ')
                imu_data[j - 1][0] = float(a_line[0])

        with open (acc_path, 'r') as f2:
            for j in range(rows):
                if j == 0:
                    a_line = f2.readline()
                    continue
                a_line = f2.readline()
                a_line = a_line.split(',')
                imu_data[j - 1][1] = float(a_line[0])
                imu_data[j - 1][2] = float(a_line[1])
                imu_data[j - 1][3] = float(a_line[2])

        with open (gyro_path, 'r') as f3:
            for j in range(rows):
                if j == 0:
                    a_line = f3.readline()
                    continue
                a_line = f3.readline()
                a_line = a_line.split(',')
                imu_data[j - 1][4] = float(a_line[0])
                imu_data[j - 1][5] = float(a_line[1])
                imu_data[j - 1][6] = float(a_line[2])
        return imu_data

# /home/xng/catkin_ws/src/inno_lio/data/raw_data/gps-0.csv
# timestamp gps_lat (deg),gps_lon (deg),gps_alt (m),gps_vN (m/s),gps_vE (m/s),gps_vD (m/s)
def read_gps_csv(dir_path):
    gps_file_path = dir_path + "/data/raw_data/gps-0.csv"
    gps_time_file_path = dir_path + "/data/raw_data/gps_time.csv"
    with open(gps_time_file_path, 'r') as f:
        rows = len(f.readlines())
    gps_data = np.zeros((rows, 7))
    print(gps_file_path)

    with open (gps_time_file_path, 'r') as f1:
        for j in range(rows):
            if j == 0:
                a_line = f1.readline()
                continue
            a_line = f1.readline()
            a_line = a_line.split(' ')
            gps_data[j - 1][0] = float(a_line[0])

    with open (gps_file_path, 'r') as f2:
        for j in range(rows):
            if j == 0:
                a_line = f2.readline()
                continue
            a_line = f2.readline()
            a_line = a_line.split(',')
            gps_data[j - 1][1] = float(a_line[0])
            gps_data[j - 1][2] = float(a_line[1])
            gps_data[j - 1][3] = float(a_line[2])
            gps_data[j - 1][4] = float(a_line[3])
            gps_data[j - 1][5] = float(a_line[4])
            gps_data[j - 1][6] = float(a_line[5])

    print(gps_data.shape)  # 利用.shape查看结构。
    return gps_data

def pub_imu(bag, imu_msg, imu_count):
        angular_v = Vector3()
        linear_a = Vector3()

        linear_a.x = float(imu_msg[imu_count][1])
        linear_a.y = float(imu_msg[imu_count][2])
        linear_a.z = float(imu_msg[imu_count][3])
        angular_v.x = float(imu_msg[imu_count][4])
        angular_v.y = float(imu_msg[imu_count][5])
        angular_v.z = float(imu_msg[imu_count][6])

        imu = Imu()
        imu.header = Header()
        # imu.header.frame_id = "innovusion"
        imu.header.frame_id = "inertial"
        imu.header.stamp = rospy.Time.from_sec(float(imu_msg[imu_count][0]))
        imu.angular_velocity = angular_v
        imu.linear_acceleration = linear_a
        # print(imu_msg[imu_count][0])
        bag.write("/imu", imu, imu.header.stamp)

def pub_gps(bag, gps_data, gps_count):
        # status = NavSatStatus()
        # if mode==0 or mode==1:
        #     status.status = NavSatStatus.STATUS_NO_FIX
        # else:
        #     status.status = NavSatStatus.STATUS_FIX
        # status.service = NavSatStatus.SERVICE_GPS
        # num_sats = UInt16()
        # num_sats.data = gps[i, 2]
        # fix = NavSatFix()
        # fix.status = status

        gps_ = NavSatFix()
        gps_.header = Header()
        gps_.header.frame_id = "inertial"
        gps_.header.stamp = rospy.Time.from_sec(float(gps_data[gps_count][0]))
        gps_.latitude = float(gps_data[gps_count][1])               # degree
        gps_.longitude = float(gps_data[gps_count][2])
        gps_.altitude = float(gps_data[gps_count][3])
    
        bag.write("/gps", gps_, gps_.header.stamp)

def make_bag(dir_path):
        bag_path = dir_path + "/data/test.bag"
        bag = rosbag.Bag(bag_path, 'w')

        imu_msg = read_imu_csv(dir_path)
        imu_size = len(imu_msg)
        imu_count = 0
        # print('timestamp: ', float(pcd_timestamp[0] / 1e6))
        for i in tqdm(range(imu_size - 1)):
                pub_imu(bag, imu_msg, imu_count)
                imu_count = imu_count + 1

        gps_data = read_gps_csv(dir_path)
        gps_size = len(gps_data)
        gps_count = 0
        # print('timestamp: ', float(pcd_timestamp[0] / 1e6))
        for i in tqdm(range(gps_size - 1)):
                pub_gps(bag, gps_data, gps_count)
                gps_count = gps_count + 1

        bag.close()       


if __name__ == '__main__':
        # dir_path = "/home/xng/data/Test/odom_test_dataset/3_tunnel"
        # 上一级目录
        dir_path = os.path.abspath(os.path.dirname(os.getcwd()))
        print(dir_path)
        # read_imu_csv(dir_path)
        # read_gps_csv(dir_path)

        make_bag(dir_path)
