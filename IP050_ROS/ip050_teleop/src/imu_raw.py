#!/usr/bin/env python3

import serial
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np
import math
import tf
from tf.transformations import quaternion_about_axis

ser = serial.Serial('/dev/ttyACM0', baudrate=115200)  # 시리얼 포트 설정 (COMx 대신 실제 포트 이름을 사용)
yaw_angle = 0.0  # Yaw angle initialization

rospy.init_node('imu_publisher')  # Initialize a ROS node
last_time = rospy.Time.now()

imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)  # Create an IMU publisher

try:
    while not rospy.is_shutdown():
        data = ser.readline().decode().strip()  # 시리얼 데이터 읽기 및 디코딩
        # print(f'data: {data}')
        # Split the data into individual values
        raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z = map(float, data.split(','))

        # Read the acceleration vals
        accel_x = raw_accel_x / 16384.0
        accel_y = raw_accel_y / 16384.0
        accel_z = raw_accel_z / 16384.0
        
        # Read the gyro vals
        gyro_x = raw_gyro_x / 131.0
        gyro_y = raw_gyro_y / 131.0
        gyro_z = raw_gyro_z / 131.0
        
        # Create a Quaternion message
        quaternion_msg = Quaternion()
        quaternion_msg.x = 0.0
        quaternion_msg.y = 0.0
        quaternion_msg.z = 0.0
        quaternion_msg.w = 0.0

        # Create an Imu message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"  # Replace with your desired frame ID
        
        # Assign the orientation to the IMU message
        imu_msg.orientation = quaternion_msg

        imu_msg.linear_acceleration.x = accel_x*9.8
        imu_msg.linear_acceleration.y = accel_y*9.8
        imu_msg.linear_acceleration.z = accel_z*9.8

        imu_msg.angular_velocity.x = gyro_x*0.0174533
        imu_msg.angular_velocity.y = gyro_y*0.0174533
        imu_msg.angular_velocity.z = gyro_z*0.0174533
        
        # Publish the IMU message
        imu_pub.publish(imu_msg)

except KeyboardInterrupt:
    ser.close()  # 프로그램 종료 시 시리얼 포트 닫기

