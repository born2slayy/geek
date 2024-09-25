#!/usr/bin/env python3

import serial
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3

ser = serial.Serial('/dev/ttyACM1', baudrate=115200)  # 시리얼 포트 설정 (실제 포트 이름으로 대체하세요)
yaw_angle = 0.0  # Yaw 각도 초기화

rospy.init_node('imu_publisher')  # ROS 노드 초기화
last_time = rospy.Time.now()

imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)  # IMU 데이터를 전송하는 퍼블리셔 생성

try:
    while not rospy.is_shutdown():
        data = ser.readline().decode('utf-8', errors='ignore').strip()  # 시리얼 데이터 읽고 디코딩
        # print(f'data: {data}')
    
        # 데이터를 개별 값으로 분리
        data_values = data.split(' ')
        if len(data_values) == 6:
            raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z = map(float, data_values)
            print(raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z)        

        # if len(data) == 6 :
        #     # 값이 6개인 경우에만 실수로 변환
        #     raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z = map(float, data_values)
            
            
        #     print(raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z)
            
            # 가속도 값 읽기
            accel_x = raw_accel_x / 16384.0
            accel_y = raw_accel_y / 16384.0
            accel_z = raw_accel_z / 16384.0

            # 자이로 값 읽기
            gyro_x = raw_gyro_x / 131.0
            gyro_y = raw_gyro_y / 131.0
            gyro_z = raw_gyro_z / 131.0

            # 쿼터니언 메시지 생성
            quaternion_msg = Quaternion()
            quaternion_msg.x = 0.0
            quaternion_msg.y = 0.0
            quaternion_msg.z = 0.0
            quaternion_msg.w = 0.0

            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"  # 원하는 프레임 ID로 대체하세요

            # 쿼터니언을 IMU 메시지에 할당
            imu_msg.orientation = quaternion_msg

            imu_msg.linear_acceleration.x = accel_x * 9.8
            imu_msg.linear_acceleration.y = accel_y * 9.8
            imu_msg.linear_acceleration.z = accel_z * 9.8

            imu_msg.angular_velocity.x = gyro_x * 0.0174533
            imu_msg.angular_velocity.y = gyro_y * 0.0174533
            imu_msg.angular_velocity.z = gyro_z * 0.0174533

            # IMU 메시지 전송
            imu_pub.publish(imu_msg)
	    
        else:
            # print("잘못된 데이터 형식:", data)
            continue
except KeyboardInterrupt:
    ser.close()  # 프로그램 종료 시 시리얼 포트 닫기
