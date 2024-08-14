#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import time

# 퍼블리시된 메시지 카운트와 시간 기록
msg_count = 0
start_time = None

def imu_listener_callback(imu_msg):
    global msg_count, start_time

    # 첫 번째 메시지를 수신한 시간을 기록
    if start_time is None:
        start_time = time.time()

    # 메시지 카운트 증가
    msg_count += 1

    # 경과 시간 계산
    elapsed_time = time.time() - start_time

    if elapsed_time >= 1.0:
        # 1초 동안 수신된 메시지 수(Hz) 계산
        hz = msg_count / elapsed_time
        rospy.loginfo(f"IMU topic publishing at {hz:.2f} Hz")

        # 카운트 및 시간 초기화
        msg_count = 0
        start_time = time.time()

def listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber('/imu', Imu, imu_listener_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
