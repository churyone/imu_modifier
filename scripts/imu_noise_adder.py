import rospy
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque

# IMU 메시지를 저장할 deque
imu_queue = deque()

# 노이즈 표준편차 설정 (필요에 따라 조정)
acc_noise_std = 0.0  # 가속도 측정 오류 노이즈 표준편차
gyro_noise_std = 0.0  # 자이로스코프 측정 오류 노이즈 표준편차
acc_bias = np.zeros(3)  # 가속도 랜덤 워크 노이즈 초기값
gyro_bias = np.zeros(3)  # 자이로스코프 랜덤 워크 노이즈 초기값
acc_w = 0  # 가속도 랜덤 워크 노이즈 표준편차
gyro_w = 0  # 자이로스코프 랜덤 워크 노이즈 표준편차

# 마지막 IMU 타임스탬프
#last_timestamp_secs = None
#last_timestamp_nsecs = None
last_timestamp = None


def add_noise_to_imu(imu_msg):
    global acc_bias, gyro_bias

    # 측정 오류 노이즈 추가
    noisy_imu = Imu()
    noisy_imu.header = imu_msg.header
    noisy_imu.linear_acceleration.x = imu_msg.linear_acceleration.x + np.random.normal(0, acc_noise_std)
    noisy_imu.linear_acceleration.y = imu_msg.linear_acceleration.y + np.random.normal(0, acc_noise_std)
    noisy_imu.linear_acceleration.z = imu_msg.linear_acceleration.z + np.random.normal(0, acc_noise_std)
    noisy_imu.angular_velocity.x = imu_msg.angular_velocity.x + np.random.normal(0, gyro_noise_std)
    noisy_imu.angular_velocity.y = imu_msg.angular_velocity.y + np.random.normal(0, gyro_noise_std)
    noisy_imu.angular_velocity.z = imu_msg.angular_velocity.z + np.random.normal(0, gyro_noise_std)
    # 방향 정보(쿼터니언)를 유지
    noisy_imu.orientation = imu_msg.orientation

    # 랜덤 워크 노이즈 추가
    acc_bias += np.random.normal(0, acc_w, 3)
    gyro_bias += np.random.normal(0, gyro_w, 3)

    noisy_imu.linear_acceleration.x += acc_bias[0]
    noisy_imu.linear_acceleration.y += acc_bias[1]
    noisy_imu.linear_acceleration.z += acc_bias[2]
    noisy_imu.angular_velocity.x += gyro_bias[0]
    noisy_imu.angular_velocity.y += gyro_bias[1]
    noisy_imu.angular_velocity.z += gyro_bias[2]

    return noisy_imu

"""
def imu_callback(imu_msg):
    global imu_queue, imu_pub, last_timestamp_secs, last_timestamp_nsecs

    # 현재 메시지의 타임스탬프를 seconds와 nanoseconds로 계산
    current_secs = imu_msg.header.stamp.secs
    current_nsecs = imu_msg.header.stamp.nsecs
            
    # 마지막 타임스탬프와 현재 타임스탬프 비교 후 순서가 맞지 않으면 삭제
    if last_timestamp_secs is not None and last_timestamp_nsecs is not None:
        if (current_secs < last_timestamp_secs) or (current_secs == last_timestamp_secs and current_nsecs <= last_timestamp_nsecs):
            rospy.logwarn(f"imu message in disorder! Duplicate or out-of-order timestamp detected. Ignoring message. current_time: {current_secs}.{current_nsecs}, last_timestamp: {last_timestamp_secs}.{last_timestamp_nsecs}")
            return
    
    last_timestamp_secs = current_secs
    last_timestamp_nsecs = current_nsecs

    # 노이즈가 추가된 IMU 메시지를 deque에 추가하고 타임스탬프 기준으로 정렬
    noisy_imu_msg = add_noise_to_imu(imu_msg)
    imu_queue.append(noisy_imu_msg)

    # 메시지를 타임스탬프 기준으로 정렬
    imu_queue = deque(sorted(imu_queue, key=lambda msg: (msg.header.stamp.secs, msg.header.stamp.nsecs)))

    
    # 큐를 순회하면서 순서가 맞지 않는 메시지 제거
    for i in range(1, len(imu_queue)):
        time_i = imu_queue[i].header.stamp.secs + imu_queue[i].header.stamp.nsecs * 1e-9
        time_i_1 = imu_queue[i-1].header.stamp.secs + imu_queue[i-1].header.stamp.nsecs * 1e-9
        dt = time_i - time_i_1
        if dt <= 0:
            rospy.logwarn("imu message in disorder! Removing out-of-order message.")
            imu_queue.remove(imu_queue[i])

    # 수정된 IMU 데이터를 퍼블리시
    imu_pub.publish(noisy_imu_msg)
    
"""
def imu_callback(imu_msg):
    global imu_queue, imu_pub, last_timestamp

    # 현재 메시지의 타임스탬프를 seconds와 nanoseconds로 계산
    current_time = imu_msg.header.stamp.secs + imu_msg.header.stamp.nsecs * 1e-9

    # 마지막 타임스탬프와 현재 타임스탬프 비교 후 순서가 맞지 않으면 삭제
    if last_timestamp is not None:
        last_time = last_timestamp
        if current_time <= last_time:
            rospy.logwarn(f"imu message in disorder! Duplicate or out-of-order timestamp detected. Ignoring message. current_time: {current_time:.9f}, last_timestamp: {last_time:.9f}")
            return
    
    last_timestamp = current_time

    # 노이즈가 추가된 IMU 메시지를 deque에 추가하고 타임스탬프 기준으로 정렬
    noisy_imu_msg = add_noise_to_imu(imu_msg)
    imu_queue.append(noisy_imu_msg)

    # 메시지를 타임스탬프 기준으로 정렬
    imu_queue = deque(sorted(imu_queue, key=lambda msg: msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9))

    # 큐에서 가장 오래된 메시지를 가져와 퍼블리시
    if imu_queue:
        oldest_msg = imu_queue.popleft()
        imu_pub.publish(oldest_msg)

    
if __name__ == '__main__':
    rospy.init_node('pointcloud_ring_adder', anonymous=True)

    # IMU 데이터를 구독
    rospy.Subscriber('/imu_raw', Imu, imu_callback)
    # 수정된 IMU 데이터를 퍼블리시
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

    rospy.spin()