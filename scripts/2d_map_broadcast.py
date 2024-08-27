#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

def broadcast_2d_map_frame():
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "2d_map"
    
    # 2D 맵에 대한 변환 설정 (필요에 따라 조정)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    # z축 회전을 위한 쿼터니언 설정
    q = quaternion_from_euler(0, 0, 0)  # (roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node('broadcast_2d_map')
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        broadcast_2d_map_frame()
        rate.sleep()
