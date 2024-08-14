#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

MAX_DISTANCE = 200.0  # 최대 거리 값을 설정합니다 (LiDAR 사양에 맞게 조정 필요)

def callback(pointcloud_msg):
    points = []
    for i, point in enumerate(pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=False)):
        x, y, z, intensity = point
        # NaN 값을 최대 거리 값으로 대체
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            x = MAX_DISTANCE
            y = 0.0
            z = 0.0
        ring = i % 16  # 예시: 간단한 링 채널 할당 (LiDAR 사양에 맞게 조정 필요)
        points.append([x, y, z, intensity, ring])

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)
    ]

    header = pointcloud_msg.header
    new_cloud = pc2.create_cloud(header, fields, points)
    
    pub.publish(new_cloud)

if __name__ == '__main__':
    rospy.init_node('pointcloud_ring_adder', anonymous=True)

    rospy.Subscriber('/lidar3D', PointCloud2, callback)
    pub = rospy.Publisher('/modified_lidar3D', PointCloud2, queue_size=10)

    rospy.spin()
