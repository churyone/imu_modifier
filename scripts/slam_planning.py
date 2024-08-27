#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, NavsatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# Occupancy Grid Map을 생성하는 함수
def create_costmap(points, resolution, grid_width, grid_height, height_threshold):
    # Z축 필터링을 통해 특정 높이 범위의 점만 사용합니다.
    filtered_points = points[(points[:, 2] > height_threshold[0]) & (points[:, 2] < height_threshold[1])]
    points_2d = filtered_points[:, :2]  # XY 평면으로 투영

    # grid의 중심점 계산
    center_x = grid_width // 2
    center_y = grid_height // 2

    # 각 포인트를 그리드 셀에 매핑하여 장애물(100)로 설정
    # 그리드 초기화
    occupancy_grid_floor = np.zeros((grid_height, grid_width), dtype=np.uint8)
    occupancy_grid_ceil = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # 포인트 좌표를 그리드 셀 단위로 변환 (내림)
    indices_floor = np.floor(points_2d / resolution).astype(int)
    indices_floor[:, 0] += center_x
    indices_floor[:, 1] += center_y

    # 포인트 좌표를 그리드 셀 단위로 변환 (올림)
    indices_ceil = np.ceil(points_2d / resolution).astype(int)
    indices_ceil[:, 0] += center_x
    indices_ceil[:, 1] += center_y

    # 내림을 적용한 포인트를 그리드에 할당
    for idx in indices_floor:
        x, y = idx
        if 0 <= x < grid_width and 0 <= y < grid_height:
            occupancy_grid_floor[y, x] = 255  # 장애물로 설정

    # 올림을 적용한 포인트를 그리드에 할당
    for idx in indices_ceil:
        x, y = idx
        if 0 <= x < grid_width and 0 <= y < grid_height:
            occupancy_grid_ceil[y, x] = 255  # 장애물로 설정

    # 최종 그리드 생성: 내림 또는 올림 중 하나라도 255이면 최종 맵에 255을 설정
    cost_map = np.where((occupancy_grid_floor == 255) | (occupancy_grid_ceil == 255), 255, 0)

    return cost_map

# PointCloud2 메시지 콜백 함수
def point_cloud_callback(msg):
    # 포인트 클라우드 데이터를 추출
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    if points.size == 0:
        rospy.logwarn("Received empty point cloud")
        return  # 포인트 클라우드가 비어 있는 경우 처리하지 않음

    # Occupancy Grid Map 생성
    occupancy_grid, resolution, grid_width, grid_height = create_costmap(points)

    # Occupancy Grid Map을 ROS 메시지로 변환
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "base_link"

    # 그리드 정보 설정
    grid_msg.info.resolution = resolution
    grid_msg.info.width = grid_width
    grid_msg.info.height = grid_height

    # 그리드의 중심을 base_link에 맞추기 위해 origin을 설정
    grid_msg.info.origin = Pose()
    grid_msg.info.origin.position.x = -(grid_width * resolution) / 2.0
    grid_msg.info.origin.position.y = -(grid_height * resolution) / 2.0

    # 그리드 데이터를 1D 리스트로 변환하여 메시지에 추가
    grid_msg.data = occupancy_grid.flatten().tolist()

    # Occupancy Grid Map 퍼블리시
    occupancy_grid_publisher.publish(grid_msg)

def point_to_gps(points,height_threshold=(0, 1.5)):
    filtered_points = points[(points[:, 2] > height_threshold[0]) & (points[:, 2] < height_threshold[1])]
    points_2d = filtered_points[:, :2]  # XY 평면으로 투영
    
"""# Occupancy Grid Map을 생성하는 함수
def create_fixed_size_occupancy_grid(points, resolution=0.2, grid_width=300, grid_height=300, height_threshold=(0, 1.5)):
    # Z축 필터링을 통해 특정 높이 범위의 점만 사용합니다.
    filtered_points = points[(points[:, 2] > height_threshold[0]) & (points[:, 2] < height_threshold[1])]
    points_2d = filtered_points[:, :2]  # XY 평면으로 투영

    # Occupancy Grid Map 초기화 (기본값 0으로 설정하여 빈 공간으로 초기화)
    occupancy_grid = np.zeros((grid_height, grid_width), dtype=int)

    # grid의 중심점 계산
    center_x = grid_width // 2
    center_y = grid_height // 2

    # 각 포인트를 그리드 셀에 매핑하여 장애물(100)로 설정
    # 그리드 초기화
    occupancy_grid_floor = np.zeros((grid_height, grid_width), dtype=int)
    occupancy_grid_ceil = np.zeros((grid_height, grid_width), dtype=int)

    # 포인트 좌표를 그리드 셀 단위로 변환 (내림)
    indices_floor = np.floor(points_2d / resolution).astype(int)
    indices_floor[:, 0] += center_x
    indices_floor[:, 1] += center_y

    # 포인트 좌표를 그리드 셀 단위로 변환 (올림)
    indices_ceil = np.ceil(points_2d / resolution).astype(int)
    indices_ceil[:, 0] += center_x
    indices_ceil[:, 1] += center_y

    # 내림을 적용한 포인트를 그리드에 할당
    for idx in indices_floor:
        x, y = idx
        if 0 <= x < grid_width and 0 <= y < grid_height:
            occupancy_grid_floor[y, x] = 100  # 장애물로 설정

    # 올림을 적용한 포인트를 그리드에 할당
    for idx in indices_ceil:
        x, y = idx
        if 0 <= x < grid_width and 0 <= y < grid_height:
            occupancy_grid_ceil[y, x] = 100  # 장애물로 설정

    # 최종 그리드 생성: 내림 또는 올림 중 하나라도 100이면 최종 맵에 100을 설정
    occupancy_grid_final = np.where((occupancy_grid_floor == 100) | (occupancy_grid_ceil == 100), 100, 0)

"""

"""# PointCloud2 메시지 콜백 함수
def point_cloud_callback(msg):
    # 포인트 클라우드 데이터를 추출
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

    if points.size == 0:
        rospy.logwarn("Received empty point cloud")
        return  # 포인트 클라우드가 비어 있는 경우 처리하지 않음

    # Occupancy Grid Map 생성
    occupancy_grid, resolution, grid_width, grid_height = create_fixed_size_occupancy_grid(points)

    # Occupancy Grid Map을 ROS 메시지로 변환
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "base_link"

    # 그리드 정보 설정
    grid_msg.info.resolution = resolution
    grid_msg.info.width = grid_width
    grid_msg.info.height = grid_height

    # 그리드의 중심을 base_link에 맞추기 위해 origin을 설정
    grid_msg.info.origin = Pose()
    grid_msg.info.origin.position.x = -(grid_width * resolution) / 2.0
    grid_msg.info.origin.position.y = -(grid_height * resolution) / 2.0

    # 그리드 데이터를 1D 리스트로 변환하여 메시지에 추가
    grid_msg.data = occupancy_grid.flatten().tolist()

    # Occupancy Grid Map 퍼블리시
    occupancy_grid_publisher.publish(grid_msg)"""

# ROS 노드 초기화
rospy.init_node('point_cloud_to_occupancy_grid', anonymous=True)
occupancy_grid_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)

# PointCloud2 토픽 구독
rospy.Subscriber('/velodyne_points', PointCloud2, point_cloud_callback, queue_size=1)

# ROS 노드 실행
rospy.spin()
