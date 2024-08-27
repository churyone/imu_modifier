#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, NavSatFix
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose

class OccupancyGridPublisher:
    def __init__(self):
        rospy.init_node('point_cloud_to_occupancy_grid', anonymous=True)

        self.occupancy_grid_publisher = rospy.Publisher('/odom/grid_map', OccupancyGrid, queue_size=1)
        self.current_pose = None

        # Initialize map parameters
        self.resolution = 0.2  # grid cell resolution
        self.grid_width = 300  # grid width in cells
        self.grid_height = 300  # grid height in cells
        self.height_threshold = (0, 1.5)  # height range for obstacles

        # Initialize the grid with unknown values (-1)
        self.grid = np.zeros((self.grid_width, self.grid_height), dtype=int)

        # Subscribe to topics
        rospy.Subscriber('/velodyne_points', PointCloud2, self.point_cloud_callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

        rospy.spin()

    def odom_callback(self, odom_msg):
        # Update current robot pose
        self.current_pose = odom_msg.pose.pose

    def point_cloud_callback(self, msg):
        # Check if the pose is available
        if self.current_pose is None:
            rospy.logwarn("Waiting for odom data to be available...")
            return

        # Extract point cloud data
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        if points.size == 0:
            rospy.logwarn("Received empty point cloud")
            return

        # Create occupancy grid based on point cloud data
        occupancy_grid, resolution, grid_width, grid_height = self.create_fixed_size_occupancy_grid(points)

        # Convert occupancy grid to ROS message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "odom"

        # Set grid information
        grid_msg.info.resolution = resolution
        grid_msg.info.width = grid_width
        grid_msg.info.height = grid_height

        # Set grid origin based on current pose
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.current_pose.position.x - (grid_width * resolution) / 2.0
        grid_msg.info.origin.position.y = self.current_pose.position.y - (grid_height * resolution) / 2.0

        # Flatten the occupancy grid data and set it to the message
        grid_msg.data = occupancy_grid.flatten().tolist()

        # Publish the occupancy grid
        self.occupancy_grid_publisher.publish(grid_msg)

    def create_fixed_size_occupancy_grid(self, points, resolution=0.2, grid_width=300, grid_height=300, height_threshold=(0, 1.5)):
        # Filter points based on height
        filtered_points = points[(points[:, 2] > height_threshold[0]) & (points[:, 2] < height_threshold[1])]
        points_2d = filtered_points[:, :2]  # Project to 2D (XY plane)

        # Initialize occupancy grid (free space = 0)
        occupancy_grid = np.zeros((grid_height, grid_width), dtype=int)

        # Calculate grid center
        center_x = grid_width // 2
        center_y = grid_height // 2

        # Map points to grid indices
        indices = np.floor(points_2d / resolution).astype(int)
        indices[:, 0] += center_x
        indices[:, 1] += center_y

        valid_indices = (indices[:, 0] >= 0) & (indices[:, 0] < grid_width) & \
                        (indices[:, 1] >= 0) & (indices[:, 1] < grid_height)

        # Mark grid cells as occupied (100)
        occupancy_grid[indices[valid_indices, 1], indices[valid_indices, 0]] = 100

        return occupancy_grid, resolution, grid_width, grid_height

if __name__ == '__main__':
    try:
        grid_publisher = OccupancyGridPublisher()
    except rospy.ROSInterruptException:
        pass
