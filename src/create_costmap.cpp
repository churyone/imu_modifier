#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

class PointCloudToCostmap
{
public:
    // 네임스페이스 초기화
    PointCloudToCostmap() : nh_("costmap")
    {
        // YAML 파일에서 파라미터 로드
        nh_.getParam("costmap_resolution", resolution_);
        nh_.getParam("costmap_width", grid_width_);
        nh_.getParam("costmap_height", grid_height_);
        nh_.getParam("height_threshold_min", height_threshold_min_);
        nh_.getParam("height_threshold_max", height_threshold_max_);

        // Costmap2D 객체 초기화
        double origin_x = -grid_width_ * resolution_ / 2.0;
        double origin_y = -grid_height_ * resolution_ / 2.0;
        costmap_ = new costmap_2d::Costmap2D(grid_width_, grid_height_, resolution_, origin_x, origin_y);

        // 퍼블리셔 초기화
        costmap_publisher_ = new costmap_2d::Costmap2DPublisher(&nh_, costmap_, "global_frame", "costmap", false);

        // PointCloud2 토픽을 구독
        point_cloud_subscriber_ = nh_.subscribe("/velodyne_points", 1, &PointCloudToCostmap::pointCloudCallback, this);
    }

    ~PointCloudToCostmap()
    {
        delete costmap_;
        delete costmap_publisher_;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // PCL PointCloud형 cloud 선언
        pcl::PointCloud<pcl::PointXYZ> cloud;
        // PointCloud2 msg를 PCL PointCloud cloud로 변환
        pcl::fromROSMsg(*msg, cloud);

        // 포인트 클라우드를 비용 맵에 추가
        for (const auto& point : cloud.points)
        {
            unsigned int mx, my;
            // pointcloud의 x, y가 costmap의 mx, my로 변환, 성공하면 true 반환
            if (costmap_->worldToMap(point.x, point.y, mx, my))
            {
                if (point.z > height_threshold_min_ && point.z < height_threshold_max_)
                {
                    costmap_->setCost(mx, my, 255); // 최대 비용(장애물)
                }
            }
        }

        // 비용 맵을 퍼블리시
        costmap_publisher_->publishCostmap();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_subscriber_;
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DPublisher* costmap_publisher_;

    // 파라미터 변수
    double resolution_;
    int grid_width_;
    int grid_height_;
    double height_threshold_min_;
    double height_threshold_max_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_to_costmap");

    PointCloudToCostmap pc_to_costmap;

    ros::spin();
    return 0;
}
