#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_etflab
{
    class Obstacles
    {
    public:
        Obstacles();

        void move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void move(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);

    private:
        float step;
        float delta_y;
        int sign;

    };
}