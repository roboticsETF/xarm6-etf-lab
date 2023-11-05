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
        const int num_obstacles = 10;
        const Eigen::Vector3f dim = Eigen::Vector3f(0.03, 0.03, 0.03);
        const Eigen::Vector3f WS_center = Eigen::Vector3f(0, 0, 0.267);
        const float WS_radius = 1.0;
		const float max_vel = 0.01; 						            // Maximal velocity for each obstacle
		std::vector<Eigen::Vector3f> velocities; 		                // Velocity vector for each obstacle
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> obstacles;  // Each obstacle represents a single cluster

    };
}