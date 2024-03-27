#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_etflab
{
    class Obstacles
    {
    public:
        Obstacles(const std::string &config_file_path);

        void move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void move(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
        bool isValid(const Eigen::Vector3f &pos, float vel);

        inline float getPeriod() const { return period; }

    private:
        size_t num_obstacles;
        Eigen::Vector3f dim;
		float robot_max_vel;                                            // Maximal velocity for each obstacle
        Eigen::Vector3f WS_center;								        // Workspace center point in [m]
        float WS_radius; 										        // Workspace radius in [m]
		float base_radius;
		bool table_included;
        float period; 						                            // Period in [s]
        float max_vel;                                                  // Maximal velocity of each obstacle in [m/s]
		std::vector<Eigen::Vector3f> velocities; 		                // Velocity vector for each obstacle
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> obstacles;  // Each obstacle represents a single cluster

    };
}