#include "environments/ConvexHulls.h"

sim_bringup::ConvexHulls::ConvexHulls(const std::string config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
}

void sim_bringup::ConvexHulls::pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    points = {std::vector<fcl::Vector3f>()};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    size_t j { 0 };
    for (size_t i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZRGB P { pcl->points[i] };
        if (P.x == 0.0 && P.y == 0.0 && P.z == 0.0)     // This point is just delimiter to distinguish clusters
        {
            points.emplace_back(std::vector<fcl::Vector3f>());
            j++;
        }
        else
        {
            points[j].emplace_back(fcl::Vector3f(P.x, P.y, P.z));
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Convex-hull %ld.\t Point: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
}

void sim_bringup::ConvexHulls::polygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    polygons = {std::vector<size_t>()};
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    size_t j { 0 };
    for (size_t i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZ P { pcl->points[i] };
        if (P.x == -1)     // This point is just delimiter to distinguish clusters
        {
            polygons.emplace_back(std::vector<size_t>());
            j++;
        }
        else
        {
            polygons[j].emplace_back(P.x);
            polygons[j].emplace_back(P.y);
            polygons[j].emplace_back(P.z);
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Convex-hull %ld.\t Polygon indices: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
}
