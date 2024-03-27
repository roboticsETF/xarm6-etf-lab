//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_CONVEX_HULLS_H
#define SIM_BRINGUP_CONVEX_HULLS_H

#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

namespace sim_bringup
{
    class ConvexHulls
    {
    public:
        ConvexHulls(const std::string config_file_path);

        inline const std::vector<std::vector<fcl::Vector3f>> &getPoints() const { return points; }
        inline const std::vector<std::vector<size_t>> &getPolygons() const { return polygons; }

        void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void polygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr polygons_subscription;

    private:
        std::vector<std::vector<fcl::Vector3f>> points;
        std::vector<std::vector<size_t>> polygons;
    };
}

#endif // SIM_BRINGUP_CONVEX_HULLS_H