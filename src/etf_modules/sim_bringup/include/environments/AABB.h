//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_AABB_H
#define SIM_BRINGUP_AABB_H

#include <Environment.h>

#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

namespace sim_bringup
{
    class AABB
    {
    public:
        AABB(const std::string &config_file_path);

        inline const std::vector<Eigen::Vector3f> &getDimensions() const { return dimensions; }
        inline const Eigen::Vector3f &getDimensions(size_t idx) const { return dimensions[idx]; }
        inline const std::vector<Eigen::Vector3f> &getPositions() const { return positions; }
        inline const Eigen::Vector3f &getPositions(size_t idx) const { return positions[idx]; }
        inline size_t getMinNumCaptures() const { return min_num_captures; }

        inline void setEnvironment(const std::shared_ptr<env::Environment> &env_) { env = env_; }
        inline void setMinNumCaptures(size_t min_num_captures_) { min_num_captures = min_num_captures_; }

        void updateEnvironment();
        void resetMeasurements();
        virtual int chooseObject() { return -1; }
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void withFilteringCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    protected:
        virtual bool whetherToRemove(const Eigen::Vector3f &object_pos, const Eigen::Vector3f &object_dim);

        std::vector<Eigen::Vector3f> dimensions;
        std::vector<Eigen::Vector3f> positions;
        std::vector<size_t> num_captures;
        size_t min_num_captures;
        std::shared_ptr<env::Environment> env;
    };
}

#endif // SIM_BRINGUP_AABB_H