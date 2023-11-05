#include "environment/Obstacles.h"

perception_etflab::Obstacles::Obstacles()
{
    // Initial settings for each obstacle
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial settings for each obstacle: ");
    Eigen::Vector3f pos, vel;
    float r, fi, theta;
    for (int i = 0; i < num_obstacles; i++)
    {
        r = float(rand()) / RAND_MAX * WS_radius;
        fi = float(rand()) / RAND_MAX * 2 * M_PI;
        theta = float(rand()) / RAND_MAX * M_PI;
        pos.x() = WS_center.x() + r * std::cos(fi) * std::sin(theta);
        pos.y() = WS_center.y() + r * std::sin(fi) * std::sin(theta);
        pos.z() = WS_center.z() + r * std::cos(theta);
        if (std::abs(pos.x()) < 0.2 + dim.head(2).norm() && pos.y() > -0.2 - dim.head(2).norm() && 
            pos.y() < 0.4 + dim.head(2).norm() && pos.z() < 0.6 + dim.z() ||  // Initial position of the robot
            pos.z() < 0)   
            i--;
        else
        {
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            cluster->emplace_back(pcl::PointXYZRGB(pos.x(), pos.y(), pos.z()));
            cluster->emplace_back(pcl::PointXYZRGB(pos.x() - dim.x()/2, pos.y() - dim.y()/2, pos.z() - dim.z()/2));
            cluster->emplace_back(pcl::PointXYZRGB(pos.x() + dim.x()/2, pos.y() + dim.y()/2, pos.z() + dim.z()/2));
            obstacles.emplace_back(cluster);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());

            vel = Eigen::Vector3f::Random(3);
            vel.normalize();
            velocities.emplace_back(vel);
        }
    }
}

void perception_etflab::Obstacles::move(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cluster;
    Eigen::Vector3f pos, vel;

    for (int i = 0; i < num_obstacles; i++)
    {
        cluster = obstacles[i];
        pos = Eigen::Vector3f(cluster->points.front().x, cluster->points.front().y, cluster->points.front().z);
        // vel = max_vel * velocities[i];
        vel = float(rand()) / RAND_MAX * max_vel * velocities[i];
        pos += vel;

        if ((pos - WS_center).norm() > WS_radius || pos.z() < 0 ||      // Out of workspace or below table
            pos.head(2).norm() < 0.1 + dim.head(2).norm() && pos.z() < WS_center.z() + 0.1 + dim.z())   // Robot base
        {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing new velocity. ");
            vel = Eigen::Vector3f::Random(3);
            vel.normalize();
            velocities[i] = vel;
            i--;
        }
        else
        {
            for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cluster->begin(); point < cluster->end(); point++)
            {
                point->x += vel.x();
                point->y += vel.y();
                point->z += vel.z();
            }
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d. Obstacle pos: (%f, %f, %f)", i, pos.x(), pos.y(), pos.z());
        }
    }
    clusters = obstacles;
}

void perception_etflab::Obstacles::move(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
}