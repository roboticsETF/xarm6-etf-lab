#include "environments/AABB.h"

sim_bringup::AABB::AABB(const std::string &config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    min_num_captures = node["cameras"]["min_num_captures"].as<size_t>();
}

void sim_bringup::AABB::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    resetMeasurements();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    for (size_t i = 0; i < pcl->size(); i += 2)
    {
        pcl::PointXYZ dim { pcl->points[i] };
        pcl::PointXYZ pos { pcl->points[i+1] };
        dimensions.emplace_back(fcl::Vector3f(dim.x, dim.y, dim.z));
        positions.emplace_back(fcl::Vector3f(pos.x, pos.y, pos.z));
        num_captures.emplace_back(1);
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f)",
        //     i/2, dim.x, dim.y, dim.z, pos.x, pos.y, pos.z);
    }
}

void sim_bringup::AABB::withFilteringCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
    pcl::moveFromROSMsg(*msg, *pcl);
    Eigen::Vector3f dim {};
    Eigen::Vector3f pos {};
    bool found { false };

    for (size_t i = 0; i < pcl->size(); i += 2)
    {
        dim << pcl->points[i].x, pcl->points[i].y, pcl->points[i].z;
        pos << pcl->points[i+1].x, pcl->points[i+1].y, pcl->points[i+1].z;

        if (whetherToRemove(pos, dim))
            continue;

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f)",  // (x, y, z) in [m]
        //     i/2, dim.x(), dim.y(), dim.z(), pos.x(), pos.y(), pos.z());
    
        // Measurements are averaged online
        for (size_t j = 0; j < dimensions.size(); j++)
        {
            if ((dim - dimensions[j]).norm() < 0.05 && (pos - positions[j]).norm() < 0.05)
            {
                dimensions[j] = (num_captures[j] * dimensions[j] + dim) / (num_captures[j] + 1);
                positions[j] = (num_captures[j] * positions[j] + pos) / (num_captures[j] + 1);
                num_captures[j]++;
                found = true;
                break;
            }
        }
        
        if (!found)
        {
            dimensions.emplace_back(dim);
            positions.emplace_back(pos);
            num_captures.emplace_back(1);
        }                
    }   
}

bool sim_bringup::AABB::whetherToRemove([[maybe_unused]] const Eigen::Vector3f &object_pos, [[maybe_unused]] const Eigen::Vector3f &object_dim)
{    
    return false;
}

void sim_bringup::AABB::updateEnvironment()
{
    env->removeObjects("table", false);
    
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (num_captures[i] >= min_num_captures)
        {
		    std::shared_ptr<env::Object> object = 
                std::make_shared<env::Box>(dimensions[i], positions[i], fcl::Quaternionf::Identity(), "dynamic_obstacle");
            env->addObject(object);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld: dim = (%f, %f, %f), pos = (%f, %f, %f), num. captures = %ld",
                i, dimensions[i].x(), dimensions[i].y(), dimensions[i].z(),                 // (x, y, z) in [m]
                positions[i].x(), positions[i].y(), positions[i].z(), num_captures[i]);     // (x, y, z) in [m]
        }
    }

    if (min_num_captures > 0)
        resetMeasurements();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Environment is updated."); 
}

// Clear previous averaged measurements and start taking new ones
void sim_bringup::AABB::resetMeasurements()
{
    dimensions.clear();
    positions.clear();
    num_captures.clear();
}
