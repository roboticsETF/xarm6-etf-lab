#include "environments/Octomap.h"

sim_bringup::Octomap::Octomap(const std::string config_file_path)
{
    std::string project_abs_path(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

    read_node = rclcpp::Node::make_shared("read_node");
    client = read_node->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
}

void sim_bringup::Octomap::read()
{    
    while (!client->wait_for_service(std::chrono::seconds(1))) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Wait for the result  
    auto request { std::make_shared<octomap_msgs::srv::GetOctomap::Request>() };
    auto result { client->async_send_request(request) };
    if (rclcpp::spin_until_future_complete(read_node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        octomap_msgs::msg::Octomap octomap_msg { result.get()->map };
        octomap::AbstractOcTree* octomap_abstract_octree { octomap_msgs::binaryMsgToMap(octomap_msg) };
        octomap_octree = dynamic_cast<octomap::OcTree*>(octomap_abstract_octree);

        fcl::OcTreef Octree(std::make_shared<const octomap::OcTree>(*octomap_octree));
        octree = std::make_shared<fcl::OcTreef>(Octree);            
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octree read successfully!");
        visualize();
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read octree!");
}

void sim_bringup::Octomap::visualize()
{
    std::vector<std::array<float, 6>> boxes { octree->toBoxes() };
    visualization_msgs::msg::MarkerArray marker_array_msg {};

    for (size_t i = 0; i < boxes.size(); i++) 
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Box %ld: (%f, %f, %f)", i, boxes[i][0], boxes[i][1], boxes[i][2]);
        visualization_msgs::msg::Marker marker {};
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "octree_boxes";
        marker.id = i;
        marker.header.frame_id = "world";
        // marker.header.stamp = now();
        marker.pose.position.x = boxes[i][0];
        marker.pose.position.y = boxes[i][1];
        marker.pose.position.z = boxes[i][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker_array_msg.markers.emplace_back(marker);
    }
    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing octree boxes...");
}
