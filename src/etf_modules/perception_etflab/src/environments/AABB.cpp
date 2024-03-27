#include "environments/AABB.h"

// Make axis-aligned bounding-box for each cluster from 'clusters'
void perception_etflab::AABB::make(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    boxes = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    Eigen::Vector4f min_point {}, max_point {};
    pcl::PointXYZ dim {}, pos {};
    size_t j { 0 };

    for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster : clusters)
    {
        // Compute AABB for each cluster
        pcl::getMinMax3D(*cluster, min_point, max_point);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB %ld. min: (%f, %f, %f), max: (%f, %f, %f)",
            j++, min_point.x(), min_point.y(), min_point.z(), max_point.x(), max_point.y(), max_point.z());

        dim.x = max_point.x() - min_point.x();
        dim.y = max_point.y() - min_point.y();
        dim.z = max_point.z() - min_point.z();
        boxes->emplace_back(dim);

        pos.x = (min_point.x() + max_point.x()) / 2;
        pos.y = (min_point.y() + max_point.y()) / 2;
        pos.z = (min_point.z() + max_point.z()) / 2;
        boxes->emplace_back(pos);
        
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "AABB position: (%f, %f, %f) and dimensions: (%f, %f, %f)", 
        //     pos.x, pos.y, pos.z, dim.x, dim.y, dim.z);
    }

}

void perception_etflab::AABB::publish()
{
    sensor_msgs::msg::PointCloud2 output_cloud_ros;
    pcl::toROSMsg(*boxes, output_cloud_ros);
	// output_cloud_ros.header.stamp = now();
	publisher->publish(output_cloud_ros);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing %ld AABBs...", boxes->size() / 2);
}

void perception_etflab::AABB::visualize()
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "AABB";
    marker.header.frame_id = "world";
    // marker.header.stamp = now();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    for (size_t i = 0; i < boxes->size(); i += 2)
    {
        marker.id = i / 2;
        marker.scale.x = boxes->points[i].x;
        marker.scale.y = boxes->points[i].y;
        marker.scale.z = boxes->points[i].z;
        marker.pose.position.x = boxes->points[i+1].x;
        marker.pose.position.y = boxes->points[i+1].y;
        marker.pose.position.z = boxes->points[i+1].z;
        marker_array_msg.markers.emplace_back(marker);
    }
    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing AABBs...");
}
