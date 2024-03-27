#include "environments/ConvexHulls.h"

// Make convex hull for each cluster from 'clusters'
void perception_etflab::ConvexHulls::make(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
{
    // Create convex-hull for each cluster
    pcl::ConvexHull<pcl::PointXYZRGB> convex_hull {};
    points = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    polygons_indices = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        std::vector<pcl::Vertices> polygons {};
        convex_hull.setInputCloud(clusters[i]);
        convex_hull.reconstruct(*points_, polygons);
        points_->emplace_back(pcl::PointXYZRGB(0.0, 0.0, 0.0, 0, 0, 0)); // This point is just delimiter to distinguish clusters

        for (pcl::Vertices &polygon : polygons)
        {
            polygons_indices->emplace_back(pcl::PointXYZ(polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]));
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Indices: (%ld, %ld, %ld)", polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]);
        }
        polygons_indices->emplace_back(pcl::PointXYZ(-1, -1, -1));  // This point is just delimiter to distinguish clusters
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Convex-hull %ld contains the following %ld points: ", i, points_->size());
        for (pcl::PointXYZRGB point : points_->points)
        {
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "(%f, %f, %f)", point.x, point.y, point.z);
            points->emplace_back(point);
        }
    }
}

void perception_etflab::ConvexHulls::publish()
{
    sensor_msgs::msg::PointCloud2 output_cloud_ros;
    pcl::toROSMsg(*points, output_cloud_ros);
	// output_cloud_ros.header.stamp = now();
	points_publisher->publish(output_cloud_ros);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing %ld points of convex-hulls...", points->size());

    pcl::toROSMsg(*polygons_indices, output_cloud_ros);
	// output_cloud_ros.header.stamp = now();
	polygons_publisher->publish(output_cloud_ros);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing %ld points of convex-hulls polygons indices...", polygons_indices->size());

}

void perception_etflab::ConvexHulls::visualize()
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "convex_hulls";
    marker.header.frame_id = "world";
    // marker.header.stamp = now();
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    size_t j = 0;
    for (size_t i = 0; i < points->size(); i++)
    {
        pcl::PointXYZRGB P = points->points[i];
        if (P.x == 0.0 && P.y == 0.0 && P.z == 0.0)     // This point is just delimiter to distinguish different clusters
        {
            marker.id = j++;
            marker_array_msg.markers.emplace_back(marker);
            marker.points.clear();
        }
        else
        {
            geometry_msgs::msg::Point point;
            point.x = P.x; 
            point.y = P.y; 
            point.z = P.z;
            marker.points.emplace_back(point);
        }
    }    

    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing convex-hulls...");
}
