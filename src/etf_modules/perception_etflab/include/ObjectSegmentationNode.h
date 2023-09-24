#include "Robot.h"
#include "environment/Clusters.h"
#include "environment/AABB.h"
#include "environment/ConvexHulls.h"
#include "environment/Obstacles.h"

// #include <geometry_msgs/msg/transform_stamped.h>
// #include <tf2_eigen/tf2_eigen.hpp>
// #include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace perception_etflab
{
	class ObjectSegmentationNode : public rclcpp::Node,
							   	   public perception_etflab::Robot,
								   public perception_etflab::Clusters,
								   public perception_etflab::AABB,
								   public perception_etflab::ConvexHulls,
								   public perception_etflab::Obstacles
	{
	public:
		ObjectSegmentationNode(const std::string node_name, const std::string config_file_path);

	protected:
		std::string input_cloud;
		std::string objects_cloud;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pcl_publisher;

		void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void publishObjectsPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
		void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
	};
}