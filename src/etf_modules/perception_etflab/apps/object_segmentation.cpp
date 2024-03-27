#include "ObjectSegmentationNode.h"

int main(int argc, char *argv[])
{
	const std::string node_name = "object_segmentation_node";
	// const std::string config_file_path = "/perception_etflab/data/sim_perception_etflab_config.yaml";
	const std::string config_file_path = "/perception_etflab/data/real_perception_etflab_config.yaml";

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<perception_etflab::ObjectSegmentationNode>(node_name, config_file_path));
	rclcpp::shutdown();
	return 0;
}