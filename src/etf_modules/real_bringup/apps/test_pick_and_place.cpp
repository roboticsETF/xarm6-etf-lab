#include "demos/PickAndPlaceNode.h"

int main(int argc, char *argv[])
{
    const std::string node_name = "pick_and_place_node";
	const std::string config_file_path = "/real_bringup/data/pick_and_place_config.yaml";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::PickAndPlaceNode>(node_name, config_file_path));
    rclcpp::shutdown();
    return 0;
}