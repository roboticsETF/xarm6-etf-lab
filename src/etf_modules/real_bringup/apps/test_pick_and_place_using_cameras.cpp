#include "demos/PickAndPlaceUsingCamerasNode.h"

int main(int argc, char *argv[])
{
    const std::string node_name = "pick_and_place_using_cameras_node";
	const std::string config_file_path = "/real_bringup/data/pick_and_place_using_cameras_config.yaml";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::PickAndPlaceUsingCamerasNode>(node_name, config_file_path));
    rclcpp::shutdown();
    return 0;
}