#include "demos/BottleAndGlassNode.h"

int main(int argc, char *argv[])
{
	const std::string node_name = "bottle_and_glass_node";
	const std::string config_file_path = "/real_bringup/data/bottle_and_glass_config.yaml";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<real_bringup::BottleAndGlassNode>(node_name, config_file_path));
    rclcpp::shutdown();
    return 0;
}