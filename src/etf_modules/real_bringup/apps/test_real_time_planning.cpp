#include "planning/RealTimePlanningNode.h"

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name = "real_time_planning_node";
	const std::string config_file_path1 = "/real_bringup/data/real_time_planning_config1.yaml";
	const std::string config_file_path2 = "/real_bringup/data/real_time_planning_config2.yaml";

	while (true)
	{
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path1));
		rclcpp::shutdown();

		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<real_bringup::RealTimePlanningNode>(node_name, config_file_path2));
		rclcpp::shutdown();
	}

	google::ShutDownCommandLineFlags();
    return 0;
}