#include "planning/RealTimePlanningNode.h"

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	const std::string node_name = "real_time_planning_node";
	const std::string config_file_path = "/sim_bringup/data/real_time_planning_config.yaml";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_bringup::RealTimePlanningNode>(node_name, config_file_path));
    rclcpp::shutdown();
	google::ShutDownCommandLineFlags();
    return 0;
}