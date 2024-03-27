#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <xArm6.h>
#include <RealVectorSpace.h>
#include <xarm_api/xarm_ros_client.h>

namespace perception_etflab
{
    class Robot
    {
    public:
        Robot(const std::string &config_file_path);

		inline float getTableRadius() const { return table_radius; }
        inline size_t getNumDOFs() const { return num_DOFs; }

		void jointsStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
		void removeFromScene(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
		void removeFromScene2(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
		void removeFromScene3(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
		void visualizeCapsules();
		void visualizeSkeleton();

		rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joints_state_subscription;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;

    private:
		std::shared_ptr<robots::AbstractRobot> robot;
		std::shared_ptr<Eigen::MatrixXf> skeleton;
		std::shared_ptr<base::State> joints_state;
		std::shared_ptr<rclcpp::Node> xarm_client_node;
		xarm_api::XArmROSClient xarm_client;
        std::vector<float> tolerance_factors;
		float table_radius;
        size_t num_DOFs;
    };
}