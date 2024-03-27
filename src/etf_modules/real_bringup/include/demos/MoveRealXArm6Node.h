#include "demos/MoveXArm6Node.h"

#include <xarm_api/xarm_ros_client.h>
#include <xarm_msgs/srv/move_cartesian.hpp>

#define deg2rad (M_PI / 180)

namespace real_bringup
{
    class MoveRealXArm6Node : public sim_bringup::MoveXArm6Node
    {
    public:
        MoveRealXArm6Node(const std::string &node_name, const std::string &config_file_path);

    protected:
        void moveXArm6Callback() override { moveRealXArm6Callback(); }
        virtual void moveRealXArm6Callback();
        void setPosition(const std::vector<float> &pose, float speed = 100, float acceleration = 1000);
        void testOrientation();

        std::shared_ptr<rclcpp::Node> xarm_client_node;
        xarm_api::XArmROSClient xarm_client;

        std::shared_ptr<rclcpp::Node> set_position_node;
        rclcpp::Client<xarm_msgs::srv::MoveCartesian>::SharedPtr set_position_client;
        
        std::vector<float> home_angles;
        std::vector<float> current_angles;
        std::vector<float> current_pose;

    private:
        enum State
        {
            going_home,
            moving_in_joint_space,
            setting_position1,
            setting_position2,
            closing_gripper,
            opening_gripper
        };
        State state;
    };
}