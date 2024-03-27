#include "demos/MoveXArm6Node.h"

sim_bringup::MoveXArm6Node::MoveXArm6Node(const std::string &node_name, const std::string &config_file_path) : 
    BaseNode(node_name, config_file_path) 
{
    state = going_home;
}

void sim_bringup::MoveXArm6Node::moveXArm6Callback()
{
    switch (state)
    {
    case going_home:
        goHome();
        state = moving_in_joint_space;
        break;

    case moving_in_joint_space:
        moveInJointSpace();
        state = going_home;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void sim_bringup::MoveXArm6Node::goHome()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home...");
    Trajectory::clear();
    Trajectory::addPoint(0.9 * period, Robot::getHomeJointsPosition());
    Trajectory::publish();
}

void sim_bringup::MoveXArm6Node::moveInJointSpace()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving in joint space...");
    std::vector<Eigen::VectorXf> path {};
    std::vector<float> time_instances {};
    Eigen::VectorXf q(6);

    q << -M_PI_2, 0, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    time_instances.emplace_back(2);
    
    q << -M_PI_2, -M_PI_4, 0, M_PI, M_PI_2, 0;
    path.emplace_back(q);
    time_instances.emplace_back(4);

    Trajectory::clear();
    Trajectory::addPath(path, time_instances);
    Trajectory::publish();
}
