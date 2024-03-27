#include "demos/MoveRealXArm6Node.h"

real_bringup::MoveRealXArm6Node::MoveRealXArm6Node(const std::string &node_name, const std::string &config_file_path) : 
    sim_bringup::MoveXArm6Node(node_name, config_file_path) 
{
    // The defined service can only be activated at initialization if that service is configured to true. 
    // If you need to customize the parameters, please create a file xarm_api/config/xarm_user_params.yaml 
    // To modify the format, refer to xarm_api/config/xarm_params.yaml
    // See 5.4 xarm_api at: https://github.com/xArm-Developer/xarm_ros2/tree/humble
    xarm_client_node = std::make_shared<rclcpp::Node>("xarm_client_node");
    xarm_client.init(xarm_client_node, "xarm");

    // xarm_client.clean_error();
    // xarm_client.clean_warn();
    // xarm_client.motion_enable(true);

    // See 6.1 Mode Explanation at: https://github.com/xArm-Developer/xarm_ros#report_type-argument
    // Mode 0: xArm controller (position) mode
    // Mode 1: External trajectory planner (position) mode
    // xarm_client.set_mode(0);
    // xarm_client.set_state(0);

    // xarm_client.set_gripper_enable(true);
    // xarm_client.set_gripper_mode(0);
    // xarm_client.set_gripper_speed(3000);

    // set_position_node = std::make_shared<rclcpp::Node>("set_position_node");
    // set_position_client = set_position_node->create_client<xarm_msgs::srv::MoveCartesian>("/xarm/set_position");
    
    for (float coord : Robot::getHomeJointsPosition())
        home_angles.emplace_back(coord);
        
    state = going_home;
}

void real_bringup::MoveRealXArm6Node::moveRealXArm6Callback()
{
    switch (state)
    {
    case going_home:
        // goHome();
        // state = moving_in_joint_space;
        state = setting_position1;
        break;

    case moving_in_joint_space:
        moveInJointSpace();
        state = going_home;
        break;

    case setting_position1:
        setPosition({500, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        // xarm_client.set_position({500, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        state = setting_position2;
        break;

    case setting_position2:
        setPosition({400, 0, 133, 0, M_PI_2, 0}, 200, 1000);
        // xarm_client.set_position({400, 0, 133, 0, M_PI_2, 0}, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, false);
        state = closing_gripper;

    case closing_gripper:
        xarm_client.set_gripper_position(0);
        // moveGripper(0);
        state = opening_gripper;
        break;

    case opening_gripper:
        xarm_client.set_gripper_position(850, true, 1);
        // moveGripper(1);
        state = going_home;
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------------------"); 
}

void real_bringup::MoveRealXArm6Node::setPosition(const std::vector<float> &pose, float speed, float acceleration)
{
    while (!set_position_client->wait_for_service(1s))
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service is ready!");
    
    auto request { std::make_shared<xarm_msgs::srv::MoveCartesian::Request>() };
    request->pose = pose;
    request->speed = speed;
    request->acc = acceleration;

    auto result { set_position_client->async_send_request(request) };
    if (rclcpp::spin_until_future_complete(set_position_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ("Message: " + result.get()->message).c_str());
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set position!");
}

void real_bringup::MoveRealXArm6Node::testOrientation()
{
    std::vector<float> current_pose {};
    xarm_client.get_position(current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector pose: (%f, %f, %f)", current_pose[0], current_pose[1], current_pose[2]);     // XYZ in [m]
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot end-effector RPY:  (%f, %f, %f)", current_pose[3], current_pose[4], current_pose[5]);     // RPY angles in [rad]
    
    Eigen::Matrix3f R {};
    Eigen::Vector3f RPY {}, YPR {};

    // For approaching from above
    // R.col(0) << 0, 0, -1;
    // R.col(1) << -current_pose[1], current_pose[0], 0;
    // R.col(1) = R.col(1) / R.col(1).norm();
    // R.col(2) << current_pose[0], current_pose[1], 0;
    // R.col(2) = R.col(2) / R.col(2).norm();

    R = Eigen::AngleAxisf(current_pose[5], Eigen::Vector3f::UnitZ()) 
      * Eigen::AngleAxisf(current_pose[4], Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(current_pose[3], Eigen::Vector3f::UnitX());
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));

    YPR = R.eulerAngles(2, 1, 0);   // R = Rz(yaw) * Ry(pich) * Rx(roll)
    RPY << YPR(2), YPR(1), YPR(0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RPY: (%f, %f, %f)", RPY(0), RPY(1), RPY(2));

    R = Eigen::AngleAxisf(RPY(2), Eigen::Vector3f::UnitZ()) 
      * Eigen::AngleAxisf(RPY(1), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(RPY(0), Eigen::Vector3f::UnitX());
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));
}
