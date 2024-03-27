#include "demos/BottleAndGlassNode.h"

real_bringup::BottleAndGlassNode::BottleAndGlassNode(const std::string &node_name, const std::string &config_file_path) : 
    MoveRealXArm6Node(node_name, config_file_path) 
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node scenario { node["scenario"] };

    YAML::Node glass_angles_approach_node { scenario["glass_angles_approach"] };
    for (size_t i = 0; i < glass_angles_approach_node.size(); i++)
        glass_angles_approach.emplace_back(glass_angles_approach_node[i].as<float>() * deg2rad);

    YAML::Node glass_angles_pick_node { scenario["glass_angles_pick"] };
    for (size_t i = 0; i < glass_angles_pick_node.size(); i++)
        glass_angles_pick.emplace_back(glass_angles_pick_node[i].as<float>() * deg2rad);

    YAML::Node bottle_pose_pick_node { scenario["bottle_pose_pick"] };
    for (size_t i = 0; i < bottle_pose_pick_node.size(); i++)
        bottle_pose_pick.emplace_back(bottle_pose_pick_node[i].as<float>());

    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    state = going_home;
    state_next = approaching_to_bottle;
    sign = 1;
}

void real_bringup::BottleAndGlassNode::bottleAndGlassCallback()
{
    switch (state)
    {
    case going_home:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        xarm_client.set_servo_angle(home_angles, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        xarm_client.set_gripper_position(850, true, 2);
        state = state_next;
        break;

    case approaching_to_bottle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching to the bottle..."); 
        xarm_client.set_position(bottle_pose_pick, -1, 0.5*Robot::getMaxLinVel(), 0.5*Robot::getMaxLinAcc(), 0, true, 1);
        state = grasping_bottle;
        break;

    case grasping_bottle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the bottle..."); 
        xarm_client.set_gripper_position(500, true, 2);
        state = raising_bottle;
        break;

    case raising_bottle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the bottle..."); 
        current_pose = bottle_pose_pick;
        current_pose[2] = 200;
        xarm_client.set_position(current_pose, -1, 0.5*Robot::getMaxLinVel(), 0.5*Robot::getMaxLinAcc(), 0, true, 1);
        state = moving_towards_glass1;
        break;

    case moving_towards_glass1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving towards the glass..."); 
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] += sign * M_PI_4;
        xarm_client.set_servo_angle(current_angles, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        state = pouring_water1;
        break;

    case pouring_water1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pouring the watter into the glass..."); 
        current_angles[5] -= sign * M_PI_2;
        xarm_client.set_servo_angle(current_angles, 0.7*Robot::getMaxVel(0), 0.7*Robot::getMaxAcc(0), 0, true, 1, -1);
        state = pouring_water2;
        break;
    
    case pouring_water2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pouring the watter into the glass..."); 
        current_angles[5] += sign * M_PI_2;
        xarm_client.set_servo_angle(current_angles, 0.7*Robot::getMaxVel(0), 0.7*Robot::getMaxAcc(0), 0, true, 1, -1);
        state = returning_bottle;
        break;

    case returning_bottle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Returning the bottle..."); 
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] -= sign * M_PI_4;
        xarm_client.set_servo_angle(current_angles, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        xarm_client.set_position(bottle_pose_pick, -1, Robot::getMaxLinVel(), Robot::getMaxLinAcc(), 0, true, 1);
        state = releasing_bottle;
        break;

    case releasing_bottle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the bottle...");
        xarm_client.set_gripper_position(850, true, 2);
        state = going_home;
        state_next = moving_towards_glass2;
        break;
    
    case moving_towards_glass2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving towards the glass...");
        xarm_client.set_servo_angle(glass_angles_approach, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        xarm_client.set_servo_angle(glass_angles_pick, 0.5*Robot::getMaxVel(0), 0.5*Robot::getMaxAcc(0), 0, true, 1, -1);
        state = grasping_glass;
        break;

    case grasping_glass:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the glass...");
        xarm_client.set_gripper_position(550, true, 2);
        state = moving_glass1;
        break;

    case moving_glass1:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the glass...");
        current_pose = bottle_pose_pick;
        current_pose[2] *= 5;
        xarm_client.set_position(current_pose, -1, Robot::getMaxLinVel(), Robot::getMaxLinAcc(), 0, true, 1);
        state = moving_glass2;
        break;
    
    case moving_glass2:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the glass...");
        glass_angles_approach[0] *= -1;
        glass_angles_pick[0] *= -1;
        sign *= -1;
        xarm_client.set_servo_angle(glass_angles_pick, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        state = releasing_glass;
        break;

    case releasing_glass:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the glass...");
        xarm_client.set_gripper_position(850, true, 2);
        xarm_client.set_servo_angle(glass_angles_approach, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        state = going_home;
        state_next = approaching_to_bottle;
        break;

    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}
