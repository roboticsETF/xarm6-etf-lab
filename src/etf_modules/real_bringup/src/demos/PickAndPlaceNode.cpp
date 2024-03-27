#include "demos/PickAndPlaceNode.h"

real_bringup::PickAndPlaceNode::PickAndPlaceNode(const std::string &node_name, const std::string &config_file_path) : 
    MoveRealXArm6Node(node_name, config_file_path) 
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node scenario { node["scenario"] };

    num_objects = scenario["num_objects"].as<size_t>();
    object_height = scenario["object_height"].as<float>();
    object_pick_z = scenario["object_pick_z"].as<float>();
    delta_theta1 = scenario["delta_theta1"].as<float>() * deg2rad;

    YAML::Node object_angles_approach_node { scenario["object_angles_approach"] };
    for (size_t i = 0; i < object_angles_approach_node.size(); i++)
        object_angles_approach.emplace_back(object_angles_approach_node[i].as<float>() * deg2rad);

    xarm_client.set_mode(0);
    xarm_client.set_state(0);

    task = approaching_to_object;
    num = 0;
}

void real_bringup::PickAndPlaceNode::pickAndPlaceCallback()
{
    switch (task)
    {
    case going_home:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going home..."); 
        xarm_client.set_servo_angle(home_angles, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        xarm_client.set_gripper_position(850, true, 2);
        num = 0;
        object_angles_approach[0] += delta_theta1;
        delta_theta1 *= -1;
        task = approaching_to_object;
        break;

    case approaching_to_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approaching to the object..."); 
        xarm_client.set_servo_angle(object_angles_approach, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        task = lowering_robot;
        break;
    
    case lowering_robot:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the robot...");
        xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - num * object_height;
        xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = grasping_object;
        break;

    case grasping_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasping the object..."); 
        xarm_client.set_gripper_position(400, true, 2);
        task = raising_object;
        break;

    case raising_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        current_pose[2] = 3*object_pick_z;
        xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = moving_object_to_goal;
        break;

    case moving_object_to_goal:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to the goal...");
        xarm_client.get_servo_angle(current_angles);
        current_angles[0] += delta_theta1;
        xarm_client.set_servo_angle(current_angles, Robot::getMaxVel(0), Robot::getMaxAcc(0), 0, true, 1, -1);
        task = lowering_object;
        break;

    case lowering_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lowering the object..."); 
        xarm_client.get_position(current_pose);
        current_pose[2] = object_pick_z - (2-num) * object_height;
        xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        task = releasing_object;
        break;

    case releasing_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object..."); 
        xarm_client.set_gripper_position(850, true, 2);
        task = raising_robot;
        break;

    case raising_robot:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the robot...");
        current_pose[2] = 3*object_pick_z;
        xarm_client.set_position(current_pose, -1, 0.7*Robot::getMaxLinVel(), 0.7*Robot::getMaxLinAcc(), 0, true, 1);
        if (++num < num_objects)
            task = approaching_to_object;
        else
            task = going_home;     // Repeat the sequence for the next object
        break;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------------------"); 
}
