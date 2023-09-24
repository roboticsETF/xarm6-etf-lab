#include "planning/PlanningNode.h"

real_bringup::PlanningNode::PlanningNode(const std::string node_name, const std::string config_file_path) : 
    BaseNode(node_name, config_file_path),
    Planner(config_file_path),
    AABB(config_file_path)
{
    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    ConfigurationReader::initConfiguration(project_abs_path + node["planner"]["configurations"].as<std::string>());

    scenario = std::make_shared<scenario::Scenario>(config_file_path, project_abs_path);
    Planner::setScenario(scenario);
    Planner::setMaxPlanningTime(node["planner"]["max_planning_time"].as<float>());  // Needs to be set again, since 'initConfiguration' resets it

    AABB::setEnvironment(scenario->getEnvironment());
    if (AABB::getMinNumCaptures() == 0)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    state = waiting;
}

void real_bringup::PlanningNode::planningCallback()
{
    switch (state)
    {
    case waiting:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting...");
        if (Robot::isReady())
        {
            scenario->setStart(Robot::getJointsState());
            state = planning;
        }
        break;

    case planning:
        if (Planner::isReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
            AABB::updateEnvironment();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
            if (Planner::solve())
            {
                Trajectory::addPath(Planner::getPath());
                state = publishing_trajectory;
            }
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the planner..."); 
        break;
    
    case publishing_trajectory:
        Trajectory::publish();
        state = executing_trajectory;
        break;

    case executing_trajectory:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");
        if (Robot::isReached(scenario->getGoal()))
        {
            std::shared_ptr<base::State> start = scenario->getStart();
            std::shared_ptr<base::State> goal = scenario->getGoal();

            // Swap start and goal for next motion, and repeat the procedure
            scenario->setStart(goal);
            scenario->setGoal(start);
            state = planning;
        }
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}
