#include "base/Planner.h"

sim_bringup::Planner::Planner(const std::string &config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    try
    {
        YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
        YAML::Node planner_node = node["planner"];
        if (!planner_node.IsDefined())
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Planner is not defined!");
            return;
        }
        
        ConfigurationReader::initConfiguration(project_abs_path + planner_node["configurations"].as<std::string>());
        planner = nullptr;
        name = planner_node["name"].as<std::string>();

        YAML::Node max_planning_time_node = planner_node["max_planning_time"];
        max_planning_time = (max_planning_time_node.IsDefined()) ? max_planning_time_node.as<int>() : INFINITY;
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
    
    ready = true;
}

bool sim_bringup::Planner::solve(std::shared_ptr<base::State> q_start, std::shared_ptr<base::State> q_goal, int max_planning_time_)
{
    ready = false;
    bool result = false;

    if (q_start == nullptr)
        q_start = scenario->getStart();
    else
        scenario->setStart(q_start);

    if (q_goal == nullptr)
        q_goal = scenario->getGoal();
    else
        scenario->setGoal(q_goal);

    if (max_planning_time_ != -1)
        max_planning_time = max_planning_time_;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Number of collision objects: %d", scenario->getEnvironment()->getNumObjects());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Start: "); std::cout << q_start << "\n";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Goal: "); std::cout << q_goal << "\n";

    try
    {
        if (name == "RGBMT*")
        {
            RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt_star::RGBMTStar>(scenario->getStateSpace(), q_start, q_goal);
        }
        else if (name == "RGBTConnect")
        {
            RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt::RGBTConnect>(scenario->getStateSpace(), q_start, q_goal);
        }
        else if (name == "RBTConnect")
        {
            RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rbt::RBTConnect>(scenario->getStateSpace(), q_start, q_goal);
        }
        else if (name == "RRTConnect")
        {
            RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
            planner = std::make_unique<planning::rrt::RRTConnect>(scenario->getStateSpace(), q_start, q_goal);
        }
        else
            throw std::domain_error("The requested static planner is not found! ");

        result = planner->solve();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t %s planning finished with %s ", 
            name.c_str(), (result ? std::string("SUCCESS!").c_str() : std::string("FAILURE!").c_str()));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Number of states in the path: %d", planner->getPath().size());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Planning time: %d [ms]", planner->getPlannerInfo()->getPlanningTime());
        if (name == "RGBMT*")
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\t Path cost: %f", planner->getPlannerInfo()->getCostConvergence().back());

        // Just for debugging (Not recommended to waste time!)
        // std::string project_abs_path = std::string(__FILE__);
        // for (int i = 0; i < 4; i++)
        //     project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        // planner->outputPlannerData(project_abs_path + "/sim_bringup/data/planner_data.log");
    }
    catch (std::exception &e)
    {
        LOG(ERROR) << e.what();
    }

    ready = true;
    return result;
}