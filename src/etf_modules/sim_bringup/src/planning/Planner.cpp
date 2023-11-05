#include "planning/Planner.h"

sim_bringup::Planner::Planner(const std::string config_file_path)
{
    std::string project_abs_path = std::string(__FILE__);
    for (int i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));

    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    name = node["planner"]["name"].as<std::string>();
    setMaxPlanningTime(node["planner"]["max_planning_time"].as<float>());
}

// 'max_planning_time' must be in [ms]
void sim_bringup::Planner::setMaxPlanningTime(float max_planning_time)
{
    if (name == "RRTConnect")
        RRTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
    else if (name == "RBTConnect")
        RBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
    else if (name == "RGBTConnect")
        RGBTConnectConfig::MAX_PLANNING_TIME = max_planning_time;
    else if (name == "RGBMT*")
        RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
    else
    {
        std::cout << "The requested planner is not found! Using RGBMT*. \n";
        RGBMTStarConfig::MAX_PLANNING_TIME = max_planning_time;
    }
}

bool sim_bringup::Planner::solve(std::shared_ptr<base::State> start)
{
    ready = false;
    bool res = false;
    if (start == nullptr)
        start = scenario->getStart();
    std::shared_ptr<base::State> goal = scenario->getGoal();
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    
    LOG(INFO) << "Number of collision objects: " << scenario->getEnvironment()->getParts().size();
    LOG(INFO) << "Number of DOFs: " << ss->getNumDimensions();
    LOG(INFO) << "State space type: " << ss->getStateSpaceType();
    LOG(INFO) << "Start: " << start;
    LOG(INFO) << "Goal: " << goal;

    try
    {
        std::unique_ptr<planning::AbstractPlanner> planner;
        if (name == "RRTConnect")
            planner = std::make_unique<planning::rrt::RRTConnect>(ss, start, goal);
        else if (name == "RBTConnect")
            planner = std::make_unique<planning::rbt::RBTConnect>(ss, start, goal);
        else if (name == "RGBTConnect")
            planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
        else if (name == "RGBMT*")
            planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start, goal);
        else
            planner = std::make_unique<planning::rbt_star::RGBMTStar>(ss, start, goal);
        
        res = planner->solve();
        LOG(INFO) << name << " planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of states in the trees: " << planner->getPlannerInfo()->getNumStates();
        LOG(INFO) << "Number of states in the path: " << planner->getPath().size();
        LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
        if (name == "RGBMT*")
            LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();

        // Just for debugging (Not recommended to waste time!)
        // std::string project_abs_path = std::string(__FILE__);
        // for (int i = 0; i < 4; i++)
        //     project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
        // planner->outputPlannerData(project_abs_path + "/sim_bringup/data/planner_data.log");

        if (res)
            path = planner->getPath();
    }
    catch (std::exception &e)
    {
        LOG(ERROR) << e.what();
    }

    ready = true;
    return res;
}