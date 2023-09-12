//
// Created by nermin on 28.08.23.
//

#include "planning/RealTimePlanningNode.h"

sim_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string node_name, const std::string config_file_path) : 
    PlanningNode(node_name, config_file_path),
    DRGBTConnect(scenario->getStateSpace(), scenario->getStart(), scenario->getGoal())
{
    YAML::Node node = YAML::LoadFile(project_abs_path + config_file_path);
    DRGBTConnectConfig::MAX_ITER_TIME = period;

    timer_computing_next_state = this->create_wall_timer(std::chrono::milliseconds(period), 
        std::bind(&RealTimePlanningNode::computingNextStateCallback, this));
    timer_replanning = this->create_wall_timer(std::chrono::milliseconds(node["planner"]["max_planning_time"].as<int>() + 50), // 50 [ms] is added as a reserve 
        std::bind(&RealTimePlanningNode::replanningCallback, this));
    
    time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    time_iter_start = time_alg_start;                      // Start the iteration clock
    time_current = time_alg_start;

    // Obtaining the inital path using specified planner
    Planner::planPath();
    predefined_path = Planner::getPath();
    state = generating_horizon;
}

void sim_bringup::RealTimePlanningNode::computingNextStateCallback()
{
    switch (state)
    {
    case generating_horizon:
        // Generate a horizon
        generateHorizon();
        // LOG(INFO) << "Initial horizon consists of " << horizon.size() << " states: ";
        // for (int i = 0; i < horizon.size(); i++)
        //     LOG(INFO) << i << ". state:\n" << horizon[i]; 
        state = moving_to_next_state;
        break;
    
    case moving_to_next_state:
        // Moving from 'q_current' towards 'q_next' for step 'DRGBTConnectConfig::STEP', where 'q_next' may change
        // LOG(INFO) << "\n\nIteration num. " << planner_info->getNumIterations()
        //           << " -----------------------------------------------------------------------------------------";

        // Update environment and check if the collision occurs
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
        AABB::updateEnvironment();
        if (!ss->isValid(q_current))
        {
            LOG(INFO) << "Collision has been occured!!! ";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            rclcpp::shutdown();
            return;
        }
        
        // Compute the horizon and the next state
        computeHorizon();
        computeNextState();
        // LOG(INFO) << "Horizon consists of " << horizon.size() << " states: ";
        // for (int i = 0; i < horizon.size(); i++)
        //     LOG(INFO) << i << ". state:\n" << horizon[i];

        // Update the robot current state
        updateCurrentState();
        Trajectory::addPath({q_current, q_next->getState()}, false, period / 1000.0);
        Trajectory::publish();
        // LOG(INFO) << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
        //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
        //                         << (status == base::State::Status::Reached  ? "Reached"  : "");
        
        // Checking the real-time execution
        time_current = std::chrono::steady_clock::now();
        float time_iter_current = getElapsedTime(time_iter_start, time_current);
        float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - time_iter_current;
        // LOG(INFO) << "Remaining iteration time is " << time_iter_remain << " [ms]. ";
        if (time_iter_remain < 0)
        {
            LOG(INFO) << "*************** Real-time is broken. " << -time_iter_remain << " [ms] exceeded!!! ***************";
        }
        time_iter_start = time_current;

        // Planner info and terminating condition
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
        planner_info->addIterationTime(getElapsedTime(time_alg_start, time_current));

        if (status != base::State::Status::Advanced)
        {
            LOG(INFO) << "Next state is " << (status == base::State::Status::Reached ? "reached!" : "not reached!") 
                << "\n***************************************************************************";
            state = generating_horizon;
        }

        if (Robot::isReached(scenario->getGoal()))
        {
            std::shared_ptr<base::State> start = scenario->getStart();
            std::shared_ptr<base::State> goal = scenario->getGoal();

            // Swap start and goal for next motion, and repeat the procedure
            scenario->setStart(goal);
            scenario->setGoal(start);
            state = generating_horizon;
        }
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "--------------------------------computingNextState finished--------------------------------\n");
}

void sim_bringup::RealTimePlanningNode::replanningCallback()
{
    // Replanning procedure assessment
    if (replanning || whetherToReplan())
    {
        try
        {
            LOG(INFO) << "Trying to replan...";
            if (Planner::planPath(q_current))   // New path is found, thus update predefined path to the goal
            {
                LOG(INFO) << "The path has been replanned.";
                // LOG(INFO) << "The path has been replanned in " << Planner::getPlannerInfo()->getPlanningTime() << " [ms]. ";
                // LOG(INFO) << "Predefined path is: ";
                predefined_path = Planner::getPath();
                // for (int i = 0; i < predefined_path.size(); i++)
                //     std::cout << predefined_path.at(i) << std::endl;
                replanning = false;
                status = base::State::Status::Reached;
                q_next = std::make_shared<planning::drbt::HorizonState>(predefined_path.front(), 0);
                horizon.clear();
                // planner_info->addRoutineTime(Planner::getPlannerInfo()->getPlanningTime(), 4);
            }
            else    // New path is not found
                throw std::runtime_error("New path is not found! ");
        }
        catch (std::exception &e)
        {
            // LOG(INFO) << "Replanning is required. " << e.what();
            replanning = true;
        }
    }
    // else
    //     LOG(INFO) << "Replanning is not required! ";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "---------------------------------replanning finished-------------------------------\n");
}
