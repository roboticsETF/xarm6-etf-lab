//
// Created by nermin on 28.08.23.
//

#include "planning/RealTimePlanningNode.h"

sim_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string node_name, const std::string config_file_path) : 
    PlanningNode(node_name, config_file_path),
    DRGBTConnect(scenario->getStateSpace(), scenario->getStart(), scenario->getGoal())
{
    DRGBTConnectConfig::MAX_ITER_TIME = BaseNode::period;
    time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    init_iteration = true;
}

void sim_bringup::RealTimePlanningNode::planningCallback()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nIteration num. %d", DRGBTConnect::planner_info->getNumIterations());
    time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock

    // Initial iteration: Obtaining the inital path using specified static planner
    if (init_iteration)
    {
        AABB::updateEnvironment();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining the inital path...");
        replan(DRGBTConnectConfig::MAX_ITER_TIME);
        
        DRGBTConnect::planner_info->setNumIterations(DRGBTConnect::planner_info->getNumIterations() + 1);
        DRGBTConnect::planner_info->addIterationTime(DRGBTConnect::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------\n");
        init_iteration = false;
        return;
    }

    // Update environment and check if the collision occurs
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating environment..."); 
    AABB::updateEnvironment();
    if (!DRGBTConnect::ss->isValid(DRGBTConnect::q_current))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "*************** Collision has been occured!!! ***************");
        // DRGBTConnect::planner_info->setSuccessState(false);
        // DRGBTConnect::planner_info->setPlanningTime(DRGBTConnect::planner_info->getIterationTimes().back());
        // rclcpp::shutdown();
        return;
    }

    if (DRGBTConnect::status != base::State::Status::Advanced)
    {
        // Generate a horizon
        DRGBTConnect::generateHorizon();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial horizon consists of %d states.", DRGBTConnect::horizon.size());
        // for (int i = 0; i < DRGBTConnect::horizon.size(); i++)
        //     std::cout << i << ". state:\n" << DRGBTConnect::horizon[i] << std::endl; 
    }

    // Moving from 'q_current' towards 'q_next'
    // Compute the horizon and the next state
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing horizon...");
    DRGBTConnect::computeHorizon();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing the next state...");
    DRGBTConnect::computeNextState();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Horizon consists of %d states.", DRGBTConnect::horizon.size());
    for (int i = 0; i < DRGBTConnect::horizon.size(); i++)
        std::cout << i << ". state:\n" << DRGBTConnect::horizon[i] << std::endl;

    // Update the robot current state
    RealTimePlanningNode::updateCurrentState();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status: %s%s%s.",
        (DRGBTConnect::status == base::State::Status::Advanced ? std::string("Advanced").c_str() : ""),
        (DRGBTConnect::status == base::State::Status::Trapped  ? std::string("Trapped").c_str()  : ""),
        (DRGBTConnect::status == base::State::Status::Reached  ? std::string("Reached").c_str()  : ""));
    
    // Replanning procedure assessment
    if (DRGBTConnect::replanning || DRGBTConnect::whetherToReplan())
    {
        float time_remaining = DRGBTConnectConfig::MAX_ITER_TIME 
                               - DRGBTConnect::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()) 
                               - 1;     // 1 [ms] is reserved for the following code lines
        replan(time_remaining);
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is not required! ");

    // Checking the real-time execution
    auto time_current = std::chrono::steady_clock::now();
    float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - DRGBTConnect::getElapsedTime(time_iter_start, time_current);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remaining iteration time is %f [ms].", time_iter_remain);
    if (time_iter_remain < 0)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "*************** Real-time is broken. %f [ms] exceeded!!! ***************",
                    -time_iter_remain);

    // Planner info and terminating condition
    DRGBTConnect::planner_info->setNumIterations(DRGBTConnect::planner_info->getNumIterations() + 1);
    DRGBTConnect::planner_info->addIterationTime(DRGBTConnect::getElapsedTime(time_alg_start, time_current));
    if (DRGBTConnect::ss->isEqual(DRGBTConnect::q_current, scenario->getGoal()))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal configuration has been successfully reached!");
        rclcpp::shutdown();

        // Swap start and goal for next motion, and repeat the procedure
        // std::shared_ptr<base::State> start = scenario->getStart();
        // std::shared_ptr<base::State> goal = scenario->getGoal();
        // scenario->setStart(goal);
        // scenario->setGoal(start);
        // init_iteration = true;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------\n");
}

void sim_bringup::RealTimePlanningNode::replan(float replanning_time)
{
    try
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", replanning_time);
        Planner::setMaxPlanningTime(replanning_time);
        
        if (Planner::solve(DRGBTConnect::q_current))   // New path is found, thus update predefined path to the goal
        {
            // First, interpolation of the predefined path is done in order that the horizon contains more states from the path
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path has been replanned.");
            DRGBTConnect::predefined_path.clear();
            DRGBTConnect::predefined_path.emplace_back(Planner::getPath().front());
            base::State::Status status;
            std::shared_ptr<base::State> q_new;
            float delta_q_max = std::sqrt(DRGBTConnect::ss->getNumDimensions()) * Robot::getMaxAngVel() 
                                * DRGBTConnectConfig::MAX_ITER_TIME / 1000;     // length of a hyper-diagonal

            for (int i = 1; i < Planner::getPath().size(); i++)
            {
                status = base::State::Status::Advanced;
                q_new = Planner::getPath()[i-1];
                while (status == base::State::Status::Advanced)
                {
                    std::tie(status, q_new) = DRGBTConnect::ss->interpolateEdge2(q_new, Planner::getPath()[i], delta_q_max);
                    DRGBTConnect::predefined_path.emplace_back(q_new);
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Predefined path is: ");
            for (int i = 0; i < DRGBTConnect::predefined_path.size(); i++)
                std::cout << i << ": (" << DRGBTConnect::predefined_path.at(i)->getCoord().transpose() << ")\n";
            
            DRGBTConnect::replanning = false;
            DRGBTConnect::status = base::State::Status::Reached;
            DRGBTConnect::q_next = std::make_shared<planning::drbt::HorizonState>(DRGBTConnect::predefined_path.front(), 0);
            DRGBTConnect::horizon.clear();
        }
        else    // New path is not found
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is required. %s", e.what());
        DRGBTConnect::replanning = true;
    }
}

// Update the current state of the robot. If possible, move towards 'q_next' for step size 'step'.
void sim_bringup::RealTimePlanningNode::updateCurrentState()
{
    std::shared_ptr<base::State> q_new = DRGBTConnect::ss->getNewState(DRGBTConnect::q_next->getStateReached());
    const float time_remain = (DRGBTConnectConfig::MAX_ITER_TIME 
                              - DRGBTConnect::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()))
                              / 1000.0;   // Conversion from [ms] to [s]
    const float delta_q_max = Robot::getMaxAngVel() * time_remain;

    if (DRGBTConnect::ss->getDistance(DRGBTConnect::q_current, q_new) > delta_q_max)
    {
        // Check whether 'q_new' can be reached considering robot max. velocity
        std::vector<std::vector<float>> limits;
        for (int i = 0; i < DRGBTConnect::ss->getNumDimensions(); i++)
            limits.emplace_back(std::vector<float>({DRGBTConnect::q_current->getCoord(i) - delta_q_max, 
                                                    DRGBTConnect::q_current->getCoord(i) + delta_q_max}));
        q_new = DRGBTConnect::ss->pruneEdge(DRGBTConnect::q_current, q_new, limits);
    }

    if (!DRGBTConnect::ss->isEqual(DRGBTConnect::q_current, q_new))
    {
        if (DRGBTConnect::ss->isEqual(q_new, DRGBTConnect::q_next->getStateReached()))
        {
            std::vector<std::shared_ptr<base::State>> path = {q_new};
            if (!DRGBTConnect::q_next->getIsReached())
                path.emplace_back(DRGBTConnect::q_next->getState());
            else if (DRGBTConnect::q_next->getIndex() != -1 && 
                     DRGBTConnect::q_next->getStatus() != planning::drbt::HorizonState::Status::Goal)
                path.emplace_back(DRGBTConnect::predefined_path[DRGBTConnect::q_next->getIndex() + 1]);
            
            DRGBTConnect::status = base::State::Status::Reached;
            Trajectory::addPath(path, false, time_remain);
        }
        else
        {
            DRGBTConnect::status = base::State::Status::Advanced;
            Trajectory::addPath({q_new, DRGBTConnect::q_next->getStateReached()}, false, time_remain);
        }
        
        q_new->setParent(DRGBTConnect::q_current);
        DRGBTConnect::q_current = q_new;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the robot current state to: ");
            std::cout << "(" << DRGBTConnect::q_current->getCoord().transpose() << ")" << std::endl;
    }
    else
    {
        DRGBTConnect::status = base::State::Status::Trapped;
        DRGBTConnect::replanning = true;
        DRGBTConnect::horizon.clear();
        Trajectory::addPath({q_new}, false, time_remain);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not updating the robot current state!");
    }

    DRGBTConnect::path.emplace_back(DRGBTConnect::q_current);
    Trajectory::publish();
}