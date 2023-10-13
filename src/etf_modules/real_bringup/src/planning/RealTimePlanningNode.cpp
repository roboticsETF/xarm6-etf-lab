//
// Created by nermin on 28.08.23.
//

#include "planning/RealTimePlanningNode.h"

typedef planning::drbt::DRGBTConnect DP;    // 'DP' is Dynamic Planner

real_bringup::RealTimePlanningNode::RealTimePlanningNode(const std::string node_name, const std::string config_file_path) : 
    PlanningNode(node_name, config_file_path),
    DP(scenario->getStateSpace(), scenario->getStart(), scenario->getGoal())
{
    DRGBTConnectConfig::MAX_ITER_TIME = BaseNode::period;
    if (Planner::getName() == "RGBMT*")
        RGBMTStarConfig::TERMINATE_WHEN_PATH_IS_FOUND = true;
    
    q_next_prev = nullptr;
    q_current_prev = nullptr;
    time_alg_start = std::chrono::steady_clock::now();      // Start the algorithm clock
}

void real_bringup::RealTimePlanningNode::planningCallback()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\nIteration num. %d", DP::planner_info->getNumIterations());
    time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock

    // Initial iteration: Obtaining the inital path using specified static planner
    if (DP::planner_info->getNumIterations() == 0)
    {
        AABB::updateEnvironment();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtaining the inital path...");
        replan(DRGBTConnectConfig::MAX_ITER_TIME);
        
        DP::planner_info->setNumIterations(DP::planner_info->getNumIterations() + 1);
        DP::planner_info->addIterationTime(DP::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------\n");
        return;
    }

    // Update environment and check if the collision occurs
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating environment..."); 
    AABB::updateEnvironment();
    if (!DP::ss->isValid(DP::q_current))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********** Robot is stopping. Collision has been occured!!! **********");
        // DP::planner_info->setSuccessState(false);
        // DP::planner_info->setPlanningTime(DP::planner_info->getIterationTimes().back());
        // rclcpp::shutdown();
        return;
    }

    if (DP::status != base::State::Status::Advanced)
    {
        // Generate a horizon
        DP::generateHorizon();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initial horizon consists of %d states.", DP::horizon.size());
        // for (int i = 0; i < DP::horizon.size(); i++)
        //     std::cout << i << ". state:\n" << DP::horizon[i] << std::endl; 
    }

    // Moving from 'q_current' towards 'q_next'
    // Compute the horizon and the next state
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing horizon...");
    DP::computeHorizon();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computing the next state...");
    DP::computeNextState();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Horizon consists of %d states.", DP::horizon.size());
    // for (int i = 0; i < DP::horizon.size(); i++)
    //     std::cout << i << ". state:\n" << DP::horizon[i] << std::endl;

    // Update the robot current state
    RealTimePlanningNode::updateCurrentState();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status: %s%s%s.",
        (DP::status == base::State::Status::Advanced ? std::string("Advanced").c_str() : ""),
        (DP::status == base::State::Status::Trapped  ? std::string("Trapped").c_str()  : ""),
        (DP::status == base::State::Status::Reached  ? std::string("Reached").c_str()  : ""));
    
    // Replanning procedure assessment
    if (DP::replanning || DP::whetherToReplan())
    {
        float time_remaining = DRGBTConnectConfig::MAX_ITER_TIME 
                               - DP::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()) 
                               - 10;     // 10 [ms] is reserved for the following code lines
        replan(time_remaining);
    }
    else
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is not required! ");

    // Checking the real-time execution
    auto time_current = std::chrono::steady_clock::now();
    float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - DP::getElapsedTime(time_iter_start, time_current);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remaining iteration time is %f [ms].", time_iter_remain);
    if (time_iter_remain < 0)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********** Real-time is broken. %f [ms] exceeded!!! **********",
                    -time_iter_remain);

    // Planner info and terminating condition
    DP::planner_info->setNumIterations(DP::planner_info->getNumIterations() + 1);
    DP::planner_info->addIterationTime(DP::getElapsedTime(time_alg_start, time_current));
    if (DP::ss->isEqual(DP::q_current, scenario->getGoal()))
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal configuration has been successfully reached!\n");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------------------\n");
}

// Replan the predefined path to the goal
void real_bringup::RealTimePlanningNode::replan(float replanning_time)
{
    try
    {
        if (replanning_time < 0)
            throw std::runtime_error("Not enough time for replanning! ");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trying to replan in %f [ms]...", replanning_time);
        Planner::setMaxPlanningTime(replanning_time);
        bool result = false;

        // std::thread th([this, &result]() 
        // {
            result = Planner::solve(DP::q_current);
        // });
        // std::this_thread::sleep_for(std::chrono::milliseconds(int(replanning_time)));
        // th.detach();

        if (result)   // New path is found, thus update predefined path to the goal
        {
            // First, interpolation of the predefined path is done in order that the horizon contains more states from the path
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path has been replanned.");
            DP::predefined_path.clear();
            DP::predefined_path.emplace_back(Planner::getPath().front());
            base::State::Status status;
            std::shared_ptr<base::State> q_new;
            float delta_q_max = std::sqrt(DP::ss->getNumDimensions()) * Robot::getMaxAngVel() 
                                * DRGBTConnectConfig::MAX_ITER_TIME / 1000;     // length of a hyper-diagonal

            for (int i = 1; i < Planner::getPath().size(); i++)
            {
                status = base::State::Status::Advanced;
                q_new = Planner::getPath()[i-1];
                while (status == base::State::Status::Advanced)
                {
                    std::tie(status, q_new) = DP::ss->interpolateEdge2(q_new, Planner::getPath()[i], delta_q_max);
                    DP::predefined_path.emplace_back(q_new);
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Predefined path is: ");
            for (int i = 0; i < DP::predefined_path.size(); i++)
                std::cout << i << ": (" << DP::predefined_path.at(i)->getCoord().transpose() << ")\n";
            
            DP::replanning = false;
            DP::status = base::State::Status::Reached;
            DP::q_next = std::make_shared<planning::drbt::HorizonState>(DP::predefined_path.front(), 0);
            DP::horizon.clear();
        }
        else    // New path is not found
            throw std::runtime_error("New path is not found! ");
    }
    catch (std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Replanning is required. %s", e.what());
        DP::replanning = true;
    }
}

// Update the current state of the robot towards 'q_next' moving with maximal velocity
void real_bringup::RealTimePlanningNode::updateCurrentState()
{
    std::shared_ptr<base::State> q_new = DP::ss->getNewState(DP::q_next->getStateReached());
    float time_remain = (DRGBTConnectConfig::MAX_ITER_TIME 
                        - DP::getElapsedTime(time_iter_start, std::chrono::steady_clock::now()));
    float delta_q_max = Robot::getMaxAngVel() * time_remain / 1000;     // Time conversion from [ms] to [s]

    try
    {
        if (time_remain < 0)
            throw std::runtime_error("Not enough time for updating the robot current state! ");

        if (DP::ss->getNorm(DP::q_current, q_new) > delta_q_max)
        {
            // Check whether 'q_new' can be reached considering robot max. velocity
            std::vector<std::vector<float>> limits;
            for (int i = 0; i < DP::ss->getNumDimensions(); i++)
                limits.emplace_back(std::vector<float>({DP::q_current->getCoord(i) - delta_q_max, 
                                                        DP::q_current->getCoord(i) + delta_q_max}));
            q_new = DP::ss->pruneEdge(DP::q_current, q_new, limits);
        }

        if (!DP::ss->isEqual(DP::q_current, q_new))
        {
            if (DP::ss->isEqual(q_new, DP::q_next->getStateReached()))
                DP::status = base::State::Status::Reached;
            else
                DP::status = base::State::Status::Advanced;
            
            q_new->setParent(DP::q_current);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the robot current state to: ");
                std::cout << "(" << DP::q_current->getCoord().transpose() << ")" << std::endl;
        }
        else
        {
            DP::status = base::State::Status::Trapped;
            DP::replanning = true;
            DP::horizon.clear();
            throw std::runtime_error("Robot is trapped! ");
        }
    }
    catch(std::exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Not updating the robot current state! %s", e.what());
        return;
    }

    if (DP::q_next != q_next_prev)
    {
        std::vector<std::shared_ptr<base::State>> path;
        std::vector<float> time_instances;
        float time;

        // if (q_current_prev != nullptr)
        // {
        //     float dist = DP::ss->getNorm(q_current_prev, DP::q_current);
        //     std::shared_ptr<base::State> q_current_new = DP::ss->interpolateEdge(q_current_prev, DP::q_current, 1.2*dist, dist);
        //     path.emplace_back(q_current_new);
        //     time = (DP::q_current->getCoord() - q_current_new->getCoord()).cwiseAbs().maxCoeff() 
        //            / Robot::getMaxAngVel() * 1000;
        //     time_instances.emplace_back(time);
        // }

        path.emplace_back(q_new);
        time = time_remain;
        time_instances.emplace_back(time);

        if (DP::status == base::State::Status::Advanced)
        {
            path.emplace_back(DP::q_next->getStateReached());
            time += (DP::q_next->getStateReached()->getCoord() - q_new->getCoord()).cwiseAbs().maxCoeff() 
                    / Robot::getMaxAngVel() * 1000;
            time_instances.emplace_back(time);
        }

        if (!DP::q_next->getIsReached())     // 'q_next_reached' is different from 'q_next'
        {
            path.emplace_back(DP::q_next->getState());
            time += (DP::q_next->getState()->getCoord() - DP::q_next->getStateReached()->getCoord()).cwiseAbs().maxCoeff() 
                    / Robot::getMaxAngVel() * 1000;
            time_instances.emplace_back(time);
        }
            
        float time_max = time_remain + DRGBTConnectConfig::MAX_ITER_TIME;
        if (time < time_max && DP::q_next->getIndex() != -1)    // If 'q_next' lies in 'predefined_path'
        {
            for (int i = DP::q_next->getIndex(); i < DP::predefined_path.size()-1; i++)
            {
                path.emplace_back(DP::predefined_path[i+1]);
                time += (DP::predefined_path[i+1]->getCoord() - DP::predefined_path[i]->getCoord()).cwiseAbs().maxCoeff() 
                        / Robot::getMaxAngVel() * 1000;
                time_instances.emplace_back(time);
                if (time > time_max)
                    break;
            }
        }
        
        q_next_prev = DP::q_next;
        Trajectory::addPath(path, time_instances);
        Trajectory::publish();
    }

    q_current_prev = DP::q_current;
    DP::q_current = q_new;
    DP::path.emplace_back(DP::q_current);
}