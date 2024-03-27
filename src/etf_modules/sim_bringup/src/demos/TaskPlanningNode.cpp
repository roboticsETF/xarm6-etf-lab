#include "demos/TaskPlanningNode.h"

sim_bringup::TaskPlanningNode::TaskPlanningNode(const std::string &node_name, const std::string &config_file_path) : 
    PlanningNode(node_name, config_file_path)
{
    YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };
    YAML::Node scenario { node["scenario"] };

    max_object_height = scenario["max_object_height"].as<float>();
    picking_object_wait_max = scenario["picking_object_wait_max"].as<size_t>();
    picking_object_wait = picking_object_wait_max;
    for (size_t i = 0; i < 3; i++)
        destination(i) = scenario["destination"][i].as<float>();

    IK_computed = -1;
    task = waiting_for_object;
    state = State::planning;
}

void sim_bringup::TaskPlanningNode::taskPlanningCallback()
{
    switch (task)
    {
    case waiting_for_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the object...");
        AABB::resetMeasurements();
        if (Robot::isReady())
            task = choosing_object;
        break;

    case choosing_object:
        obj_idx = chooseObject();
        if (obj_idx != -1)
        {
            task = computing_IK;
            IK_computed = computeObjectApproachAndPickStates();
        }
        else
            task = waiting_for_object;
        break;

    case computing_IK:
        if (IK_computed == 1)
        {
            AABB::resetMeasurements();
            scenario->setStart(Robot::getJointsPositionPtr());
            scenario->setGoal(q_object_approach1);
            task = planning;
            task_next = going_towards_object;
            IK_computed = -1;
        }
        else if (IK_computed == 0)
            task = waiting_for_object;
        
        break;

    case going_towards_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going towards the object...");
        Trajectory::clear();
        Trajectory::addPath({q_object_approach1, q_object_approach2, q_object_pick});
        Trajectory::publish();
        task = picking_object;
        break;

    case picking_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Picking the object...");
        picking_object_wait--;
        if (picking_object_wait == 0)
        {
            picking_object_wait = picking_object_wait_max;
            task = raising_object;
        }
        break;

    case raising_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Raising the object...");
        Trajectory::clear();
        Trajectory::addPath({q_object_pick, q_object_approach1});
        Trajectory::publish();
        task = moving_object_to_destination;
        break;

    case moving_object_to_destination:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving the object to destination...");
        AABB::resetMeasurements();
        scenario->setStart(q_object_approach1);
        scenario->setGoal(q_goal);
        task = planning;
        task_next = releasing_object;
        break;
    
    case releasing_object:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Releasing the object...");
        task = waiting_for_object;
        break;

    case planning:
        planningCase();
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}

void sim_bringup::TaskPlanningNode::planningCase()
{
    switch (state)
    {
    case State::waiting:
        break;
    
    case State::planning:
        if (Planner::isReady())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating the environment..."); 
            AABB::updateEnvironment();

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning the path..."); 
            if (Planner::solve())
            {
                Trajectory::clear();
                Trajectory::addPath(Planner::getPath());
                state = State::publishing_trajectory;
            }
        }
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for the planner..."); 
        break;
    
    case State::publishing_trajectory:
        Trajectory::publish();
        state = State::executing_trajectory;
        break;

    case State::executing_trajectory:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing trajectory...");            
        if (Robot::isReached(scenario->getGoal()))
        {
            state = State::planning;
            task = task_next;
        }
        break;
    }
}

bool sim_bringup::TaskPlanningNode::computeObjectApproachAndPickStates()
{
    const Eigen::Vector3f pos { AABB::getPositions(obj_idx) };
    const Eigen::Vector3f dim { AABB::getDimensions(obj_idx) };

    // For approaching from above
    KDL::Vector n(pos.x(), pos.y(), 0); n.Normalize();
    KDL::Vector s(pos.y(), -pos.x(), 0); s.Normalize();
    KDL::Vector a(0, 0, -1);
    
    // For approaching by side
    // KDL::Vector n(0, 0, -1);
    // KDL::Vector s(-pos.y(), pos.x(), 0); s.Normalize();
    // KDL::Vector a(pos.x(), pos.y(), 0); a.Normalize();

    KDL::Rotation R(n, s, a);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotation matrix: ");
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector n: (%f, %f, %f)", R(0,0), R(1,0), R(2,0));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector s: (%f, %f, %f)", R(0,1), R(1,1), R(2,1));
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vector a: (%f, %f, %f)", R(0,2), R(1,2), R(2,2));
    
    float fi { std::atan2(pos.y(), pos.x()) };
    float r { pos.head(2).norm() };
    float r_crit { 0.3 };
    if (r < r_crit)
        r += 1.5 * dim.head(2).norm();
    else
        r -= 1.5 * dim.head(2).norm();

    float p_pick_z { pos.z() };
    if (dim.z() > 0.14)
        p_pick_z += dim.z() / 2 - 0.07;    // finger length is 7 [cm]

    KDL::Vector p_approach1(r * float(cos(fi)), 
                            r * float(sin(fi)), 
                            p_pick_z + 2 * max_object_height);
    KDL::Vector p_approach2(r * float(cos(fi)), 
                            r * float(sin(fi)), 
                            p_pick_z);
    KDL::Vector p_pick(pos.x(), 
                       pos.y(), 
                       p_pick_z);
    size_t num { 0 };
    std::shared_ptr<base::State> q_init { Robot::getJointsPositionPtr() };
    while (num++ <= 100)
    {
        q_object_approach1 = Robot::getRobot()->computeInverseKinematics(R, p_approach1, q_init);
        if (q_object_approach1 == nullptr)
        {
            q_init = nullptr;
            continue;
        }
        
        // It is convenient for the purpose when picking objects from above
        if ((r > r_crit && std::abs(q_object_approach1->getCoord(3)) < 0.1) ||
            (r <= r_crit && std::abs(q_object_approach1->getCoord(3) - (-M_PI)) < 0.1))
            break;
    }
    if (q_object_approach1 == nullptr)
        return false;

    // Just to ensure that 'q_object_approach2' is relatively close to 'q_object_approach1'
    q_object_approach2 = Robot::getRobot()->computeInverseKinematics(R, p_approach2, q_object_approach1);
    if (q_object_approach2 == nullptr || std::abs(q_object_approach1->getCoord(3) - q_object_approach2->getCoord(3)) > 0.1)
        return false;

    // Just to ensure that 'q_object_pick' is relatively close to 'q_object_approach2'
    q_object_pick = Robot::getRobot()->computeInverseKinematics(R, p_pick, q_object_approach2); 
    if (q_object_pick == nullptr || std::abs(q_object_approach2->getCoord(3) - q_object_pick->getCoord(3)) > 0.1)
        return false;

    KDL::Vector p_goal(destination.x(), destination.y(), destination.z() + dim.z());
    KDL::Vector n_goal(destination.x(), destination.y(), 0); n_goal.Normalize();
    KDL::Vector s_goal(destination.y(), -destination.x(), 0); s_goal.Normalize();
    KDL::Vector a_goal(0, 0, -1);
    KDL::Rotation R_goal(n_goal, s_goal, a_goal);
    Eigen::VectorXf goal_angles { q_object_pick->getCoord() };
    goal_angles(0) = std::atan2(destination.y(), destination.x());
    q_goal = Robot::getRobot()->computeInverseKinematics(R_goal, p_goal, std::make_shared<base::RealVectorSpaceState>(goal_angles));
    if (q_goal == nullptr)
        return false;
    
    // Just to take a shorter angle
    if (std::abs(std::atan2(pos.y(), pos.x()) - q_goal->getCoord(0)) > M_PI)
    {
        if (q_goal->getCoord(0) > 0)
            q_goal->setCoord(q_goal->getCoord(0) - 2*M_PI, 0);
        else
            q_goal->setCoord(q_goal->getCoord(0) + 2*M_PI, 0);
    }

    return true;
}

int sim_bringup::TaskPlanningNode::chooseObject()
{
    float z_max { -INFINITY };
    int obj_idx_ { -1 };
    for (size_t i = 0; i < positions.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %ld. dim = (%f, %f, %f), pos = (%f, %f, %f). Num. captures %ld.",
            i, dimensions[i].x(), dimensions[i].y(), dimensions[i].z(), 
               positions[i].x(), positions[i].y(), positions[i].z(), num_captures[i]);
        if (num_captures[i] >= min_num_captures && positions[i].z() > z_max && positions[i].z() < max_object_height)  // Pick only "small" objects
        {
            z_max = positions[i].z();
            obj_idx_ = i;
        }
    }

    if (obj_idx_ != -1)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Object %d is recognized at the position (%f, %f, %f).", 
            obj_idx_, positions[obj_idx_].x(), positions[obj_idx_].y(), positions[obj_idx_].z());
    
    return obj_idx_;
}

bool sim_bringup::TaskPlanningNode::whetherToRemove(const Eigen::Vector3f &object_pos, [[maybe_unused]] const Eigen::Vector3f &object_dim)
{
    // Remove the destination box from the scene
    if (object_pos.x() < -0.4 && std::abs(object_pos.y()) < 0.2 && object_pos.z() < 0.25)
        return true;
    
    return false;
}
