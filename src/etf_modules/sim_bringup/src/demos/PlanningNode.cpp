#include "demos/PlanningNode.h"

sim_bringup::PlanningNode::PlanningNode(const std::string &node_name, const std::string &config_file_path) : 
    BaseNode(node_name, config_file_path),
    AABB(config_file_path),
    Octomap(config_file_path),
    ConvexHulls(config_file_path)
{
    AABB::setEnvironment(scenario->getEnvironment());
    if (AABB::getMinNumCaptures() == 1)
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::callback, this, std::placeholders::_1));
    else
        AABB::subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
            ("/bounding_boxes", 10, std::bind(&AABB::withFilteringCallback, this, std::placeholders::_1));

    // Octomap::marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>
    //     ("/occupied_cells_vis_array", 10);
    
    // ConvexHulls::points_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
    //     ("/convex_hulls", 10, std::bind(&ConvexHulls::pointsCallback, this, std::placeholders::_1));
    // ConvexHulls::polygons_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
    //     ("/convex_hulls_polygons", 10, std::bind(&ConvexHulls::polygonsCallback, this, std::placeholders::_1));
    
    state = waiting;
    q_start = scenario->getStart();
    q_goal = scenario->getGoal();
}

void sim_bringup::PlanningNode::planningCallback()
{
    switch (state)
    {
    case waiting:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting...");
        if (Robot::isReady())
        {
            scenario->setStart(Robot::getJointsPositionPtr());
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
                Trajectory::clear();
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
            rclcpp::shutdown();
            return;
        }
        break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------------------------------------\n");
}
