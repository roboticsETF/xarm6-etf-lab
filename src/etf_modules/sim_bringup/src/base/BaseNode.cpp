#include "base/BaseNode.h"

sim_bringup::BaseNode::BaseNode(const std::string &node_name, const std::string &config_file_path) : 
    Node(node_name),
    Trajectory(config_file_path),
    Planner(config_file_path)
{
    project_abs_path = std::string(__FILE__);
    for (size_t i = 0; i < 4; i++)
        project_abs_path = project_abs_path.substr(0, project_abs_path.find_last_of("/\\"));
    
    try
    {
        YAML::Node node { YAML::LoadFile(project_abs_path + config_file_path) };

        Trajectory::publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>
            ("/xarm6_traj_controller/joint_trajectory", 10);
        
        Robot::joints_state_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
            ("/xarm6_traj_controller/state", 10, std::bind(&Robot::jointsStateCallback, this, std::placeholders::_1));
        Robot::gripper_node = std::make_shared<rclcpp::Node>("gripper_node");
        Robot::gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>
            (gripper_node, "/xarm_gripper/gripper_action");

        period = node["period"].as<float>();
        timer = this->create_wall_timer(std::chrono::microseconds(size_t(period * 1e6)), std::bind(&BaseNode::baseCallback, this));

        std::shared_ptr<env::Environment> env { nullptr };
        if (node["environment"].IsDefined())
            env = std::make_shared<env::Environment>(config_file_path, project_abs_path);
        else
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Predefined environment is not set up! Be careful since table is not added to the scene!");
                
        std::shared_ptr<base::StateSpace> ss { nullptr };
        std::string state_space = node["robot"]["space"].as<std::string>();
        if (state_space == "RealVectorSpace")
            ss = std::make_shared<base::RealVectorSpace>(Robot::getNumDOFs(), Robot::getRobot(), env);
        else if (state_space == "RealVectorSpaceFCL")
            ss = std::make_shared<base::RealVectorSpaceFCL>(Robot::getNumDOFs(), Robot::getRobot(), env);
        else
            throw std::logic_error("State space does not exist!");

        YAML::Node q_start_node { node["robot"]["q_start" ]};
        YAML::Node q_goal_node { node["robot"]["q_goal"] };
        std::shared_ptr<base::State> q_start { nullptr };
        std::shared_ptr<base::State> q_goal { nullptr };
        if (q_start_node.IsDefined() && q_goal_node.IsDefined())
        {
            if (q_start_node.size() != Robot::getNumDOFs() || q_goal_node.size() != Robot::getNumDOFs())
                throw std::logic_error("Start or goal size is not correct!");
        
            Eigen::VectorXf q_start_vec(Robot::getNumDOFs());
            Eigen::VectorXf q_goal_vec(Robot::getNumDOFs());
            for (size_t i = 0; i < Robot::getNumDOFs(); i++)
            {
                q_start_vec(i) = q_start_node[i].as<float>();
                q_goal_vec(i) = q_goal_node[i].as<float>();
            }
            q_start = std::make_shared<base::RealVectorSpaceState>(q_start_vec);
            q_goal = std::make_shared<base::RealVectorSpaceState>(q_goal_vec);
        }
        else
        {
            q_start = nullptr;
            q_goal = nullptr;
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Start or goal configuration is not defined!");
        }
        
        Planner::scenario = std::make_shared<scenario::Scenario>(ss, q_start, q_goal);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << "\n";
    }
}

sim_bringup::BaseNode::~BaseNode() {}