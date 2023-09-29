#include "base/Trajectory.h"

real_bringup::Trajectory::Trajectory(float max_ang_vel)
{
    Trajectory::max_ang_vel = max_ang_vel;
    msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

void real_bringup::Trajectory::addPoint(std::shared_ptr<base::State> point, float time_instance)
{
    points.emplace_back(point->getCoord());
    time_instances.emplace_back(time_instance);
}

void real_bringup::Trajectory::addPoint(const Eigen::VectorXf &point, float time_instance)
{
    points.emplace_back(point);
    time_instances.emplace_back(time_instance);
}

// Add and parametrize 'path' in order to satisfy the maximal angular velocity 'max_ang_vel'
// 'time_offset' in [ms]: If passed, each time instance from 'time_instances' is shifted for the value of 'time_offset'
// 'omit_first_conf': If true, the first configuration from 'path' is omitted
void real_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, float time_offset, bool omit_first_conf)
{
    Eigen::VectorXf q = path[0]->getCoord();
    Eigen::VectorXf q_next;
    float time = time_offset / 1000;
    clear();

    if (!omit_first_conf)
    {
        points.emplace_back(q);
        time_instances.emplace_back(time);
    }

    for (int i = 1; i < path.size(); i++)
    {
        q_next = path[i]->getCoord();
        time += (q_next - q).cwiseAbs().maxCoeff() / max_ang_vel;
        points.emplace_back(q_next);
        time_instances.emplace_back(time);
        q = q_next;
    }
}

// 'time_instances_' are in [ms]
void real_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances_)
{
    clear();
    for (int i = 0; i < path.size(); i++)
    {
        points.emplace_back(path[i]->getCoord());
        time_instances.emplace_back(time_instances_[i] / 1000);     // Conversion from [ms] to [s]
    }
}

// 'time_instances_' are in [ms]
void real_bringup::Trajectory::addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances_)
{
    clear();
    points = path;
    for (int i = 0; i < path.size(); i++)
        time_instances.emplace_back(time_instances_[i] / 1000);     // Conversion from [ms] to [s]
}

void real_bringup::Trajectory::publish()
{
    if (points.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "There is no trajectory to publish!");
        return;
    }
    
    msg.points.clear();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory: ");
    for (int i = 0; i < points.size(); i++)
    {
        Eigen::VectorXf q = points[i];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Num. %d. Time: %f [s]. Point: (%f, %f, %f, %f, %f, %f)", 
            i, time_instances[i], q(0), q(1), q(2), q(3), q(4), q(5));

        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int j = 0; j < q.size(); j++)
            point.positions.emplace_back(q(j));

        point.time_from_start.sec = int32_t(time_instances[i]);
        point.time_from_start.nanosec = (time_instances[i] - point.time_from_start.sec) * 1e9;
        msg.points.emplace_back(point);
    }

    publisher->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing trajectory ...");
}

void real_bringup::Trajectory::clear()
{
    points.clear();
    time_instances.clear();
}