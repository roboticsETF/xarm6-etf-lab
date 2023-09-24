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
// 'omit_first_conf': If true, the first configuration from 'path' is omitted
// 'time_offset' in [s]: If passed, each time instance from 'time_instances' is shifted for the value of 'time_offset'
// 'delta_time': If not passed, configurations from 'path' remain the same, while their time instances are computed
// 'delta_time': If passed, configurations are interpolated rendering their adjacent time instances equidistant with 'delta_time'
void real_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, 
                                       bool omit_first_conf, float time_offset, float delta_time)
{
    Eigen::VectorXf q = path[0]->getCoord();
    Eigen::VectorXf q_next;
    float time = time_offset;
    clear();

    if (!omit_first_conf)
    {
        points.emplace_back(q);
        time_instances.emplace_back(time);
    }

    if (delta_time == 0)
    {
        for (int i = 1; i < path.size(); i++)
        {
            q_next = path[i]->getCoord();
            time += (q_next - q).norm() / max_ang_vel;
            points.emplace_back(q_next);
            time_instances.emplace_back(time);
            q = q_next;
        }
    }
    else
    {
        float max_delta_angle = max_ang_vel * delta_time;
        for (int i = 1; i < path.size(); i++)
        {
            q_next = path[i]->getCoord();
            float d = (q_next - q).norm();
            bool status = false;
            while (!status)
            {
                if (d > max_delta_angle)
                {
                    q += ((q_next - q) / d) * max_delta_angle;
                    d -= max_delta_angle;
                }
                else
                {
                    q = q_next;
                    status = true;
                }
                points.emplace_back(q);
                time += delta_time;
                time_instances.emplace_back(time);
            }
        }
    }
}

void real_bringup::Trajectory::addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances_)
{
    clear();    
    for (int i = 0; i < path.size(); i++)
        points.emplace_back(path[i]->getCoord());
    
    time_instances = time_instances_;
}

void real_bringup::Trajectory::addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances_)
{
    clear();    
    points = path;
    time_instances = time_instances_;
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