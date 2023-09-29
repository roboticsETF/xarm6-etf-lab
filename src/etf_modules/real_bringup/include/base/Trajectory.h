#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <RealVectorSpaceState.h>

namespace real_bringup
{
    class Trajectory
    {
    public:
        Trajectory(float max_ang_vel);

        inline void setMaxAngVel(float max_ang_vel_) { max_ang_vel = max_ang_vel_; }
        
        void addPoint(std::shared_ptr<base::State> point, float time_instance);
        void addPoint(const Eigen::VectorXf &point, float time_instance);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, float time_offset = 0, bool omit_first_conf = false);
        void addPath(const std::vector<std::shared_ptr<base::State>> &path, const std::vector<float> &time_instances_);
        void addPath(const std::vector<Eigen::VectorXf> &path, const std::vector<float> &time_instances_);
        void publish();
        void clear();
        
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;

    private:
        trajectory_msgs::msg::JointTrajectory msg;
        std::vector<Eigen::VectorXf> points;
        std::vector<float> time_instances;
        float max_ang_vel;
    };
}