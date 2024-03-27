//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_BASE_NODE_H
#define SIM_BRINGUP_BASE_NODE_H

#include "base/Trajectory.h"
#include "base/Planner.h"

#include <RealVectorSpace.h>
#include <RealVectorSpaceFCL.h>

using namespace std::chrono_literals;

namespace sim_bringup
{
    class BaseNode : public rclcpp::Node,
                     public sim_bringup::Trajectory,
                     public sim_bringup::Planner
    {
    public:
        BaseNode(const std::string &node_name, const std::string &config_file_path);
        virtual ~BaseNode() = 0;

        virtual void baseCallback() = 0;

        rclcpp::TimerBase::SharedPtr timer;
        float period;                             // Period of basic callback function in [s]

    protected:
        std::string project_abs_path;

    };
}

#endif // SIM_BRINGUP_BASE_NODE_H