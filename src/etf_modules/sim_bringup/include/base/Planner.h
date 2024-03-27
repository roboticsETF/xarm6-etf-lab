//
// Created by nermin on 28.08.23.
//

#ifndef SIM_BRINGUP_PLANNER_H
#define SIM_BRINGUP_PLANNER_H

#include <RRTConnect.h>
#include <RBTConnect.h>
#include <RGBTConnect.h>
#include <RGBMTStar.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace sim_bringup
{
    class Planner
    {
    public:
        Planner(const std::string &config_file_path);

        inline const std::unique_ptr<planning::AbstractPlanner> &getPlanner() const { return planner; }
        inline planning::PlannerType getPlannerType() const { return planner_type; }
        inline const std::vector<std::shared_ptr<base::State>> &getPath() const { return planner->getPath(); }
        inline float getPlanningTime() const { return planner->getPlannerInfo()->getPlanningTime(); }
        inline bool isReady() const { return ready; }

        bool solve(std::shared_ptr<base::State> q_start = nullptr, std::shared_ptr<base::State> q_goal = nullptr, 
                   float max_planning_time_ = -1);

        std::shared_ptr<scenario::Scenario> scenario;

    private:
        std::unique_ptr<planning::AbstractPlanner> planner;
        planning::PlannerType planner_type;
        float max_planning_time;                                  // In [s]
        bool ready;
    };
}

#endif // SIM_BRINGUP_PLANNER_H