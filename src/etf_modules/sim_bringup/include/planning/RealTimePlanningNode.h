//
// Created by nermin on 28.08.23.
//

#include "planning/PlanningNode.h"

#include <DRGBTConnect.h>

namespace sim_bringup
{
    class RealTimePlanningNode : public sim_bringup::PlanningNode, 
                                 public planning::drbt::DRGBTConnect
    {
    public:
        RealTimePlanningNode(const std::string node_name, const std::string config_file_path);

    protected:
        void planningCallback() override;
        void replan(float replanning_time);
        void updateCurrentState();

        std::chrono::_V2::steady_clock::time_point time_alg_start, time_iter_start;

    private:
        bool init_iteration;
    };
}