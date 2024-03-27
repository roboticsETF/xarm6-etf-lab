//
// Created by nermin on 28.08.23.
//

#include "base/BaseNode.h"
#include "environments/AABB.h"

#include <DRGBT.h>
#include <thread>

namespace sim_bringup
{
    class RealTimePlanningNode : public sim_bringup::BaseNode, 
                                 public sim_bringup::AABB, 
                                 public planning::drbt::DRGBT
    {
    public:
        RealTimePlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void baseCallback() override { planningCallback(); }
        void planningCallback();
        void replan(float max_planning_time) override;
        void computeTrajectory();
        
        std::shared_ptr<scenario::Scenario> scenario;
    };
}
