//
// Created by nermin on 28.08.23.
//

#include "demos/PlanningNode.h"

namespace sim_bringup
{
    class TaskPlanningNode : public sim_bringup::PlanningNode
    {
    public:
        TaskPlanningNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void planningCallback() override { taskPlanningCallback(); }
        void taskPlanningCallback();
        void planningCase();
        bool computeObjectApproachAndPickStates();
        int chooseObject() override;
        bool whetherToRemove(const Eigen::Vector3f &object_pos, const Eigen::Vector3f &object_dim) override;

        enum Task 
        {
            waiting_for_object,
            choosing_object,
            computing_IK,
            going_towards_object,
            picking_object,
            raising_object,
            releasing_object,
            moving_object_to_destination,
            planning
        };
        Task task, task_next;

    private:
        std::shared_ptr<base::State> q_object_approach1, q_object_approach2;
        std::shared_ptr<base::State> q_object_pick;
        std::shared_ptr<base::State> q_goal;
        int obj_idx;
        int IK_computed;
        size_t picking_object_wait, picking_object_wait_max;
        float max_object_height;
        Eigen::Vector3f destination;
    };
}
