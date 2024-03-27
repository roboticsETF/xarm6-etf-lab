#include "demos/MoveRealXArm6Node.h"

namespace real_bringup
{
    class PickAndPlaceNode : public real_bringup::MoveRealXArm6Node
    {
    public:
        PickAndPlaceNode(const std::string &node_name, const std::string &config_file_path);

    protected:    
        void moveRealXArm6Callback() override { pickAndPlaceCallback(); }
        virtual void pickAndPlaceCallback();

    private:
        size_t num;
        size_t num_objects;
        float object_height;
        float object_pick_z;
        float delta_theta1;
        std::vector<float> object_angles_approach;

        enum Task
        {
            going_home,
            approaching_to_object,
            lowering_robot,
            grasping_object,
            raising_object,
            moving_object_to_goal,
            lowering_object,
            releasing_object,
            raising_robot
        };
        Task task;
    };
}