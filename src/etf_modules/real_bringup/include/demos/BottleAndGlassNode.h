#include "demos/MoveRealXArm6Node.h"

namespace real_bringup
{
    class BottleAndGlassNode : public real_bringup::MoveRealXArm6Node
    {
    public:
        BottleAndGlassNode(const std::string &node_name, const std::string &config_file_path);

    protected:
        void moveRealXArm6Callback() override { bottleAndGlassCallback(); }
        void bottleAndGlassCallback();

    private:
        int sign;
        std::vector<float> glass_angles_approach;
        std::vector<float> glass_angles_pick;
        std::vector<float> bottle_pose_pick;

        enum State
        {
            going_home,
            approaching_to_bottle,
            grasping_bottle,
            raising_bottle,
            moving_towards_glass1,
            moving_towards_glass2,
            pouring_water1,
            pouring_water2,
            returning_bottle,
            releasing_bottle,
            grasping_glass,
            moving_glass1,
            moving_glass2,
            releasing_glass
        };
        State state, state_next;
    };
}