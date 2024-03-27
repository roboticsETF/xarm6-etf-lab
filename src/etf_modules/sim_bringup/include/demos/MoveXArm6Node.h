#include "base/BaseNode.h"

namespace sim_bringup
{
    class MoveXArm6Node : public sim_bringup::BaseNode
    {
    public:
        MoveXArm6Node(const std::string &node_name, const std::string &config_file_path);

    protected:
        void baseCallback() override { moveXArm6Callback(); }
        virtual void moveXArm6Callback();
        void goHome();
        void moveInJointSpace();

    private:
        enum State
        {
            going_home,
            moving_in_joint_space
        };
        State state;
    };
}
