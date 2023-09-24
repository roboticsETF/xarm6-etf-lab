#include <RRTConnect.h>
#include <RBTConnect.h>
#include <RGBTConnect.h>
#include <RGBMTStar.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <glog/logging.h>

namespace real_bringup
{
    class Planner
    {
    public:
        Planner(const std::string config_file_path);

        inline const std::vector<std::shared_ptr<base::State>> &getPath() { return path; }
        inline void setName(const std::string &name_) { name = name_; }
        void setMaxPlanningTime(float max_planning_time);
        inline void setScenario(std::shared_ptr<scenario::Scenario> &scenario_) { scenario = scenario_; }

        bool solve(std::shared_ptr<base::State> start = nullptr);
        inline const bool isReady() { return ready; }

    private:
        std::string name;
        bool ready = true;
        std::vector<std::shared_ptr<base::State>> path;
        std::shared_ptr<scenario::Scenario> scenario;
    };
}