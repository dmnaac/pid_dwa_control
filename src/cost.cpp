#include <ros/console.h>
#include "pid_dwa_control/cost.h"

namespace FOLLOWING
{
    Cost::Cost() {}

    Cost::Cost(const float obs_cost, const float goal_cost, const float direction_cost, const float speed_cost, const float path_cost, const float total_cost) : obs_cost_(obs_cost), goal_cost_(goal_cost), direction_cost_(direction_cost), speed_cost_(speed_cost), path_cost_(path_cost), total_cost_(total_cost)
    {
    }

    void Cost::show()
    {
        ROS_INFO_STREAM("Cost: " << total_cost_);
        ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
        ROS_INFO_STREAM("\tGoal cost: " << goal_cost_);
        ROS_INFO_STREAM("\tDirection cost: " << direction_cost_);
        ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
        ROS_INFO_STREAM("\tPath cost:" << path_cost_);
    }

    void Cost::calc_total_cost()
    {
        total_cost_ = obs_cost_ + goal_cost_ + speed_cost_ + path_cost_;
    }

    Cost::~Cost()
    {
    }
}