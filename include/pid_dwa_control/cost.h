#ifndef PID_DWA_CONTROL_COST_H
#define PID_DWA_CONTROL_COST_H

namespace FOLLOWING
{
    class Cost
    {
    private:
        /* data */
    public:
        Cost(const float obs_cost, const float goal_cost, const float direction_cost, const float speed_cost, const float path_cost, const float total_cost);
        ~Cost();

        void show();
        void calc_total_cost();

        float obs_cost_;
        float goal_cost_;
        float direction_cost_;
        float speed_cost_;
        float path_cost_;
        float total_cost_;
    };
}

#endif // FOLLOWING_CONTROLLER_STATE_H