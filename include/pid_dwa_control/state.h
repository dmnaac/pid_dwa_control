#ifndef PID_DWA_CONTROL_STATE_H
#define PID_DWA_CONTROL_STATE_H

namespace FOLLOWING
{
    class State
    {
    private:
        /* data */
    public:
        State();
        State(const double x, const double y, const double yaw, const double vel_x, const double vel_yaw);
        ~State();

        double x_;
        double y_;
        double yaw_;
        double vel_x_;
        double vel_yaw_;
    };
}

#endif // FOLLOWING_CONTROLLER_STATE_H