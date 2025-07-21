#include "pid_dwa_control/state.h"

namespace FOLLOWING
{
    State::State() : x_(0.0), y_(0.0), yaw_(0.0), vel_x_(0.0), vel_yaw_(0.0)
    {
    }

    State::State(const double x, const double y, const double yaw, const double vel_x, const double vel_yaw) : x_(x), y_(y), yaw_(yaw), vel_x_(vel_x), vel_yaw_(vel_yaw)
    {
    }

    State::~State()
    {
    }
}