#include "state.h"

namespace FOLLOWING
{
    State::State(const double x, const double y, const double yaw, const double vel_x, const double vel_yaw) : x_(x), y_(y), yaw_(yaw), vel_x_(vel_x), vel_yaw_(vel_yaw)
    {
    }

    State::~State()
    {
    }
}