#include "pid_dwa_control/window.h"

namespace FOLLOWING
{
    Window::Window() : min_vel_x_(0.0), max_vel_x_(0.0), min_vel_yaw_(0.0), max_vel_yaw_(0.0)
    {
    }

    Window::~Window()
    {
    }

    void Window::show()
    {
        ROS_INFO_STREAM("Window: ");
        ROS_INFO_STREAM("\tLinear: ");
        ROS_INFO_STREAM("\t\tmin_vel_x: " << min_vel_x_);
        ROS_INFO_STREAM("\t\tmax_vel_x: " << max_vel_x_);
        ROS_INFO_STREAM("\tAngular: ");
        ROS_INFO_STREAM("\t\tmin_vel_yaw: " << min_vel_yaw_);
        ROS_INFO_STREAM("\t\tmax_vel_yaw: " << max_vel_yaw_);
    }
}