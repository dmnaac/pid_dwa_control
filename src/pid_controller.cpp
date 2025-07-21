#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ros/ros.h>

#include "pid_dwa_control/pid_controller.h"

namespace FOLLOWING
{
    PID_controller::PID_controller(double kp, double ki, double kd, double deadband,
                                   double min_pid_out, double max_pid_out,
                                   double min_err_int, double max_err_int, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          min_pid_out_(min_pid_out), max_pid_out_(max_pid_out),
          min_err_int_(min_err_int), max_err_int_(max_err_int),
          deadband_(deadband),
          err_cur_(0.0), err_last_(0.0), err_int_(0.0),
          err_der_(0.0), pid_ref_(0.0), pid_out_(0.0)
    {
        if (dt <= 0.0)
        {
            throw std::invalid_argument("dt < 0");
        }

        if (min_pid_out >= max_pid_out)
        {
            throw std::invalid_argument("min_pid_out > max_pid_out");
        }

        if (min_err_int >= max_err_int)
        {
            throw std::invalid_argument("min_err_int > max_err_int");
        }

        ROS_INFO_STREAM("PID controller is ready.");
    }

    PID_controller::~PID_controller()
    {
    }

    double PID_controller::calc_output(double pos_x, double dt)
    {
        dt_ = dt;

        err_cur_ = pid_ref_ - pos_x;

        // Deadband
        const double deadband_half = deadband_ / 2.0;
        const double bounded_error =
            (err_cur_ <= deadband_half && err_cur_ >= -deadband_half) ? 0.0 : err_cur_;

        const double derivative = (bounded_error - err_last_) / dt_;

        err_int_ += bounded_error * dt_;
        err_int_ = std::clamp(err_int_, min_err_int_, max_err_int_);

        const double p_term = kp_ * bounded_error;
        const double i_term = ki_ * err_int_;
        const double d_term = kd_ * derivative;
        double output = p_term + i_term + d_term;

        output = std::clamp(output, min_pid_out_, max_pid_out_);

        err_last_ = bounded_error;

        pid_out_ = output;
        return output;
    }
}