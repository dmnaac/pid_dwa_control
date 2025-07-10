#include <algorithm>
#include <string>

#include "pid_dwa_control/dwa_planner.h"

void DWA_planner::load_params()
{
    local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.01);
    local_nh_.param<double>("MAX_LINEAR_ACCELERATION", max_linear_acceleration_, 0.5);
    local_nh_.param<double>("MAX_LINEAR_DECELERATION", max_linear_deceleration_, 2.0);
    local_nh_.param<double>("MAX_ANGULAR_ACCELERATION", max_angular_acceleration_, 3.2);
    local_nh_.param<double>("MAX_IN_PLACE_ANGULAR_VELOCITY", max_in_place_angular_velocity_, 0.6);
    local_nh_.param<double>("MAX_LINEAR_VELOCITY", max_linear_velocity_, 1.0);
    local_nh_.param<double>("MAX_ANGULAR_VELOCITY", max_angular_velocity_, 1.0);
    local_nh_.param<double>("MIN_IN_PLACE_ANGULAR_VELOCITY", min_in_place_angular_velocity_, 0.3);
    local_nh_.param<double>("MIN_LINEAR_VELOCITY", min_linear_velocity_, 0.0);
    local_nh_.param<double>("MIN_ANGULAR_VELOCITY", min_angular_velocity_, 0.05);

    local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, 1.0);
    local_nh_.param<double>("OBS_RANGE", obs_range_, 2.5);
    local_nh_.param<double>("PREDICT_TIME", predict_time_, 3.0);
    local_nh_.param<double>("ROBOT_RADIUS", robot_radius_, 0.1);
    local_nh_.param<double>("SIM_DIRECTION", sim_direction_, M_PI / 2.0);
    local_nh_.param<double>("SIM_PERIOD", sim_period_, 0.1);
    local_nh_.param<double>("SIM_TIME_SAMPLES", sim_time_samples_, 10);
    local_nh_.param<double>("SLOW_VELOCITY_TH", slow_velocity_th_, 0.1);
    local_nh_.param<double>("SPEED_COST_GAIN", speed_cost_gain_, 0.4);
    local_nh_.param<double>("TARGET_LINEAR_VELOCITY", target_linear_velocity_, 0.55);
    local_nh_.param<double>("GOAL_COST_GAIN", goal_cost_gain_, 0.8);
    local_nh_.param<double>("DIRECTION_COST_GAIN", direction_cost_gain_, 1.0);
    local_nh_.param<bool>("USE_FOOTPRINT", use_footprint_, true);
    local_nh_.param<bool>("USE_PATH_COST", use_path_cost_, false);
    local_nh_.param<double>("VELOCITY_SAMPLES_X", velocity_samples_x_, 3);
    local_nh_.param<double>("VELOCITY_SAMPLES_Y", velocity_samples_y_, 20);
    local_nh_.param<double>("VELOCITY_SAMPLES_YAW", velocity_samples_yaw_, 20);

    target_linear_velocity_ = std::min(target_linear_velocity_, max_linear_velocity_);
}