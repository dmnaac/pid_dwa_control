#include <algorithm>
#include <string>

#include "pid_dwa_control/dwa_planner.h"

namespace FOLLOWING
{
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
    {
        // Make sure that the value we're looking at is either a double or an int.
        if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
            value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            std::string &value_string = value;
            ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                      full_param_name.c_str(), value_string.c_str());
            throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
    }

    std::vector<geometry_msgs::Point32> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc,
                                                                const std::string &full_param_name)
    {
        // Make sure we have an array of at least 3 elements.
        if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            footprint_xmlrpc.size() < 3)
        {
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                      full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                                     "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }

        std::vector<geometry_msgs::Point32> footprint;
        geometry_msgs::Point32 pt;

        for (int i = 0; i < footprint_xmlrpc.size(); ++i)
        {
            // Make sure each element of the list is an array of size 2. (x and y coordinates)
            XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                point.size() != 2)
            {
                ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                          "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                          full_param_name.c_str());
                throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                         "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            }

            pt.x = getNumberFromXMLRPC(point[0], full_param_name);
            pt.y = getNumberFromXMLRPC(point[1], full_param_name);

            footprint.push_back(pt);
        }
        return footprint;
    }

    void DWA_planner::load_params()
    {
        local_nh_.param<bool>("USE_FOOTPRINT", use_footprint_, true);
        local_nh_.param<bool>("USE_SPEED_COST", use_speed_cost_, true);
        local_nh_.param<bool>("USE_PATH_COST", use_path_cost_, false);

        // DWA planner params
        local_nh_.param<double>("TARGET_LINEAR_VELOCITY", target_linear_velocity_, 0.55);
        local_nh_.param<double>("PREDICT_TIME", predict_time_, 3.0);
        local_nh_.param<int>("SIM_TIME_SAMPLES", sim_time_samples_, 10);
        local_nh_.param<double>("SIM_PERIOD", sim_period_, 0.1);
        local_nh_.param<double>("SIM_DIRECTION", sim_direction_, M_PI / 2.0);
        local_nh_.param<double>("SLOW_VELOCITY_TH", slow_velocity_th_, 0.1);
        local_nh_.param<int>("VELOCITY_SAMPLES_X", velocity_samples_x_, 3);
        local_nh_.param<int>("VELOCITY_SAMPLES_Y", velocity_samples_y_, 3);
        local_nh_.param<int>("VELOCITY_SAMPLES_YAW", velocity_samples_yaw_, 20);

        local_nh_.param<double>("OBSTACLE_COST_GAIN", obs_cost_gain_, 1.2);
        local_nh_.param<double>("GOAL_COST_GAIN", goal_cost_gain_, 0.8);
        local_nh_.param<double>("DIRECTION_COST_GAIN", direction_cost_gain_, 0.2);
        local_nh_.param<double>("SPEED_COST_GAIN", speed_cost_gain_, 0.4);

        local_nh_.param<double>("OBS_RANGE", obs_range_, 2.5);

        // Robot params
        local_nh_.param<double>("ROBOT_RADIUS", robot_radius_, 0.1);
        local_nh_.param<double>("FOOTPRINT_PADDING", footprint_padding_, 0.01);

        local_nh_.param<double>("MAX_LINEAR_VELOCITY", max_linear_velocity_, 1.0);
        local_nh_.param<double>("MIN_LINEAR_VELOCITY", min_linear_velocity_, 0.0);
        local_nh_.param<double>("MAX_ANGULAR_VELOCITY", max_angular_velocity_, 1.0);
        local_nh_.param<double>("MIN_ANGULAR_VELOCITY", min_angular_velocity_, 0.05);
        // local_nh_.param<double>("MAX_IN_PLACE_ANGULAR_VELOCITY", max_in_place_angular_velocity_, 0.6);
        // local_nh_.param<double>("MIN_IN_PLACE_ANGULAR_VELOCITY", min_in_place_angular_velocity_, 0.3);
        local_nh_.param<double>("MAX_LINEAR_ACCELERATION", max_linear_acceleration_, 0.5);
        local_nh_.param<double>("MAX_LINEAR_DECELERATION", max_linear_deceleration_, 2.0);
        local_nh_.param<double>("MAX_ANGULAR_ACCELERATION", max_angular_acceleration_, 3.2);

        target_linear_velocity_ = std::min(target_linear_velocity_, max_linear_velocity_);

        if (use_footprint_)
        {
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            if (!local_nh_.getParam("FOOTPRINT", footprint_xmlrpc))
            {
                ROS_ERROR("USE_FOOTPRINT but not found FOOTPRINT param");
            }
            else
            {
                footprint_points_ = makeFootprintFromXMLRPC(footprint_xmlrpc, "FOOTPRINT");
            }

            if (footprint_points_.size() < 3)
            {
                ROS_ERROR("Footprint must have at least 3 points");
            }
        }
    }
}