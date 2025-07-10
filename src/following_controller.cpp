#include <iostream>
#include <cmath>
#include <algorithm>
#include "pid_dwa_control/following_controller.h"

namespace FOLLOWING
{
    following_controller::following_controller() : local_nh_("~"), scale_vel_x_(2.0), scale_vel_yaw_(2.5), target_id_(-1),
                                                   laser_sub_(nh_, "scan", 10), target_sub_(nh, "mono_following/target", 1), dwa_planner_()
    {
        local_nh_.param<bool>("enable_back", enable_back_, true);
        local_nh_.param<double>("max_linear_velocity", max_vel_x_, 0.2);
        local_nh_.param<double>("max_angular_velocity", max_vel_yaw_, 0.5);
        local_nh_.param<double>("gain_linear_velocity", gain_vel_x_, 0.3);
        local_nh_.param<double>("gain_angular_velocity", gain_vel_yaw_, 0.3);
        local_nh_.param<double>("distance", distance_, 1.0);
        local_nh_.param<double>("timeout", timeout_, 1.0);
        local_nh_.param<double>("scan_angle_resolution", scan_angle_resolution_, 0.87);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), laser_sub_, target_sub_);
        sync.registerCallback(boost::bind(&following_controller::target_callback, _1, _2));

        double rate = 10;
        control_dt_ = 1.0 / rate;

        xy_pid_controller_ptr_ = std::make_unique<PID_controller>(0.3, 0.0, 0.1, 0.0, -max_vel_x_, max_vel_x_, -0.1, 0.1, control_dt_);

        th_pid_controller_ptr_ = std::make_unique<PID_controller>(1.0, 0.5, 0.2, 0.0, -max_vel_yaw_, max_vel_yaw_, -0.2, 0.2, control_dt_);

        last_time_ = ros::Time::now();

        ROS_INFO("Mono Controlling Node is Ready!");
    }

    void create_obs_list(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        obs_list_.clear();
        float angle = scan->angle_min;
        const int angle_index_step = static_cast<int>(scan_angle_resolution_ / scan->angle_increment);
        for (int i = 0; i < scan->ranges.size(); i++)
        {
            const float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max || i % angle_index_step != 0)
            {
                angle += scan->angle_increment;
                continue;
            }
            geometry_msgs::Pose pose;
            pose.position.x = range * cos(angle);
            pose.position.y = range * sin(angle);
            obs_list_.poses.push_back(pose);
            angle += scan->angle_increment;
        }
    }

    void following_controller::target_callback(const mono_following::TargetPerson::ConstPtr &targetMsg, const sensor_msgs::LaserScanConstPtr &laserScanMsg)
    {
        target_msg = *targetMsg;
        create_obs_list(laserScanMsg);
        if (target_msg.pose.pose.position.x == 0 || target_msg.pose.pose.position.x > 3.5)
        {
            cmd_vel_pub_.publish(Twist());
            return;
        }

        double px = target_msg.pose.pose.position.x;
        double py = target_msg.pose.pose.position.y;
        double rx = distance_;
        double ry = 0.0;
        double th_err = std::atan2(py, px);
        double p_err = px - rx;

        double w = th_pid_controller_ptr_->calc_output(-th_err, control_dt_) * scale_vel_yaw_;
        double v = xy_pid_controller_ptr_->calc_output(-p_err, control_dt_) * scale_vel_x_;
        double vx = 0.0;
        double vz = w / 2.0;

        const double angle_threshold = M_PI / 4.0;
        if (std::abs(th_err) < angle_threshold)
        {
            double min_vel_x = enable_back_ ? -max_vel_x_ : 0.0;
            vx = std::clamp(vx, min_vel_x, max_vel_x_);
        }
        else
        {
            ROS_INFO("Rotation too big");
        }

        const Eigen::Vector3d goal(px, py, tf::getYaw(target_msg.pose.pose.orientation));
        std::vector<State> trajectory = dwa_planner_.generate_trajectory(vz, goal);
        dwa_planner_.set_obs_list(obs_list_);
        geometry_msgs::Twist cmd_vel;

        if (!dwa_planner_.check_collision(trajectory))
        {
            cmd_vel.linear.x = vx;
            cmd_vel.angular.z = vz;
            cmd_vel_pub_.publish(cmd_vel);
        }
        else
        {
            ROS_INFO("Collision! DWA is replanning.");
            cmd_vel = dwa_planner_.calc_cmd_vel(goal);
            cmd_vel_pub_.publish(cmd_vel);
        }

        last_time_ = target_msg.header.stamp;
    }

    following_controller::spin()
    {
        ros::Duration elapsed = ros::Time::now() - last_time_;
        if (elapsed.toSec() > timeout_)
        {
            ROS_INFO(elapsed.toSec());
            ROS_WARN("Timeout!");
            last_time_ = ros::Time::now();
        }
    }
}