#include <iostream>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include "pid_dwa_control/following_controller.h"

namespace FOLLOWING
{
    following_controller::following_controller(ros::NodeHandle nh) : nh_(nh), local_nh_("~"), scale_vel_x_(2.0), scale_vel_yaw_(2.5), target_id_(-1), dwa_planner_(local_nh_), tf_listener_(tf_buffer_), laser_sub_(nh_, "/scan_master", 100), target_sub_(nh_, "/mono_following/target", 100)
    {
        local_nh_.param<bool>("enable_back", enable_back_, true);
        local_nh_.param<bool>("enable_dwa", enable_dwa_, false);
        local_nh_.param<double>("max_linear_velocity", max_vel_x_, 0.2);
        local_nh_.param<double>("max_angular_velocity", max_vel_yaw_, 0.5);
        local_nh_.param<double>("gain_linear_velocity", gain_vel_x_, 0.3);
        local_nh_.param<double>("gain_angular_velocity", gain_vel_yaw_, 0.3);
        local_nh_.param<double>("distance", distance_, 1.0);
        local_nh_.param<double>("timeout", timeout_, 1.0);
        local_nh_.param<double>("scan_angle_resolution", scan_angle_resolution_, 0.087);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_x", 1);

        predict_footprint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/predict_footprint", 1);

        predict_trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("/predict_trajectory", 1);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), laser_sub_, target_sub_);

        // message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh_, "/scan_master", 10);
        // message_filters::Subscriber<spencer_tracking_msgs::TargetPerson> target_sub(nh_, "/mono_following/target", 1);
        // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, spencer_tracking_msgs::TargetPerson> SyncPolicy;
        // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), laser_sub_, target_sub_);
        sync_->registerCallback(std::bind(&following_controller::target_callback, this, std::placeholders::_1, std::placeholders::_2));

        double rate = 10;
        control_dt_ = 1.0 / rate;

        xy_pid_controller_ptr_ = std::make_unique<PID_controller>(0.3, 0.0, 0.1, 0.0, -max_vel_x_, max_vel_x_, -0.1, 0.1, control_dt_);

        th_pid_controller_ptr_ = std::make_unique<PID_controller>(1.0, 0.5, 0.2, 0.0, -max_vel_yaw_, max_vel_yaw_, -0.2, 0.2, control_dt_);

        last_time_ = ros::Time::now();
        cmd_vel_ = geometry_msgs::Twist();
        last_cmd_vel_ = geometry_msgs::Twist();

        ROS_INFO("Controlling Node is Ready!");
    }

    following_controller::~following_controller() {}

    void following_controller::create_obs_list(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        obs_list_.poses.clear();
        float angle = scan->angle_min;
        const int angle_index_step = static_cast<int>(scan_angle_resolution_ / scan->angle_increment);
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("base_link", scan->header.frame_id, ros::Time(0), ros::Duration(0.1));

        for (int i = 0; i < scan->ranges.size(); i++)
        {
            const float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max || i % angle_index_step != 0)
            {
                angle += scan->angle_increment;
                continue;
            }

            geometry_msgs::PointStamped obs_lidar;
            obs_lidar.header = scan->header;
            obs_lidar.point.x = range * std::cos(angle);
            obs_lidar.point.y = range * std::sin(angle);
            obs_lidar.point.z = 0.0;

            geometry_msgs::PointStamped obs_robot;
            tf2::doTransform(obs_lidar, obs_robot, transform);

            geometry_msgs::Pose pose;
            pose.position = obs_robot.point;
            obs_list_.poses.push_back(pose);
            angle += scan->angle_increment;
        }
    }

    void following_controller::target_callback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg, const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg)
    {
        // ROS_INFO("Processing synchronized messages");
        spencer_tracking_msgs::TargetPerson target_msg;
        target_msg = *targetMsg;
        following_controller::create_obs_list(laserScanMsg);
        // ROS_INFO_STREAM("Obstacle list size: " << obs_list_.poses.size());
        if (target_msg.pose.pose.position.x == 0 || target_msg.pose.pose.position.x > 3.5)
        {
            cmd_vel_pub_.publish(geometry_msgs::Twist());
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
        double vyaw = w / 2.0;
        // ROS_INFO_STREAM("PID output vx: " << v << " vyaw: " << w);

        const double angle_threshold = M_PI / 4.0;
        if (std::abs(th_err) < angle_threshold)
        {
            double min_vel_x = enable_back_ ? -max_vel_x_ : 0.0;
            vx = std::clamp(v, min_vel_x, max_vel_x_);
        }
        else
        {
            ROS_INFO("Rotation too big");
        }

        // const Eigen::Vector3d goal(px, py, tf::getYaw(target_msg.pose.pose.orientation));
        const Eigen::Vector3d goal(px, py, 0.0);
        std::vector<State> trajectory = dwa_planner_.generate_trajectory(vx, vyaw);
        // ROS_INFO_STREAM("Trajectory size: " << trajectory.size());
        dwa_planner_.set_obs_list(obs_list_);
        // ROS_INFO_STREAM("Obstacle list size: " << dwa_planner_.get_obs_list().poses.size());

        // ROS_INFO_STREAM("DWA planner input: vx: " << vx << " vyaw: " << vyaw);

        if (!dwa_planner_.check_collision(trajectory))
        {
            cmd_vel_.linear.x = vx;
            cmd_vel_.angular.z = vyaw;
            cmd_vel_pub_.publish(cmd_vel_);
            ROS_INFO_STREAM("Execute PID velocity command: vx: " << vx << ", vyaw: " << vyaw);
            last_cmd_vel_ = cmd_vel_;

            dwa_planner_.visualize_trajectory(trajectory, predict_trajectory_pub_);
        }
        else
        {
            ROS_INFO("Collision!");
            if (!enable_dwa_)
            {
                cmd_vel_pub_.publish(geometry_msgs::Twist());
                return;
            }

            dwa_planner_.set_cur_cmd_vel(last_cmd_vel_);
            cmd_vel_ = dwa_planner_.calc_cmd_vel(goal);
            cmd_vel_pub_.publish(cmd_vel_);
            ROS_INFO_STREAM("Execute DWA velocity command: vx: " << cmd_vel_.linear.x << ", vyaw:" << cmd_vel_.angular.z);
            last_cmd_vel_ = cmd_vel_;
        }

        last_time_ = target_msg.header.stamp;
    }

    void following_controller::spin()
    {
        if (last_time_.isZero())
        {
            last_time_ = ros::Time::now();
            return;
        }

        ros::Duration elapsed = ros::Time::now() - last_time_;
        if (elapsed.toSec() > timeout_)
        {
            // ROS_INFO("Elapsed time: %.2f seconds", elapsed.toSec());
            // ROS_WARN("Timeout! No message received for %.2f seconds", timeout_);
            last_time_ = ros::Time::now();
        }
    }
}