#include <iostream>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>.

#include "pid_dwa_control/following_controller.h"

namespace FOLLOWING
{
    following_controller::following_controller(ros::NodeHandle nh) : nh_(nh), local_nh_("~"), scale_vel_x_(2.0), scale_vel_yaw_(2.5), target_id_(-1), dwa_planner_(local_nh_), tf_listener_(tf_buffer_), laser_sub_(nh_, "/scan_master", 100), odom_sub_(nh_, "/odom", 100), target_(), last_target_(),
    {
        local_nh_.param<bool>("enable_back", enable_back_, true);
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

        target_sub_ = nh_.subscribe("/mono_following/target", 100, &following_controller::target_register, this);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), laser_sub_, odom_sub_);

        sync_->registerCallback(std::bind(&following_controller::target_callback, this, std::placeholders::_1, std::placeholders::_2));

        double rate = 10;
        control_dt_ = 1.0 / rate;

        xy_pid_controller_ptr_ = std::make_unique<PID_controller>(0.3, 0.0, 0.1, 0.0, -max_vel_x_, max_vel_x_, -0.1, 0.1, control_dt_);

        th_pid_controller_ptr_ = std::make_unique<PID_controller>(1.0, 0.5, 0.2, 0.0, -max_vel_yaw_, max_vel_yaw_, -0.2, 0.2, control_dt_);

        last_time_ = ros::Time::now();
        cmd_vel_ = geometry_msgs::Twist();
        last_cmd_vel_ = geometry_msgs::Twist();
        pid_vel_ = geometry_msgs::Twist();

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

    Eigen::Isometry3d following_controller::odomToTransform(const nav_msgs::Odometry::ConstPtr &odom)
    {
        Eigen::Vector3d position(
            odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z);

        tf2::Quaternion q(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        Eigen::Matrix3d rotation;
        rotation << m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2];

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() = rotation;
        transform.translation() = position;
        return transform;
    }

    void following_controller::target_register(const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg)
    {
        spencer_tracking_msgs::TargetPerson target_msg;
        target_msg = *targetMsg;
        if (target_msg.pose.pose.position.x == 0 || target_msg.pose.pose.position.x > 3.5)
        {
            cmd_vel_pub_.publish(geometry_msgs::Twist());
            target_.is_valid_ = false;
            return;
        }
        else
        {
            target_.setTarget(target_msg);
            target_.is_valid_ = true;
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

        const double angle_threshold = M_PI / 4.0;
        if (std::abs(th_err) < angle_threshold)
        {
            double min_vel_x = enable_back_ ? -max_vel_x_ : 0.0;
            vx = std::clamp(v, min_vel_x, max_vel_x_);

            pid_vel_.linear.x = vx;
            pid_vel_.angular.z = vyaw;
        }
        else
        {
            ROS_INFO("Rotation too big");
        }
    }

    void following_controller::target_callback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg, const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        if (!target_.is_valid_)
        {
            return;
        }

        following_controller::create_obs_list(laserScanMsg);

        if (target_.getTimestamp() > last_target_.getTimestamp())
        {
            ROS_INFO_STREAM("Received new goal");
            dwa_planner_.set_obs_list(obs_list_);

            double px = target_.pose_.pose.position.x;
            double py = target_.pose_.pose.position.y;
            double vx = pid_vel_.linear.x;
            double vyaw = pid_vel_.angular.z;
            const Eigen::Vector3d goal(px, py, 0.0);
            std::vector<State> trajectory = dwa_planner_.generate_trajectory(vx, vyaw);

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
                ROS_INFO("Collision! DWA is replanning.");
                dwa_planner_.set_cur_cmd_vel(last_cmd_vel_);
                cmd_vel_ = dwa_planner_.calc_cmd_vel(goal);
                last_transform_ = odomToTransform(odomMsg);
                cmd_vel_pub_.publish(cmd_vel_);
                ROS_INFO_STREAM("Execute DWA velocity command: vx: " << cmd_vel_.linear.x << ", vyaw: " << cmd_vel_.angular.z);
                last_cmd_vel_ = cmd_vel_;
            }

            last_target_ = target_;
        }
        else
        {
            ROS_INFO_STREAM("Same goal");
            double px = target_.pose_.pose.position.x;
            double py = target_.pose_.pose.position.y;
            // const Eigen::Vector3d goal(px, py, tf::getYaw(target_msg.pose.pose.orientation));
            const Eigen::Vector3d goal(px, py, 0.0);
            transform_ = odomToTransform(odomMsg);
            Eigen::Isometry3d relative_transform = last_transform_.inverse() * transform_;
            double pos_x = relative_transform.translation().x();
            double pos_y = relative_transform.translation().y();
            const Eigen::Vector3d latest_position(pos_x, pos_y, 0.0);
            double dist = (latest_position.segment(0, 2) - goal.segment(0, 2)).norm();
            if (dist < 0.1)
            {
                ROS_WARN_STREAM("Target is lost! Stop!");
                cmd_vel_pub_.publish(geometry_msgs::Twist());
            }
            else
            {
                dwa_planner_.set_obs_list(obs_list_);
                dwa_planner_.set_cur_cmd_vel(last_cmd_vel_);
                Eigen::Vector3d updated_goal = relative_transform * goal;
                cmd_vel_ = dwa_planner_.calc_cmd_vel(updated_goal);
                cmd_vel_pub_.publish(cmd_vel_);
                ROS_WARN_STREAM("Move to last goal: vx:" << cmd_vel_.linear_x << ", yaw: " << cmd_vel_.angualr.z);
                last_cmd_vel_ = cmd_vel_;
            }
        }
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