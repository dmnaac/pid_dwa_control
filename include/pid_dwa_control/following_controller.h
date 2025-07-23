#ifndef PID_DWA_CONTROL_FOLLOWING_CONTROLLER_H
#define PID_DWA_CONTROL_FOLLOWING_CONTROLLER_H

#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>

#include <spencer_tracking_msgs/TargetPerson.h>

#include "pid_dwa_control/dwa_planner.h"
#include "pid_dwa_control/pid_controller.h"
#include "pid_dwa_control/target.h"

namespace FOLLOWING
{
    class following_controller
    {
    private:
        bool enable_back_;
        double max_vel_x_;
        double max_vel_yaw_;
        double gain_vel_x_;
        double gain_vel_yaw_;
        double distance_;
        double timeout_;
        double scale_vel_x_;
        double scale_vel_yaw_;
        int target_id_;
        double control_dt_;
        double scan_angle_resolution_;

        ros::NodeHandle nh_;
        ros::NodeHandle local_nh_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher predict_footprint_pub_;
        ros::Publisher predict_trajectory_pub_;

        ros::Time last_time_;

        std::unique_ptr<PID_controller> xy_pid_controller_ptr_;
        std::unique_ptr<PID_controller> th_pid_controller_ptr_;

        DWA_planner dwa_planner_;
        geometry_msgs::PoseArray obs_list_;
        geometry_msgs::Twist cmd_vel_;
        geometry_msgs::Twist last_cmd_vel_;
        geometry_msgs::Twist pid_vel_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
        message_filters::Subscriber<spencer_tracking_msgs::TargetPerson> target_sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry, spencer_tracking_msgs::TargetPerson> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        Target target_;
        Target last_target_;

        Eigen::Isometry3d last_transform_;
        Eigen::Isometry3d transform_;

        Eigen::Isometry3d
        odomToTransform(const nav_msgs::Odometry::ConstPtr &odom);

        bool has_reached_;
        bool isTargetValid_;

    public:
        following_controller(ros::NodeHandle nh);
        ~following_controller();

        void create_obs_list(const sensor_msgs::LaserScan::ConstPtr &scan);
        void calc_pid_vel(const spencer_tracking_msgs::TargetPerson target_msg);
        void target_callback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg, const nav_msgs::Odometry::ConstPtr &odomMsg, const spencer_tracking_msgs::TargetPerson::ConstPtr &targetMsg);
        void spin();
    };
}
#endif //  FOLLOWING_CONTROLLER_FOLLOWING_CONTROLLER_H