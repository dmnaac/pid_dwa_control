#ifndef FOLLOWING_CONTROLLER_FOLLOWING_CONTROLLER_H
#define FOLLOWING_CONTROLLER_FOLLOWING_CONTROLLER_H

#include <memory>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "dwa_planner.h"
#include "pid_controller.h"

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
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        message_filters::Subscriber<TargetPerson> target_sub_;

        ros::Time last_time_;

        std::unique_ptr<PID_controller> xy_pid_controller_ptr_;
        std::unique_ptr<PID_controller> th_pid_controller_ptr_;

        DWA_planner dwa_planner_;
        geometry_msgs::PoseArray obs_list_;
        geometry_msgs::Twist pid_cmd_vel_;

    public:
        following_controller();
        ~following_controller();

        void create_obs_list(const sensor_msgs::LaserScan::ConstPtr &scan);
        void target_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void spin();
    };
}
#endif //  FOLLOWING_CONTROLLER_FOLLOWING_CONTROLLER_H