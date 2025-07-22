#include "pid_dwa_control/target.h"

namespace FOLLOWING
{
    Target::Target()
    {
        pose_.header.stamp = ros::Time::Time();
        pose_.pose.position.x = 0.0;
        pose_.pose.position.y = 0.0;
        pose_.pose.position.z = 0.0;
        pose_.pose.orientation.x = 0.0;
        pose_.pose.orientation.y = 0.0;
        pose_.pose.orientation.z = 0.0;
        pose_.pose.orientation.w = 1.0;

        twist_.header.stamp = ros::Time::Time();
        twist_.twist.linear.x = 0.0;
        twist_.twist.linear.y = 0.0;
        twist_.twist.linear.z = 0.0;
        twist_.twist.angular.x = 0.0;
        twist_.twist.angular.y = 0.0;
        twist_.twist.angular.z = 0.0;

        is_valid_ = false;
    }

    geometry_msgs::PoseStamped Target::getPose()
    {
        return pose_;
    }

    geometry_msgs::TwistStamped Target::getTwist()
    {
        return twist_;
    }

    ros::Time Target::getTimestamp()
    {
        return pose_.header.stamp;
    }

    void Target::setTarget(const spencer_tracking_msgs::TargetPerson target)
    {
        pose_.header.stamp = target.header.stamp;
        pose_.pose = target.pose;
        twist_.header.stamp = target.header.stamp;
        twist_.twist = target.twist;
    }
}