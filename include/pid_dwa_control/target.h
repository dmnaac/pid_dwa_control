#ifndef PID_DWA_CONTROL_TARGET_H
#define PID_DWA_CONTROL_TARGET_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "spencer_tracking_msgs/TargetPerson.h"

namespace FOLLOWING
{
    class Target
    {
    private:
        geometry_msgs::PoseStamped pose_;
        geometry_msgs::TwistStamped twist_;

    public:
        Target();
        ~Target();

        geometry_msgs::PoseStamped getPose();
        geometry_msgs::TwistStamped getTwist();
        ros::Time getTimestamp();
        void setTarget(const spencer_tracking_msgs::TargetPerson target);

        bool is_valid_;
    };
}

#endif // PID_DWA_CONTROL_TARGET_H