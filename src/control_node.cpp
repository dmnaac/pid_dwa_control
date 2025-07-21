#include "pid_dwa_control/following_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_dwa_control_node");
    ros::NodeHandle nh;
    FOLLOWING::following_controller follower(nh);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        follower.spin();
        loop_rate.sleep();
    }
    return 0;
}