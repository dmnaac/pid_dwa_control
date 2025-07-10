#ifndef FOLLOWING_CONTROLLER_WINDOW_H
#define FOLLOWING_CONTROLLER_WINDOW_H

namespace FOLLOWING
{
    class Window
    {
    private:
        /* data */
    public:
        Window(/* args */);
        ~Window();

        void show();

        double min_vel_x_;
        double max_vel_x_;
        double min_vel_yaw_;
        double max_vel_yaw_;
    };
}

#endif // FOLLOWING_CONTROLLER_WINDOW_H