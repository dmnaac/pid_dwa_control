#ifndef FOLLOWING_CONTROLLER_PID_CONTROLLER_H
#define FOLLOWING_CONTROLLER_PID_CONTROLLER_H

namespace FOLLOWING
{
    class PID_controller
    {
    private:
        double err_cur_;
        double err_last_;
        double err_int_;
        double err_der_;
        double pid_ref_;
        double pid_out_;

        double kp_;
        double ki_;
        double kd_;

        double dt_;
        double max_err_int_;
        double min_err_int_;
        double max_pid_out_;
        double min_pid_out_;

        double deadband_;

    public:
        PID_controller(double kp, double ki, double kd, double deadband, double min_pid_out, double max_pid_out, double min_err_int, double max_err_int, double dt = 0.1);
        ~PID_controller();

        double calc_output(double pos_x, double dt);
    };
}

#endif // FOLLOWING_CONTROLLER_PID_CONTROLLER_H