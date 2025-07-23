#ifndef PID_DWA_CONTROL_DWA_PLANNER_H
#define PID_DWA_CONTROL_DWA_PLANNER_H

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "pid_dwa_control/state.h"
#include "pid_dwa_control/cost.h"
#include "pid_dwa_control/window.h"

namespace FOLLOWING
{
    class DWA_planner
    {

    public:
        DWA_planner(ros::NodeHandle private_nh);
        ~DWA_planner();

        void sim_one_step(FOLLOWING::State &state, const double vel_x, const double vel_yaw);
        std::vector<FOLLOWING::State> generate_trajectory(const double vel_yaw, const Eigen::Vector3d &goal);
        std::vector<FOLLOWING::State> generate_trajectory(const double vel_x, const double vel_yaw);
        geometry_msgs::PolygonStamped move_footprint(const FOLLOWING::State &step);

        bool is_point_in_triangle(const geometry_msgs::Point &point, const geometry_msgs::Polygon &triangle);
        bool is_point_in_footprint(const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const FOLLOWING::State &step);
        bool check_collision(const std::vector<FOLLOWING::State> &trajectory);

        FOLLOWING::Window calc_dynamic_window();
        void set_cur_cmd_vel(const geometry_msgs::Twist &cmd_vel);
        void set_obs_list(const geometry_msgs::PoseArray &obs_list);
        geometry_msgs::PoseArray get_obs_list();

        geometry_msgs::Point calc_intersection(const geometry_msgs::Point &obstacle, const FOLLOWING::State &state, geometry_msgs::PolygonStamped footprint);
        float calc_dist_to_footprint(const geometry_msgs::Point &obstacle, const FOLLOWING::State &state);

        float calc_goal_cost(const std::vector<FOLLOWING::State> &trajectory, const Eigen::Vector3d &goal);
        float calc_direction_cost(const std::vector<FOLLOWING::State> &trajectory, const Eigen::Vector3d &goal);
        float calc_obs_cost(const std::vector<FOLLOWING::State> &trajectory);
        float calc_speed_cost(const std::vector<FOLLOWING::State> &trajectory);
        FOLLOWING::Cost calc_traj_cost(const std::vector<FOLLOWING::State> &trajectory, const Eigen::Vector3d &goal);

        void normalize_costs(std::vector<FOLLOWING::Cost> &costs);

        std::vector<FOLLOWING::State> dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<FOLLOWING::State>, bool>> &trajectories);

        geometry_msgs::Twist calc_cmd_vel(const Eigen::Vector3d &goal);

        void load_params();

        visualization_msgs::Marker create_marker_msg(
            const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
            const geometry_msgs::PolygonStamped &footprint = geometry_msgs::PolygonStamped());
        void visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub);

        void visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub);
        void visualize_trajectories(const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub);

        std::vector<State> best_trajectory_to_show_;

    private:
        ros::NodeHandle local_nh_;
        double sim_direction_;
        int sim_time_samples_;
        int velocity_samples_x_;
        int velocity_samples_y_;
        int velocity_samples_yaw_;
        double predict_time_;
        double sim_period_;
        bool use_footprint_;
        bool use_path_cost_;
        bool use_speed_cost_;
        double robot_radius_;
        double footprint_padding_;
        geometry_msgs::PoseArray obs_list_;
        geometry_msgs::Twist cur_cmd_vel_;
        double slow_velocity_th_;
        double min_linear_velocity_;
        double max_linear_velocity_;
        double min_angular_velocity_;
        double max_angular_velocity_;
        double target_linear_velocity_;
        double max_linear_acceleration_;
        double max_linear_deceleration_;
        double max_angular_acceleration_;

        double obs_range_;

        double obs_cost_gain_;
        double goal_cost_gain_;
        double speed_cost_gain_;
        double direction_cost_gain_;

        std::vector<geometry_msgs::Point32> footprint_points_;
        geometry_msgs::PolygonStamped footprint_;

        void generate_footprint();
    };
}

#endif // FOLLOWING_CONTROLLER_DWA_PLANNER_H