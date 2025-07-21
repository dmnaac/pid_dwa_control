#include <algorithm>
#include <vector>
#include <cmath>

#include "pid_dwa_control/dwa_planner.h"

namespace FOLLOWING
{
    DWA_planner::DWA_planner(ros::NodeHandle private_nh) : local_nh_(private_nh)
    {
        DWA_planner::load_params();
        DWA_planner::generate_footprint();
        ROS_INFO("DWA Planner is Ready!");
    }

    DWA_planner::~DWA_planner()
    {
    }

    void DWA_planner::set_cur_cmd_vel(const geometry_msgs::Twist &cmd_vel)
    {
        cur_cmd_vel_ = cmd_vel;
    }

    void DWA_planner::set_obs_list(const geometry_msgs::PoseArray &obs_list)
    {
        obs_list_ = obs_list;
    }

    geometry_msgs::PoseArray DWA_planner::get_obs_list()
    {
        return obs_list_;
    }

    void DWA_planner::generate_footprint()
    {
        if (use_footprint_)
        {
            // ROS_INFO_STREAM("Add padding");
            std::vector<geometry_msgs::Point32> padded_points;
            for (int i = 0; i < footprint_points_.size(); i++)
            {
                geometry_msgs::Point32 point;
                int direction_x = (footprint_points_[i].x < 0) ? -1 : 1;
                point.x = footprint_points_[i].x + direction_x * footprint_padding_;
                int direction_y = (footprint_points_[i].y < 0) ? -1 : 1;
                point.y = footprint_points_[i].y + direction_y * footprint_padding_;
                padded_points.push_back(point);
            }

            int num_segments = 10;
            for (int i = 0; i < padded_points.size() - 1; i++)
            {
                for (int j = 1; j < num_segments; j++)
                {
                    // The first point is the same as the padded_points_[i]
                    // When this point is the first, it will not be added to the footprint, therefore j starts from 1

                    double t = static_cast<double>(j) / num_segments;
                    geometry_msgs::Point32 point;
                    point.x = padded_points[i].x + t * (padded_points[i + 1].x - padded_points[i].x);
                    point.y = padded_points[i].y + t * (padded_points[i + 1].y - padded_points[i].y);
                    footprint_.polygon.points.push_back(point);
                }
            }
            // Process the last padded footprint point
            for (int j = 1; j < num_segments; j++)
            {
                double t = static_cast<double>(j) / num_segments;
                geometry_msgs::Point32 point;
                int point_index = padded_points.size() - 1;
                point.x = padded_points[point_index].x + t * (padded_points[0].x - padded_points[point_index].x);
                point.y = padded_points[point_index].y + t * (padded_points[0].y - padded_points[point_index].y);
                footprint_.polygon.points.push_back(point);
            }
        }
        else
        {
            const int plot_num = 20;
            for (int i = 0; i < plot_num; i++)
            {
                geometry_msgs::Point32 point;
                point.x = (robot_radius_ + footprint_padding_) * cos(2 * M_PI * i / plot_num);
                point.y = robot_radius_ * sin(2 * M_PI * i / plot_num);
                footprint_.polygon.points.push_back(point);
            }
        }

        footprint_.header.stamp = ros::Time::now();
        footprint_.header.frame_id = "base_link";
    }

    visualization_msgs::Marker DWA_planner::create_marker_msg(
        const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory,
        const geometry_msgs::PolygonStamped &footprint)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = scale;
        marker.color = color;
        marker.color.a = 0.8;
        marker.lifetime = ros::Duration(0.05);

        geometry_msgs::Point p;
        if (footprint.polygon.points.empty())
        {
            for (const auto &point : trajectory)
            {
                p.x = point.x_;
                p.y = point.y_;
                marker.points.push_back(p);
            }
        }
        else
        {
            for (const auto &point : footprint.polygon.points)
            {
                p.x = point.x;
                p.y = point.y;
                marker.points.push_back(p);
            }
            p.x = footprint.polygon.points.front().x;
            p.y = footprint.polygon.points.front().y;
            marker.points.push_back(p);
        }

        return marker;
    }

    void DWA_planner::visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub)
    {
        std_msgs::ColorRGBA color;
        color.b = 1.0;
        visualization_msgs::MarkerArray v_footprints;
        for (int i = 0; i < trajectory.size(); i++)
        {
            const geometry_msgs::PolygonStamped moved_footprint = move_footprint(trajectory[i]);
            visualization_msgs::Marker v_footprint = create_marker_msg(i, 0.05 * 0.2, color, trajectory, moved_footprint);
            v_footprints.markers.push_back(v_footprint);
        }
        pub.publish(v_footprints);
    }

    void DWA_planner::visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub)
    {
        std_msgs::ColorRGBA color;
        color.b = 1.0;
        visualization_msgs::Marker v_trajectory = create_marker_msg(0, 0.05 * 0.2, color, trajectory);
        pub.publish(v_trajectory);
    }

    void DWA_planner::visualize_trajectories(
        const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub)
    {
        visualization_msgs::MarkerArray v_trajectories;
        for (int i = 0; i < trajectories.size(); i++)
        {
            std_msgs::ColorRGBA color;
            if (trajectories[i].second)
            {
                color.g = 1.0;
            }
            else
            {
                color.r = 0.5;
                color.b = 0.5;
            }
            visualization_msgs::Marker v_trajectory = create_marker_msg(i, 0.05 * 0.4, color, trajectories[i].first);
            v_trajectories.markers.push_back(v_trajectory);
        }
        pub.publish(v_trajectories);
    }

    void DWA_planner::sim_one_step(State &state, const double vel_x, const double vel_yaw)
    {
        const double sim_time_step = predict_time_ / static_cast<double>(sim_time_samples_);
        state.yaw_ += vel_yaw * sim_time_step;
        state.x_ += vel_x * std::cos(state.yaw_) * sim_time_step;
        state.y_ += vel_x * std::sin(state.yaw_) * sim_time_step;
        state.vel_x_ = vel_x;
        state.vel_yaw_ = vel_yaw;
    }

    std::vector<State> DWA_planner::generate_trajectory(const double vel_yaw, const Eigen::Vector3d &goal)
    {
        const double target_direction = atan2(goal.y(), goal.x()) > 0 ? sim_direction_ : -sim_direction_;
        const double predict_time = target_direction / (vel_yaw + __DBL_EPSILON__);
        std::vector<State> trajectory;
        trajectory.resize(sim_time_samples_);
        State state;
        for (int i = 0; i < sim_time_samples_; i++)
        {
            sim_one_step(state, 0.0, vel_yaw);
            trajectory[i] = state;
        }
        return trajectory;
    }

    std::vector<State> DWA_planner::generate_trajectory(const double vel_x, const double vel_yaw)
    {
        std::vector<State> trajectory;
        trajectory.resize(sim_time_samples_);
        State state;
        for (int i = 0; i < sim_time_samples_; i++)
        {
            sim_one_step(state, vel_x, vel_yaw);
            trajectory[i] = state;
        }
        return trajectory;
    }

    geometry_msgs::PolygonStamped DWA_planner::move_footprint(const State &step)
    {
        // ROS_INFO_STREAM("Move footprint for one step");
        // ROS_INFO_STREAM("Footprint points size: " << footprint_points_.size());
        geometry_msgs::PolygonStamped moved_footprint;

        for (auto &point : footprint_.polygon.points)
        {
            Eigen::VectorXf point_in(2);
            point_in << point.x, point.y;
            Eigen::Matrix2f rotation;
            rotation = Eigen::Rotation2Df(step.yaw_);
            Eigen::VectorXf point_out(2);
            point_out = rotation * point_in;

            geometry_msgs::Point32 moved_point;
            moved_point.x = point_out.x() + step.x_;
            moved_point.y = point_out.y() + step.y_;

            moved_footprint.polygon.points.push_back(moved_point);
        }

        // ROS_INFO_STREAM("Finished moving footprint for one step");
        moved_footprint.header.stamp = ros::Time::now();
        moved_footprint.header.frame_id = "base_link";
        return moved_footprint;
    }

    bool DWA_planner::is_point_in_triangle(const geometry_msgs::Point &point, const geometry_msgs::Polygon &triangle)
    {
        if (triangle.points.size() != 3)
        {
            ROS_ERROR("triangle size is not 3");
            exit(1);
        }

        // ROS_INFO_STREAM("Check triangle");

        const Eigen::Vector3d point_A(triangle.points[0].x, triangle.points[0].y, 0.0);
        const Eigen::Vector3d point_B(triangle.points[1].x, triangle.points[1].y, 0.0);
        const Eigen::Vector3d point_C(triangle.points[2].x, triangle.points[2].y, 0.0);
        const Eigen::Vector3d point_D(point.x, point.y, 0.0);

        const Eigen::Vector3d vector_AB = point_B - point_A;
        const Eigen::Vector3d vector_BD = point_D - point_B;
        const Eigen::Vector3d cross_AB_BD = vector_AB.cross(vector_BD);
        const Eigen::Vector3d vector_BC = point_C - point_B;
        const Eigen::Vector3d vector_CD = point_D - point_C;
        const Eigen::Vector3d cross_BC_CD = vector_BC.cross(vector_CD);
        const Eigen::Vector3d vector_CA = point_A - point_C;
        const Eigen::Vector3d vector_AD = point_D - point_A;
        const Eigen::Vector3d cross_CA_AD = vector_CA.cross(vector_AD);

        if (cross_AB_BD.z() * cross_BC_CD.z() > 0 && cross_BC_CD.z() * cross_CA_AD.z() > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool DWA_planner::is_point_in_footprint(const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const State &step)
    {
        geometry_msgs::Point32 point;
        point.x = step.x_;
        point.y = step.y_;

        // ROS_INFO_STREAM("Check footprint");
        for (int i = 0; i < footprint.polygon.points.size(); i++)
        {
            geometry_msgs::Polygon triangle;
            triangle.points.push_back(point);
            triangle.points.push_back(footprint.polygon.points[i]);

            if (i == footprint.polygon.points.size() - 1)
            {
                triangle.points.push_back(footprint.polygon.points[0]);
            }
            else
            {
                triangle.points.push_back(footprint.polygon.points[i + 1]);
            }

            if (is_point_in_triangle(obstacle, triangle))
            {
                return true;
            }
        }

        return false;
    }

    bool DWA_planner::check_collision(const std::vector<State> &trajectory)
    {
        for (auto &step : trajectory)
        {
            for (auto &obs : obs_list_.poses)
            {
                const geometry_msgs::PolygonStamped moved_footprint = move_footprint(step);
                if (is_point_in_footprint(obs.position, moved_footprint, step))
                {
                    return true;
                }
            }
        }

        return false;
    }

    Window DWA_planner::calc_dynamic_window()
    {
        Window window;
        window.min_vel_x_ = std::max((cur_cmd_vel_.linear.x - max_linear_deceleration_ * sim_period_), min_linear_velocity_);
        window.max_vel_x_ = std::min((cur_cmd_vel_.linear.x + max_linear_acceleration_ * sim_period_), target_linear_velocity_);
        window.min_vel_yaw_ = std::max((cur_cmd_vel_.angular.z - max_angular_acceleration_ * sim_period_), -max_angular_velocity_);
        window.max_vel_yaw_ = std::min((cur_cmd_vel_.angular.z + max_angular_acceleration_ * sim_period_), max_angular_velocity_);
        return window;
    }

    geometry_msgs::Point DWA_planner::calc_intersection(
        const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint)
    {
        const Eigen::Vector3d ray_start(obstacle.x, obstacle.y, 0.0);
        const Eigen::Vector3d ray_end(state.x_, state.y_, 0.0);
        const Eigen::Vector3d ray_direction = ray_end - ray_start;

        const double INF = 1e6;
        geometry_msgs::Point invalid_point;
        invalid_point.x = INF;
        invalid_point.y = INF;

        for (size_t i = 0; i < footprint.polygon.points.size(); ++i)
        {
            const Eigen::Vector3d edge_start(
                footprint.polygon.points[i].x,
                footprint.polygon.points[i].y,
                0.0);

            size_t next_idx = (i == footprint.polygon.points.size() - 1) ? 0 : i + 1;
            const Eigen::Vector3d edge_end(
                footprint.polygon.points[next_idx].x,
                footprint.polygon.points[next_idx].y,
                0.0);

            const Eigen::Vector3d edge_direction = edge_end - edge_start;

            const double deno = ray_direction.cross(edge_direction).z();

            if (std::abs(deno) < 1e-10)
                continue;

            const double s = (edge_start - ray_start).cross(edge_direction).z() / deno;
            const double t = ray_direction.cross(ray_start - edge_start).z() / deno;

            if (s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0)
            {
                geometry_msgs::Point intersection;
                intersection.x = ray_start.x() + s * ray_direction.x();
                intersection.y = ray_start.y() + s * ray_direction.y();
                return intersection;
            }
        }

        return invalid_point;
    }

    float DWA_planner::calc_dist_to_footprint(const geometry_msgs::Point &obstacle, const State &state)
    {
        const geometry_msgs::PolygonStamped footprint = move_footprint(state);
        if (is_point_in_footprint(obstacle, footprint, state))
        {
            return 0.0;
        }
        else
        {
            geometry_msgs::Point intersection = calc_intersection(obstacle, state, footprint);
            return hypot((obstacle.x - intersection.x), (obstacle.y - intersection.y));
        }
    }

    float DWA_planner::calc_goal_cost(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
    {
        Eigen::Vector3d last_position(trajectory.back().x_, trajectory.back().y_, trajectory.back().yaw_);
        return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
    }

    float DWA_planner::calc_direction_cost(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
    {
        float direction_cost = 0.0;

        for (const auto &state : trajectory)
        {
            float diff = fabs(state.yaw_ - goal[2]);
            if (diff > M_PI / 4.0)
            {
                return 1e6;
            }
            else
            {
                direction_cost += diff;
            }
        }

        return direction_cost;
    }

    float DWA_planner::calc_obs_cost(const std::vector<State> &trajectory)
    {
        float min_dist = obs_range_;
        for (const auto &state : trajectory)
        {
            for (const auto &obs : obs_list_.poses)
            {
                float dist;
                if (use_footprint_)
                {
                    dist = calc_dist_to_footprint(obs.position, state);
                }
                else
                {
                    dist = hypot((obs.position.x - state.x_), (obs.position.y - state.y_)) - robot_radius_ - footprint_padding_;
                }

                if (dist < __DBL_EPSILON__)
                {
                    return 1e6;
                }
                min_dist = std::min(min_dist, dist);
            }
        }
        return obs_range_ - min_dist;
    }

    float DWA_planner::calc_speed_cost(const std::vector<State> &trajectory)
    {
        if (!use_speed_cost_)
        {
            return 0.0;
        }
        const Window dynamic_window = calc_dynamic_window();
        return dynamic_window.max_vel_x_ - trajectory.front().vel_x_;
    }

    Cost DWA_planner::calc_traj_cost(const std::vector<State> &trajectory, const Eigen::Vector3d &goal)
    {
        Cost cost;
        cost.goal_cost_ = calc_goal_cost(trajectory, goal);
        cost.direction_cost_ = calc_direction_cost(trajectory, goal);
        cost.obs_cost_ = calc_obs_cost(trajectory);
        cost.speed_cost_ = calc_speed_cost(trajectory);
        cost.path_cost_ = 0.0;
        cost.calc_total_cost();
        return cost;
    }

    void DWA_planner::normalize_costs(std::vector<Cost> &costs)
    {
        Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

        for (const auto &cost : costs)
        {
            if (cost.obs_cost_ != 1e6)
            {
                min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
                max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
                min_cost.goal_cost_ = std::min(min_cost.goal_cost_, cost.goal_cost_);
                max_cost.goal_cost_ = std::max(max_cost.goal_cost_, cost.goal_cost_);
                min_cost.direction_cost_ = std::min(min_cost.direction_cost_, cost.direction_cost_);
                max_cost.direction_cost_ = std::max(max_cost.direction_cost_, cost.direction_cost_);

                if (use_speed_cost_)
                {
                    min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
                    max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
                }
                if (use_path_cost_)
                {
                    min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
                    max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
                }
            }
        }

        for (auto &cost : costs)
        {
            if (cost.obs_cost_ != 1e6)
            {
                cost.obs_cost_ = (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + __DBL_EPSILON__);
                cost.goal_cost_ = (cost.goal_cost_ - min_cost.goal_cost_) / (max_cost.goal_cost_ - min_cost.goal_cost_ + __DBL_EPSILON__);
                cost.direction_cost_ = (cost.direction_cost_ - min_cost.direction_cost_) / (max_cost.direction_cost_ - min_cost.direction_cost_ + __DBL_EPSILON__);

                if (use_speed_cost_)
                {
                    cost.speed_cost_ = (cost.speed_cost_ - min_cost.speed_cost_) / (max_cost.speed_cost_ - min_cost.speed_cost_ + __DBL_EPSILON__);
                }
                if (use_path_cost_)
                {
                    cost.path_cost_ = (cost.path_cost_ - min_cost.path_cost_) / (max_cost.path_cost_ - min_cost.path_cost_ + __DBL_EPSILON__);
                }
            }
        }
    }

    std::vector<State> DWA_planner::dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories)
    {
        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 1e6);
        const Window dynamic_window = calc_dynamic_window();
        std::vector<State> best_trajectory;
        best_trajectory.resize(sim_time_samples_);
        std::vector<Cost> costs;
        const size_t costs_size = velocity_samples_x_ * (velocity_samples_yaw_ + 1);
        costs.reserve(costs_size);

        const double vel_x_step = std::max((dynamic_window.max_vel_x_ - dynamic_window.min_vel_x_) / (velocity_samples_x_ - 1), __DBL_EPSILON__);
        const double vel_yaw_step = std::max((dynamic_window.max_vel_yaw_ - dynamic_window.min_vel_yaw_) / (velocity_samples_yaw_ - 1), __DBL_EPSILON__);

        int avai_traj_num = 0;
        for (int i = 0; i < velocity_samples_x_; i++)
        {
            const double vel_x = dynamic_window.min_vel_x_ + vel_x_step * i;
            for (int j = 0; j < velocity_samples_yaw_; j++)
            {
                std::pair<std::vector<State>, bool> trajectory;
                double vel_yaw = dynamic_window.min_vel_yaw_ + vel_yaw_step * j;
                if (vel_x < slow_velocity_th_)
                {
                    vel_yaw = vel_yaw > 0 ? std::max(vel_yaw, min_angular_velocity_) : std::min(vel_yaw, -min_angular_velocity_);
                }
                trajectory.first = generate_trajectory(vel_x, vel_yaw);
                const Cost cost = calc_traj_cost(trajectory.first, goal);
                costs.push_back(cost);
                if (cost.obs_cost_ == 1e6)
                {
                    trajectory.second = false;
                }
                else
                {
                    trajectory.second = true;
                    avai_traj_num++;
                }
                trajectories.push_back(trajectory);
            }

            if (dynamic_window.min_vel_yaw_ < 0.0 && 0.0 < dynamic_window.max_vel_yaw_)
            {
                std::pair<std::vector<State>, bool> trajectory;
                trajectory.first = generate_trajectory(vel_x, 0.0);
                const Cost cost = calc_traj_cost(trajectory.first, goal);
                costs.push_back(cost);
                if (cost.obs_cost_ == 1e6)
                {
                    trajectory.second = false;
                }
                else
                {
                    trajectory.second = true;
                    avai_traj_num++;
                }
                trajectories.push_back(trajectory);
            }
        }

        if (avai_traj_num == 0)
        {
            ROS_ERROR_THROTTLE(1.0, "No available trajectory");
            best_trajectory = generate_trajectory(0.0, 0.0);
        }
        else
        {
            normalize_costs(costs);
            for (int i = 0; i < costs.size(); i++)
            {
                if (costs[i].obs_cost_ != 1e6)
                {
                    costs[i].goal_cost_ *= goal_cost_gain_;
                    costs[i].obs_cost_ *= obs_cost_gain_;
                    costs[i].speed_cost_ *= speed_cost_gain_;
                    costs[i].direction_cost_ *= direction_cost_gain_;
                    costs[i].calc_total_cost();
                    if (costs[i].total_cost_ < min_cost.total_cost_)
                    {
                        min_cost = costs[i];
                        best_trajectory = trajectories[i].first;
                    }
                }
            }
        }

        ROS_INFO("===");
        ROS_INFO_STREAM("(vel_x, vel_yaw) = (" << best_trajectory[0].vel_x_ << ", " << best_trajectory[0].vel_yaw_ << ")");
        min_cost.show();
        ROS_INFO_STREAM("Num of trajectories available: " << avai_traj_num << " of " << trajectories.size());
        ROS_INFO(" ");

        return best_trajectory;
    }

    geometry_msgs::Twist DWA_planner::calc_cmd_vel(const Eigen::Vector3d &goal)
    {
        geometry_msgs::Twist cmd_vel;
        std::pair<std::vector<State>, bool> best_trajectory;
        std::vector<std::pair<std::vector<State>, bool>> trajectories;
        const size_t trajectoris_size = velocity_samples_x_ * (velocity_samples_yaw_ + 1);
        trajectories.reserve(trajectoris_size);
        best_trajectory.first = dwa_planning(goal, trajectories);
        cmd_vel.linear.x = best_trajectory.first.front().vel_x_;
        cmd_vel.angular.z = best_trajectory.first.front().vel_yaw_;
        return cmd_vel;
    }
}