#pragma once
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
namespace local_planner_checks {
struct Limits {
 double acc_lim_x{}, acc_lim_theta{}, max_vel_x{}, min_vel_trans{}, max_vel_theta{}, min_vel_theta{};
 double theta_stopped_vel{}, trans_stopped_vel{}, xy_goal_tolerance{}, yaw_goal_tolerance{};
};
inline bool stopped(const nav_msgs::msg::Odometry &o, double w, double v) {
 return std::abs(o.twist.twist.angular.z) <= w &&
        std::hypot(o.twist.twist.linear.x, o.twist.twist.linear.y) <= v;
}
}
