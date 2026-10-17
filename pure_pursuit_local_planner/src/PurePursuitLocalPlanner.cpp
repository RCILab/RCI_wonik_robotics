#include "pure_pursuit_local_planner/PurePursuitLocalPlanner.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitLocalPlanner, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner
{

namespace
{

tf2::Quaternion createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

double slew(double last, double target, double max_step)
{
    return std::min(std::max(target, last - max_step), last + max_step);
}

// Index of the path point closest to pos, searching forward from 'from' (the robot does not go back).
std::size_t find_closest_point(const std::vector<tf2::Vector3> &path, std::size_t from, const tf2::Vector3 &pos)
{
    std::size_t best = from;
    double best_dist = (path[from] - pos).length2();
    for (std::size_t i = from + 1; i < path.size(); ++i)
    {
        const double d = (path[i] - pos).length2();
        if (d < best_dist)
        {
            best = i;
            best_dist = d;
        }
    }
    return best;
}

// Point at arc length 'dist' along the path from index i (clamped to the path end).
tf2::Vector3 move_along_path(const std::vector<tf2::Vector3> &path, std::size_t i, double dist)
{
    for (; i + 1 < path.size(); ++i)
    {
        const double len = (path[i + 1] - path[i]).length();
        if (len >= dist)
        {
            return path[i] + (path[i + 1] - path[i]) * (len > 1e-9 ? dist / len : 0.0);
        }
        dist -= len;
    }
    return path.back();
}

double length_from(const std::vector<tf2::Vector3> &path, std::size_t i)
{
    double sum = 0;
    for (; i + 1 < path.size(); ++i)
    {
        sum += (path[i + 1] - path[i]).length();
    }
    return sum;
}

} // namespace

PurePursuitLocalPlanner::PurePursuitLocalPlanner()
{
}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner()
{
}

bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);

    cmd_vel = geometry_msgs::Twist();

    if (!m_odometry)
    {
        ROS_WARN_NAMED("PurePursuitLocalPlanner", "Waiting for odometry ...");
        return false;
    }
    if (m_global_plan.empty())
    {
        ROS_WARN_NAMED("PurePursuitLocalPlanner", "Global plan is empty!");
        return false;
    }

    // compute delta time (ROS time, so the limits hold in simulation time as well)
    const ros::Time time_now = ros::Time::now();
    const double dt = std::min(std::max((time_now - m_last_time).toSec(), 0.0), 0.1);
    m_last_time = time_now;

    // get latest global to local transform (map to odom)
    tf2::Stamped<tf2::Transform> global_to_local;
    try
    {
        auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), ros::Duration(0));
        tf2::fromMsg(msg, global_to_local);
    }
    catch (...)
    {
        ROS_WARN_NAMED("PurePursuitLocalPlanner", "lookupTransform(%s, %s) failed", m_local_frame.c_str(),
                       m_global_frame.c_str());
        return false;
    }

    // transform plan to local frame (odom)
    std::vector<tf2::Vector3> local_plan;
    local_plan.reserve(m_global_plan.size());
    for (const auto &pose : m_global_plan)
    {
        tf2::Vector3 p = global_to_local * tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0);
        p.setZ(0);
        local_plan.push_back(p);
    }

    // get latest local pose
    tf2::Transform local_pose;
    tf2::fromMsg(m_odometry->pose.pose, local_pose);
    const double yaw = tf2::getYaw(local_pose.getRotation());
    const tf2::Vector3 pos(local_pose.getOrigin().x(), local_pose.getOrigin().y(), 0);
    const tf2::Transform robot(createQuaternionFromYaw(yaw), pos);
    const bool is_stopped = base_local_planner::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                        m_limits.trans_stopped_vel);

    // closest point, remaining path and lookahead target
    m_closest = find_closest_point(local_plan, std::min(m_closest, local_plan.size() - 1), pos);
    const double remaining = length_from(local_plan, m_closest) + (local_plan[m_closest] - pos).length();
    const double lookahead = m_lookahead_dist + std::max(m_last_cmd_vel.linear.x, 0.0) * m_lookahead_time;
    const tf2::Vector3 target_pos = move_along_path(local_plan, m_closest, lookahead);
    const tf2::Vector3 local_target = robot.inverse() * target_pos;
    const double yaw_error = ::atan2(local_target.y(), local_target.x());

    if (m_state == STATE_IDLE)
    {
        m_state = (std::fabs(yaw_error) > m_start_yaw_error) ? STATE_ROTATING : STATE_TRANSLATING;
    }

    double control_vel_x = 0;
    double control_yawrate = 0;

    if (remaining <= m_limits.xy_goal_tolerance && m_state != STATE_ROTATING)
    {
        if (is_stopped && m_state != STATE_GOAL)
        {
            m_state = STATE_GOAL;
            ROS_INFO_NAMED("PurePursuitLocalPlanner", "end of path reached");
        }
    }
    else if (m_state == STATE_ROTATING)
    {
        if (std::fabs(yaw_error) > 0.25 * m_start_yaw_error)
        {
            const double profile = std::sqrt(2 * 0.8 * m_limits.acc_lim_theta * std::fabs(yaw_error));
            control_yawrate = std::copysign(
                std::max(std::min(profile, m_limits.max_vel_theta), m_limits.min_vel_theta), yaw_error);
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_limits.acc_lim_theta * dt);
        }
        else if (is_stopped)
        {
            m_state = STATE_TRANSLATING;
        }
    }
    else if (std::fabs(yaw_error) > 2 * m_start_yaw_error)
    {
        // the path turns too sharply to follow on the move: stop and rotate
        m_state = STATE_ROTATING;
    }
    else
    {
        const double dist_sq = local_target.x() * local_target.x() + local_target.y() * local_target.y();
        const double curvature = dist_sq > 1e-6 ? 2 * local_target.y() / dist_sq : 0;

        // speed: braking profile toward the path end and the curvature limit (as max_curve_vel in neo)
        double vel_x = std::min(std::sqrt(2 * m_dec_lim_x * remaining), m_limits.max_vel_x);
        if (std::fabs(curvature) > 1e-3)
        {
            vel_x = std::min(vel_x, m_max_curve_vel / std::fabs(curvature));
        }
        vel_x = std::max(vel_x, m_limits.min_vel_trans);

        // footprint check along the path ahead, up to the braking distance plus min_stop_dist
        const double check_dist = vel_x * vel_x / (2 * m_dec_lim_x) + m_min_stop_dist;
        base_local_planner::CostmapModel world_model(*m_cost_map->getCostmap());
        const std::vector<geometry_msgs::Point> footprint = m_cost_map->getRobotFootprint();
        bool blocked = false;
        double travelled = 0;
        for (std::size_t i = m_closest; i + 1 < local_plan.size() && travelled <= check_dist && !blocked; ++i)
        {
            const tf2::Vector3 d = local_plan[i + 1] - local_plan[i];
            travelled += d.length();
            if (travelled > 0.05 || i == m_closest)
            {
                blocked = world_model.footprintCost(local_plan[i].x(), local_plan[i].y(), ::atan2(d.y(), d.x()),
                                                    footprint) == -1;
            }
        }
        if (blocked)
        {
            if (m_state != STATE_BLOCKED)
            {
                ROS_WARN_NAMED("PurePursuitLocalPlanner", "obstacle on the path ahead, stopping");
            }
            m_state = STATE_BLOCKED;
            vel_x = 0;
        }
        else
        {
            m_state = STATE_TRANSLATING;
        }

        control_vel_x = std::min(vel_x, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt);

        // yaw rate = v * curvature, low passed and slew limited (smooth re-steering of the drive units)
        control_yawrate = control_vel_x * curvature;
        control_yawrate = control_yawrate * m_low_pass_gain + m_last_cmd_vel.angular.z * (1 - m_low_pass_gain);
        control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_trans_acc_lim_theta * dt);
    }

    cmd_vel.linear.x = std::min(std::max(control_vel_x, 0.0), m_limits.max_vel_x);
    cmd_vel.angular.z = std::min(std::max(control_yawrate, -m_limits.max_vel_theta), m_limits.max_vel_theta);
    m_last_cmd_vel = cmd_vel;

    // publish the path ahead (local frame) for visualization
    nav_msgs::Path local_path;
    local_path.header.frame_id = m_local_frame;
    local_path.header.stamp = m_odometry->header.stamp;
    for (std::size_t i = m_closest; i < local_plan.size(); ++i)
    {
        geometry_msgs::PoseStamped p;
        p.header = local_path.header;
        p.pose.position.x = local_plan[i].x();
        p.pose.position.y = local_plan[i].y();
        p.pose.orientation.w = 1;
        local_path.poses.push_back(p);
    }
    m_local_plan_pub.publish(local_path);
    return true;
}

bool PurePursuitLocalPlanner::isGoalReached()
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);

    if (!m_odometry)
    {
        return false;
    }
    if (m_global_plan.empty())
    {
        return true;
    }
    const bool is_reached = m_state == STATE_GOAL &&
        base_local_planner::stopped(*m_odometry, m_limits.theta_stopped_vel, m_limits.trans_stopped_vel);
    if (!m_is_goal_reached || !is_reached)
    {
        m_first_goal_reached_time = ros::Time::now();
    }
    m_is_goal_reached = is_reached;
    return is_reached && (ros::Time::now() - m_first_goal_reached_time).toSec() >= m_goal_tune_time;
}

bool PurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (plan.empty())
    {
        return false;
    }
    m_global_plan = plan;
    m_global_frame = plan.front().header.frame_id;
    m_closest = 0;
    m_last_time = ros::Time::now();
    m_first_goal_reached_time = ros::Time();
    m_is_goal_reached = false;
    // A replanned path while driving continues from the current motion: keep translating unless the
    // new path starts far off the heading (decided in the next control step).
    m_last_cmd_vel = geometry_msgs::Twist();
    {
        boost::mutex::scoped_lock lock(m_odometry_mutex);
        if (m_odometry)
        {
            m_last_cmd_vel.linear.x = std::max(m_odometry->twist.twist.linear.x, 0.0);
            m_last_cmd_vel.angular.z = m_odometry->twist.twist.angular.z;
        }
    }
    m_state = (m_last_cmd_vel.linear.x > m_limits.trans_stopped_vel) ? STATE_TRANSLATING : STATE_IDLE;
    return true;
}

void PurePursuitLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~/" + name);

    m_limits.acc_lim_x = private_nh.param<double>("acc_lim_x", 0.3);
    m_limits.acc_lim_theta = private_nh.param<double>("acc_lim_theta", 0.6);
    m_limits.max_vel_x = private_nh.param<double>("max_vel_x", 0.5);
    m_limits.min_vel_trans = private_nh.param<double>("min_trans_vel", 0.05);
    m_limits.max_vel_theta = private_nh.param<double>("max_rot_vel", 0.6);
    m_limits.min_vel_theta = private_nh.param<double>("min_rot_vel", 0.05);
    m_limits.theta_stopped_vel = private_nh.param<double>("rot_stopped_vel", 0.02);
    m_limits.trans_stopped_vel = private_nh.param<double>("trans_stopped_vel", 0.02);
    m_limits.xy_goal_tolerance = private_nh.param<double>("xy_goal_tolerance", 0.1);

    m_dec_lim_x = private_nh.param<double>("dec_lim_x", 0.3);
    m_lookahead_dist = private_nh.param<double>("lookahead_dist", 0.6);
    m_lookahead_time = private_nh.param<double>("lookahead_time", 0.5);
    m_start_yaw_error = private_nh.param<double>("start_yaw_error", 0.5);
    m_max_curve_vel = private_nh.param<double>("max_curve_vel", 0.3);
    m_trans_acc_lim_theta = private_nh.param<double>("trans_acc_lim_theta", 0.4);
    m_low_pass_gain = private_nh.param<double>("low_pass_gain", 0.5);
    m_min_stop_dist = private_nh.param<double>("min_stop_dist", 0.3);
    m_goal_tune_time = private_nh.param<double>("goal_tune_time", 0.3);
    m_local_frame = private_nh.param<std::string>("local_frame", "odom");
    const std::string odom_topic = private_nh.param<std::string>("odom_topic", "/odom");

    m_tf = tf;
    m_cost_map = costmap_ros;
    m_base_frame = costmap_ros->getBaseFrameID();

    m_odom_sub = nh.subscribe(odom_topic, 1, &PurePursuitLocalPlanner::odomCallback, this);
    m_local_plan_pub = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

    ROS_INFO_NAMED("PurePursuitLocalPlanner", "base_frame=%s, local_frame=%s, max_vel_x=%.2f, max_curve_vel=%.2f",
                   m_base_frame.c_str(), m_local_frame.c_str(), m_limits.max_vel_x, m_max_curve_vel);
}

void PurePursuitLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);
    m_odometry = msg;
}

} // namespace pure_pursuit_local_planner
