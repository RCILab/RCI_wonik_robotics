#include "stop_go_local_planner/StopGoLocalPlanner.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(stop_go_local_planner::StopGoLocalPlanner, nav_core::BaseLocalPlanner)

namespace stop_go_local_planner
{

namespace
{

tf2::Quaternion createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

tf2::Vector3 toVector(const geometry_msgs::PoseStamped &pose)
{
    return tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0);
}

// Segment end points of a polyline: a new segment starts where the direction differs from the
// current segment's direction by more than corner_angle. Steps shorter than 2 cm are ignored when
// estimating directions.
std::vector<tf2::Vector3> extract_vertices(const std::vector<geometry_msgs::PoseStamped> &plan, double corner_angle)
{
    std::vector<tf2::Vector3> vertices{toVector(plan.front())};
    bool have_dir = false;
    double segment_dir = 0;
    tf2::Vector3 last = vertices.front();

    for (std::size_t i = 1; i < plan.size(); ++i)
    {
        const tf2::Vector3 p = toVector(plan[i]);
        if ((p - last).length() < 0.02)
        {
            continue;
        }
        const double dir = ::atan2(p.y() - last.y(), p.x() - last.x());
        if (!have_dir)
        {
            segment_dir = dir;
            have_dir = true;
        }
        else if (std::fabs(angles::shortest_angular_distance(segment_dir, dir)) > corner_angle)
        {
            vertices.push_back(last);
            segment_dir = dir;
        }
        last = p;
    }
    vertices.push_back(toVector(plan.back()));
    return vertices;
}

double slew(double last, double target, double max_step)
{
    return std::min(std::max(target, last - max_step), last + max_step);
}

} // namespace

StopGoLocalPlanner::StopGoLocalPlanner()
{
}

StopGoLocalPlanner::~StopGoLocalPlanner()
{
}

bool StopGoLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);

    cmd_vel = geometry_msgs::Twist();

    if (!m_odometry)
    {
        ROS_WARN_NAMED("StopGoLocalPlanner", "Waiting for odometry ...");
        return false;
    }
    if (m_vertices.size() < 2)
    {
        ROS_WARN_NAMED("StopGoLocalPlanner", "Global plan is empty!");
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
        ROS_WARN_NAMED("StopGoLocalPlanner", "lookupTransform(%s, %s) failed", m_local_frame.c_str(),
                       m_global_frame.c_str());
        return false;
    }

    // get latest local pose
    tf2::Transform local_pose;
    tf2::fromMsg(m_odometry->pose.pose, local_pose);
    const double yaw = tf2::getYaw(local_pose.getRotation());
    const tf2::Vector3 pos(local_pose.getOrigin().x(), local_pose.getOrigin().y(), 0);
    const bool is_stopped = base_local_planner::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                        m_limits.trans_stopped_vel);

    // current segment in the local frame
    const tf2::Vector3 seg_start = global_to_local * m_vertices[m_segment];
    const tf2::Vector3 seg_end = global_to_local * m_vertices[m_segment + 1];
    tf2::Vector3 seg_dir = seg_end - seg_start;
    seg_dir.setZ(0);
    const double seg_len = seg_dir.length();
    const bool is_last_segment = m_segment + 2 >= m_vertices.size();

    double target_yaw = yaw;
    double along = 0;
    if (seg_len > 1e-3)
    {
        seg_dir /= seg_len;
        target_yaw = ::atan2(seg_dir.y(), seg_dir.x());
        along = (pos - seg_start).dot(seg_dir);
    }
    const double remaining = seg_len - along;

    // a zero-length plan, or the last segment done: hold the goal orientation
    tf2::Stamped<tf2::Transform> goal_global;
    tf2::fromMsg(m_global_plan.back(), goal_global);
    const double goal_yaw = tf2::getYaw((global_to_local * goal_global).getRotation());
    const bool final_rotation = (seg_len <= 1e-3 || m_state == STATE_GOAL) && is_last_segment;
    if (final_rotation)
    {
        target_yaw = goal_yaw;
    }
    const double yaw_error = angles::shortest_angular_distance(yaw, target_yaw);

    if (m_state == STATE_IDLE)
    {
        m_state = (std::fabs(yaw_error) > m_start_yaw_error) ? STATE_ROTATING : STATE_TRANSLATING;
        ROS_INFO_NAMED("StopGoLocalPlanner", "segment %zu/%zu: %s (yaw_error=%.1f deg, length=%.2f m)",
                       m_segment + 1, m_vertices.size() - 1,
                       m_state == STATE_ROTATING ? "rotating in place" : "translating",
                       angles::to_degrees(yaw_error), seg_len);
    }

    double control_vel_x = 0;
    double control_yawrate = 0;

    if (m_state == STATE_ROTATING || (m_state == STATE_GOAL && final_rotation))
    {
        if (std::fabs(yaw_error) > m_limits.yaw_goal_tolerance)
        {
            // braking profile toward the target heading, at 80 % of the angular acceleration limit
            const double profile = std::sqrt(2 * 0.8 * m_limits.acc_lim_theta * std::fabs(yaw_error));
            control_yawrate = std::copysign(
                std::max(std::min(profile, m_limits.max_vel_theta), m_limits.min_vel_theta), yaw_error);
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_limits.acc_lim_theta * dt);
        }
        else if (is_stopped && m_state == STATE_ROTATING)
        {
            m_state = (seg_len > 1e-3) ? STATE_TRANSLATING : STATE_GOAL;
        }
    }
    else if (m_state == STATE_TRANSLATING)
    {
        if (remaining <= m_limits.xy_goal_tolerance)
        {
            // segment end reached: stop, then continue with the next segment
            if (is_stopped)
            {
                if (is_last_segment)
                {
                    m_state = STATE_GOAL;
                    ROS_INFO_NAMED("StopGoLocalPlanner", "route end reached");
                }
                else
                {
                    m_segment++;
                    m_state = STATE_IDLE;
                }
            }
        }
        else if (std::fabs(yaw_error) > 2 * m_start_yaw_error)
        {
            // pushed far off the segment heading: stop and rotate again
            m_state = STATE_ROTATING;
        }
        else
        {
            // pure pursuit on the current segment
            const double lookahead = std::min(std::max(along, 0.0) + m_lookahead_dist, seg_len);
            const tf2::Vector3 target_pos = seg_start + seg_dir * lookahead;
            const tf2::Vector3 local_target =
                tf2::Transform(createQuaternionFromYaw(yaw), pos).inverse() * target_pos;
            const double dist_sq = local_target.x() * local_target.x() + local_target.y() * local_target.y();
            const double curvature = dist_sq > 1e-6 ? 2 * local_target.y() / dist_sq : 0;

            // speed: braking profile toward the segment end, never below min_vel_trans before it
            const double profile = std::sqrt(2 * m_dec_lim_x * std::max(remaining, 0.0));
            control_vel_x = std::max(std::min(profile, m_limits.max_vel_x), m_limits.min_vel_trans);
            control_vel_x = std::min(control_vel_x, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt);

            // yaw rate = v * curvature, low passed and slew limited: a step in the yaw rate re-steers
            // the drive units, which the base answers with a stop-steer-go cycle.
            control_yawrate = control_vel_x * curvature;
            control_yawrate =
                control_yawrate * m_low_pass_gain + m_last_cmd_vel.angular.z * (1 - m_low_pass_gain);
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_trans_acc_lim_theta * dt);
        }
    }

    cmd_vel.linear.x = std::min(std::max(control_vel_x, 0.0), m_limits.max_vel_x);
    cmd_vel.angular.z = std::min(std::max(control_yawrate, -m_limits.max_vel_theta), m_limits.max_vel_theta);
    m_last_cmd_vel = cmd_vel;

    // publish the rest of the route (local frame) for visualization
    nav_msgs::Path local_path;
    local_path.header.frame_id = m_local_frame;
    local_path.header.stamp = m_odometry->header.stamp;
    geometry_msgs::PoseStamped p;
    p.header = local_path.header;
    p.pose.orientation.w = 1;
    p.pose.position.x = pos.x();
    p.pose.position.y = pos.y();
    local_path.poses.push_back(p);
    for (std::size_t i = m_segment + 1; i < m_vertices.size(); ++i)
    {
        const tf2::Vector3 v = global_to_local * m_vertices[i];
        p.pose.position.x = v.x();
        p.pose.position.y = v.y();
        local_path.poses.push_back(p);
    }
    m_local_plan_pub.publish(local_path);

    if (m_update_counter++ % 20 == 0)
    {
        ROS_DEBUG_NAMED("StopGoLocalPlanner",
                        "state=%d, segment=%zu, remaining=%.3f, yaw_error=%.3f, cmd_vel=%.3f, cmd_yawrate=%.3f",
                        m_state, m_segment, remaining, yaw_error, cmd_vel.linear.x, cmd_vel.angular.z);
    }
    return true;
}

bool StopGoLocalPlanner::isGoalReached()
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);

    if (!m_odometry)
    {
        return false;
    }
    if (m_vertices.size() < 2)
    {
        return true;
    }

    const bool is_stopped = base_local_planner::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                        m_limits.trans_stopped_vel);
    bool is_reached = false;
    if (m_state == STATE_GOAL && is_stopped)
    {
        tf2::Stamped<tf2::Transform> global_to_local;
        try
        {
            auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), ros::Duration(0));
            tf2::fromMsg(msg, global_to_local);
        }
        catch (...)
        {
            return false;
        }
        tf2::Stamped<tf2::Transform> goal_global;
        tf2::fromMsg(m_global_plan.back(), goal_global);
        const auto goal_local = global_to_local * goal_global;
        const double yaw_error = std::fabs(angles::shortest_angular_distance(
            tf2::getYaw(m_odometry->pose.pose.orientation), tf2::getYaw(goal_local.getRotation())));
        is_reached = yaw_error < m_limits.yaw_goal_tolerance;
    }

    if (!m_is_goal_reached || !is_reached)
    {
        m_first_goal_reached_time = ros::Time::now();
    }
    m_is_goal_reached = is_reached;
    return is_reached && (ros::Time::now() - m_first_goal_reached_time).toSec() >= m_goal_tune_time;
}

bool StopGoLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (plan.empty())
    {
        return false;
    }
    m_global_plan = plan;
    m_global_frame = plan.front().header.frame_id;
    m_vertices = extract_vertices(plan, m_corner_angle);
    m_segment = 0;
    m_state = STATE_IDLE;
    m_last_time = ros::Time::now();
    m_first_goal_reached_time = ros::Time();
    m_is_goal_reached = false;
    // Start the slew limits from the current motion, so a plan handed over while driving (e.g. from
    // another controller) continues without a dip in speed.
    m_last_cmd_vel = geometry_msgs::Twist();
    {
        boost::mutex::scoped_lock lock(m_odometry_mutex);
        if (m_odometry)
        {
            m_last_cmd_vel.linear.x = std::max(m_odometry->twist.twist.linear.x, 0.0);
            m_last_cmd_vel.angular.z = m_odometry->twist.twist.angular.z;
        }
    }
    ROS_INFO_NAMED("StopGoLocalPlanner", "new plan: %zu poses, %zu segment(s)", plan.size(), m_vertices.size() - 1);
    return true;
}

void StopGoLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~/" + name);

    m_limits.acc_lim_x = private_nh.param<double>("acc_lim_x", 0.3);
    m_limits.acc_lim_theta = private_nh.param<double>("acc_lim_theta", 0.6);
    m_limits.max_vel_x = private_nh.param<double>("max_vel_x", 1.0);
    m_limits.min_vel_trans = private_nh.param<double>("min_trans_vel", 0.05);
    m_limits.max_vel_theta = private_nh.param<double>("max_rot_vel", 0.6);
    m_limits.min_vel_theta = private_nh.param<double>("min_rot_vel", 0.05);
    m_limits.theta_stopped_vel = private_nh.param<double>("rot_stopped_vel", 0.02);
    m_limits.trans_stopped_vel = private_nh.param<double>("trans_stopped_vel", 0.02);
    m_limits.xy_goal_tolerance = private_nh.param<double>("xy_goal_tolerance", 0.03);
    m_limits.yaw_goal_tolerance = private_nh.param<double>("yaw_goal_tolerance", 0.02);

    m_dec_lim_x = private_nh.param<double>("dec_lim_x", 0.3);
    m_lookahead_dist = private_nh.param<double>("lookahead_dist", 0.8);
    m_start_yaw_error = private_nh.param<double>("start_yaw_error", 0.26);
    m_corner_angle = private_nh.param<double>("corner_angle", 0.17);
    m_trans_acc_lim_theta = private_nh.param<double>("trans_acc_lim_theta", 0.4);
    m_low_pass_gain = private_nh.param<double>("low_pass_gain", 0.5);
    m_goal_tune_time = private_nh.param<double>("goal_tune_time", 0.3);
    m_local_frame = private_nh.param<std::string>("local_frame", "odom");
    const std::string odom_topic = private_nh.param<std::string>("odom_topic", "/odom");

    m_tf = tf;
    m_cost_map = costmap_ros;
    m_base_frame = costmap_ros->getBaseFrameID();

    m_odom_sub = nh.subscribe(odom_topic, 1, &StopGoLocalPlanner::odomCallback, this);
    m_local_plan_pub = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

    ROS_INFO_NAMED("StopGoLocalPlanner", "base_frame=%s, local_frame=%s, max_vel_x=%.2f, max_rot_vel=%.2f",
                   m_base_frame.c_str(), m_local_frame.c_str(), m_limits.max_vel_x, m_limits.max_vel_theta);
}

void StopGoLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);
    m_odometry = msg;
}

} // namespace stop_go_local_planner
