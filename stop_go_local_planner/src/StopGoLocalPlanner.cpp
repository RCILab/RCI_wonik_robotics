#include "stop_go_local_planner/StopGoLocalPlanner.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

#include <local_planner_checks/footprint_checker.h>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(stop_go_local_planner::StopGoLocalPlanner, nav2_core::Controller)

namespace stop_go_local_planner
{

namespace checks = local_planner_checks;

namespace
{

tf2::Quaternion createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

tf2::Vector3 toVector(const geometry_msgs::msg::PoseStamped &pose)
{
    return tf2::Vector3(pose.pose.position.x, pose.pose.position.y, 0);
}

// Segment end points of a polyline: a new segment starts where the direction differs from the
// current segment's direction by more than corner_angle. Steps shorter than 2 cm are ignored when
// estimating directions.
std::vector<tf2::Vector3> extract_vertices(const std::vector<geometry_msgs::msg::PoseStamped> &plan, double corner_angle)
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

bool StopGoLocalPlanner::odometryFresh(const rclcpp::Time &now) const
{
    const double age = (now - m_odometry->header.stamp).seconds();
    return std::fabs(age) <= m_odom_timeout;
}

bool StopGoLocalPlanner::commandIsFree(const geometry_msgs::msg::Twist &cmd, double planned_speed, double rotation_left,
                                       const tf2::Vector3 &pos, double yaw)
{
    const double v_cmd = std::hypot(cmd.linear.x, cmd.linear.y);
    if (v_cmd < 1e-3 && std::fabs(cmd.angular.z) < 1e-3)
    {
        return true; // stopping
    }
    nav2_costmap_2d::Costmap2D *costmap = m_cost_map->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    const checks::FootprintChecker checker(*costmap, m_cost_map->getRobotFootprint());
    const checks::Pose2D robot{pos.x(), pos.y(), yaw};

    checks::CheckResult result;
    if (v_cmd >= 1e-3)
    {
        // the motion this command starts, over the distance the base needs to stop from the measured,
        // commanded or planned speed (whichever is highest)
        const double v_meas = std::hypot(m_odometry->twist.twist.linear.x, m_odometry->twist.twist.linear.y);
        const double dist = checks::stoppingDistance(v_meas, std::max(v_cmd, planned_speed), m_dec_lim_x,
                                                     m_collision_latency, m_collision_margin);
        result = checker.checkTwist(robot, cmd.linear.x, cmd.linear.y, cmd.angular.z, dist / v_cmd);
    }
    else
    {
        // turning in place: the whole rest of the turn (the corners sweep a circle)
        result = checker.checkRotation(robot, rotation_left + std::copysign(m_rotation_margin, cmd.angular.z));
    }

    if (!result.free())
    {
        if (!m_blocked)
        {
            RCLCPP_WARN(m_node->get_logger(), "%s: %s after %.2f m / %.0f deg, stopping until it is clear",
                           v_cmd >= 1e-3 ? "driving" : "turning in place", checks::toString(result.collision),
                           result.distance, angles::to_degrees(std::fabs(result.angle)));
        }
        m_blocked = true;
        return false;
    }
    if (m_blocked)
    {
        RCLCPP_INFO(m_node->get_logger(), "the way is clear again");
    }
    m_blocked = false;
    return true;
}

bool StopGoLocalPlanner::computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel)
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);

    cmd_vel = geometry_msgs::msg::Twist();

    if (!m_odometry)
    {
        RCLCPP_WARN(m_node->get_logger(), "Waiting for odometry ...");
        return false;
    }
    if (m_vertices.size() < 2)
    {
        RCLCPP_WARN(m_node->get_logger(), "Global plan is empty!");
        return false;
    }

    // compute delta time (ROS time, so the limits hold in simulation time as well)
    const rclcpp::Time time_now = m_node->now();
    if (!odometryFresh(time_now))
    {
        RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 1000, "odometry is %.2f s old: not driving on it",
                                (time_now - m_odometry->header.stamp).seconds());
        m_last_cmd_vel = geometry_msgs::msg::Twist();
        return false;
    }
    const double dt = std::min(std::max((time_now - m_last_time).seconds(), 0.0), 0.1);
    m_last_time = time_now;

    // get latest global to local transform (map to odom)
    tf2::Stamped<tf2::Transform> global_to_local;
    try
    {
        auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0));
        tf2::fromMsg(msg, global_to_local);
    }
    catch (...)
    {
        RCLCPP_WARN(m_node->get_logger(), "lookupTransform(%s, %s) failed", m_local_frame.c_str(),
                       m_global_frame.c_str());
        return false;
    }

    // get latest local pose
    tf2::Transform local_pose;
    tf2::fromMsg(m_odometry->pose.pose, local_pose);
    const double yaw = tf2::getYaw(local_pose.getRotation());
    const tf2::Vector3 pos(local_pose.getOrigin().x(), local_pose.getOrigin().y(), 0);
    const bool is_stopped = local_planner_checks::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                        m_limits.trans_stopped_vel);
    const double xy_tol = m_limits.xy_goal_tolerance;

    // A segment end missed by more than the tolerance: drive the rest sideways if it is small (and the
    // base can), otherwise approach the end again from here as a new segment.
    auto approach_again = [&](double dist) {
        tf2::Vector3 here = global_to_local.inverse() * pos;
        here.setZ(0);
        m_vertices[m_segment] = here;
        m_state = STATE_IDLE;
        RCLCPP_WARN(m_node->get_logger(), "%.2f m off the end of segment %zu: approaching it again", dist,
                       m_segment + 1);
    };
    auto missed = [&](double dist) {
        if (m_holonomic && dist <= m_adjust_max_dist)
        {
            m_state = STATE_ADJUSTING;
            RCLCPP_INFO(m_node->get_logger(), "%.3f m off the end of segment %zu: adjusting", dist, m_segment + 1);
        }
        else
        {
            approach_again(dist);
        }
    };

    // current segment in the local frame (recomputed after approach_again() changed its start)
    tf2::Vector3 seg_start, seg_end, seg_dir;
    double seg_len = 0, along = 0, remaining = 0, dist_end = 0, target_yaw = yaw;
    const bool is_last_segment = m_segment + 2 >= m_vertices.size();
    auto update_segment = [&]() {
        seg_start = global_to_local * m_vertices[m_segment];
        seg_end = global_to_local * m_vertices[m_segment + 1];
        seg_start.setZ(0);
        seg_end.setZ(0);
        seg_dir = seg_end - seg_start;
        seg_len = seg_dir.length();
        along = 0;
        target_yaw = yaw;
        if (seg_len > 1e-3)
        {
            seg_dir /= seg_len;
            target_yaw = ::atan2(seg_dir.y(), seg_dir.x());
            along = (pos - seg_start).dot(seg_dir);
        }
        remaining = seg_len - along;
        dist_end = (seg_end - pos).length();
    };
    update_segment();

    // moved off the goal after reaching it (e.g. a localization correction)
    if (m_state == STATE_GOAL && dist_end > xy_tol)
    {
        missed(dist_end);
        update_segment();
    }

    // a zero-length plan, or the last segment done: hold the goal orientation
    tf2::Transform goal_global;
    tf2::fromMsg(m_global_plan.back().pose, goal_global);
    const double goal_yaw = tf2::getYaw((global_to_local * goal_global).getRotation());
    const bool final_rotation = (seg_len <= 1e-3 || m_state == STATE_GOAL) && is_last_segment;
    if (final_rotation)
    {
        target_yaw = goal_yaw;
    }
    double yaw_error = angles::shortest_angular_distance(yaw, target_yaw);

    if (m_state == STATE_IDLE)
    {
        m_state = (std::fabs(yaw_error) > m_start_yaw_error) ? STATE_ROTATING : STATE_TRANSLATING;
        RCLCPP_INFO(m_node->get_logger(), "segment %zu/%zu: %s (yaw_error=%.1f deg, length=%.2f m)",
                       m_segment + 1, m_vertices.size() - 1,
                       m_state == STATE_ROTATING ? "rotating in place" : "translating",
                       angles::to_degrees(yaw_error), seg_len);
    }

    auto segment_done = [&]() {
        if (is_last_segment)
        {
            m_state = STATE_GOAL;
            RCLCPP_INFO(m_node->get_logger(), "route end reached (%.3f m off)", dist_end);
        }
        else
        {
            m_segment++;
            m_state = STATE_IDLE;
        }
    };

    double control_vel_x = 0;
    double control_vel_y = 0;
    double control_yawrate = 0;
    double planned_speed = 0; // speed the profile asks for, before the acceleration ramp
    double rotation_left = 0; // rest of an in-place turn

    if (m_state == STATE_ROTATING || (m_state == STATE_GOAL && final_rotation))
    {
        if (std::fabs(yaw_error) > m_limits.yaw_goal_tolerance)
        {
            // braking profile toward the target heading, at 80 % of the angular acceleration limit
            const double profile = std::sqrt(2 * 0.8 * m_limits.acc_lim_theta * std::fabs(yaw_error));
            control_yawrate = std::copysign(
                std::max(std::min(profile, m_limits.max_vel_theta), m_limits.min_vel_theta), yaw_error);
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_limits.acc_lim_theta * dt);
            rotation_left = yaw_error;
        }
        else if (is_stopped && m_state == STATE_ROTATING)
        {
            m_state = (seg_len > 1e-3) ? STATE_TRANSLATING : STATE_GOAL;
        }
    }
    else if (m_state == STATE_TRANSLATING)
    {
        if (remaining <= xy_tol)
        {
            // at (or past) the segment end along the line: stop, then check where the robot really is
            if (is_stopped)
            {
                if (dist_end <= xy_tol)
                {
                    segment_done();
                }
                else
                {
                    missed(dist_end);
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
            planned_speed = std::max(std::min(profile, m_limits.max_vel_x), m_limits.min_vel_trans);
            control_vel_x = std::min(planned_speed, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt);

            // yaw rate = v * curvature, low passed and slew limited: a step in the yaw rate re-steers
            // the drive units, which the base answers with a stop-steer-go cycle.
            control_yawrate = control_vel_x * curvature;
            control_yawrate =
                control_yawrate * m_low_pass_gain + m_last_cmd_vel.angular.z * (1 - m_low_pass_gain);
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_trans_acc_lim_theta * dt);
        }
    }
    else if (m_state == STATE_ADJUSTING)
    {
        if (dist_end > m_adjust_max_dist)
        {
            approach_again(dist_end);
        }
        else if (dist_end <= 0.5 * xy_tol)
        {
            if (is_stopped)
            {
                segment_done();
            }
        }
        else
        {
            // straight to the segment end at low speed, without turning
            const tf2::Vector3 local_target = tf2::Transform(createQuaternionFromYaw(yaw), pos).inverse() * seg_end;
            const double d = std::hypot(local_target.x(), local_target.y());
            planned_speed = std::min(std::max(dist_end, m_limits.min_vel_trans), m_adjust_vel);
            control_vel_x = local_target.x() / d * planned_speed;
            control_vel_y = local_target.y() / d * planned_speed;
        }
    }

    geometry_msgs::msg::Twist cmd;
    if (m_state == STATE_ADJUSTING)
    {
        cmd.linear.x = control_vel_x;
        cmd.linear.y = control_vel_y;
    }
    else
    {
        cmd.linear.x = std::min(std::max(control_vel_x, 0.0), m_limits.max_vel_x);
    }
    cmd.angular.z = std::min(std::max(control_yawrate, -m_limits.max_vel_theta), m_limits.max_vel_theta);

    if (!commandIsFree(cmd, planned_speed, rotation_left, pos, yaw))
    {
        cmd = geometry_msgs::msg::Twist(); // wait: no avoidance on the route
    }
    cmd_vel = cmd;
    m_last_cmd_vel = cmd;

    // publish the rest of the route (local frame) for visualization
    nav_msgs::msg::Path local_path;
    local_path.header.frame_id = m_local_frame;
    local_path.header.stamp = m_odometry->header.stamp;
    geometry_msgs::msg::PoseStamped p;
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
    m_local_plan_pub->publish(local_path);

    if (m_update_counter++ % 20 == 0)
    {
        RCLCPP_DEBUG(m_node->get_logger(),
                        "state=%d, segment=%zu, remaining=%.3f, dist_end=%.3f, yaw_error=%.3f, cmd=(%.3f, %.3f, %.3f)",
                        m_state, m_segment, remaining, dist_end, yaw_error, cmd.linear.x, cmd.linear.y,
                        cmd.angular.z);
    }
    return true;
}

bool StopGoLocalPlanner::isGoalReached()
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);

    if (!m_odometry || !odometryFresh(m_node->now()))
    {
        return false;
    }
    if (m_vertices.size() < 2)
    {
        return true;
    }

    bool is_reached = false;
    if (m_state == STATE_GOAL)
    {
        tf2::Stamped<tf2::Transform> global_to_local;
        try
        {
            auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0));
            tf2::fromMsg(msg, global_to_local);
        }
        catch (...)
        {
            return false;
        }
        tf2::Transform goal_global;
        tf2::fromMsg(m_global_plan.back().pose, goal_global);
        const tf2::Transform goal_local = global_to_local * goal_global;
        const checks::Pose2D goal{goal_local.getOrigin().x(), goal_local.getOrigin().y(),
                                  tf2::getYaw(goal_local.getRotation())};
        const checks::Pose2D robot{m_odometry->pose.pose.position.x, m_odometry->pose.pose.position.y,
                                   tf2::getYaw(m_odometry->pose.pose.orientation)};
        const bool is_stopped = local_planner_checks::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                            m_limits.trans_stopped_vel);
        is_reached = checks::goalReached(robot, goal, is_stopped, m_limits.xy_goal_tolerance,
                                         m_limits.yaw_goal_tolerance);
    }

    if (!m_is_goal_reached || !is_reached)
    {
        m_first_goal_reached_time = m_node->now();
    }
    m_is_goal_reached = is_reached;
    return is_reached && (m_node->now() - m_first_goal_reached_time).seconds() >= m_goal_tune_time;
}

bool StopGoLocalPlanner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &plan)
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
    m_last_time = m_node->now();
    m_first_goal_reached_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    m_is_goal_reached = false;
    m_blocked = false;
    // Start the slew limits from the current motion, so a plan handed over while driving (e.g. from
    // another controller) continues without a dip in speed.
    m_last_cmd_vel = geometry_msgs::msg::Twist();
    {
        std::lock_guard<std::mutex> lock(m_odometry_mutex);
        if (m_odometry && odometryFresh(m_node->now()))
        {
            m_last_cmd_vel.linear.x = std::max(m_odometry->twist.twist.linear.x, 0.0);
            m_last_cmd_vel.angular.z = m_odometry->twist.twist.angular.z;
        }
    }
    RCLCPP_INFO(m_node->get_logger(), "new plan: %zu poses, %zu segment(s)", plan.size(), m_vertices.size() - 1);
    return true;
}

void StopGoLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    m_node = parent.lock();
    if (!m_node) throw std::runtime_error("Controller parent expired");
    m_name = std::move(name);

    m_limits.acc_lim_x = param<double>("acc_lim_x", 0.3);
    m_limits.acc_lim_theta = param<double>("acc_lim_theta", 0.6);
    m_limits.max_vel_x = param<double>("max_vel_x", 1.0);
    m_limits.min_vel_trans = param<double>("min_trans_vel", 0.05);
    m_limits.max_vel_theta = param<double>("max_rot_vel", 0.6);
    m_limits.min_vel_theta = param<double>("min_rot_vel", 0.05);
    m_limits.theta_stopped_vel = param<double>("rot_stopped_vel", 0.02);
    m_limits.trans_stopped_vel = param<double>("trans_stopped_vel", 0.02);
    m_limits.xy_goal_tolerance = param<double>("xy_goal_tolerance", 0.03);
    m_limits.yaw_goal_tolerance = param<double>("yaw_goal_tolerance", 0.02);

    m_dec_lim_x = param<double>("dec_lim_x", 0.3);
    m_lookahead_dist = param<double>("lookahead_dist", 0.8);
    m_start_yaw_error = param<double>("start_yaw_error", 0.26);
    m_corner_angle = param<double>("corner_angle", 0.17);
    m_trans_acc_lim_theta = param<double>("trans_acc_lim_theta", 0.4);
    m_low_pass_gain = param<double>("low_pass_gain", 0.5);
    m_goal_tune_time = param<double>("goal_tune_time", 0.3);
    m_holonomic = param<bool>("holonomic", true);
    m_adjust_max_dist = param<double>("adjust_max_dist", 0.3);
    m_adjust_vel = param<double>("adjust_vel", 0.1);
    m_odom_timeout = param<double>("odom_timeout", 0.5);
    m_collision_latency = param<double>("collision_latency", 0.3);
    m_collision_margin = param<double>("collision_margin", 0.1);
    m_rotation_margin = param<double>("rotation_margin", 0.1);
    m_local_frame = param<std::string>("local_frame", "odom");
    const std::string odom_topic = param<std::string>("odom_topic", "/odom");

    m_tf = tf;
    m_cost_map = costmap_ros;
    m_base_frame = costmap_ros->getBaseFrameID();

    m_odom_sub = m_node->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), std::bind(&StopGoLocalPlanner::odomCallback, this, std::placeholders::_1));
    m_local_plan_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_name + "/local_plan", 1);
    m_default_max_vel_x = m_limits.max_vel_x;
    m_last_time = m_node->now();

    RCLCPP_INFO(m_node->get_logger(), "base_frame=%s, local_frame=%s, max_vel_x=%.2f, max_rot_vel=%.2f",
                   m_base_frame.c_str(), m_local_frame.c_str(), m_limits.max_vel_x, m_limits.max_vel_theta);
}

void StopGoLocalPlanner::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);
    m_odometry = msg;
}


void StopGoLocalPlanner::cleanup() { m_odom_sub.reset(); m_local_plan_pub.reset(); m_odometry.reset(); m_global_plan.clear(); }
void StopGoLocalPlanner::activate() { m_local_plan_pub->on_activate(); }
void StopGoLocalPlanner::deactivate() { m_local_plan_pub->on_deactivate(); }
void StopGoLocalPlanner::setPlan(const nav_msgs::msg::Path &path) {
    if (!setPlan(path.poses)) throw nav2_core::PlannerException("Empty controller path");
}
geometry_msgs::msg::TwistStamped StopGoLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = m_node->now();
    cmd.header.frame_id = m_base_frame;
    if (!computeVelocityCommands(cmd.twist)) throw nav2_core::PlannerException("Controller input unavailable or stale");
    return cmd;
}
void StopGoLocalPlanner::setSpeedLimit(const double &limit, const bool &percentage) {
    m_limits.max_vel_x = limit <= 0 ? m_default_max_vel_x :
        std::min(m_default_max_vel_x, percentage ? m_default_max_vel_x * limit / 100.0 : limit);
}

} // namespace stop_go_local_planner
