#include "pure_pursuit_local_planner/PurePursuitLocalPlanner.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>

#include <local_planner_checks/footprint_checker.h>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitLocalPlanner, nav2_core::Controller)

namespace pure_pursuit_local_planner
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

// Point at arc length 'dist' along the path from index i. Past the path end the path is extended
// straight in the direction of its last step, so the target never closes in on the robot (that would
// make the curvature, and the steering, swing near the end).
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
    for (std::size_t k = path.size() - 1; k > 0; --k)
    {
        const tf2::Vector3 step = path[k] - path[k - 1];
        if (step.length() > 1e-6)
        {
            return path.back() + step.normalized() * dist;
        }
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

bool PurePursuitLocalPlanner::odometryFresh(const rclcpp::Time &now) const
{
    return std::fabs((now - m_odometry->header.stamp).seconds()) <= m_odom_timeout;
}

PurePursuitLocalPlanner::~PurePursuitLocalPlanner()
{
}

bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel)
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);

    cmd_vel = geometry_msgs::msg::Twist();

    if (!m_odometry)
    {
        RCLCPP_WARN(m_node->get_logger(), "Waiting for odometry ...");
        return false;
    }
    if (m_global_plan.empty())
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
    const bool is_stopped = local_planner_checks::stopped(*m_odometry, m_limits.theta_stopped_vel,
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
    double planned_speed = 0;
    double target_yawrate = 0; // yaw rate of the pursuit arc, before the low pass and slew limit
    double rotation_left = 0;

    if (remaining <= m_limits.xy_goal_tolerance && m_state != STATE_ROTATING)
    {
        if (is_stopped && m_state != STATE_GOAL)
        {
            m_state = STATE_GOAL;
            RCLCPP_INFO(m_node->get_logger(), "end of path reached");
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
            rotation_left = yaw_error;
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
        const double pursuit_curvature = dist_sq > 1e-6 ? 2 * local_target.y() / dist_sq : 0;

        // The drive units' steering angles follow the curvature, not the yaw rate: low pass the curvature and
        // limit its rate of change (max_curvature_rate), so the steering moves smoothly also while the speed
        // changes and when a new path is handed over. A steering step makes the base stop, re-steer and
        // start again.
        double curvature = pursuit_curvature;
        if (m_have_curvature)
        {
            curvature = curvature * m_low_pass_gain + m_curvature * (1 - m_low_pass_gain);
            curvature = slew(m_curvature, curvature, m_max_curvature_rate * dt);
        }
        m_curvature = curvature;
        m_have_curvature = true;

        // speed: braking profile toward the path end and the curvature limit (as max_curve_vel in neo)
        double vel_x = std::min(std::sqrt(2 * m_dec_lim_x * remaining), m_limits.max_vel_x);
        const double sharpest = std::max(std::fabs(pursuit_curvature), std::fabs(curvature));
        if (sharpest > 1e-3)
        {
            vel_x = std::min(vel_x, m_max_curve_vel / sharpest);
        }
        vel_x = std::max(vel_x, m_limits.min_vel_trans);
        planned_speed = vel_x;

        control_vel_x = std::min(vel_x, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt);

        // yaw rate = v * curvature, slew limited; the pursuit arc itself for the collision check
        control_yawrate = control_vel_x * curvature;
        target_yawrate = control_vel_x * pursuit_curvature;
        control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_trans_acc_lim_theta * dt);
        m_state = STATE_TRANSLATING;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::min(std::max(control_vel_x, 0.0), m_limits.max_vel_x);
    cmd.angular.z = std::min(std::max(control_yawrate, -m_limits.max_vel_theta), m_limits.max_vel_theta);
    if (!commandIsFree(cmd, planned_speed, target_yawrate, rotation_left, local_plan, robot, yaw))
    {
        if (m_state == STATE_TRANSLATING)
        {
            m_state = STATE_BLOCKED; // re-evaluated every cycle; continues once the way is clear
        }
        cmd = geometry_msgs::msg::Twist();
    }
    cmd_vel = cmd;
    m_last_cmd_vel = cmd;
    if (m_state != STATE_TRANSLATING || cmd.linear.x < 1e-3)
    {
        m_have_curvature = false; // stopping: the base re-aligns its steering before it drives again
    }

    // publish the path ahead (local frame) for visualization
    nav_msgs::msg::Path local_path;
    local_path.header.frame_id = m_local_frame;
    local_path.header.stamp = m_odometry->header.stamp;
    for (std::size_t i = m_closest; i < local_plan.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header = local_path.header;
        p.pose.position.x = local_plan[i].x();
        p.pose.position.y = local_plan[i].y();
        p.pose.orientation.w = 1;
        local_path.poses.push_back(p);
    }
    m_local_plan_pub->publish(local_path);
    return true;
}

bool PurePursuitLocalPlanner::commandIsFree(const geometry_msgs::msg::Twist &cmd, double planned_speed,
                                            double target_yawrate, double rotation_left,
                                            const std::vector<tf2::Vector3> &local_plan, const tf2::Transform &robot,
                                            double yaw)
{
    const double v_cmd = std::fabs(cmd.linear.x);
    if (v_cmd < 1e-3 && std::fabs(cmd.angular.z) < 1e-3)
    {
        return true; // stopping
    }
    nav2_costmap_2d::Costmap2D *costmap = m_cost_map->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    const checks::FootprintChecker checker(*costmap, m_cost_map->getRobotFootprint());
    const checks::Pose2D start{robot.getOrigin().x(), robot.getOrigin().y(), yaw};

    checks::CheckResult result;
    const char *what = "";
    if (v_cmd >= 1e-3)
    {
        // over the distance the base needs to stop (measured, commanded or planned speed) plus min_stop_dist:
        // 1. the motion this controller drives from where the robot is (it may be off the path, or cut a
        //    corner): the commanded speed, the yaw rate ramping from the command toward the pursuit arc at
        //    trans_acc_lim_theta, as the next control steps will command it
        const double dist = checks::stoppingDistance(m_odometry->twist.twist.linear.x, std::max(v_cmd, planned_speed),
                                                     m_dec_lim_x, m_collision_latency, m_min_stop_dist);
        std::vector<checks::Pose2D> motion{start};
        double wz = cmd.angular.z;
        const double dt = 0.05;
        for (double s = 0; s < dist && motion.size() < 2000; s += v_cmd * dt)
        {
            wz = slew(wz, target_yawrate, m_trans_acc_lim_theta * dt);
            motion.push_back(checks::integrateTwist(motion.back(), cmd.linear.x, 0, wz, dt));
        }
        result = checker.checkPath(motion, dist);
        what = "driving";
        // 2. the path ahead from its closest point
        //    The heading at a path point is taken toward the point path_heading_dist further on, not toward
        //    the next sample: the robot does not follow the planner's centimetre-scale wiggles, and on a body
        //    reaching far from the turning centre (SD: 1.07 m) a few degrees of wiggle would swing the
        //    checked footprint into obstacles the robot passes well clear of.
        if (result.free())
        {
            std::vector<checks::Pose2D> path;
            std::size_t ahead = m_closest;
            for (std::size_t i = m_closest; i < local_plan.size(); ++i)
            {
                const tf2::Vector3 &a = local_plan[i];
                ahead = std::max(ahead, i);
                while (ahead + 1 < local_plan.size() && (local_plan[ahead] - a).length() < m_path_heading_dist)
                {
                    ++ahead;
                }
                const tf2::Vector3 &b = local_plan[ahead];
                const double heading = (b - a).length() > 1e-6 ? ::atan2(b.y() - a.y(), b.x() - a.x())
                                                               : (path.empty() ? yaw : path.back().yaw);
                path.push_back({a.x(), a.y(), heading});
            }
            result = checker.checkPath(path, dist);
            what = "path ahead";
        }
    }
    else
    {
        result = checker.checkRotation(start, rotation_left + std::copysign(m_rotation_margin, cmd.angular.z));
        what = "turning in place";
    }

    if (!result.free())
    {
        if (!m_blocked)
        {
            RCLCPP_WARN(m_node->get_logger(), "%s: %s after %.2f m / %.0f deg, stopping", what,
                           checks::toString(result.collision), result.distance,
                           angles::to_degrees(std::fabs(result.angle)));
        }
        setBlocked(true);
        return false;
    }
    if (m_blocked)
    {
        RCLCPP_INFO(m_node->get_logger(), "the way is clear again");
    }
    setBlocked(false);
    return true;
}

void PurePursuitLocalPlanner::setBlocked(bool blocked)
{
    if (blocked != m_blocked || !m_blocked_published)
    {
        std_msgs::msg::Bool msg;
        msg.data = blocked;
        m_blocked_pub->publish(msg);
        m_blocked_published = true;
    }
    m_blocked = blocked;
}

bool PurePursuitLocalPlanner::isGoalReached()
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);

    if (!m_odometry || !odometryFresh(m_node->now()))
    {
        return false;
    }
    if (m_global_plan.empty())
    {
        return true;
    }
    bool is_reached = false;
    if (m_state == STATE_GOAL)
    {
        // the end of the path by actual position (any heading: the route controller turns afterwards)
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
        const geometry_msgs::msg::Point &e = m_global_plan.back().pose.position;
        const tf2::Vector3 end = global_to_local * tf2::Vector3(e.x, e.y, 0);
        const checks::Pose2D robot{m_odometry->pose.pose.position.x, m_odometry->pose.pose.position.y, 0};
        is_reached = checks::goalReached(robot, {end.x(), end.y(), 0}, local_planner_checks::stopped(
            *m_odometry, m_limits.theta_stopped_vel, m_limits.trans_stopped_vel), m_limits.xy_goal_tolerance, M_PI);
    }
    if (!m_is_goal_reached || !is_reached)
    {
        m_first_goal_reached_time = m_node->now();
    }
    m_is_goal_reached = is_reached;
    return is_reached && (m_node->now() - m_first_goal_reached_time).seconds() >= m_goal_tune_time;
}

bool PurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &plan)
{
    if (plan.empty())
    {
        return false;
    }
    m_global_plan = plan;
    m_global_frame = plan.front().header.frame_id;
    m_closest = 0;
    m_last_time = m_node->now();
    m_first_goal_reached_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    m_is_goal_reached = false;
    setBlocked(false);
    // A replanned path while driving continues from the current motion: keep translating unless the
    // new path starts far off the heading (decided in the next control step).
    m_last_cmd_vel = geometry_msgs::msg::Twist();
    m_have_curvature = false;
    {
        std::lock_guard<std::mutex> lock(m_odometry_mutex);
        if (m_odometry && odometryFresh(m_node->now()))
        {
            m_last_cmd_vel.linear.x = std::max(m_odometry->twist.twist.linear.x, 0.0);
            m_last_cmd_vel.angular.z = m_odometry->twist.twist.angular.z;
            if (m_last_cmd_vel.linear.x > m_limits.trans_stopped_vel)
            {
                m_curvature = m_last_cmd_vel.angular.z / m_last_cmd_vel.linear.x; // the steering it has now
                m_have_curvature = true;
            }
        }
    }
    m_state = (m_last_cmd_vel.linear.x > m_limits.trans_stopped_vel) ? STATE_TRANSLATING : STATE_IDLE;
    return true;
}

void PurePursuitLocalPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    m_node = parent.lock();
    if (!m_node) throw std::runtime_error("Controller parent expired");
    m_name = std::move(name);

    m_limits.acc_lim_x = param<double>("acc_lim_x", 0.3);
    m_limits.acc_lim_theta = param<double>("acc_lim_theta", 0.6);
    m_limits.max_vel_x = param<double>("max_vel_x", 0.5);
    m_limits.min_vel_trans = param<double>("min_trans_vel", 0.05);
    m_limits.max_vel_theta = param<double>("max_rot_vel", 0.6);
    m_limits.min_vel_theta = param<double>("min_rot_vel", 0.05);
    m_limits.theta_stopped_vel = param<double>("rot_stopped_vel", 0.02);
    m_limits.trans_stopped_vel = param<double>("trans_stopped_vel", 0.02);
    m_limits.xy_goal_tolerance = param<double>("xy_goal_tolerance", 0.1);

    m_dec_lim_x = param<double>("dec_lim_x", 0.3);
    m_lookahead_dist = param<double>("lookahead_dist", 0.6);
    m_lookahead_time = param<double>("lookahead_time", 0.5);
    m_start_yaw_error = param<double>("start_yaw_error", 0.5);
    m_max_curve_vel = param<double>("max_curve_vel", 0.3);
    m_trans_acc_lim_theta = param<double>("trans_acc_lim_theta", 0.4);
    m_low_pass_gain = param<double>("low_pass_gain", 0.5);
    m_max_curvature_rate = param<double>("max_curvature_rate", 1.0);
    m_min_stop_dist = param<double>("min_stop_dist", 0.3);
    m_path_heading_dist = param<double>("path_heading_dist", 0.3);
    m_odom_timeout = param<double>("odom_timeout", 0.5);
    m_collision_latency = param<double>("collision_latency", 0.3);
    m_rotation_margin = param<double>("rotation_margin", 0.1);
    m_goal_tune_time = param<double>("goal_tune_time", 0.3);
    m_local_frame = param<std::string>("local_frame", "odom");
    const std::string odom_topic = param<std::string>("odom_topic", "/odom");

    m_tf = tf;
    m_cost_map = costmap_ros;
    m_base_frame = costmap_ros->getBaseFrameID();

    m_odom_sub = m_node->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(), std::bind(&PurePursuitLocalPlanner::odomCallback, this, std::placeholders::_1));
    m_local_plan_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_name + "/local_plan", 1);
    m_default_max_vel_x = m_limits.max_vel_x;
    m_last_time = m_node->now();
    m_blocked_pub = m_node->create_publisher<std_msgs::msg::Bool>("~/" + m_name + "/blocked", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(m_node->get_logger(), "base_frame=%s, local_frame=%s, max_vel_x=%.2f, max_curve_vel=%.2f",
                   m_base_frame.c_str(), m_local_frame.c_str(), m_limits.max_vel_x, m_max_curve_vel);
}

void PurePursuitLocalPlanner::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
{
    std::lock_guard<std::mutex> lock(m_odometry_mutex);
    m_odometry = msg;
}


void PurePursuitLocalPlanner::cleanup() { m_odom_sub.reset(); m_local_plan_pub.reset(); m_blocked_pub.reset(); m_odometry.reset(); m_global_plan.clear(); }
void PurePursuitLocalPlanner::activate() { m_local_plan_pub->on_activate(); m_blocked_pub->on_activate(); m_blocked_published = false; setBlocked(false); }
void PurePursuitLocalPlanner::deactivate() { setBlocked(false); m_blocked_pub->on_deactivate(); m_local_plan_pub->on_deactivate(); }
void PurePursuitLocalPlanner::setPlan(const nav_msgs::msg::Path &path) {
    if (!setPlan(path.poses)) throw nav2_core::PlannerException("Empty controller path");
}
geometry_msgs::msg::TwistStamped PurePursuitLocalPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = m_node->now();
    cmd.header.frame_id = m_base_frame;
    if (!computeVelocityCommands(cmd.twist)) throw nav2_core::PlannerException("Controller input unavailable or stale");
    return cmd;
}
void PurePursuitLocalPlanner::setSpeedLimit(const double &limit, const bool &percentage) {
    m_limits.max_vel_x = limit <= 0 ? m_default_max_vel_x :
        std::min(m_default_max_vel_x, percentage ? m_default_max_vel_x * limit / 100.0 : limit);
}

} // namespace pure_pursuit_local_planner
