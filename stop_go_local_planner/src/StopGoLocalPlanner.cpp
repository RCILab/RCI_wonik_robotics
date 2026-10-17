#include "stop_go_local_planner/StopGoLocalPlanner.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <local_planner_checks/footprint_checker.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(stop_go_local_planner::StopGoLocalPlanner, nav_core::BaseLocalPlanner)

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

bool StopGoLocalPlanner::odometryFresh(const ros::Time &now) const
{
    const double age = (now - m_odometry->header.stamp).toSec();
    return std::fabs(age) <= m_odom_timeout;
}

bool StopGoLocalPlanner::commandIsFree(const geometry_msgs::Twist &cmd, double planned_speed, double rotation_left,
                                       const tf2::Vector3 &pos, double yaw)
{
    const double v_cmd = std::hypot(cmd.linear.x, cmd.linear.y);
    if (v_cmd < 1e-3 && std::fabs(cmd.angular.z) < 1e-3)
    {
        return true; // stopping
    }
    costmap_2d::Costmap2D *costmap = m_cost_map->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
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
            ROS_WARN_NAMED("StopGoLocalPlanner", "%s: %s after %.2f m / %.0f deg, stopping until it is clear",
                           v_cmd >= 1e-3 ? "driving" : "turning in place", checks::toString(result.collision),
                           result.distance, angles::to_degrees(std::fabs(result.angle)));
        }
        m_blocked = true;
        return false;
    }
    if (m_blocked)
    {
        ROS_INFO_NAMED("StopGoLocalPlanner", "the way is clear again");
    }
    m_blocked = false;
    return true;
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
    if (!odometryFresh(time_now))
    {
        ROS_WARN_THROTTLE_NAMED(1.0, "StopGoLocalPlanner", "odometry is %.2f s old: not driving on it",
                                (time_now - m_odometry->header.stamp).toSec());
        m_last_cmd_vel = geometry_msgs::Twist();
        return false;
    }
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
    const double xy_tol = m_limits.xy_goal_tolerance;

    // A segment end missed by more than the tolerance and by little: drive the rest out without turning
    // (ADJUSTING), sideways too if the base can; otherwise approach the end again from here as a new
    // segment.
    auto approach_again = [&](double dist) {
        tf2::Vector3 here = global_to_local.inverse() * pos;
        here.setZ(0);
        m_vertices[m_segment] = here;
        m_state = STATE_IDLE;
        ROS_WARN_NAMED("StopGoLocalPlanner", "%.2f m off the end of segment %zu: approaching it again", dist,
                       m_segment + 1);
    };
    const tf2::Transform robot_pose(createQuaternionFromYaw(yaw), pos);

    // current segment in the local frame (recomputed after approach_again() changed its start)
    tf2::Vector3 seg_start, seg_end, seg_dir;
    double seg_len = 0, along = 0, remaining = 0, dist_end = 0, target_yaw = yaw;
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

    // The part of the way to the segment end the base can drive without turning, in the robot frame: all
    // of it on a holonomic base, only the part along its heading otherwise (forward or backward).
    auto drivable = [&]() {
        const tf2::Vector3 target = robot_pose.inverse() * seg_end;
        return tf2::Vector3(target.x(), m_holonomic ? target.y() : 0.0, 0.0);
    };
    auto missed = [&](double dist) {
        // a non-holonomic base can drive a miss out only if what is left beside it is within tolerance
        const double beside = (robot_pose.inverse() * seg_end).y();
        if (dist <= m_adjust_max_dist && (m_holonomic || std::fabs(beside) <= xy_tol))
        {
            m_state = STATE_ADJUSTING;
            ROS_INFO_NAMED("StopGoLocalPlanner", "%.3f m off the end of segment %zu: adjusting%s", dist,
                           m_segment + 1, m_holonomic ? "" : " along the heading");
        }
        else
        {
            approach_again(dist);
        }
    };

    // A plan handed over at a corner (e.g. a controller switch while turning there) starts with the stub
    // of the leg the robot has just finished: skip it instead of turning back to it. (A short first
    // segment the robot stands at the start of is driven as usual.)
    if (m_segment == 0 && m_state == STATE_IDLE && m_vertices.size() > 2 && !m_started &&
        dist_end <= m_adjust_max_dist && dist_end < (seg_start - pos).length())
    {
        ROS_INFO_NAMED("StopGoLocalPlanner", "already at the end of segment 1 (%.3f m off): starting with segment 2",
                       dist_end);
        m_segment = 1;
        update_segment();
    }
    m_started = true;
    const bool is_last_segment = m_segment + 2 >= m_vertices.size();

    // moved off the goal after reaching it (e.g. a localization correction)
    if (m_state == STATE_GOAL && dist_end > xy_tol)
    {
        missed(dist_end);
        update_segment();
    }

    // a zero-length plan, or the last segment done: hold the goal orientation
    tf2::Stamped<tf2::Transform> goal_global;
    tf2::fromMsg(m_global_plan.back(), goal_global);
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
        ROS_INFO_NAMED("StopGoLocalPlanner", "segment %zu/%zu: %s (yaw_error=%.1f deg, length=%.2f m)",
                       m_segment + 1, m_vertices.size() - 1,
                       m_state == STATE_ROTATING ? "rotating in place" : "translating",
                       angles::to_degrees(yaw_error), seg_len);
    }

    auto segment_done = [&]() {
        if (is_last_segment)
        {
            m_state = STATE_GOAL;
            ROS_INFO_NAMED("StopGoLocalPlanner", "route end reached (%.3f m off)", dist_end);
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
            // pure pursuit on the current segment's line. The target stays a full lookahead ahead, past the
            // segment end if need be (the speed profile does the stopping): a target closing in on the robot
            // would turn a centimetre of offset into a large curvature, and steering, before every stop.
            const double lookahead = std::max(along, 0.0) + m_lookahead_dist;
            const tf2::Vector3 target_pos = seg_start + seg_dir * lookahead;
            const tf2::Vector3 local_target = robot_pose.inverse() * target_pos;
            const double dist_sq = local_target.x() * local_target.x() + local_target.y() * local_target.y();
            double curvature = dist_sq > 1e-6 ? 2 * local_target.y() / dist_sq : 0;

            // speed: braking profile toward the segment end, never below min_vel_trans before it
            const double profile = std::sqrt(2 * m_dec_lim_x * std::max(remaining, 0.0));
            planned_speed = std::max(std::min(profile, m_limits.max_vel_x), m_limits.min_vel_trans);
            control_vel_x = std::min(planned_speed, m_last_cmd_vel.linear.x + m_limits.acc_lim_x * dt);

            // The drive units' steering angles follow the curvature of the motion, not the yaw rate, so
            // the curvature is low passed and rate limited (max_curvature_rate; the yaw rate v * curvature is
            // slew limited on top): the steering then moves smoothly also while the speed changes and when a
            // plan is handed over while driving. A steering step makes the base stop, re-steer and start
            // again (stop-steer-go).
            if (m_have_curvature)
            {
                curvature = curvature * m_low_pass_gain + m_curvature * (1 - m_low_pass_gain);
                curvature = slew(m_curvature, curvature, m_max_curvature_rate * dt);
            }
            m_curvature = curvature;
            m_have_curvature = true;
            control_yawrate = control_vel_x * curvature;
            control_yawrate = slew(m_last_cmd_vel.angular.z, control_yawrate, m_trans_acc_lim_theta * dt);
        }
    }
    else if (m_state == STATE_ADJUSTING)
    {
        const tf2::Vector3 way = drivable();
        const double d = way.length();
        if (dist_end > m_adjust_max_dist)
        {
            approach_again(dist_end);
        }
        else if (d <= 0.5 * xy_tol)
        {
            if (is_stopped)
            {
                if (dist_end <= xy_tol)
                {
                    segment_done();
                }
                else
                {
                    approach_again(dist_end); // what is left beside the heading is out of tolerance
                }
            }
        }
        else
        {
            // straight toward the segment end at low speed, without turning
            planned_speed = std::min(std::max(d, m_limits.min_vel_trans), m_adjust_vel);
            control_vel_x = way.x() / d * planned_speed;
            control_vel_y = way.y() / d * planned_speed;
        }
    }

    geometry_msgs::Twist cmd;
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
        cmd = geometry_msgs::Twist(); // wait: no avoidance on the route
    }
    cmd_vel = cmd;
    m_last_cmd_vel = cmd;
    if (m_state != STATE_TRANSLATING || cmd.linear.x < 1e-3)
    {
        m_have_curvature = false; // stopping: the base re-aligns its steering before it drives again
    }

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
                        "state=%d, segment=%zu, remaining=%.3f, dist_end=%.3f, yaw_error=%.3f, cmd=(%.3f, %.3f, %.3f)",
                        m_state, m_segment, remaining, dist_end, yaw_error, cmd.linear.x, cmd.linear.y,
                        cmd.angular.z);
    }
    return true;
}

bool StopGoLocalPlanner::isGoalReached()
{
    boost::mutex::scoped_lock lock(m_odometry_mutex);

    if (!m_odometry || !odometryFresh(ros::Time::now()))
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
            auto msg = m_tf->lookupTransform(m_local_frame, m_global_frame, ros::Time(), ros::Duration(0));
            tf2::fromMsg(msg, global_to_local);
        }
        catch (...)
        {
            return false;
        }
        tf2::Stamped<tf2::Transform> goal_global;
        tf2::fromMsg(m_global_plan.back(), goal_global);
        const tf2::Transform goal_local = global_to_local * goal_global;
        const checks::Pose2D goal{goal_local.getOrigin().x(), goal_local.getOrigin().y(),
                                  tf2::getYaw(goal_local.getRotation())};
        const checks::Pose2D robot{m_odometry->pose.pose.position.x, m_odometry->pose.pose.position.y,
                                   tf2::getYaw(m_odometry->pose.pose.orientation)};
        const bool is_stopped = base_local_planner::stopped(*m_odometry, m_limits.theta_stopped_vel,
                                                            m_limits.trans_stopped_vel);
        is_reached = checks::goalReached(robot, goal, is_stopped, m_limits.xy_goal_tolerance,
                                         m_limits.yaw_goal_tolerance);
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
    m_blocked = false;
    m_started = false;
    // Start the slew limits from the current motion, so a plan handed over while driving (e.g. from
    // another controller) continues without a dip in speed.
    m_last_cmd_vel = geometry_msgs::Twist();
    m_have_curvature = false;
    {
        boost::mutex::scoped_lock lock(m_odometry_mutex);
        if (m_odometry && odometryFresh(ros::Time::now()))
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
    m_max_curvature_rate = private_nh.param<double>("max_curvature_rate", 1.0);
    m_goal_tune_time = private_nh.param<double>("goal_tune_time", 0.3);
    m_holonomic = private_nh.param<bool>("holonomic", true);
    m_adjust_max_dist = private_nh.param<double>("adjust_max_dist", 0.3);
    m_adjust_vel = private_nh.param<double>("adjust_vel", 0.1);
    m_odom_timeout = private_nh.param<double>("odom_timeout", 0.5);
    m_collision_latency = private_nh.param<double>("collision_latency", 0.3);
    m_collision_margin = private_nh.param<double>("collision_margin", 0.1);
    m_rotation_margin = private_nh.param<double>("rotation_margin", 0.1);
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
