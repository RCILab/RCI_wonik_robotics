#ifndef STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_
#define STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_

#include <string>
#include <vector>

#include <base_local_planner/local_planner_limits.h>
#include <boost/thread.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

namespace stop_go_local_planner
{

// Stop-and-go route follower, structured after neo_local_planner (Neobotix).
// The plan is split into straight segments where its direction changes by more than
// corner_angle. Each segment starts with an in-place rotation to its heading (if the heading
// error exceeds start_yaw_error), is followed with pure pursuit (yaw rate = v * curvature) and ends
// in a stop. A segment end (and the goal) counts only when the robot is within xy_goal_tolerance of
// it; a small miss (sideways, overshoot, or a localization correction) is driven out without turning
// (ADJUSTING: in any direction on a holonomic base; forward or backward along the heading on a
// non-holonomic one, if what is left beside the heading is within tolerance), a larger one by
// approaching the point again as a new segment.
//
// No obstacle avoidance: the robot keeps to the route. Every command is checked against the local
// costmap with the swept footprint (local_planner_checks): driving over the stopping distance from the
// measured, commanded and planned speed, turning in place over the whole remaining turn. A blocked
// command becomes a stop, and the robot waits until the way is clear.
class StopGoLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    StopGoLocalPlanner();
    ~StopGoLocalPlanner() override;

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool odometryFresh(const ros::Time &now) const;
    // Collision check of cmd from the robot pose; rotation_left is the rest of an in-place turn.
    bool commandIsFree(const geometry_msgs::Twist &cmd, double planned_speed, double rotation_left,
                       const tf2::Vector3 &pos, double yaw);

  private:
    tf2_ros::Buffer *m_tf = nullptr;
    costmap_2d::Costmap2DROS *m_cost_map = nullptr;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;
    std::vector<tf2::Vector3> m_vertices; // segment end points of the plan, global frame

    boost::mutex m_odometry_mutex;
    nav_msgs::Odometry::ConstPtr m_odometry;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_local_plan_pub;

    std::string m_global_frame = "map";
    std::string m_local_frame = "odom";
    std::string m_base_frame = "base_footprint";

    base_local_planner::LocalPlannerLimits m_limits = {};

    double m_dec_lim_x = 0;       // [m/s^2] braking profile toward each stop
    double m_lookahead_dist = 0;  // [m] pure pursuit lookahead on the current segment
    double m_start_yaw_error = 0; // [rad] rotate in place first when the heading error exceeds this
    double m_corner_angle = 0;    // [rad] heading change that splits the plan into segments
    double m_trans_acc_lim_theta = 0; // [rad/s^2] yaw-rate slew while translating
    double m_low_pass_gain = 0;   // curvature low pass while translating (1 = off)
    double m_goal_tune_time = 0;  // [s] stay within tolerance this long before the goal counts
    bool m_holonomic = true;      // the base can move sideways (ADJUSTING; false: along its heading only)
    double m_adjust_max_dist = 0; // [m] larger misses of a segment end are approached again as a segment
    double m_adjust_vel = 0;      // [m/s] speed limit while ADJUSTING
    double m_odom_timeout = 0;    // [s] older odometry is not used (the controller fails)
    double m_collision_latency = 0; // [s] reaction time in the stopping distance
    double m_collision_margin = 0;  // [m] added to the stopping distance
    double m_rotation_margin = 0;   // [rad] added to the remaining turn in the rotation check

    enum state_t
    {
        STATE_IDLE,
        STATE_ROTATING,
        STATE_TRANSLATING,
        STATE_ADJUSTING,
        STATE_GOAL
    };

    state_t m_state = STATE_IDLE;
    std::size_t m_segment = 0;
    ros::Time m_last_time;
    ros::Time m_first_goal_reached_time;
    bool m_is_goal_reached = false;
    bool m_blocked = false;
    bool m_started = false; // the first control step of the current plan has run
    uint64_t m_update_counter = 0;
    geometry_msgs::Twist m_last_cmd_vel;
    double m_curvature = 0;        // [1/m] filtered curvature of the last translating step
    bool m_have_curvature = false; // m_curvature is the steering the base is driving with
};

} // namespace stop_go_local_planner

#endif // STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_
