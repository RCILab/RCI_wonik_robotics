#ifndef PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_
#define PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_

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

namespace pure_pursuit_local_planner
{

// Pure pursuit path follower for curved paths (e.g. a global planner's detour around an obstacle),
// structured after neo_local_planner (Neobotix). Rotates in place first if the path starts off the
// robot's heading, then follows without stopping: yaw rate = v * curvature, speed limited by the
// curvature (max_curve_vel) and a braking profile toward the end of the path. Every command is checked
// on the local costmap with the swept footprint (local_planner_checks): the motion the controller drives
// from the robot's actual pose (yaw rate ramping toward the pursuit arc) and the path ahead, both over
// the stopping distance from the measured,
// commanded or planned speed plus min_stop_dist, and in-place turns over the whole remaining turn.
// A blocked command becomes a stop (STATE_BLOCKED); the route supervisor replans the detour.
class PurePursuitLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    PurePursuitLocalPlanner();
    ~PurePursuitLocalPlanner() override;

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;
    bool isGoalReached() override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool odometryFresh(const ros::Time &now) const;
    bool commandIsFree(const geometry_msgs::Twist &cmd, double planned_speed, double target_yawrate,
                       double rotation_left, const std::vector<tf2::Vector3> &local_plan, const tf2::Transform &robot,
                       double yaw);

  private:
    tf2_ros::Buffer *m_tf = nullptr;
    costmap_2d::Costmap2DROS *m_cost_map = nullptr;
    std::vector<geometry_msgs::PoseStamped> m_global_plan;

    boost::mutex m_odometry_mutex;
    nav_msgs::Odometry::ConstPtr m_odometry;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_local_plan_pub;

    std::string m_global_frame = "map";
    std::string m_local_frame = "odom";
    std::string m_base_frame = "base_footprint";

    base_local_planner::LocalPlannerLimits m_limits = {};

    double m_dec_lim_x = 0;           // [m/s^2] braking profile toward the end of the path
    double m_lookahead_dist = 0;      // [m]
    double m_lookahead_time = 0;      // [s] lookahead grows with speed
    double m_start_yaw_error = 0;     // [rad] rotate in place first when the path starts off this much
    double m_max_curve_vel = 0;       // [rad/s] yaw rate limit in curves, caps the speed
    double m_trans_acc_lim_theta = 0; // [rad/s^2] yaw-rate slew while translating
    double m_low_pass_gain = 0;       // yaw-rate low pass while translating (1 = off)
    double m_min_stop_dist = 0;       // [m] footprint check reaches this far beyond the braking distance
    double m_goal_tune_time = 0;      // [s]
    double m_odom_timeout = 0;        // [s] older odometry is not used (the controller fails)
    double m_collision_latency = 0;   // [s] reaction time in the stopping distance
    double m_rotation_margin = 0;     // [rad] added to the remaining turn in the rotation check

    enum state_t
    {
        STATE_IDLE,
        STATE_ROTATING,
        STATE_TRANSLATING,
        STATE_BLOCKED,
        STATE_GOAL
    };

    state_t m_state = STATE_IDLE;
    std::size_t m_closest = 0;
    ros::Time m_last_time;
    ros::Time m_first_goal_reached_time;
    bool m_is_goal_reached = false;
    bool m_blocked = false;
    geometry_msgs::Twist m_last_cmd_vel;
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_
