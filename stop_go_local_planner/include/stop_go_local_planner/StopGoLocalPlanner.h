#ifndef STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_
#define STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_

#include <string>
#include <vector>

#include <local_planner_checks/controller_types.hpp>
#include <mutex>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

namespace stop_go_local_planner
{

// Stop-and-go route follower, structured after neo_local_planner (Neobotix).
// The plan is split into straight segments where its direction changes by more than
// corner_angle. Each segment starts with an in-place rotation to its heading (if the heading
// error exceeds start_yaw_error), is followed with pure pursuit (yaw rate = v * curvature) and ends
// in a stop. A segment end (and the goal) counts only when the robot is within xy_goal_tolerance of
// it; a small miss (sideways, overshoot, or a localization correction) is driven out sideways
// (ADJUSTING, holonomic bases), a larger one by approaching the point again as a new segment.
//
// No obstacle avoidance: the robot keeps to the route. Every command is checked against the local
// costmap with the swept footprint (local_planner_checks): driving over the stopping distance from the
// measured, commanded and planned speed, turning in place over the whole remaining turn. A blocked
// command becomes a stop, and the robot waits until the way is clear.
class StopGoLocalPlanner : public nav2_core::Controller
{
  public:
    StopGoLocalPlanner();
    ~StopGoLocalPlanner() override;

    bool computeVelocityCommands(geometry_msgs::msg::Twist &cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &plan);
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    void setPlan(const nav_msgs::msg::Path &path) override;
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker) override;
    void setSpeedLimit(const double &limit, const bool &percentage) override;

  private:
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    bool odometryFresh(const rclcpp::Time &now) const;
    // Collision check of cmd from the robot pose; rotation_left is the rest of an in-place turn.
    bool commandIsFree(const geometry_msgs::msg::Twist &cmd, double planned_speed, double rotation_left,
                       const tf2::Vector3 &pos, double yaw);

  private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr m_node;
    std::string m_name;
    double m_default_max_vel_x = 0;
    template<class T> T param(const std::string &name, const T &value) {
        const auto key = m_name + "." + name;
        if (!m_node->has_parameter(key)) m_node->declare_parameter<T>(key, value);
        return m_node->get_parameter(key).get_value<T>();
    }
    std::shared_ptr<tf2_ros::Buffer> m_tf;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> m_cost_map;
    std::vector<geometry_msgs::msg::PoseStamped> m_global_plan;
    std::vector<tf2::Vector3> m_vertices; // segment end points of the plan, global frame

    std::mutex m_odometry_mutex;
    nav_msgs::msg::Odometry::ConstSharedPtr m_odometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr m_local_plan_pub;

    std::string m_global_frame = "map";
    std::string m_local_frame = "odom";
    std::string m_base_frame = "base_footprint";

    local_planner_checks::Limits m_limits = {};

    double m_dec_lim_x = 0;       // [m/s^2] braking profile toward each stop
    double m_lookahead_dist = 0;  // [m] pure pursuit lookahead on the current segment
    double m_start_yaw_error = 0; // [rad] rotate in place first when the heading error exceeds this
    double m_corner_angle = 0;    // [rad] heading change that splits the plan into segments
    double m_trans_acc_lim_theta = 0; // [rad/s^2] yaw-rate slew while translating
    double m_low_pass_gain = 0;   // yaw-rate low pass while translating (1 = off)
    double m_goal_tune_time = 0;  // [s] stay within tolerance this long before the goal counts
    bool m_holonomic = true;      // the base can move sideways (ADJUSTING)
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
    rclcpp::Time m_last_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time m_first_goal_reached_time{0, 0, RCL_ROS_TIME};
    bool m_is_goal_reached = false;
    bool m_blocked = false;
    uint64_t m_update_counter = 0;
    geometry_msgs::msg::Twist m_last_cmd_vel;
};

} // namespace stop_go_local_planner

#endif // STOP_GO_LOCAL_PLANNER_STOPGOLOCALPLANNER_H_
