#ifndef PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_
#define PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_

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

namespace pure_pursuit_local_planner
{

// Pure pursuit path follower for curved paths (e.g. a global planner's detour around an obstacle),
// structured after neo_local_planner (Neobotix). Rotates in place first if the path starts off the
// robot's heading, then follows without stopping: yaw rate = v * curvature, speed limited by the
// curvature (max_curve_vel) and a braking profile toward the end of the path. Every command is checked
// on the local costmap with the swept footprint (local_planner_checks): the motion the command starts
// from the robot's actual pose and the path ahead, both over the stopping distance from the measured,
// commanded or planned speed plus min_stop_dist, and in-place turns over the whole remaining turn.
// A blocked command becomes a stop (STATE_BLOCKED); the route supervisor replans the detour.
class PurePursuitLocalPlanner : public nav2_core::Controller
{
  public:
    PurePursuitLocalPlanner();
    ~PurePursuitLocalPlanner() override;

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
    bool commandIsFree(const geometry_msgs::msg::Twist &cmd, double planned_speed, double target_yawrate, double rotation_left,
                       const std::vector<tf2::Vector3> &local_plan, const tf2::Transform &robot, double yaw);

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

    std::mutex m_odometry_mutex;
    nav_msgs::msg::Odometry::ConstSharedPtr m_odometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr m_local_plan_pub;

    std::string m_global_frame = "map";
    std::string m_local_frame = "odom";
    std::string m_base_frame = "base_footprint";

    local_planner_checks::Limits m_limits = {};

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
    rclcpp::Time m_last_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time m_first_goal_reached_time{0, 0, RCL_ROS_TIME};
    bool m_is_goal_reached = false;
    bool m_blocked = false;
    geometry_msgs::msg::Twist m_last_cmd_vel;
};

} // namespace pure_pursuit_local_planner

#endif // PURE_PURSUIT_LOCAL_PLANNER_PUREPURSUITLOCALPLANNER_H_
