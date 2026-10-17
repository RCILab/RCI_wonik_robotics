#include <cmath>
#include <limits>
#include <nav2_core/goal_checker.hpp>
#include <nav2_core/progress_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "local_planner_checks/footprint_checker.h"
namespace local_planner_checks {
class SettledGoalChecker : public nav2_core::GoalChecker {
 rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
 double xy_{.03}, yaw_{.02}, linear_{.02}, angular_{.02}, hold_{.3};
 rclcpp::Time since_{0,0,RCL_ROS_TIME}; bool inside_{false};
public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &name,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
  node_=parent.lock();
  auto p=[&](const std::string &key,double def) {
   const auto full=name+"."+key;
   if(!node_->has_parameter(full)) node_->declare_parameter(full,def);
   return node_->get_parameter(full).as_double();
  };
  xy_=p("xy_goal_tolerance",.03);yaw_=p("yaw_goal_tolerance",.02);
  linear_=p("trans_stopped_vel",.02);angular_=p("rot_stopped_vel",.02);hold_=p("goal_tune_time",.3);
 }
 void reset() override {inside_=false;}
 bool isGoalReached(const geometry_msgs::msg::Pose &p,const geometry_msgs::msg::Pose &g,
                    const geometry_msgs::msg::Twist &v) override {
  const bool hit=goalReached({p.position.x,p.position.y,tf2::getYaw(p.orientation)},
    {g.position.x,g.position.y,tf2::getYaw(g.orientation)},
    std::hypot(v.linear.x,v.linear.y)<=linear_ && std::abs(v.angular.z)<=angular_,xy_,yaw_);
  const auto now=node_->now();
  if(!hit) {inside_=false;return false;}
  if(!inside_ || now<since_) {since_=now;inside_=true;}
  return (now-since_).seconds()>=hold_;
 }
 bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &v) override {
  p.position.x=p.position.y=xy_;p.position.z=std::numeric_limits<double>::lowest();
  tf2::Quaternion q;q.setRPY(0,0,yaw_);p.orientation=tf2::toMsg(q);
  v.linear.x=v.linear.y=linear_;v.angular.z=angular_;
  v.linear.z=v.angular.x=v.angular.y=std::numeric_limits<double>::lowest();return true;
 }
};
// Route stops are intentional; drive and sensor supervision remain enabled.
class RouteProgressChecker : public nav2_core::ProgressChecker {
public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,const std::string &) override {}
 void reset() override {}
 bool check(geometry_msgs::msg::PoseStamped &) override {return true;}
};
}
PLUGINLIB_EXPORT_CLASS(local_planner_checks::SettledGoalChecker,nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(local_planner_checks::RouteProgressChecker,nav2_core::ProgressChecker)
