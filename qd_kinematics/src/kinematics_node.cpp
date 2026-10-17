#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "qd_kinematics/kinematics_model.hpp"
#include "qd_kinematics/odometry_integrator.hpp"
#include "qd_kinematics/stamp_monitor.hpp"
#include "qd_kinematics/steer_drive_controller.hpp"

using qd_kinematics::ModuleState;
using qd_kinematics::StampMonitor;

// cmd_vel -> steer positions / drive velocities (gazebo_ros_control), joint_states -> odom + TF.
// Every time is ros::Time, so with /use_sim_time the node runs on Gazebo's /clock.
//
// Input validity (each checked separately):
//   cmd_vel       older than cmd_timeout_sec -> zero command (the controller decelerates at linear_dec);
//                 a non-finite command is taken as a zero command.
//   joint_states  a message is rejected when a required joint is missing, an array is shorter than
//                 the name list, a value is not finite, it has no stamp or a stamp in the future.
//                 Stamps are classified by StampMonitor (duplicates and late/reordered samples
//                 dropped, gaps not integrated). Only this node's ROS clock going back (simulation
//                 reset) resets odometry and the state machine; an old stamp never does.
//                 Without an accepted, recent sample for joint_state_timeout_sec the node commands
//                 zero wheel speed, holds the steer commands and restarts the state machine; it does
//                 not drive on old feedback.
// These timeouts only stop what this node commands. The drive unit driver needs its own watchdog on
// the command topics (see docs/safety.md): if this node dies, nothing here can stop the wheels.
class KinematicsNode
{
  public:
    KinematicsNode() : pnh_("~")
    {
    }

    bool init()
    {
        std::string drive_type;
        std::vector<double> module_x;
        std::vector<double> module_y;
        double fixed_axle_x = 0.0;
        pnh_.param<std::string>("drive_type", drive_type, "swerve");
        pnh_.param("module_x", module_x, std::vector<double>{0.330, -0.330});
        pnh_.param("module_y", module_y, std::vector<double>{0.135, -0.135});
        pnh_.param("fixed_axle_x", fixed_axle_x, 0.0);

        pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
        pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
        pnh_.param("wheel_diameter", wheel_diameter_, 0.32);
        pnh_.param("control_rate_hz", control_rate_hz_, 100.0);
        pnh_.param("cmd_timeout_sec", cmd_timeout_sec_, 0.5);
        pnh_.param("joint_state_timeout_sec", joint_state_timeout_sec_, 0.2);
        pnh_.param("stamp_future_tolerance_sec", stamp_future_tolerance_sec_, 0.05);
        pnh_.param("publish_tf", publish_tf_, true);
        pnh_.param("pose_covariance_diagonal", pose_cov_, std::vector<double>{1e-3, 1e-3, 1e3, 1e3, 1e3, 1e-2});
        pnh_.param("twist_covariance_diagonal", twist_cov_, std::vector<double>{1e-3, 1e-3, 1e3, 1e3, 1e3, 1e-2});
        pnh_.param("steer_joints", steer_joints_, std::vector<std::string>{"front_steer_joint", "rear_steer_joint"});
        pnh_.param("drive_joints", drive_joints_, std::vector<std::string>{"front_drive_joint", "rear_drive_joint"});

        double odom_max_dt = 0.1;
        pnh_.param("odom_max_dt", odom_max_dt, 0.1);
        odom_.setMaxDt(odom_max_dt);
        stamps_ = StampMonitor(odom_max_dt);

        qd_kinematics::ControllerParams ctrl;
        double hysteresis_deg = ctrl.steer_hysteresis * 180.0 / M_PI;
        pnh_.param("linear_acc", ctrl.linear_acc, ctrl.linear_acc);
        pnh_.param("linear_dec", ctrl.linear_dec, ctrl.linear_dec);
        pnh_.param("max_steer_vel", ctrl.max_steer_vel, ctrl.max_steer_vel);
        pnh_.param("steer_hysteresis_deg", hysteresis_deg, hysteresis_deg);
        pnh_.param("steer_track_limit", ctrl.steer_track_limit, ctrl.steer_track_limit);
        pnh_.param("creep_speed_max", ctrl.creep_speed_max, ctrl.creep_speed_max);
        pnh_.param("wait_stop_timeout", ctrl.wait_stop_timeout, ctrl.wait_stop_timeout);
        pnh_.param("align_timeout", ctrl.align_timeout, ctrl.align_timeout);
        pnh_.param("creep_timeout", ctrl.creep_timeout, ctrl.creep_timeout);
        pnh_.param("max_start_attempts", ctrl.max_start_attempts, ctrl.max_start_attempts);
        pnh_.param("drive_track_abs_tol", ctrl.drive_track_abs_tol, ctrl.drive_track_abs_tol);
        pnh_.param("drive_track_rel_tol", ctrl.drive_track_rel_tol, ctrl.drive_track_rel_tol);
        pnh_.param("drive_fault_time", ctrl.drive_fault_time, ctrl.drive_fault_time);
        ctrl.steer_hysteresis = hysteresis_deg * M_PI / 180.0;

        if (control_rate_hz_ <= 0.0)
        {
            ROS_WARN("control_rate_hz <= 0.0, using 100.0");
            control_rate_hz_ = 100.0;
        }
        if (cmd_timeout_sec_ <= 0.0 || joint_state_timeout_sec_ <= 0.0)
        {
            ROS_ERROR("cmd_timeout_sec and joint_state_timeout_sec must be > 0");
            return false;
        }
        if (pose_cov_.size() != 6 || twist_cov_.size() != 6)
        {
            ROS_ERROR("pose/twist_covariance_diagonal must have 6 elements");
            return false;
        }
        if (module_x.empty() || module_x.size() != module_y.size())
        {
            ROS_ERROR("module_x and module_y must be non-empty and the same length");
            return false;
        }
        if (wheel_diameter_ <= 0.0)
        {
            ROS_ERROR("wheel_diameter must be > 0");
            return false;
        }

        std::vector<qd_kinematics::ModulePosition> modules;
        for (size_t i = 0; i < module_x.size(); ++i)
        {
            modules.push_back({module_x[i], module_y[i]});
        }

        try
        {
            if (drive_type == "swerve")
            {
                model_ = std::make_shared<qd_kinematics::SwerveKinematics>(modules);
            }
            else if (drive_type == "tricycle")
            {
                if (modules.size() != 1)
                {
                    ROS_ERROR("drive_type 'tricycle' needs exactly one module");
                    return false;
                }
                model_ = std::make_shared<qd_kinematics::TricycleKinematics>(modules[0], fixed_axle_x);
            }
            else
            {
                ROS_ERROR("unknown drive_type '%s' (swerve | tricycle)", drive_type.c_str());
                return false;
            }
            controller_ = std::make_unique<qd_kinematics::SteerDriveController>(model_, ctrl);
        }
        catch (const std::invalid_argument &e)
        {
            ROS_ERROR("invalid kinematics geometry: %s", e.what());
            return false;
        }
        num_modules_ = model_->numModules();

        if (steer_joints_.size() != num_modules_ || drive_joints_.size() != num_modules_)
        {
            ROS_ERROR("steer_joints/drive_joints must have %zu entries", num_modules_);
            return false;
        }

        // Topics are relative so they can be remapped from the launch file.
        pub_steer_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>("steer_controller/command", 10);
        pub_drive_cmd_ = nh_.advertise<std_msgs::Float64MultiArray>("drive_controller/command", 10);
        pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
        pub_debug_state_ = pnh_.advertise<std_msgs::String>("debug_state", 10);
        pub_fault_ = pnh_.advertise<std_msgs::String>("fault", 1, true);
        sub_cmd_vel_ = nh_.subscribe("cmd_vel", 10, &KinematicsNode::receiveCmd, this);
        sub_joint_state_ = nh_.subscribe("joint_states", 10, &KinematicsNode::receiveJointState, this);
        srv_clear_fault_ = pnh_.advertiseService("clear_fault", &KinematicsNode::clearFault, this);
        publishFault();

        control_timer_ =
            nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &KinematicsNode::controlLoop, this);

        ROS_INFO("qd_kinematics node initialized");
        ROS_INFO("  drive_type: %s (%zu module%s)", model_->name().c_str(), num_modules_, num_modules_ > 1 ? "s" : "");
        ROS_INFO("  use_sim_time: %s", ros::Time::isSimTime() ? "true" : "false");
        ROS_INFO("  control_rate_hz: %.1f", control_rate_hz_);
        ROS_INFO("  cmd_timeout_sec: %.2f, joint_state_timeout_sec: %.2f", cmd_timeout_sec_, joint_state_timeout_sec_);
        ROS_INFO("  steer_hysteresis_deg: %.1f", hysteresis_deg);
        return true;
    }

  private:
    void receiveCmd(const geometry_msgs::Twist::ConstPtr &msg)
    {
        last_cmd_time_ = ros::Time::now();
        has_cmd_ = true;
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z))
        {
            ++rejected_cmd_;
            ROS_ERROR_THROTTLE(1.0, "cmd_vel is not finite: taken as a zero command");
            target_cmd_ = geometry_msgs::Twist();
            return;
        }
        target_cmd_ = *msg;
    }

    void rejectJointState(const char *why)
    {
        ++rejected_feedback_;
        ROS_WARN_THROTTLE(2.0, "joint_states rejected: %s", why);
    }

    // The ROS clock went back (Gazebo world reset, bag restart): odometry, stamp history and the state
    // machine start over. Called with the current time by every callback.
    void checkClock(const ros::Time &now)
    {
        if (has_last_now_ && now < last_now_)
        {
            ROS_WARN("ROS time jumped back %.3f s: resetting odometry and the drive state machine",
                     (last_now_ - now).toSec());
            odom_.reset();
            stamps_.reset();
            controller_->reset();
            has_last_control_time_ = false;
            has_state_ = false;
        }
        last_now_ = now;
        has_last_now_ = true;
    }

    void receiveJointState(const sensor_msgs::JointState::ConstPtr &msg)
    {
        const ros::Time now = ros::Time::now();
        checkClock(now);
        auto index_of = [&](const std::string &name) -> int {
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == name)
                {
                    return static_cast<int>(i);
                }
            }
            return -1;
        };

        // ---- structure and values ----
        if ((!msg->position.empty() && msg->position.size() != msg->name.size()) ||
            (!msg->velocity.empty() && msg->velocity.size() != msg->name.size()))
        {
            rejectJointState("position/velocity arrays do not match the name list");
            return;
        }
        const double radius = wheel_diameter_ * 0.5;
        std::vector<ModuleState> state(num_modules_);
        for (size_t m = 0; m < num_modules_; ++m)
        {
            const int i_s = index_of(steer_joints_[m]);
            const int i_d = index_of(drive_joints_[m]);
            if (i_s < 0 || i_d < 0 || static_cast<size_t>(i_s) >= msg->position.size() ||
                static_cast<size_t>(i_d) >= msg->velocity.size())
            {
                rejectJointState("a required steer/drive joint (position/velocity) is missing");
                return;
            }
            state[m] = {msg->position[i_s], msg->velocity[i_d] * radius};
            if (!std::isfinite(state[m].angle) || !std::isfinite(state[m].speed))
            {
                rejectJointState("non-finite steer position or drive velocity");
                return;
            }
        }

        // ---- measurement time ----
        if (msg->header.stamp.isZero())
        {
            rejectJointState("no stamp (the measurement time is required)");
            return;
        }
        const ros::Time meas_time = msg->header.stamp;
        if ((meas_time - now).toSec() > stamp_future_tolerance_sec_)
        {
            ++rejected_future_;
            ROS_WARN_THROTTLE(2.0, "joint_states rejected: stamp %.3f s in the future (clock offset?)",
                              (meas_time - now).toSec());
            return;
        }
        const StampMonitor::Result r = stamps_.check(meas_time.toSec());
        if (!StampMonitor::accepted(r))
        {
            ROS_WARN_THROTTLE(2.0, "joint_states dropped: %s stamp (duplicates %lu, out of order %lu)",
                              StampMonitor::name(r), static_cast<unsigned long>(stamps_.duplicates),
                              static_cast<unsigned long>(stamps_.out_of_order));
            return;
        }
        if (r == StampMonitor::Result::GAP)
        {
            ROS_WARN_THROTTLE(2.0, "joint_states gap: odometry is not integrated across it (%lu gaps so far)",
                              static_cast<unsigned long>(stamps_.gaps));
        }

        // Odometry always follows the measurement time ...
        publishOdometry(state, meas_time);

        // ... but only a recent sample may be used for control.
        const double age = (now - meas_time).toSec();
        if (age > joint_state_timeout_sec_)
        {
            ++late_feedback_;
            ROS_WARN_THROTTLE(2.0, "joint_states %.3f s old on arrival: not used for control", age);
            return;
        }
        latest_state_ = state;
        has_state_ = true;
        last_feedback_time_ = now;
    }

    void publishOdometry(const std::vector<ModuleState> &state, const ros::Time &meas_time)
    {
        odom_.update(model_->bodyTwist(state), meas_time.toSec());
        const auto &pose = odom_.pose();
        const auto &twist = odom_.twist();

        nav_msgs::Odometry odom;
        odom.header.stamp = meas_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, pose.yaw);
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = twist.vx;
        odom.twist.twist.linear.y = twist.vy;
        odom.twist.twist.angular.z = twist.wz;
        for (size_t i = 0; i < 6; ++i)
        {
            odom.pose.covariance[i * 6 + i] = pose_cov_[i];
            odom.twist.covariance[i * 6 + i] = twist_cov_[i];
        }
        pub_odometry_.publish(odom);

        if (publish_tf_)
        {
            geometry_msgs::TransformStamped t;
            t.header = odom.header;
            t.child_frame_id = base_frame_;
            t.transform.translation.x = pose.x;
            t.transform.translation.y = pose.y;
            t.transform.rotation = odom.pose.pose.orientation;
            odom_broadcaster_.sendTransform(t);
        }
    }

    bool feedbackFresh(const ros::Time &now) const
    {
        const double age = (now - last_feedback_time_).toSec();
        return has_state_ && age >= 0.0 && age <= joint_state_timeout_sec_;
    }

    void publishCommand(const std::vector<double> &steer, const std::vector<double> &drive)
    {
        std_msgs::Float64MultiArray steer_cmd;
        std_msgs::Float64MultiArray drive_cmd;
        steer_cmd.data = steer;
        drive_cmd.data = drive;
        pub_steer_cmd_.publish(steer_cmd);
        pub_drive_cmd_.publish(drive_cmd);
        last_steer_cmd_ = steer;
    }

    void controlLoop(const ros::TimerEvent &)
    {
        const ros::Time now_t = ros::Time::now();
        checkClock(now_t);
        double dt = 1.0 / control_rate_hz_;
        if (has_last_control_time_)
        {
            dt = (now_t - last_control_time_).toSec();
            if (dt == 0.0)
            {
                return; // clock paused
            }
        }
        last_control_time_ = now_t;
        has_last_control_time_ = true;

        if (!has_state_)
        {
            return; // no feedback yet: nothing is commanded
        }

        geometry_msgs::Twist cmd_msg;
        const bool cmd_timed_out = !has_cmd_ || (now_t - last_cmd_time_).toSec() > cmd_timeout_sec_;
        if (!cmd_timed_out)
        {
            cmd_msg = target_cmd_;
        }

        if (!feedbackFresh(now_t))
        {
            if (!feedback_lost_)
            {
                ROS_ERROR("joint_states not received for %.2f s: commanding zero wheel speed and holding the steering",
                          (now_t - last_feedback_time_).toSec());
                feedback_lost_ = true;
            }
            // Restart with stop -> steer-align -> creep once feedback is back (a FAULT is kept).
            controller_->reset();
            std::vector<double> steer = last_steer_cmd_;
            if (steer.size() != num_modules_)
            {
                steer.clear();
                for (const auto &s : latest_state_)
                {
                    steer.push_back(s.angle);
                }
            }
            publishCommand(steer, std::vector<double>(num_modules_, 0.0));
            publishDebug(cmd_msg, cmd_timed_out, now_t);
            return;
        }
        if (feedback_lost_)
        {
            ROS_WARN("joint_states fresh again: restarting with stop-steer-go");
            feedback_lost_ = false;
        }

        const qd_kinematics::Twist2D cmd{cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z};
        std::vector<ModuleState> command;
        controller_->update(cmd, latest_state_, dt, command);

        if (controller_->faulted() != reported_fault_)
        {
            reported_fault_ = controller_->faulted();
            if (reported_fault_)
            {
                ROS_ERROR("drive FAULT: %s. Wheels stopped; call ~clear_fault after checking the drive units.",
                          controller_->faultReason().c_str());
            }
            publishFault();
        }

        std::vector<double> steer;
        std::vector<double> drive;
        const double radius = wheel_diameter_ * 0.5;
        for (const auto &c : command)
        {
            steer.push_back(c.angle);
            drive.push_back(c.speed / radius);
        }
        publishCommand(steer, drive);
        publishDebug(cmd_msg, cmd_timed_out, now_t);
    }

    bool clearFault(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res)
    {
        if (!controller_->faulted())
        {
            res.success = true;
            res.message = "no fault";
            return true;
        }
        if (!feedbackFresh(ros::Time::now()))
        {
            res.success = false;
            res.message = "joint_states are not fresh; the fault is kept";
            return true;
        }
        ROS_WARN("drive FAULT cleared by request (was: %s)", controller_->faultReason().c_str());
        controller_->clearFault();
        reported_fault_ = false;
        publishFault();
        res.success = true;
        res.message = "fault cleared";
        return true;
    }

    void publishFault()
    {
        std_msgs::String msg;
        msg.data = controller_->faultReason();
        pub_fault_.publish(msg);
    }

    void publishDebug(const geometry_msgs::Twist &cmd, bool cmd_timed_out, const ros::Time &now)
    {
        const auto &dbg = controller_->debugState();
        auto vec = [](const std::vector<double> &v) {
            std::ostringstream s;
            s << std::fixed << std::setprecision(3) << "(";
            for (size_t i = 0; i < v.size(); ++i)
            {
                s << (i ? "," : "") << v[i];
            }
            s << ")";
            return s.str();
        };

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << "mode="
            << (feedback_lost_ ? "FEEDBACK_TIMEOUT" : qd_kinematics::SteerDriveController::modeName(dbg.mode))
            << " align_ok=" << (dbg.align_ok ? 1 : 0) << " speed_sync_ok=" << (dbg.speed_sync_ok ? 1 : 0)
            << " drive_stopped=" << (dbg.drive_stopped ? 1 : 0) << " near_zero_cmd=" << (dbg.near_zero_cmd ? 1 : 0)
            << " balance_on=" << (dbg.drive_balance_active ? 1 : 0) << " balance_corr=" << vec(dbg.balance_corr)
            << " cmd_timeout=" << (cmd_timed_out ? 1 : 0) << " cmd_in=(" << cmd.linear.x << "," << cmd.linear.y
            << "," << cmd.angular.z << ")"
            << " th_cur=" << vec(dbg.th_current) << " th_tgt=" << vec(dbg.th_target) << " dir_err=" << vec(dbg.dir_err)
            << " v_cur=" << vec(dbg.v_current) << " v_tgt=" << vec(dbg.v_target) << " v_cmd=" << vec(dbg.v_cmd)
            << " hold_t=(" << dbg.stop_time << "," << dbg.align_time << "," << dbg.sync_time << ")"
            << " phase_t=" << dbg.phase_time << " attempts=" << dbg.start_attempts
            << " fb_age=" << (has_state_ ? (now - last_feedback_time_).toSec() : -1.0)
            << " rejected(fb,future,late,cmd)=(" << rejected_feedback_ << "," << rejected_future_ << ","
            << late_feedback_ << "," << rejected_cmd_ << ")"
            << " stamps(dup,order,gap)=(" << stamps_.duplicates << "," << stamps_.out_of_order << ","
            << stamps_.gaps << ")";
        if (controller_->faulted())
        {
            oss << " fault=\"" << controller_->faultReason() << "\"";
        }

        std_msgs::String dbg_msg;
        dbg_msg.data = oss.str();
        pub_debug_state_.publish(dbg_msg);
        ROS_INFO_THROTTLE(0.5, "%s", dbg_msg.data.c_str());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    std::shared_ptr<qd_kinematics::KinematicsModel> model_;
    std::unique_ptr<qd_kinematics::SteerDriveController> controller_;
    qd_kinematics::OdometryIntegrator odom_;
    StampMonitor stamps_;
    size_t num_modules_{0};

    ros::Publisher pub_steer_cmd_;
    ros::Publisher pub_drive_cmd_;
    ros::Publisher pub_odometry_;
    ros::Publisher pub_debug_state_;
    ros::Publisher pub_fault_;
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_joint_state_;
    ros::ServiceServer srv_clear_fault_;
    ros::Timer control_timer_;
    tf2_ros::TransformBroadcaster odom_broadcaster_;

    geometry_msgs::Twist target_cmd_;
    std::vector<ModuleState> latest_state_;
    std::vector<double> last_steer_cmd_;
    ros::Time last_cmd_time_;
    ros::Time last_feedback_time_;
    ros::Time last_control_time_;
    ros::Time last_now_;

    bool has_cmd_{false};
    bool has_state_{false};
    bool has_last_control_time_{false};
    bool has_last_now_{false};
    bool feedback_lost_{false};
    bool reported_fault_{false};
    bool publish_tf_{true};

    unsigned long rejected_feedback_{0};
    unsigned long rejected_future_{0};
    unsigned long late_feedback_{0};
    unsigned long rejected_cmd_{0};

    double wheel_diameter_{0.32};
    double control_rate_hz_{100.0};
    double cmd_timeout_sec_{0.5};
    double joint_state_timeout_sec_{0.2};
    double stamp_future_tolerance_sec_{0.05};
    std::vector<double> pose_cov_;
    std::vector<double> twist_cov_;

    std::string odom_frame_;
    std::string base_frame_;
    std::vector<std::string> steer_joints_;
    std::vector<std::string> drive_joints_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_node");
    KinematicsNode node;
    if (!node.init())
    {
        return 1;
    }
    ros::spin();
    return 0;
}
