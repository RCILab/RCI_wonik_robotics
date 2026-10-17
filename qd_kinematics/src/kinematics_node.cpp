#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "qd_kinematics/kinematics_model.hpp"
#include "qd_kinematics/odometry_integrator.hpp"
#include "qd_kinematics/stamp_monitor.hpp"
#include "qd_kinematics/steer_drive_controller.hpp"

using qd_kinematics::ModuleState;
using qd_kinematics::StampMonitor;

// cmd_vel -> steer positions / drive velocities (ros2_control), joint_states -> odom + TF.
// Every time is rclcpp::Time, so with /use_sim_time the node runs on Gazebo's /clock.
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
class KinematicsNode : public rclcpp::Node
{
  public:
    KinematicsNode() : rclcpp::Node("kinematics_node")
    {
    }

    bool init()
    {
        std::string drive_type;
        std::vector<double> module_x;
        std::vector<double> module_y;
        double fixed_axle_x = 0.0;
        drive_type = declare_parameter<std::string>("drive_type", "swerve");
        module_x = declare_parameter("module_x", std::vector<double>{0.330, -0.330});
        module_y = declare_parameter("module_y", std::vector<double>{0.135, -0.135});
        fixed_axle_x = declare_parameter("fixed_axle_x", 0.0);

        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
        wheel_diameter_ = declare_parameter("wheel_diameter", 0.32);
        control_rate_hz_ = declare_parameter("control_rate_hz", 100.0);
        cmd_timeout_sec_ = declare_parameter("cmd_timeout_sec", 0.5);
        joint_state_timeout_sec_ = declare_parameter("joint_state_timeout_sec", 0.2);
        stamp_future_tolerance_sec_ = declare_parameter("stamp_future_tolerance_sec", 0.05);
        publish_tf_ = declare_parameter("publish_tf", true);
        pose_cov_ = declare_parameter("pose_covariance_diagonal", std::vector<double>{1e-3, 1e-3, 1e3, 1e3, 1e3, 1e-2});
        twist_cov_ = declare_parameter("twist_covariance_diagonal", std::vector<double>{1e-3, 1e-3, 1e3, 1e3, 1e3, 1e-2});
        steer_joints_ = declare_parameter("steer_joints", std::vector<std::string>{"front_steer_joint", "rear_steer_joint"});
        drive_joints_ = declare_parameter("drive_joints", std::vector<std::string>{"front_drive_joint", "rear_drive_joint"});

        double odom_max_dt = 0.1;
        odom_max_dt = declare_parameter("odom_max_dt", 0.1);
        odom_.setMaxDt(odom_max_dt);
        stamps_ = StampMonitor(odom_max_dt);

        qd_kinematics::ControllerParams ctrl;
        double hysteresis_deg = ctrl.steer_hysteresis * 180.0 / M_PI;
        ctrl.linear_acc = declare_parameter("linear_acc", ctrl.linear_acc);
        ctrl.linear_dec = declare_parameter("linear_dec", ctrl.linear_dec);
        ctrl.max_steer_vel = declare_parameter("max_steer_vel", ctrl.max_steer_vel);
        hysteresis_deg = declare_parameter("steer_hysteresis_deg", hysteresis_deg);
        ctrl.steer_track_limit = declare_parameter("steer_track_limit", ctrl.steer_track_limit);
        ctrl.creep_speed_max = declare_parameter("creep_speed_max", ctrl.creep_speed_max);
        ctrl.wait_stop_timeout = declare_parameter("wait_stop_timeout", ctrl.wait_stop_timeout);
        ctrl.align_timeout = declare_parameter("align_timeout", ctrl.align_timeout);
        ctrl.creep_timeout = declare_parameter("creep_timeout", ctrl.creep_timeout);
        ctrl.max_start_attempts = declare_parameter("max_start_attempts", ctrl.max_start_attempts);
        ctrl.drive_track_abs_tol = declare_parameter("drive_track_abs_tol", ctrl.drive_track_abs_tol);
        ctrl.drive_track_rel_tol = declare_parameter("drive_track_rel_tol", ctrl.drive_track_rel_tol);
        ctrl.drive_fault_time = declare_parameter("drive_fault_time", ctrl.drive_fault_time);
        ctrl.steer_hysteresis = hysteresis_deg * M_PI / 180.0;

        if (control_rate_hz_ <= 0.0)
        {
            RCLCPP_WARN(get_logger(), "control_rate_hz <= 0.0, using 100.0");
            control_rate_hz_ = 100.0;
        }
        if (cmd_timeout_sec_ <= 0.0 || joint_state_timeout_sec_ <= 0.0)
        {
            RCLCPP_ERROR(get_logger(), "cmd_timeout_sec and joint_state_timeout_sec must be > 0");
            return false;
        }
        if (pose_cov_.size() != 6 || twist_cov_.size() != 6)
        {
            RCLCPP_ERROR(get_logger(), "pose/twist_covariance_diagonal must have 6 elements");
            return false;
        }
        if (module_x.empty() || module_x.size() != module_y.size())
        {
            RCLCPP_ERROR(get_logger(), "module_x and module_y must be non-empty and the same length");
            return false;
        }
        if (wheel_diameter_ <= 0.0)
        {
            RCLCPP_ERROR(get_logger(), "wheel_diameter must be > 0");
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
                    RCLCPP_ERROR(get_logger(), "drive_type 'tricycle' needs exactly one module");
                    return false;
                }
                model_ = std::make_shared<qd_kinematics::TricycleKinematics>(modules[0], fixed_axle_x);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "unknown drive_type '%s' (swerve | tricycle)", drive_type.c_str());
                return false;
            }
            controller_ = std::make_unique<qd_kinematics::SteerDriveController>(model_, ctrl);
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(get_logger(), "invalid kinematics geometry: %s", e.what());
            return false;
        }
        num_modules_ = model_->numModules();

        if (steer_joints_.size() != num_modules_ || drive_joints_.size() != num_modules_)
        {
            RCLCPP_ERROR(get_logger(), "steer_joints/drive_joints must have %zu entries", num_modules_);
            return false;
        }

        // Topics are relative so they can be remapped from the launch file.
        pub_steer_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("steer_controller/commands", 10);
        pub_drive_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("drive_controller/commands", 10);
        pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_debug_state_ = create_publisher<std_msgs::msg::String>("~/debug_state", 10);
        pub_fault_ = create_publisher<std_msgs::msg::String>("~/fault", rclcpp::QoS(1).transient_local());
        sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&KinematicsNode::receiveCmd, this, std::placeholders::_1));
        sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(), std::bind(&KinematicsNode::receiveJointState, this, std::placeholders::_1));
        srv_clear_fault_ = create_service<std_srvs::srv::Trigger>("~/clear_fault", [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) { clearFault(*req, *res); });
        publishFault();

        control_timer_ =
            rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate_hz_), std::bind(&KinematicsNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "qd_kinematics node initialized");
        RCLCPP_INFO(get_logger(), "  drive_type: %s (%zu module%s)", model_->name().c_str(), num_modules_, num_modules_ > 1 ? "s" : "");
        RCLCPP_INFO(get_logger(), "  use_sim_time: %s", get_parameter("use_sim_time").as_bool() ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  control_rate_hz: %.1f", control_rate_hz_);
        RCLCPP_INFO(get_logger(), "  cmd_timeout_sec: %.2f, joint_state_timeout_sec: %.2f", cmd_timeout_sec_, joint_state_timeout_sec_);
        RCLCPP_INFO(get_logger(), "  steer_hysteresis_deg: %.1f", hysteresis_deg);
        return true;
    }

  private:
    void receiveCmd(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
    {
        last_cmd_time_ = now();
        has_cmd_ = true;
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z))
        {
            ++rejected_cmd_;
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "cmd_vel is not finite: taken as a zero command");
            target_cmd_ = geometry_msgs::msg::Twist();
            return;
        }
        target_cmd_ = *msg;
    }

    void rejectJointState(const char *why)
    {
        ++rejected_feedback_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states rejected: %s", why);
    }

    // The ROS clock went back (Gazebo world reset, bag restart): odometry, stamp history and the state
    // machine start over. Called with the current time by every callback.
    void checkClock(const rclcpp::Time &now)
    {
        if (has_last_now_ && now < last_now_)
        {
            RCLCPP_WARN(get_logger(), "ROS time jumped back %.3f s: resetting odometry and the drive state machine",
                     (last_now_ - now).seconds());
            odom_.reset();
            stamps_.reset();
            controller_->reset();
            has_last_control_time_ = false;
            has_state_ = false;
        }
        last_now_ = now;
        has_last_now_ = true;
    }

    void receiveJointState(const sensor_msgs::msg::JointState::ConstSharedPtr &msg)
    {
        const rclcpp::Time now = this->now();
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
        if (rclcpp::Time(msg->header.stamp).nanoseconds() == 0)
        {
            rejectJointState("no stamp (the measurement time is required)");
            return;
        }
        const rclcpp::Time meas_time = msg->header.stamp;
        if ((meas_time - now).seconds() > stamp_future_tolerance_sec_)
        {
            ++rejected_future_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states rejected: stamp %.3f s in the future (clock offset?)",
                              (meas_time - now).seconds());
            return;
        }
        const StampMonitor::Result r = stamps_.check(meas_time.seconds());
        if (!StampMonitor::accepted(r))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states dropped: %s stamp (duplicates %lu, out of order %lu)",
                              StampMonitor::name(r), static_cast<unsigned long>(stamps_.duplicates),
                              static_cast<unsigned long>(stamps_.out_of_order));
            return;
        }
        if (r == StampMonitor::Result::GAP)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states gap: odometry is not integrated across it (%lu gaps so far)",
                              static_cast<unsigned long>(stamps_.gaps));
        }

        // Odometry always follows the measurement time ...
        publishOdometry(state, meas_time);

        // ... but only a recent sample may be used for control.
        const double age = (now - meas_time).seconds();
        if (age > joint_state_timeout_sec_)
        {
            ++late_feedback_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states %.3f s old on arrival: not used for control", age);
            return;
        }
        latest_state_ = state;
        has_state_ = true;
        last_feedback_time_ = now;
    }

    void publishOdometry(const std::vector<ModuleState> &state, const rclcpp::Time &meas_time)
    {
        odom_.update(model_->bodyTwist(state), meas_time.seconds());
        const auto &pose = odom_.pose();
        const auto &twist = odom_.twist();

        nav_msgs::msg::Odometry odom;
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
        pub_odometry_->publish(odom);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header = odom.header;
            t.child_frame_id = base_frame_;
            t.transform.translation.x = pose.x;
            t.transform.translation.y = pose.y;
            t.transform.rotation = odom.pose.pose.orientation;
            odom_broadcaster_.sendTransform(t);
        }
    }

    bool feedbackFresh(const rclcpp::Time &now) const
    {
        const double age = (now - last_feedback_time_).seconds();
        return has_state_ && age >= 0.0 && age <= joint_state_timeout_sec_;
    }

    void publishCommand(const std::vector<double> &steer, const std::vector<double> &drive)
    {
        std_msgs::msg::Float64MultiArray steer_cmd;
        std_msgs::msg::Float64MultiArray drive_cmd;
        steer_cmd.data = steer;
        drive_cmd.data = drive;
        pub_steer_cmd_->publish(steer_cmd);
        pub_drive_cmd_->publish(drive_cmd);
        last_steer_cmd_ = steer;
    }

    void controlLoop()
    {
        const rclcpp::Time now_t = now();
        checkClock(now_t);
        double dt = 1.0 / control_rate_hz_;
        if (has_last_control_time_)
        {
            dt = (now_t - last_control_time_).seconds();
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

        geometry_msgs::msg::Twist cmd_msg;
        const bool cmd_timed_out = !has_cmd_ || (now_t - last_cmd_time_).seconds() > cmd_timeout_sec_;
        if (!cmd_timed_out)
        {
            cmd_msg = target_cmd_;
        }

        if (!feedbackFresh(now_t))
        {
            if (!feedback_lost_)
            {
                RCLCPP_ERROR(get_logger(), "joint_states not received for %.2f s: commanding zero wheel speed and holding the steering",
                          (now_t - last_feedback_time_).seconds());
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
            RCLCPP_WARN(get_logger(), "joint_states fresh again: restarting with stop-steer-go");
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
                RCLCPP_ERROR(get_logger(), "drive FAULT: %s. Wheels stopped; call ~clear_fault after checking the drive units.",
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

    bool clearFault(std_srvs::srv::Trigger::Request &, std_srvs::srv::Trigger::Response &res)
    {
        if (!controller_->faulted())
        {
            res.success = true;
            res.message = "no fault";
            return true;
        }
        if (!feedbackFresh(now()))
        {
            res.success = false;
            res.message = "joint_states are not fresh; the fault is kept";
            return true;
        }
        RCLCPP_WARN(get_logger(), "drive FAULT cleared by request (was: %s)", controller_->faultReason().c_str());
        controller_->clearFault();
        reported_fault_ = false;
        publishFault();
        res.success = true;
        res.message = "fault cleared";
        return true;
    }

    void publishFault()
    {
        std_msgs::msg::String msg;
        msg.data = controller_->faultReason();
        pub_fault_->publish(msg);
    }

    void publishDebug(const geometry_msgs::msg::Twist &cmd, bool cmd_timed_out, const rclcpp::Time &now)
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
            << " fb_age=" << (has_state_ ? (now - last_feedback_time_).seconds() : -1.0)
            << " rejected(fb,future,late,cmd)=(" << rejected_feedback_ << "," << rejected_future_ << ","
            << late_feedback_ << "," << rejected_cmd_ << ")"
            << " stamps(dup,order,gap)=(" << stamps_.duplicates << "," << stamps_.out_of_order << ","
            << stamps_.gaps << ")";
        if (controller_->faulted())
        {
            oss << " fault=\"" << controller_->faultReason() << "\"";
        }

        std_msgs::msg::String dbg_msg;
        dbg_msg.data = oss.str();
        pub_debug_state_->publish(dbg_msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", dbg_msg.data.c_str());
    }


    std::shared_ptr<qd_kinematics::KinematicsModel> model_;
    std::unique_ptr<qd_kinematics::SteerDriveController> controller_;
    qd_kinematics::OdometryIntegrator odom_;
    StampMonitor stamps_;
    size_t num_modules_{0};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steer_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_drive_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fault_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_fault_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    tf2_ros::TransformBroadcaster odom_broadcaster_{*this};

    geometry_msgs::msg::Twist target_cmd_;
    std::vector<ModuleState> latest_state_;
    std::vector<double> last_steer_cmd_;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_now_{0, 0, RCL_ROS_TIME};

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
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsNode>();
    if (!node->init()) { rclcpp::shutdown(); return 1; }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
