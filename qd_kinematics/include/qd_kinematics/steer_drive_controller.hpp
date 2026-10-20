#ifndef QD_KINEMATICS__STEER_DRIVE_CONTROLLER_HPP_
#define QD_KINEMATICS__STEER_DRIVE_CONTROLLER_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "qd_kinematics/kinematics_model.hpp"

namespace qd_kinematics
{

struct ControllerParams
{
    // Acceleration / steer-rate limits.
    double linear_acc{0.10};   // [m/s^2]
    double linear_dec{0.25};   // [m/s^2]
    double max_steer_vel{1.5}; // [rad/s]

    // Steer targets are folded into [-pi/2, pi/2) by flipping the drive direction.
    // Near the +-pi/2 boundary (pure lateral motion) the fold would flip the target by
    // pi on every sign change of the tiny longitudinal component. The previous fold is
    // kept until the target leaves [-pi/2 - h, pi/2 + h]. Set 0 to disable; the steer
    // hardware must reach +-(pi/2 + h).
    double steer_hysteresis{10.0 * M_PI / 180.0}; // [rad]

    // Steering range: steer commands stay within [-steer_limit, steer_limit] and move through that
    // interval, never across +-pi (the steer hardware cannot). Needs pi/2 + steer_hysteresis <=
    // steer_limit. 0 = continuous steering, which turns the shortest way round.
    double steer_limit{0.0}; // [rad]

    // Drive wheel speed limit. A command that needs a faster wheel has every wheel speed scaled by the
    // same factor: same steer angles and path curvature, lower speed. 0 = no limit.
    double max_wheel_speed{0.0}; // [m/s]

    // Steer command is rate-limited from the last command, not from the measured angle,
    // so small per-tick steps still accumulate at high control rates. This bounds how far
    // the command may lead the measured angle.
    double steer_track_limit{0.3}; // [rad]

    // Direction-vector based alignment threshold [rad].
    double dir_err_thresh{2.0 * M_PI / 180.0};
    // Below this target speed, direction check is skipped.
    double dir_check_min_speed{0.03};
    // Wheel speed considered effectively stopped [m/s].
    double speed_zero_tol{0.02};
    // Per-wheel speed synchronization thresholds in CREEP.
    double speed_sync_abs_tol{0.04};
    double speed_sync_rel_tol{0.15};
    double speed_sync_floor{0.05};
    // Low-pass for measured wheel speed used by mode gating/balance.
    double vel_meas_filter_alpha{0.9};
    // DRIVE mode wheel-speed balance when all wheel targets are almost equal.
    double drive_balance_gain{0.35};
    double drive_balance_max{0.10};
    double drive_balance_target_gap{0.03};
    double drive_balance_min_speed{0.05};
    double drive_balance_wz_gate{0.05};
    // Slew-rate limit for balance correction to avoid command spikes.
    double drive_balance_corr_slew{0.60};

    // Mode transition confirmation windows [s].
    double stop_hold_sec{0.10};
    double align_hold_sec{1.00};
    double sync_hold_sec{1.00};
    // CREEP speed cap [m/s] before full DRIVE release.
    double creep_speed_max{0.08};

    // Start-up supervision. CREEP is left for DRIVE only when the MEASURED wheel speeds follow the
    // creep reference; the commanded speeds converging proves nothing about the drive units.
    // WAIT_STOP: the wheels must stop within (fastest commanded wheel speed / linear_dec +
    // wait_stop_timeout) seconds, otherwise FAULT (a drive that does not stop is not retried).
    double wait_stop_timeout{1.0}; // [s]
    // STEER_ALIGN must hold alignment within align_timeout and CREEP must sync within creep_timeout.
    // A miss goes back to WAIT_STOP for another start attempt; after max_start_attempts misses
    // without reaching DRIVE the controller enters FAULT.
    double align_timeout{5.0}; // [s]
    double creep_timeout{3.0}; // [s]
    int max_start_attempts{2};
    // DRIVE: a wheel whose measured speed stays further than max(drive_track_abs_tol,
    // drive_track_rel_tol * |command|) from its command for drive_fault_time -> FAULT
    // (drive unit not responding, or only some modules responding).
    double drive_track_abs_tol{0.15}; // [m/s]
    double drive_track_rel_tol{0.5};
    double drive_fault_time{1.0}; // [s]
};

// Stop-steer-go controller shared by every KinematicsModel.
// DRIVE -> (misaligned) WAIT_STOP -> (stopped) STEER_ALIGN -> (aligned) CREEP -> (synced) DRIVE
// FAULT (from the start-up supervision or the DRIVE tracking check above) commands zero wheel speed
// and holds the measured steer angles. It is latched: neither new commands nor reset() clear it,
// only clearFault().
class SteerDriveController
{
  public:
    enum class Mode
    {
        DRIVE,
        WAIT_STOP,
        STEER_ALIGN,
        CREEP,
        FAULT
    };

    struct DebugState
    {
        Mode mode{Mode::DRIVE};
        bool align_ok{true};
        bool speed_sync_ok{true};
        bool drive_stopped{true};
        bool near_zero_cmd{true};
        bool drive_balance_active{false};
        double stop_time{0.0};
        double align_time{0.0};
        double sync_time{0.0};
        double phase_time{0.0};
        int start_attempts{0};
        bool verify_start{false};
        std::vector<double> dir_err;
        std::vector<double> th_target;
        std::vector<double> th_current;
        std::vector<double> v_target;
        std::vector<double> v_current;
        std::vector<double> v_cmd;
        std::vector<double> balance_corr;
    };

    SteerDriveController(std::shared_ptr<const KinematicsModel> model, const ControllerParams &params);

    // One control step. `measured` and `command` hold one entry per module.
    // dt is the actual elapsed time since the previous call [s].
    void update(const Twist2D &cmd, const std::vector<ModuleState> &measured, double dt,
                std::vector<ModuleState> &command);

    // Restarts the state machine (e.g. after a clock jump or a feedback dropout). Keeps a FAULT.
    // The next motion goes through WAIT_STOP -> STEER_ALIGN -> CREEP even if the wheels already point
    // the right way, so the measured wheel speeds are checked again before DRIVE.
    void reset();

    // Leaves FAULT; the next motion goes through the full, measured start sequence (as after reset()).
    void clearFault();

    bool faulted() const
    {
        return mode_ == Mode::FAULT;
    }

    // Why the controller entered FAULT (empty when not faulted).
    const std::string &faultReason() const
    {
        return fault_reason_;
    }

    const KinematicsModel &model() const
    {
        return *model_;
    }

    const DebugState &debugState() const
    {
        return debug_;
    }

    static const char *modeName(Mode mode);

  private:
    double foldTarget(std::size_t i, double raw_angle, double &speed) const;
    double rampVelocity(double current, double target, double dt) const;
    double limitSteer(std::size_t i, double measured, double target, double dt);
    void setMode(Mode mode);
    void enterFault(const std::string &reason);
    void startAttemptFailed(const std::string &what);

    std::shared_ptr<const KinematicsModel> model_;
    ControllerParams p_;

    Mode mode_{Mode::DRIVE};
    bool initialized_{false};

    std::vector<double> v_meas_filt_;
    std::vector<double> v_cmd_state_;
    std::vector<double> th_cmd_state_;
    std::vector<double> th_target_prev_;
    std::vector<double> balance_corr_state_;

    double stop_time_{0.0};
    double align_time_{0.0};
    double sync_time_{0.0};
    double phase_time_{0.0};    // time in the current mode [s]
    double phase_deadline_{0.0}; // WAIT_STOP / STEER_ALIGN / CREEP time limit [s]
    int start_attempts_{0};
    bool verify_start_{false}; // next motion must pass CREEP (set by reset() and clearFault())
    std::vector<double> track_time_; // per wheel: time off its command in DRIVE [s]
    std::string fault_reason_;

    DebugState debug_;
};

// Fold `theta` into [-pi/2, pi/2), flipping `speed` once per pi folded.
double foldHalfPi(double theta, double &speed);

double normalizeAngle(double a);

} // namespace qd_kinematics

#endif // QD_KINEMATICS__STEER_DRIVE_CONTROLLER_HPP_
