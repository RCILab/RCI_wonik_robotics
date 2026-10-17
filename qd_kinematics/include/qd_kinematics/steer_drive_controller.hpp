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
};

// Stop-steer-go controller shared by every KinematicsModel.
// DRIVE -> (misaligned) WAIT_STOP -> (stopped) STEER_ALIGN -> (aligned) CREEP -> (synced) DRIVE
class SteerDriveController
{
  public:
    enum class Mode
    {
        DRIVE,
        WAIT_STOP,
        STEER_ALIGN,
        CREEP
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

    void reset();

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

    DebugState debug_;
};

// Fold `theta` into [-pi/2, pi/2), flipping `speed` once per pi folded.
double foldHalfPi(double theta, double &speed);

double normalizeAngle(double a);

} // namespace qd_kinematics

#endif // QD_KINEMATICS__STEER_DRIVE_CONTROLLER_HPP_
