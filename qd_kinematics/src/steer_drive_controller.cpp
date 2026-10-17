#include "qd_kinematics/steer_drive_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace qd_kinematics
{

namespace
{

// Absorbs floating-point drift when accumulated dt is compared with a hold time.
constexpr double kTimeEps = 1e-9;

double signNonzero(double v)
{
    return (v < 0.0) ? -1.0 : 1.0;
}

double directionError(double th_now, double v_now, double th_tgt, double v_tgt, double sign_switch_speed)
{
    const double now_sign = (std::abs(v_now) < sign_switch_speed) ? signNonzero(v_tgt) : signNonzero(v_now);
    const double tgt_sign = signNonzero(v_tgt);

    const double dot = now_sign * tgt_sign * (std::cos(th_now) * std::cos(th_tgt) + std::sin(th_now) * std::sin(th_tgt));
    return std::acos(std::clamp(dot, -1.0, 1.0));
}

double slewToward(double state, double target, double max_step)
{
    return state + std::clamp(target - state, -max_step, max_step);
}

} // namespace

double foldHalfPi(double theta, double &speed)
{
    int k = 0;
    double r = std::remquo(theta, M_PI, &k);
    if (k & 1)
    {
        speed = -speed;
    }

    const double half_pi = 0.5 * M_PI;
    if (r >= half_pi)
    {
        r -= M_PI;
        speed = -speed;
    }
    if (r < -half_pi)
    {
        r += M_PI;
        speed = -speed;
    }
    return r;
}

double normalizeAngle(double a)
{
    return std::remainder(a, 2.0 * M_PI);
}

const char *SteerDriveController::modeName(Mode mode)
{
    switch (mode)
    {
    case Mode::DRIVE:
        return "DRIVE";
    case Mode::WAIT_STOP:
        return "WAIT_STOP";
    case Mode::STEER_ALIGN:
        return "STEER_ALIGN";
    case Mode::CREEP:
        return "CREEP";
    }
    return "UNKNOWN";
}

SteerDriveController::SteerDriveController(std::shared_ptr<const KinematicsModel> model,
                                           const ControllerParams &params)
    : model_(std::move(model)), p_(params)
{
    if (!model_ || model_->numModules() == 0)
    {
        throw std::invalid_argument("SteerDriveController needs a model with at least one module");
    }
    reset();
}

void SteerDriveController::reset()
{
    const std::size_t n = model_->numModules();
    mode_ = Mode::DRIVE;
    initialized_ = false;
    v_meas_filt_.assign(n, 0.0);
    v_cmd_state_.assign(n, 0.0);
    th_cmd_state_.assign(n, 0.0);
    th_target_prev_.assign(n, 0.0);
    balance_corr_state_.assign(n, 0.0);
    stop_time_ = 0.0;
    align_time_ = 0.0;
    sync_time_ = 0.0;
}

double SteerDriveController::foldTarget(std::size_t i, double raw_angle, double &speed) const
{
    double angle = foldHalfPi(raw_angle, speed);
    if (p_.steer_hysteresis <= 0.0)
    {
        return angle;
    }

    // The same wheel velocity is also reachable as (angle -+ pi, -speed). Keep that
    // alternative while it stays inside the widened band and is closer to the last target.
    const double alt = (angle < 0.0) ? angle + M_PI : angle - M_PI;
    const double ref = th_target_prev_[i];
    if (std::abs(alt) <= 0.5 * M_PI + p_.steer_hysteresis &&
        std::abs(normalizeAngle(alt - ref)) < std::abs(normalizeAngle(angle - ref)))
    {
        speed = -speed;
        return alt;
    }
    return angle;
}

double SteerDriveController::rampVelocity(double current, double target, double dt) const
{
    if (std::abs(target) < 0.01)
    {
        // Decelerate toward zero.
        return slewToward(current, target, p_.linear_dec * dt);
    }
    return slewToward(current, target, p_.linear_acc * dt);
}

double SteerDriveController::limitSteer(std::size_t i, double measured, double target, double dt)
{
    // Keep the command within steer_track_limit of the measured angle so it cannot run away
    // when the steer motor lags or is blocked.
    const double lead = normalizeAngle(th_cmd_state_[i] - measured);
    if (std::abs(lead) > p_.steer_track_limit)
    {
        th_cmd_state_[i] = measured + std::copysign(p_.steer_track_limit, lead);
    }

    // Step along the shortest path while staying on the joint's own (possibly unwrapped) branch.
    const double diff = normalizeAngle(target - th_cmd_state_[i]);
    const double max_step = p_.max_steer_vel * dt;
    th_cmd_state_[i] += std::clamp(diff, -max_step, max_step);
    return th_cmd_state_[i];
}

void SteerDriveController::update(const Twist2D &cmd, const std::vector<ModuleState> &measured, double dt,
                                  std::vector<ModuleState> &command)
{
    const std::size_t n = model_->numModules();
    if (measured.size() != n)
    {
        throw std::invalid_argument("measured module count does not match the kinematics model");
    }
    dt = std::clamp(dt, 1e-4, 0.1);

    if (!initialized_)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            th_cmd_state_[i] = measured[i].angle;
            th_target_prev_[i] = measured[i].angle;
        }
        initialized_ = true;
    }

    // ---- Geometric targets ----
    std::vector<Vec2> wheel_vel;
    model_->moduleVelocities(cmd, wheel_vel);

    bool all_zero = true;
    for (const auto &v : wheel_vel)
    {
        all_zero = all_zero && (std::hypot(v[0], v[1]) < 1e-6);
    }

    std::vector<double> th_target(n);
    std::vector<double> v_target(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        if (all_zero)
        {
            th_target[i] = measured[i].angle;
            v_target[i] = 0.0;
        }
        else
        {
            v_target[i] = std::hypot(wheel_vel[i][0], wheel_vel[i][1]);
            th_target[i] = foldTarget(i, std::atan2(wheel_vel[i][1], wheel_vel[i][0]), v_target[i]);
        }
        th_target_prev_[i] = th_target[i];
    }

    // ---- Measurements ----
    const double a = p_.vel_meas_filter_alpha;
    for (std::size_t i = 0; i < n; ++i)
    {
        v_meas_filt_[i] = a * v_meas_filt_[i] + (1.0 - a) * measured[i].speed;
    }

    std::vector<double> dir_err(n, 0.0);
    bool align_ok = true;
    bool all_meas_stopped = true;
    bool near_zero_cmd = true;
    double max_target = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        if (std::abs(v_target[i]) >= p_.dir_check_min_speed)
        {
            dir_err[i] = directionError(measured[i].angle, v_meas_filt_[i], th_target[i], v_target[i],
                                        p_.dir_check_min_speed);
            align_ok = align_ok && (dir_err[i] < p_.dir_err_thresh);
        }
        all_meas_stopped = all_meas_stopped && (std::abs(v_meas_filt_[i]) < p_.speed_zero_tol);
        near_zero_cmd = near_zero_cmd && (std::abs(v_target[i]) < p_.speed_zero_tol);
        max_target = std::max(max_target, std::abs(v_target[i]));
    }
    const bool is_misaligned = !align_ok;

    stop_time_ = all_meas_stopped ? stop_time_ + dt : 0.0;
    const bool drive_stopped = stop_time_ >= p_.stop_hold_sec - kTimeEps;

    const double creep_scale = (max_target > p_.creep_speed_max) ? (p_.creep_speed_max / max_target) : 1.0;

    // In CREEP, sync is checked against the creep-limited reference, not the full target.
    bool meas_sync_ok = true;
    bool cmd_sync_ok = true;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double ref = (mode_ == Mode::CREEP) ? v_target[i] * creep_scale : v_target[i];
        const double denom = std::max(std::abs(ref), p_.speed_sync_floor);

        const double meas_err = std::abs(v_meas_filt_[i] - ref);
        meas_sync_ok = meas_sync_ok && (meas_err < p_.speed_sync_abs_tol) && (meas_err / denom < p_.speed_sync_rel_tol);

        // Fallback: if the command has converged, release CREEP even when measured velocity
        // lags because of stiction/simulation artifacts.
        const double cmd_err = std::abs(v_cmd_state_[i] - ref);
        cmd_sync_ok = cmd_sync_ok && (cmd_err < p_.speed_sync_abs_tol) && (cmd_err / denom < p_.speed_sync_rel_tol);
    }
    const bool speed_sync_ok = meas_sync_ok || cmd_sync_ok;

    // ---- Mode transitions ----
    if (near_zero_cmd)
    {
        mode_ = Mode::DRIVE;
        align_time_ = 0.0;
        sync_time_ = 0.0;
    }

    switch (mode_)
    {
    case Mode::DRIVE:
        if (is_misaligned)
        {
            mode_ = Mode::WAIT_STOP;
            align_time_ = 0.0;
            sync_time_ = 0.0;
        }
        break;
    case Mode::WAIT_STOP:
        if (drive_stopped)
        {
            mode_ = Mode::STEER_ALIGN;
            align_time_ = 0.0;
            sync_time_ = 0.0;
        }
        break;
    case Mode::STEER_ALIGN:
        align_time_ = align_ok ? align_time_ + dt : 0.0;
        if (align_time_ >= p_.align_hold_sec - kTimeEps)
        {
            mode_ = Mode::CREEP;
            sync_time_ = 0.0;
        }
        break;
    case Mode::CREEP:
        if (is_misaligned)
        {
            mode_ = Mode::WAIT_STOP;
            align_time_ = 0.0;
            sync_time_ = 0.0;
            break;
        }
        sync_time_ = (align_ok && speed_sync_ok) ? sync_time_ + dt : 0.0;
        if (sync_time_ >= p_.sync_hold_sec - kTimeEps)
        {
            mode_ = Mode::DRIVE;
        }
        break;
    }

    // ---- Per-mode setpoints ----
    std::vector<double> final_th(n);
    std::vector<double> final_v(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        switch (mode_)
        {
        case Mode::DRIVE:
            final_th[i] = th_target[i];
            final_v[i] = v_target[i];
            break;
        case Mode::WAIT_STOP:
            final_th[i] = measured[i].angle;
            final_v[i] = 0.0;
            break;
        case Mode::STEER_ALIGN:
            final_th[i] = th_target[i];
            final_v[i] = 0.0;
            break;
        case Mode::CREEP:
            final_th[i] = th_target[i];
            final_v[i] = v_target[i] * creep_scale;
            break;
        }
    }

    // ---- DRIVE-mode wheel speed balance (straight/lateral motion only) ----
    bool balance_active = false;
    const double corr_step_max = p_.drive_balance_corr_slew * dt;
    if (mode_ == Mode::DRIVE && n >= 2)
    {
        double v_min = v_target[0];
        double v_max = v_target[0];
        bool same_direction = true;
        for (std::size_t i = 0; i < n; ++i)
        {
            v_min = std::min(v_min, v_target[i]);
            v_max = std::max(v_max, v_target[i]);
            same_direction = same_direction && (v_target[i] * v_target[0] > 0.0);
        }
        const bool similar_target = (v_max - v_min) < p_.drive_balance_target_gap;
        const bool enough_speed = max_target >= p_.drive_balance_min_speed;
        const bool low_yaw_rate = std::abs(cmd.wz) < p_.drive_balance_wz_gate;
        balance_active = same_direction && similar_target && enough_speed && low_yaw_rate;
    }

    if (balance_active)
    {
        double mean_meas = 0.0;
        for (double v : v_meas_filt_)
        {
            mean_meas += v;
        }
        mean_meas /= static_cast<double>(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            // Factor 2 keeps the original front/rear tuning: for N=2, 2*(v_i - mean) = v_f - v_r.
            const double desired =
                std::clamp(p_.drive_balance_gain * 2.0 * (v_meas_filt_[i] - mean_meas), -p_.drive_balance_max,
                           p_.drive_balance_max);
            balance_corr_state_[i] = slewToward(balance_corr_state_[i], desired, corr_step_max);
            final_v[i] -= balance_corr_state_[i];

            // Keep commanded sign consistent with kinematic target direction.
            final_v[i] = (v_target[0] >= 0.0) ? std::max(0.0, final_v[i]) : std::min(0.0, final_v[i]);
        }
    }
    else
    {
        // Smoothly release residual correction when gating is off.
        for (auto &c : balance_corr_state_)
        {
            c = slewToward(c, 0.0, corr_step_max);
        }
    }

    // ---- Rate limits ----
    command.resize(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        command[i].angle = limitSteer(i, measured[i].angle, final_th[i], dt);

        // Ramp from the last commanded velocity, not the measured one: with static friction
        // the measured speed stays at zero and a measured-based ramp would never break away.
        command[i].speed = rampVelocity(v_cmd_state_[i], final_v[i], dt);
        if (near_zero_cmd && std::abs(command[i].speed) < p_.speed_zero_tol)
        {
            command[i].speed = 0.0;
        }
        v_cmd_state_[i] = command[i].speed;
    }

    // ---- Debug ----
    debug_.mode = mode_;
    debug_.align_ok = align_ok;
    debug_.speed_sync_ok = speed_sync_ok;
    debug_.drive_stopped = drive_stopped;
    debug_.near_zero_cmd = near_zero_cmd;
    debug_.drive_balance_active = balance_active;
    debug_.stop_time = stop_time_;
    debug_.align_time = align_time_;
    debug_.sync_time = sync_time_;
    debug_.dir_err = dir_err;
    debug_.th_target = th_target;
    debug_.v_target = v_target;
    debug_.v_current = v_meas_filt_;
    debug_.balance_corr = balance_corr_state_;
    debug_.th_current.resize(n);
    debug_.v_cmd.resize(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        debug_.th_current[i] = measured[i].angle;
        debug_.v_cmd[i] = command[i].speed;
    }
}

} // namespace qd_kinematics
