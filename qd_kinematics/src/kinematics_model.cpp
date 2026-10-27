#include "qd_kinematics/kinematics_model.hpp"

#include <cmath>
#include <stdexcept>

namespace qd_kinematics
{

SwerveKinematics::SwerveKinematics(const std::vector<ModulePosition> &modules)
{
    if (modules.size() < 2)
    {
        throw std::invalid_argument("swerve kinematics needs at least 2 modules");
    }
    modules_ = modules;

    for (const auto &m : modules_)
    {
        centroid_.x += m.x;
        centroid_.y += m.y;
    }
    centroid_.x /= static_cast<double>(modules_.size());
    centroid_.y /= static_cast<double>(modules_.size());

    for (const auto &m : modules_)
    {
        const double dx = m.x - centroid_.x;
        const double dy = m.y - centroid_.y;
        spread_ += dx * dx + dy * dy;
    }
    if (spread_ < 1e-9)
    {
        throw std::invalid_argument("swerve modules must not all share one position");
    }
}

void SwerveKinematics::moduleVelocities(const Twist2D &twist, std::vector<Vec2> &velocities) const
{
    velocities.resize(modules_.size());
    for (std::size_t i = 0; i < modules_.size(); ++i)
    {
        velocities[i] = {twist.vx - twist.wz * modules_[i].y, twist.vy + twist.wz * modules_[i].x};
    }
}

Twist2D SwerveKinematics::bodyTwist(const std::vector<ModuleState> &states) const
{
    // Minimise sum |v_i - (v + w x r_i)|^2:
    //   w = sum (r_i - r_c) x (v_i - v_c) / sum |r_i - r_c|^2,   v = v_c - w x r_c
    const std::size_t n = modules_.size();
    std::vector<Vec2> v(n);
    Vec2 v_mean{0.0, 0.0};
    for (std::size_t i = 0; i < n; ++i)
    {
        v[i] = {states[i].speed * std::cos(states[i].angle), states[i].speed * std::sin(states[i].angle)};
        v_mean[0] += v[i][0];
        v_mean[1] += v[i][1];
    }
    v_mean[0] /= static_cast<double>(n);
    v_mean[1] /= static_cast<double>(n);

    double cross_sum = 0.0;
    for (std::size_t i = 0; i < n; ++i)
    {
        const double rx = modules_[i].x - centroid_.x;
        const double ry = modules_[i].y - centroid_.y;
        cross_sum += rx * (v[i][1] - v_mean[1]) - ry * (v[i][0] - v_mean[0]);
    }

    Twist2D out;
    out.wz = cross_sum / spread_;
    out.vx = v_mean[0] + out.wz * centroid_.y;
    out.vy = v_mean[1] - out.wz * centroid_.x;
    return out;
}

TricycleKinematics::TricycleKinematics(const ModulePosition &drive_wheel, double fixed_axle_x)
    : fixed_axle_x_(fixed_axle_x), wheelbase_(drive_wheel.x - fixed_axle_x)
{
    if (std::abs(wheelbase_) < 1e-6)
    {
        throw std::invalid_argument("tricycle drive wheel must not sit on the fixed axle");
    }
    modules_ = {drive_wheel};
}

Twist2D TricycleKinematics::feasible(const Twist2D &twist) const
{
    return {twist.vx, -twist.wz * fixed_axle_x_, twist.wz};
}

void TricycleKinematics::moduleVelocities(const Twist2D &twist, std::vector<Vec2> &velocities) const
{
    const Twist2D t = feasible(twist);
    velocities.resize(1);
    velocities[0] = {t.vx - t.wz * modules_[0].y, t.wz * wheelbase_};
}

Twist2D TricycleKinematics::bodyTwist(const std::vector<ModuleState> &states) const
{
    const double wvx = states[0].speed * std::cos(states[0].angle);
    const double wvy = states[0].speed * std::sin(states[0].angle);

    Twist2D out;
    out.wz = wvy / wheelbase_;
    out.vx = wvx + out.wz * modules_[0].y;
    out.vy = -out.wz * fixed_axle_x_;
    return out;
}

} // namespace qd_kinematics
