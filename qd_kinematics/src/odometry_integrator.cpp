#include "qd_kinematics/odometry_integrator.hpp"

#include <cmath>

namespace qd_kinematics
{

void OdometryIntegrator::reset(const Pose2D &pose)
{
    pose_ = pose;
    prev_twist_ = Twist2D{};
    prev_stamp_ = 0.0;
    has_prev_ = false;
}

bool OdometryIntegrator::update(const Twist2D &twist, double stamp_sec)
{
    if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz) || !std::isfinite(stamp_sec))
    {
        return false; // dropped; the next valid sample continues from the last valid one
    }
    if (!has_prev_)
    {
        prev_twist_ = twist;
        prev_stamp_ = stamp_sec;
        has_prev_ = true;
        return false;
    }

    const double dt = stamp_sec - prev_stamp_;
    if (dt <= 0.0)
    {
        return false;
    }

    bool integrated = false;
    if (dt <= max_dt_)
    {
        const double vx = 0.5 * (prev_twist_.vx + twist.vx);
        const double vy = 0.5 * (prev_twist_.vy + twist.vy);
        const double wz = 0.5 * (prev_twist_.wz + twist.wz);
        const double yaw_mid = pose_.yaw + 0.5 * wz * dt;

        pose_.x += (vx * std::cos(yaw_mid) - vy * std::sin(yaw_mid)) * dt;
        pose_.y += (vx * std::sin(yaw_mid) + vy * std::cos(yaw_mid)) * dt;
        pose_.yaw = std::remainder(pose_.yaw + wz * dt, 2.0 * M_PI);
        integrated = true;
    }

    prev_twist_ = twist;
    prev_stamp_ = stamp_sec;
    return integrated;
}

} // namespace qd_kinematics
