#ifndef QD_KINEMATICS__ODOMETRY_INTEGRATOR_HPP_
#define QD_KINEMATICS__ODOMETRY_INTEGRATOR_HPP_

#include "qd_kinematics/kinematics_model.hpp"

namespace qd_kinematics
{

struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

// Dead-reckoning with trapezoidal (mean of previous and current twist) integration.
class OdometryIntegrator
{
  public:
    // Gaps longer than max_dt (sensor dropout, paused sim) are not integrated.
    void setMaxDt(double max_dt)
    {
        if (max_dt > 0.0)
        {
            max_dt_ = max_dt;
        }
    }

    // Returns true when the pose was advanced. Non-increasing stamps and non-finite input are ignored.
    bool update(const Twist2D &twist, double stamp_sec);

    void reset(const Pose2D &pose = Pose2D{});

    const Pose2D &pose() const
    {
        return pose_;
    }

    const Twist2D &twist() const
    {
        return prev_twist_;
    }

  private:
    Pose2D pose_;
    Twist2D prev_twist_;
    double prev_stamp_{0.0};
    bool has_prev_{false};
    double max_dt_{0.1};
};

} // namespace qd_kinematics

#endif // QD_KINEMATICS__ODOMETRY_INTEGRATOR_HPP_
