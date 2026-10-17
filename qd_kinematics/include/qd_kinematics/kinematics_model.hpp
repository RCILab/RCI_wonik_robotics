#ifndef QD_KINEMATICS__KINEMATICS_MODEL_HPP_
#define QD_KINEMATICS__KINEMATICS_MODEL_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace qd_kinematics
{

// Planar body twist expressed in the base frame.
struct Twist2D
{
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
};

// One steer+drive module: steer angle [rad] and wheel ground speed [m/s].
struct ModuleState
{
    double angle{0.0};
    double speed{0.0};
};

// Module (steered drive wheel) contact point in the base frame [m].
struct ModulePosition
{
    double x{0.0};
    double y{0.0};
};

using Vec2 = std::array<double, 2>;

// Common interface for every drive layout (Quad Drive #1/#2/#3, Single Drive).
// It only describes geometry; the stop-steer-go state machine and the
// acceleration limits live in SteerDriveController and are shared.
class KinematicsModel
{
  public:
    virtual ~KinematicsModel() = default;

    virtual std::string name() const = 0;

    std::size_t numModules() const
    {
        return modules_.size();
    }

    const std::vector<ModulePosition> &modules() const
    {
        return modules_;
    }

    // Twist the platform can actually produce (e.g. a tricycle cannot move sideways).
    virtual Twist2D feasible(const Twist2D &twist) const
    {
        return twist;
    }

    // Inverse kinematics: ground velocity vector each module must have for `twist`.
    virtual void moduleVelocities(const Twist2D &twist, std::vector<Vec2> &velocities) const = 0;

    // Forward kinematics: body twist from measured module states.
    virtual Twist2D bodyTwist(const std::vector<ModuleState> &states) const = 0;

  protected:
    std::vector<ModulePosition> modules_;
};

// N >= 2 independently steered drive modules at arbitrary positions, plus passive casters.
// Covers Quad Drive #1 (lateral pair), #2 (diagonal pair), #3 (inline pair, any base origin).
class SwerveKinematics : public KinematicsModel
{
  public:
    explicit SwerveKinematics(const std::vector<ModulePosition> &modules);

    std::string name() const override
    {
        return "swerve";
    }

    void moduleVelocities(const Twist2D &twist, std::vector<Vec2> &velocities) const override;

    // Rigid-body least squares over all modules; exact for consistent measurements.
    Twist2D bodyTwist(const std::vector<ModuleState> &states) const override;

  private:
    ModulePosition centroid_;
    double spread_{0.0}; // sum |r_i - centroid|^2
};

// One steered drive wheel plus a passive fixed axle (Single Drive / pallet truck).
// The fixed axle lies along the base y axis at x = fixed_axle_x, so points on it
// cannot slide sideways: vy = -wz * fixed_axle_x.
class TricycleKinematics : public KinematicsModel
{
  public:
    TricycleKinematics(const ModulePosition &drive_wheel, double fixed_axle_x);

    std::string name() const override
    {
        return "tricycle";
    }

    Twist2D feasible(const Twist2D &twist) const override;

    void moduleVelocities(const Twist2D &twist, std::vector<Vec2> &velocities) const override;

    Twist2D bodyTwist(const std::vector<ModuleState> &states) const override;

  private:
    double fixed_axle_x_{0.0};
    double wheelbase_{0.0}; // drive.x - fixed_axle_x
};

} // namespace qd_kinematics

#endif // QD_KINEMATICS__KINEMATICS_MODEL_HPP_
