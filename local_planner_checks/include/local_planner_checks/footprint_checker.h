#ifndef LOCAL_PLANNER_CHECKS_FOOTPRINT_CHECKER_H_
#define LOCAL_PLANNER_CHECKS_FOOTPRINT_CHECKER_H_

#include <array>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace local_planner_checks
{

struct Pose2D
{
    double x = 0;
    double y = 0;
    double yaw = 0;
};

// What the footprint hits. Only NONE is safe: an unobserved cell (NO_INFORMATION) or a pose outside the
// costmap is treated like an obstacle.
enum class Collision
{
    NONE,
    LETHAL,
    UNKNOWN,
    OFF_MAP
};

const char *toString(Collision c);

struct CheckResult
{
    Collision collision = Collision::NONE;
    Pose2D pose;          // first pose that is not free
    double distance = 0;  // [m] travelled by the footprint origin up to that pose
    double angle = 0;     // [rad] turned up to that pose

    bool free() const
    {
        return collision == Collision::NONE;
    }
};

// Swept-footprint collision checks on a costmap, shared by the local planners so that they judge
// collisions the same way. Every check covers the whole footprint polygon (not just its outline) at
// the start pose, at the end pose and at intermediate poses spaced so that no footprint point moves
// more than half a costmap cell between two of them (the corners of a turning rectangle included).
// The footprint may be any simple polygon in the robot frame, convex or not (e.g. a pallet truck's body
// and fork), and need not be centred on the robot frame origin, about which the robot turns.
// The caller holds the costmap mutex.
class FootprintChecker
{
  public:
    FootprintChecker(nav2_costmap_2d::Costmap2D &costmap, std::vector<geometry_msgs::msg::Point> footprint);

    Collision checkPose(const Pose2D &pose) const;

    // Constant body twist (vx, vy [m/s], wz [rad/s] in the robot frame) from `start` for `duration` [s].
    CheckResult checkTwist(const Pose2D &start, double vx, double vy, double wz, double duration) const;

    // Turning in place by `angle` [rad] (signed) from `start`.
    CheckResult checkRotation(const Pose2D &start, double angle) const;

    // A path (poses with the heading the robot has there), from path[0] for `length` [m] of arc length,
    // interpolated between the poses.
    CheckResult checkPath(const std::vector<Pose2D> &path, double length) const;

    // Distance of the farthest footprint point from the origin [m].
    double circumscribedRadius() const
    {
        return m_radius;
    }

  private:
    // Cells covered by the polygon (map cell corners `polygon`, the same vertices in continuous map
    // coordinates `vertices`): the outline cells and every cell whose centre lies inside.
    void fillPolygon(const std::vector<nav2_costmap_2d::MapLocation> &polygon,
                     const std::vector<std::array<double, 2>> &vertices,
                     std::vector<nav2_costmap_2d::MapLocation> &cells) const;

    nav2_costmap_2d::Costmap2D &m_costmap;
    std::vector<geometry_msgs::msg::Point> m_footprint;
    double m_radius = 0;
    double m_step = 0; // [m] largest allowed motion of a footprint point between two checked poses
    bool m_convex = true; // convex footprints use costmap_2d's faster convex fill
};

// Distance the robot needs to stop: it keeps max(|v_measured|, |v_command|) for `latency` [s], then
// brakes at `dec` [m/s^2]; plus `margin` [m]. The measured speed counts because the base cannot stop
// faster than it is actually moving.
double stoppingDistance(double v_measured, double v_command, double dec, double latency, double margin);

// Pose after driving a constant body twist for t seconds.
Pose2D integrateTwist(const Pose2D &start, double vx, double vy, double wz, double t);

// A goal counts as reached only with the robot at the goal: actual XY distance within xy_tolerance
// (not just the distance left along the path), heading within yaw_tolerance, and stopped.
bool goalReached(const Pose2D &robot, const Pose2D &goal, bool stopped, double xy_tolerance, double yaw_tolerance);

} // namespace local_planner_checks

#endif // LOCAL_PLANNER_CHECKS_FOOTPRINT_CHECKER_H_
