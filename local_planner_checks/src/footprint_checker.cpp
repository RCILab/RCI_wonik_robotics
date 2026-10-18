#include "local_planner_checks/footprint_checker.h"

#include <algorithm>
#include <cmath>

#include <angles/angles.h>
#include <nav2_costmap_2d/cost_values.hpp>

namespace local_planner_checks
{

const char *toString(Collision c)
{
    switch (c)
    {
    case Collision::NONE:
        return "free";
    case Collision::LETHAL:
        return "obstacle";
    case Collision::UNKNOWN:
        return "unobserved space";
    case Collision::OFF_MAP:
        return "outside the costmap";
    }
    return "?";
}

FootprintChecker::FootprintChecker(nav2_costmap_2d::Costmap2D &costmap, std::vector<geometry_msgs::msg::Point> footprint)
    : m_costmap(costmap), m_footprint(std::move(footprint))
{
    for (const auto &p : m_footprint)
    {
        m_radius = std::max(m_radius, std::hypot(p.x, p.y));
    }
    m_step = 0.5 * m_costmap.getResolution();

    // convex: every turn along the outline goes the same way
    const std::size_t n = m_footprint.size();
    int turns = 0;
    for (std::size_t i = 0; i < n && n >= 3; ++i)
    {
        const auto &a = m_footprint[i];
        const auto &b = m_footprint[(i + 1) % n];
        const auto &c = m_footprint[(i + 2) % n];
        const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        if (std::fabs(cross) > 1e-12)
        {
            const int sign = cross > 0 ? 1 : -1;
            if (turns != 0 && sign != turns)
            {
                m_convex = false;
                break;
            }
            turns = sign;
        }
    }
}

void FootprintChecker::fillPolygon(const std::vector<nav2_costmap_2d::MapLocation> &polygon,
                                   const std::vector<std::array<double, 2>> &vertices,
                                   std::vector<nav2_costmap_2d::MapLocation> &cells) const
{
    m_costmap.polygonOutlineCells(polygon, cells);
    unsigned int x0 = polygon.front().x;
    unsigned int x1 = x0;
    unsigned int y0 = polygon.front().y;
    unsigned int y1 = y0;
    for (const auto &p : polygon)
    {
        x0 = std::min(x0, p.x);
        x1 = std::max(x1, p.x);
        y0 = std::min(y0, p.y);
        y1 = std::max(y1, p.y);
    }
    const std::size_t n = vertices.size();
    for (unsigned int y = y0; y <= y1; ++y)
    {
        const double cy = y + 0.5;
        for (unsigned int x = x0; x <= x1; ++x)
        {
            const double cx = x + 0.5;
            bool inside = false; // even-odd rule
            for (std::size_t i = 0, j = n - 1; i < n; j = i++)
            {
                const auto &a = vertices[i];
                const auto &b = vertices[j];
                if ((a[1] > cy) != (b[1] > cy) && cx < a[0] + (cy - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
                {
                    inside = !inside;
                }
            }
            if (inside)
            {
                cells.push_back({x, y});
            }
        }
    }
}

Collision FootprintChecker::checkPose(const Pose2D &pose) const
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!m_costmap.worldToMap(pose.x, pose.y, mx, my))
    {
        return Collision::OFF_MAP;
    }
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double res = m_costmap.getResolution();
    std::vector<nav2_costmap_2d::MapLocation> polygon;
    std::vector<std::array<double, 2>> vertices; // continuous map coordinates [cells]
    for (const auto &p : m_footprint)
    {
        const double wx = pose.x + c * p.x - s * p.y;
        const double wy = pose.y + s * p.x + c * p.y;
        nav2_costmap_2d::MapLocation loc;
        if (!m_costmap.worldToMap(wx, wy, loc.x, loc.y))
        {
            return Collision::OFF_MAP;
        }
        polygon.push_back(loc);
        vertices.push_back({(wx - m_costmap.getOriginX()) / res, (wy - m_costmap.getOriginY()) / res});
    }
    std::vector<nav2_costmap_2d::MapLocation> cells;
    if (polygon.size() < 3)
    {
        cells.push_back({mx, my}); // no footprint: the origin cell only
    }
    else if (m_convex)
    {
        m_costmap.convexFillCells(polygon, cells);
    }
    else
    {
        fillPolygon(polygon, vertices, cells);
    }
    bool unknown = false;
    for (const auto &cell : cells)
    {
        const unsigned char cost = m_costmap.getCost(cell.x, cell.y);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
            return Collision::LETHAL;
        }
        unknown = unknown || cost == nav2_costmap_2d::NO_INFORMATION;
    }
    return unknown ? Collision::UNKNOWN : Collision::NONE;
}

Pose2D integrateTwist(const Pose2D &start, double vx, double vy, double wz, double t)
{
    double dx = 0;
    double dy = 0;
    const double th = wz * t;
    if (std::fabs(th) < 1e-9)
    {
        dx = vx * t;
        dy = vy * t;
    }
    else
    {
        dx = (vx * std::sin(th) + vy * (std::cos(th) - 1)) / wz;
        dy = (vx * (1 - std::cos(th)) + vy * std::sin(th)) / wz;
    }
    const double c = std::cos(start.yaw);
    const double s = std::sin(start.yaw);
    return {start.x + c * dx - s * dy, start.y + s * dx + c * dy, angles::normalize_angle(start.yaw + th)};
}

CheckResult FootprintChecker::checkTwist(const Pose2D &start, double vx, double vy, double wz, double duration) const
{
    CheckResult result;
    const double point_speed = std::hypot(vx, vy) + std::fabs(wz) * m_radius; // fastest footprint point
    duration = std::max(duration, 0.0);
    int steps = 0;
    if (point_speed > 1e-9 && duration > 0)
    {
        steps = static_cast<int>(std::ceil(point_speed * duration / m_step));
        steps = std::min(std::max(steps, 1), 2000);
    }
    for (int k = 0; k <= steps; ++k) // k = 0: start pose, k = steps: end pose
    {
        const double t = steps > 0 ? duration * k / steps : 0.0;
        const Pose2D pose = integrateTwist(start, vx, vy, wz, t);
        const Collision c = checkPose(pose);
        if (c != Collision::NONE)
        {
            result.collision = c;
            result.pose = pose;
            result.distance = std::hypot(vx, vy) * t;
            result.angle = wz * t;
            return result;
        }
    }
    return result;
}

CheckResult FootprintChecker::checkRotation(const Pose2D &start, double angle) const
{
    return checkTwist(start, 0, 0, angle >= 0 ? 1.0 : -1.0, std::fabs(angle));
}

CheckResult FootprintChecker::checkPath(const std::vector<Pose2D> &path, double length) const
{
    CheckResult result;
    if (path.empty())
    {
        return result;
    }
    auto check = [&](const Pose2D &pose, double travelled) {
        const Collision c = checkPose(pose);
        if (c != Collision::NONE)
        {
            result.collision = c;
            result.pose = pose;
            result.distance = travelled;
            return false;
        }
        return true;
    };
    if (!check(path.front(), 0))
    {
        return result;
    }
    double travelled = 0;
    for (std::size_t i = 0; i + 1 < path.size() && travelled < length; ++i)
    {
        const Pose2D &a = path[i];
        const Pose2D &b = path[i + 1];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        const double turn = angles::shortest_angular_distance(a.yaw, b.yaw);
        const double motion = seg + std::fabs(turn) * m_radius;
        const int steps = std::max(1, static_cast<int>(std::ceil(motion / m_step)));
        for (int k = 1; k <= steps; ++k)
        {
            double f = static_cast<double>(k) / steps;
            bool last = false;
            if (travelled + seg * f >= length) // end of the checked length: check exactly there
            {
                f = seg > 1e-9 ? (length - travelled) / seg : 1.0;
                last = true;
            }
            const Pose2D pose{a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, angles::normalize_angle(a.yaw + turn * f)};
            if (!check(pose, travelled + seg * f))
            {
                return result;
            }
            if (last)
            {
                return result;
            }
        }
        travelled += seg;
    }
    return result;
}

double stoppingDistance(double v_measured, double v_command, double dec, double latency, double margin)
{
    const double v = std::max(std::fabs(v_measured), std::fabs(v_command));
    return v * std::max(latency, 0.0) + v * v / (2 * std::max(dec, 1e-3)) + std::max(margin, 0.0);
}

bool goalReached(const Pose2D &robot, const Pose2D &goal, bool stopped, double xy_tolerance, double yaw_tolerance)
{
    return stopped && std::hypot(goal.x - robot.x, goal.y - robot.y) <= xy_tolerance &&
           std::fabs(angles::shortest_angular_distance(robot.yaw, goal.yaw)) <= yaw_tolerance;
}

} // namespace local_planner_checks
