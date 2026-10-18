#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include "local_planner_checks/footprint_checker.h"

using namespace local_planner_checks;

namespace
{

// cobot_v2 footprint (config/cobot_v2/footprint.yaml)
std::vector<geometry_msgs::msg::Point> footprint()
{
    std::vector<geometry_msgs::msg::Point> f(4);
    f[0].x = 0.452;
    f[0].y = 0.352;
    f[1].x = 0.452;
    f[1].y = -0.352;
    f[2].x = -0.452;
    f[2].y = -0.352;
    f[3].x = -0.452;
    f[3].y = 0.352;
    return f;
}

// 10 m x 10 m, 5 cm cells, origin (-5, -5), all free.
struct Map
{
    Map() : costmap(200, 200, 0.05, -5.0, -5.0, nav2_costmap_2d::FREE_SPACE)
    {
    }

    void fill(double x0, double y0, double x1, double y1, unsigned char cost)
    {
        for (double x = x0; x <= x1 + 1e-9; x += 0.025)
        {
            for (double y = y0; y <= y1 + 1e-9; y += 0.025)
            {
                unsigned int mx = 0;
                unsigned int my = 0;
                if (costmap.worldToMap(x, y, mx, my))
                {
                    costmap.setCost(mx, my, cost);
                }
            }
        }
    }

    nav2_costmap_2d::Costmap2D costmap;
};

} // namespace

// D: a wall beside the robot, outside the body but inside the circle its corners sweep.
TEST(FootprintChecker, RotationNextToWallIsBlocked)
{
    Map map;
    map.fill(-2.0, 0.42, 2.0, 0.47, nav2_costmap_2d::LETHAL_OBSTACLE); // wall 7 cm from the side
    FootprintChecker checker(map.costmap, footprint());
    const Pose2D start{0, 0, 0};

    EXPECT_EQ(checker.checkPose(start), Collision::NONE);
    const CheckResult turn = checker.checkRotation(start, M_PI / 2);
    EXPECT_EQ(turn.collision, Collision::LETHAL);
    EXPECT_GT(std::fabs(turn.angle), 0.0); // the start pose is free, a corner hits during the turn
    EXPECT_LT(std::fabs(turn.angle), 0.5);
    EXPECT_FALSE(checker.checkRotation(start, -M_PI / 2).free()); // the other way the rear corner hits

    // driving along the wall is fine
    EXPECT_TRUE(checker.checkTwist(start, 0.5, 0, 0, 2.0).free());

    // with the wall beyond the corner circle (0.573 m, plus the cell it ends in) the robot may turn
    Map far;
    far.fill(-2.0, 0.66, 2.0, 0.70, nav2_costmap_2d::LETHAL_OBSTACLE);
    FootprintChecker far_checker(far.costmap, footprint());
    EXPECT_TRUE(far_checker.checkRotation(start, M_PI).free());
}

// E: the path is clear, but the arc the robot is actually commanded onto is not.
TEST(FootprintChecker, CommandedArcIsCheckedNotJustThePath)
{
    Map map;
    map.fill(0.9, 0.75, 1.2, 0.95, nav2_costmap_2d::LETHAL_OBSTACLE);
    FootprintChecker checker(map.costmap, footprint());

    std::vector<Pose2D> path;
    for (int i = 0; i <= 60; ++i)
    {
        path.push_back({0.05 * i, 0, 0});
    }
    EXPECT_TRUE(checker.checkPath(path, 3.0).free());

    // vx 0.5 m/s, wz 0.5 rad/s: left arc of radius 1 m, as pure pursuit commands when off the path
    const CheckResult arc = checker.checkTwist({0, 0, 0}, 0.5, 0, 0.5, 2.0);
    EXPECT_EQ(arc.collision, Collision::LETHAL);
    EXPECT_GT(arc.distance, 0.3);
}

TEST(FootprintChecker, UnobservedAndOffMapAreNotFree)
{
    Map map;
    map.fill(1.5, -0.1, 1.6, 0.1, nav2_costmap_2d::NO_INFORMATION);
    FootprintChecker checker(map.costmap, footprint());
    EXPECT_EQ(checker.checkTwist({0, 0, 0}, 1.0, 0, 0, 2.0).collision, Collision::UNKNOWN);
    // driving off the costmap (it ends at x = 5)
    EXPECT_EQ(checker.checkTwist({4.0, -2.0, 0}, 1.0, 0, 0, 1.0).collision, Collision::OFF_MAP);
    EXPECT_EQ(checker.checkPose({6.0, 0, 0}), Collision::OFF_MAP);
}

TEST(FootprintChecker, StartAndEndPosesAreChecked)
{
    Map map;
    map.fill(1.43, -0.1, 1.45, 0.1, nav2_costmap_2d::LETHAL_OBSTACLE); // just reached by the bumper at x = 1.0
    FootprintChecker checker(map.costmap, footprint());
    EXPECT_TRUE(checker.checkTwist({0, 0, 0}, 1.0, 0, 0, 0.9).free());
    const CheckResult end = checker.checkTwist({0, 0, 0}, 1.0, 0, 0, 1.0);
    EXPECT_EQ(end.collision, Collision::LETHAL);
    EXPECT_GT(end.distance, 0.9);

    // an obstacle already under the footprint blocks every motion, at distance 0
    Map under;
    under.fill(0.3, 0.2, 0.35, 0.25, nav2_costmap_2d::LETHAL_OBSTACLE);
    FootprintChecker under_checker(under.costmap, footprint());
    const CheckResult start = under_checker.checkTwist({0, 0, 0}, -0.5, 0, 0, 1.0);
    EXPECT_EQ(start.collision, Collision::LETHAL);
    EXPECT_DOUBLE_EQ(start.distance, 0.0);
    EXPECT_FALSE(under_checker.checkRotation({0, 0, 0}, 0.1).free());
}

TEST(FootprintChecker, InflationIsNotACollision)
{
    Map map;
    map.fill(-2.0, -2.0, 2.0, 2.0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    FootprintChecker checker(map.costmap, footprint());
    EXPECT_TRUE(checker.checkRotation({0, 0, 0}, M_PI).free());
}

// SD footprint (config/sd/footprint.yaml): the body ahead of the turning centre (fork roller axle) and a
// narrow fork behind it, not convex.
std::vector<geometry_msgs::msg::Point> sdFootprint()
{
    const double xy[8][2] = {{1.068, 0.403},  {0.612, 0.403},  {0.612, 0.172},  {-0.132, 0.172},
                             {-0.132, -0.172}, {0.612, -0.172}, {0.612, -0.403}, {1.068, -0.403}};
    std::vector<geometry_msgs::msg::Point> f(8);
    for (int i = 0; i < 8; ++i)
    {
        f[i].x = xy[i][0];
        f[i].y = xy[i][1];
    }
    return f;
}

TEST(FootprintChecker, NonConvexFootprintLeavesTheNotchFree)
{
    // an obstacle beside the fork (inside the convex hull of the footprint, outside the footprint)
    Map notch;
    notch.fill(0.20, 0.25, 0.40, 0.30, nav2_costmap_2d::LETHAL_OBSTACLE);
    FootprintChecker checker(notch.costmap, sdFootprint());
    EXPECT_EQ(checker.checkPose({0, 0, 0}), Collision::NONE);
    EXPECT_EQ(checker.checkPose({0, 0.15, 0}), Collision::LETHAL); // 15 cm to the left the fork is on it

    // one under the fork or the body is a collision
    Map fork;
    fork.fill(0.20, 0.0, 0.25, 0.05, nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(FootprintChecker(fork.costmap, sdFootprint()).checkPose({0, 0, 0}), Collision::LETHAL);
    Map body;
    body.fill(0.95, 0.35, 1.0, 0.38, nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(FootprintChecker(body.costmap, sdFootprint()).checkPose({0, 0, 0}), Collision::LETHAL);
}

TEST(FootprintChecker, TurnsAboutTheOriginNotTheBodyCentre)
{
    // a crate 1 m to the right of the SD's turning centre: turning counterclockwise swings the body away
    // from it, turning clockwise onto it
    Map map;
    map.fill(0.0, -1.1, 0.5, -0.95, nav2_costmap_2d::LETHAL_OBSTACLE);
    FootprintChecker checker(map.costmap, sdFootprint());
    EXPECT_TRUE(checker.checkRotation({0, 0, 0}, M_PI / 2).free());
    const CheckResult cw = checker.checkRotation({0, 0, 0}, -M_PI / 2);
    EXPECT_EQ(cw.collision, Collision::LETHAL);
    EXPECT_GT(std::fabs(cw.angle), 0.5);
    EXPECT_NEAR(checker.circumscribedRadius(), std::hypot(1.068, 0.403), 1e-9);
}

TEST(StoppingDistance, UsesTheFasterOfMeasuredAndCommanded)
{
    // measured 1.0 m/s while the new command is 0.2 m/s: the base still has to stop from 1.0 m/s
    EXPECT_NEAR(stoppingDistance(1.0, 0.2, 0.5, 0.2, 0.1), 1.0 * 0.2 + 1.0 / (2 * 0.5) + 0.1, 1e-12);
    EXPECT_NEAR(stoppingDistance(-0.4, 0.0, 0.5, 0.0, 0.0), 0.16, 1e-12);
    EXPECT_NEAR(stoppingDistance(0.0, 0.0, 0.5, 0.2, 0.1), 0.1, 1e-12);
}

TEST(IntegrateTwist, ArcAndStraight)
{
    const Pose2D a = integrateTwist({0, 0, 0}, 1.0, 0, 1.0, M_PI / 2); // quarter circle of radius 1
    EXPECT_NEAR(a.x, 1.0, 1e-9);
    EXPECT_NEAR(a.y, 1.0, 1e-9);
    EXPECT_NEAR(a.yaw, M_PI / 2, 1e-9);
    const Pose2D b = integrateTwist({1, 2, M_PI / 2}, 0.5, 0.2, 0, 2.0); // body x along world y
    EXPECT_NEAR(b.x, 1 - 0.4, 1e-9);
    EXPECT_NEAR(b.y, 2 + 1.0, 1e-9);
}

// F: along-track distance is not enough for a goal.
TEST(GoalReached, NeedsActualPositionHeadingAndStop)
{
    const Pose2D goal{5, 0, 0};
    EXPECT_FALSE(goalReached({5, 1, 0}, goal, true, 0.03, 0.02));  // beside the goal (on the line x = 5)
    EXPECT_FALSE(goalReached({5.2, 0, 0}, goal, true, 0.03, 0.02)); // overshot
    EXPECT_TRUE(goalReached({5.02, 0.01, 0.01}, goal, true, 0.03, 0.02));
    EXPECT_FALSE(goalReached({5.02, 0.01, 0.01}, goal, false, 0.03, 0.02)); // still moving
    EXPECT_FALSE(goalReached({5, 0, 0.1}, goal, true, 0.03, 0.02));         // heading off
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
