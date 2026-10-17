#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "qd_kinematics/kinematics_model.hpp"
#include "qd_kinematics/odometry_integrator.hpp"
#include "qd_kinematics/steer_drive_controller.hpp"

using namespace qd_kinematics;

namespace
{

std::vector<ModuleState> toStates(const KinematicsModel &model, const Twist2D &twist)
{
    std::vector<Vec2> v;
    model.moduleVelocities(twist, v);
    std::vector<ModuleState> s(v.size());
    for (size_t i = 0; i < v.size(); ++i)
    {
        s[i].speed = std::hypot(v[i][0], v[i][1]);
        s[i].angle = foldHalfPi(std::atan2(v[i][1], v[i][0]), s[i].speed);
    }
    return s;
}

void expectRoundTrip(const KinematicsModel &model, bool holonomic)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> lin(-0.6, 0.6);
    std::uniform_real_distribution<double> ang(-1.0, 1.0);

    std::vector<Twist2D> cases = {{0.3, 0, 0}, {0, 0.3, 0}, {0, 0, 0.5}, {0.3, 0, 0.3}, {0.2, 0.1, 0.2}};
    for (int i = 0; i < 500; ++i)
    {
        cases.push_back({lin(rng), lin(rng), ang(rng)});
    }

    for (auto t : cases)
    {
        if (!holonomic)
        {
            t = model.feasible(t);
        }
        const Twist2D out = model.bodyTwist(toStates(model, t));
        EXPECT_NEAR(out.vx, t.vx, 1e-9);
        EXPECT_NEAR(out.vy, t.vy, 1e-9);
        EXPECT_NEAR(out.wz, t.wz, 1e-9);
    }
}

// Ideal plant: wheels follow the command exactly.
struct IdealPlant
{
    explicit IdealPlant(std::shared_ptr<const KinematicsModel> model, ControllerParams p = {})
        : ctrl(model, p), state(model->numModules())
    {
    }

    void step(const Twist2D &cmd, double dt)
    {
        ctrl.update(cmd, state, dt, command);
        state = command;
    }

    SteerDriveController ctrl;
    std::vector<ModuleState> state;
    std::vector<ModuleState> command;
};

} // namespace

TEST(SwerveKinematics, RoundTripQuadDrive1Lateral)
{
    expectRoundTrip(SwerveKinematics({{0.0, 0.5}, {0.0, -0.5}}), true);
}

TEST(SwerveKinematics, RoundTripQuadDrive2Diagonal)
{
    expectRoundTrip(SwerveKinematics({{0.46, 0.30}, {-0.46, -0.30}}), true);
}

TEST(SwerveKinematics, RoundTripQuadDrive3InlineOffsetOrigin)
{
    expectRoundTrip(SwerveKinematics({{0.9, 0.0}, {-0.1, 0.0}}), true);
}

TEST(SwerveKinematics, RoundTripFourModules)
{
    expectRoundTrip(SwerveKinematics({{0.4, 0.3}, {0.4, -0.3}, {-0.4, 0.3}, {-0.4, -0.3}}), true);
}

TEST(SwerveKinematics, ParallelWheelsWithDifferentSpeedsMeanRotation)
{
    // Old ICR formula returned wz = 0 here (QD#1, forward + turn).
    SwerveKinematics model({{0.0, 0.5}, {0.0, -0.5}});
    const Twist2D out = model.bodyTwist({{0.0, 0.15}, {0.0, 0.45}});
    EXPECT_NEAR(out.vx, 0.3, 1e-12);
    EXPECT_NEAR(out.wz, 0.3, 1e-12);
}

TEST(SwerveKinematics, RejectsDegenerateGeometry)
{
    EXPECT_THROW(SwerveKinematics({{0.3, 0.0}}), std::invalid_argument);
    EXPECT_THROW(SwerveKinematics({{0.3, 0.1}, {0.3, 0.1}}), std::invalid_argument);
}

TEST(TricycleKinematics, RoundTripSingleDrive)
{
    expectRoundTrip(TricycleKinematics({0.785, 0.0}, 0.0), false);
    expectRoundTrip(TricycleKinematics({0.0, 0.05}, -0.785), false);
}

TEST(TricycleKinematics, LateralVelocityIsRemoved)
{
    TricycleKinematics model({0.785, 0.0}, 0.0);
    const Twist2D f = model.feasible({0.2, 0.3, 0.1});
    EXPECT_DOUBLE_EQ(f.vx, 0.2);
    EXPECT_DOUBLE_EQ(f.vy, 0.0);
    EXPECT_DOUBLE_EQ(f.wz, 0.1);

    // Spin in place about the axle center: drive wheel points sideways.
    const auto s = toStates(model, {0.0, 0.0, 0.5});
    EXPECT_NEAR(std::abs(s[0].angle), M_PI / 2.0, 1e-12);
    EXPECT_NEAR(std::abs(s[0].speed), 0.5 * 0.785, 1e-12);
}

TEST(TricycleKinematics, RejectsWheelOnAxle)
{
    EXPECT_THROW(TricycleKinematics({0.2, 0.0}, 0.2), std::invalid_argument);
}

TEST(SteerDriveController, LateralMotionDoesNotFlipWithHysteresis)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.135}, {-0.33, -0.135}});
    IdealPlant plant(model);

    // Pure lateral command with a tiny longitudinal component whose sign alternates.
    double first_angle = 0.0;
    for (int k = 0; k < 2000; ++k)
    {
        const double vx = (k % 2 == 0) ? 1e-4 : -1e-4;
        plant.step({vx, 0.2, 0.0}, 0.01);
        const auto &dbg = plant.ctrl.debugState();
        if (k == 300)
        {
            first_angle = dbg.th_target[0];
        }
        if (k > 300)
        {
            ASSERT_NEAR(dbg.th_target[0], first_angle, 0.01) << "target flipped at step " << k;
        }
    }
    EXPECT_EQ(plant.ctrl.debugState().mode, SteerDriveController::Mode::DRIVE);
    EXPECT_NEAR(std::abs(plant.state[0].speed), 0.2, 1e-6);
}

TEST(SteerDriveController, LateralMotionFlipsWithoutHysteresis)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.135}, {-0.33, -0.135}});
    ControllerParams p;
    p.steer_hysteresis = 0.0;
    IdealPlant plant(model, p);

    plant.step({1e-4, 0.2, 0.0}, 0.01);
    const double a = plant.ctrl.debugState().th_target[0];
    plant.step({-1e-4, 0.2, 0.0}, 0.01);
    const double b = plant.ctrl.debugState().th_target[0];
    EXPECT_NEAR(std::abs(a - b), M_PI, 0.01);
}

TEST(SteerDriveController, HysteresisReleasesOutsideBand)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.0}, {-0.33, 0.0}});
    IdealPlant plant(model);
    // Start just inside +90 deg, then rotate the commanded direction to 120 deg.
    for (int k = 0; k < 500; ++k)
    {
        plant.step({0.01, 0.2, 0.0}, 0.01);
    }
    const double band = M_PI / 2.0 + ControllerParams{}.steer_hysteresis;
    for (int deg = 85; deg <= 120; ++deg)
    {
        const double th = deg * M_PI / 180.0;
        plant.step({0.2 * std::cos(th), 0.2 * std::sin(th), 0.0}, 0.01);
        EXPECT_LE(std::abs(plant.ctrl.debugState().th_target[0]), band + 1e-9);
    }
    EXPECT_NEAR(plant.ctrl.debugState().th_target[0], (120.0 - 180.0) * M_PI / 180.0, 1e-6);
}

TEST(SteerDriveController, RateLimitsIndependentOfControlRate)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.135}, {-0.33, -0.135}});
    ControllerParams p;
    for (double hz : {50.0, 100.0, 1000.0})
    {
        IdealPlant plant(model, p);
        const int steps = static_cast<int>(std::lround(1.0 * hz));
        for (int k = 0; k < steps; ++k)
        {
            plant.step({0.3, 0.0, 0.0}, 1.0 / hz);
        }
        EXPECT_NEAR(plant.state[0].speed, p.linear_acc * 1.0, 1e-6) << hz << " Hz";
    }
}

TEST(SteerDriveController, StopSteerGoReachesTarget)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.0, 0.5}, {0.0, -0.5}});
    IdealPlant plant(model);
    // Drive forward, then command a lateral move: needs WAIT_STOP -> STEER_ALIGN -> CREEP -> DRIVE.
    for (int k = 0; k < 500; ++k)
    {
        plant.step({0.2, 0.0, 0.0}, 0.01);
    }
    bool saw_align = false;
    for (int k = 0; k < 2000; ++k)
    {
        plant.step({0.0, 0.2, 0.0}, 0.01);
        saw_align = saw_align || plant.ctrl.debugState().mode == SteerDriveController::Mode::STEER_ALIGN;
    }
    EXPECT_TRUE(saw_align);
    EXPECT_EQ(plant.ctrl.debugState().mode, SteerDriveController::Mode::DRIVE);
    const Twist2D body = model->bodyTwist(plant.state);
    EXPECT_NEAR(body.vx, 0.0, 1e-6);
    EXPECT_NEAR(body.vy, 0.2, 1e-6);
}

TEST(SteerDriveController, TricycleDrivesArc)
{
    auto model = std::make_shared<TricycleKinematics>(ModulePosition{0.785, 0.0}, 0.0);
    IdealPlant plant(model);
    for (int k = 0; k < 3000; ++k)
    {
        plant.step({0.2, 0.0, 0.2}, 0.01);
    }
    const Twist2D body = model->bodyTwist(plant.state);
    EXPECT_NEAR(body.vx, 0.2, 1e-6);
    EXPECT_NEAR(body.vy, 0.0, 1e-9);
    EXPECT_NEAR(body.wz, 0.2, 1e-6);
}

TEST(OdometryIntegrator, TrapezoidExactForLinearRamp)
{
    OdometryIntegrator odom;
    const double acc = 0.5;
    const double dt = 0.02;
    for (int k = 0; k <= 100; ++k)
    {
        odom.update({acc * k * dt, 0.0, 0.0}, k * dt);
    }
    // x = a T^2 / 2 with T = 2 s; trapezoid is exact for a linear velocity profile.
    EXPECT_NEAR(odom.pose().x, 0.5 * acc * 4.0, 1e-12);
}

TEST(OdometryIntegrator, ClosesCircle)
{
    OdometryIntegrator odom;
    const double v = 0.3;
    const double w = 0.5;
    const double dt = 0.01;
    const int steps = static_cast<int>(std::lround(2.0 * M_PI / w / dt));
    for (int k = 0; k <= steps; ++k)
    {
        odom.update({v, 0.0, w}, k * dt);
    }
    EXPECT_NEAR(odom.pose().x, 0.0, 2e-3);
    EXPECT_NEAR(odom.pose().y, 0.0, 2e-3);
}

TEST(OdometryIntegrator, SkipsGapsAndStaleStamps)
{
    OdometryIntegrator odom;
    odom.setMaxDt(0.1);
    odom.update({1.0, 0.0, 0.0}, 0.0);
    EXPECT_FALSE(odom.update({1.0, 0.0, 0.0}, 0.5)); // gap
    EXPECT_DOUBLE_EQ(odom.pose().x, 0.0);
    EXPECT_FALSE(odom.update({1.0, 0.0, 0.0}, 0.4)); // stale
    EXPECT_TRUE(odom.update({1.0, 0.0, 0.0}, 0.55));
    EXPECT_NEAR(odom.pose().x, 0.05, 1e-12);
}
