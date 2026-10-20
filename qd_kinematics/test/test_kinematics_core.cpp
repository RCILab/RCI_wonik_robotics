#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include "qd_kinematics/kinematics_model.hpp"
#include "qd_kinematics/odometry_integrator.hpp"
#include "qd_kinematics/stamp_monitor.hpp"
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

TEST(SteerDriveController, LimitedSteeringMovesThroughItsRange)
{
    // +-140 deg steering at +130 deg, target -60 deg: the shortest way round (+170 deg, across 180)
    // leaves the range, so the command has to go back through 0 (-190 deg).
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.135}, {-0.33, -0.135}});
    ControllerParams p;
    p.steer_limit = 140.0 * M_PI / 180.0;
    IdealPlant plant(model, p);
    for (auto &s : plant.state)
    {
        s.angle = 130.0 * M_PI / 180.0;
    }
    const double th = -60.0 * M_PI / 180.0;
    double previous = plant.state[0].angle;
    for (int k = 0; k < 2000; ++k)
    {
        plant.step({0.2 * std::cos(th), 0.2 * std::sin(th), 0.0}, 0.01);
        for (const auto &s : plant.state)
        {
            ASSERT_LE(std::abs(s.angle), p.steer_limit + 1e-9) << "step " << k;
        }
        ASSERT_LE(plant.state[0].angle, previous + 1e-9) << "steer reversed at step " << k;
        previous = plant.state[0].angle;
    }
    EXPECT_EQ(plant.ctrl.debugState().mode, SteerDriveController::Mode::DRIVE);
    EXPECT_NEAR(plant.state[0].angle, th, 1e-6);
    const Twist2D body = model->bodyTwist(plant.state);
    EXPECT_NEAR(body.vx, 0.2 * std::cos(th), 1e-6);
    EXPECT_NEAR(body.vy, 0.2 * std::sin(th), 1e-6);
}

TEST(SteerDriveController, WheelSpeedLimitKeepsCurvature)
{
    auto model = std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.33, 0.135}, {-0.33, -0.135}});
    ControllerParams p;
    p.max_wheel_speed = 0.5;
    IdealPlant plant(model, p);
    const Twist2D cmd{0.8, 0.0, 1.0};
    std::vector<Vec2> v;
    model->moduleVelocities(cmd, v);
    double fastest = 0.0;
    for (const auto &w : v)
    {
        fastest = std::max(fastest, std::hypot(w[0], w[1]));
    }
    ASSERT_GT(fastest, p.max_wheel_speed);
    const double scale = p.max_wheel_speed / fastest;

    for (int k = 0; k < 3000; ++k)
    {
        plant.step(cmd, 0.01);
        for (const auto &s : plant.state)
        {
            ASSERT_LE(std::abs(s.speed), p.max_wheel_speed + 1e-9) << "step " << k;
        }
    }
    EXPECT_EQ(plant.ctrl.debugState().mode, SteerDriveController::Mode::DRIVE);
    const Twist2D body = model->bodyTwist(plant.state);
    EXPECT_NEAR(body.vx, scale * cmd.vx, 1e-6);
    EXPECT_NEAR(body.vy, 0.0, 1e-6);
    EXPECT_NEAR(body.wz, scale * cmd.wz, 1e-6);
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

// ---- Start-up supervision and faults ----

namespace
{

// Steering follows the command; drive i turns only if responds[i] (otherwise it measures 0).
struct PartialPlant
{
    PartialPlant(std::shared_ptr<const KinematicsModel> model, std::vector<bool> responds_in)
        : ctrl(model, ControllerParams{}), state(model->numModules()), responds(std::move(responds_in))
    {
    }

    void step(const Twist2D &cmd, double dt)
    {
        ctrl.update(cmd, state, dt, command);
        for (size_t i = 0; i < state.size(); ++i)
        {
            state[i].angle = command[i].angle;
            state[i].speed = responds[i] ? command[i].speed : 0.0;
        }
    }

    SteerDriveController::Mode mode() const
    {
        return ctrl.debugState().mode;
    }

    SteerDriveController ctrl;
    std::vector<ModuleState> state;
    std::vector<ModuleState> command;
    std::vector<bool> responds;
};

std::shared_ptr<SwerveKinematics> cobotV2()
{
    return std::make_shared<SwerveKinematics>(std::vector<ModulePosition>{{0.1975, -0.13}, {-0.1975, 0.13}});
}

// Runs a sideways command (needs stop -> steer-align -> creep) for `seconds`; returns whether DRIVE was
// ever reached after the sequence had started.
bool driveSideways(PartialPlant &plant, double seconds)
{
    bool started = false;
    bool reached_drive = false;
    for (int k = 0; k < static_cast<int>(seconds * 100); ++k)
    {
        plant.step({0.0, 0.2, 0.0}, 0.01);
        started = started || plant.mode() != SteerDriveController::Mode::DRIVE;
        reached_drive = reached_drive || (started && plant.mode() == SteerDriveController::Mode::DRIVE);
    }
    return reached_drive;
}

} // namespace

TEST(SteerDriveController, CreepNeedsMeasuredWheelSpeed)
{
    // Steering works, the drive wheels never turn: the commanded creep speed converges, but that must
    // not count as a successful start.
    PartialPlant plant(cobotV2(), {false, false});
    EXPECT_FALSE(driveSideways(plant, 15.0));
    EXPECT_EQ(plant.mode(), SteerDriveController::Mode::FAULT);
    EXPECT_TRUE(plant.ctrl.faulted());
    EXPECT_NE(plant.ctrl.faultReason().find("creep"), std::string::npos) << plant.ctrl.faultReason();
    EXPECT_EQ(plant.ctrl.debugState().start_attempts, ControllerParams{}.max_start_attempts);
    for (const auto &c : plant.command)
    {
        EXPECT_EQ(c.speed, 0.0);
    }
}

TEST(SteerDriveController, OneModuleNotDrivingFaults)
{
    PartialPlant plant(cobotV2(), {true, false});
    EXPECT_FALSE(driveSideways(plant, 15.0));
    EXPECT_EQ(plant.mode(), SteerDriveController::Mode::FAULT);
}

TEST(SteerDriveController, WheelStallInDriveFaults)
{
    PartialPlant plant(cobotV2(), {true, true});
    EXPECT_TRUE(driveSideways(plant, 6.0));
    ASSERT_EQ(plant.mode(), SteerDriveController::Mode::DRIVE);

    plant.responds[1] = false; // rear drive stalls while driving
    double fault_after = -1.0;
    for (int k = 0; k < 300 && fault_after < 0.0; ++k)
    {
        plant.step({0.0, 0.2, 0.0}, 0.01);
        if (plant.ctrl.faulted())
        {
            fault_after = (k + 1) * 0.01;
        }
    }
    ASSERT_GT(fault_after, 0.0);
    EXPECT_LT(fault_after, ControllerParams{}.drive_fault_time + 0.5);
    for (int k = 0; k < 300; ++k)
    {
        plant.step({0.0, 0.2, 0.0}, 0.01);
    }
    EXPECT_EQ(plant.command[0].speed, 0.0); // decelerated to zero at linear_dec
    EXPECT_EQ(plant.command[1].speed, 0.0);
}

TEST(SteerDriveController, FaultIsLatchedUntilCleared)
{
    PartialPlant plant(cobotV2(), {false, false});
    driveSideways(plant, 15.0);
    ASSERT_TRUE(plant.ctrl.faulted());

    // New commands, a zero command and a reset() do not clear it; the wheels stay commanded to zero.
    plant.responds = {true, true};
    for (int k = 0; k < 500; ++k)
    {
        plant.step({0.3, 0.0, 0.0}, 0.01);
        ASSERT_TRUE(plant.ctrl.faulted());
        ASSERT_EQ(plant.command[0].speed, 0.0);
    }
    plant.step({0.0, 0.0, 0.0}, 0.01);
    plant.ctrl.reset();
    plant.step({0.3, 0.0, 0.0}, 0.01);
    EXPECT_TRUE(plant.ctrl.faulted());

    // Only clearFault() does; with working drives the robot then drives normally.
    plant.ctrl.clearFault();
    EXPECT_FALSE(plant.ctrl.faulted());
    driveSideways(plant, 6.0);
    EXPECT_EQ(plant.mode(), SteerDriveController::Mode::DRIVE);
    EXPECT_NEAR(plant.state[0].speed, plant.ctrl.debugState().v_target[0], 1e-9);
    EXPECT_GT(std::abs(plant.state[0].speed), 0.1);
}

TEST(SteerDriveController, RestartAfterResetChecksMeasuredSpeed)
{
    // Driving straight, then a feedback dropout (reset()); the wheels still point the right way but the
    // drives no longer turn. The restart must creep and verify, not ramp straight to the command.
    PartialPlant plant(cobotV2(), {true, true});
    for (int k = 0; k < 600; ++k)
    {
        plant.step({0.4, 0.0, 0.0}, 0.01);
    }
    ASSERT_EQ(plant.mode(), SteerDriveController::Mode::DRIVE);
    for (int k = 0; k < 300; ++k) // stop
    {
        plant.step({0.0, 0.0, 0.0}, 0.01);
    }
    plant.ctrl.reset();
    plant.responds = {false, false};
    double max_cmd = 0.0;
    bool drive = false;
    for (int k = 0; k < 1500; ++k)
    {
        plant.step({0.4, 0.0, 0.0}, 0.01);
        max_cmd = std::max(max_cmd, std::abs(plant.command[0].speed));
        drive = drive || plant.mode() == SteerDriveController::Mode::DRIVE;
    }
    EXPECT_FALSE(drive);
    EXPECT_LE(max_cmd, ControllerParams{}.creep_speed_max + 1e-9);
    EXPECT_TRUE(plant.ctrl.faulted());

    // the same after clearing the fault with the drives still dead
    plant.ctrl.clearFault();
    max_cmd = 0.0;
    drive = false;
    for (int k = 0; k < 1500; ++k)
    {
        plant.step({0.4, 0.0, 0.0}, 0.01);
        max_cmd = std::max(max_cmd, std::abs(plant.command[0].speed));
        drive = drive || plant.mode() == SteerDriveController::Mode::DRIVE;
    }
    EXPECT_FALSE(drive);
    EXPECT_LE(max_cmd, ControllerParams{}.creep_speed_max + 1e-9);
    EXPECT_TRUE(plant.ctrl.faulted());

    // with working drives the verified restart reaches DRIVE
    plant.ctrl.clearFault();
    plant.responds = {true, true};
    for (int k = 0; k < 600; ++k)
    {
        plant.step({0.4, 0.0, 0.0}, 0.01);
    }
    EXPECT_EQ(plant.mode(), SteerDriveController::Mode::DRIVE);
    EXPECT_GT(plant.command[0].speed, 0.3);
}

TEST(SteerDriveController, NonFiniteInputCommandsZeroSpeed)
{
    PartialPlant plant(cobotV2(), {true, true});
    for (int k = 0; k < 500; ++k)
    {
        plant.step({0.3, 0.0, 0.0}, 0.01);
    }
    ASSERT_GT(plant.command[0].speed, 0.1);

    // A NaN command is a stop command: the speed ramps down, every output stays finite.
    for (int k = 0; k < 300; ++k)
    {
        plant.step({std::nan(""), 0.0, 0.0}, 0.01);
        for (const auto &c : plant.command)
        {
            ASSERT_TRUE(std::isfinite(c.angle) && std::isfinite(c.speed));
        }
    }
    EXPECT_EQ(plant.command[0].speed, 0.0);

    // A non-finite measurement: zero speed immediately, steering held, nothing NaN.
    plant.state[1].speed = std::numeric_limits<double>::infinity();
    std::vector<ModuleState> command;
    plant.ctrl.update({0.3, 0.0, 0.0}, plant.state, 0.01, command);
    for (const auto &c : command)
    {
        EXPECT_EQ(c.speed, 0.0);
        EXPECT_TRUE(std::isfinite(c.angle));
    }
}

TEST(StampMonitor, ClassifiesFeedbackStamps)
{
    using R = StampMonitor::Result;
    StampMonitor m(0.1);
    EXPECT_EQ(m.check(10.00), R::FIRST);
    EXPECT_EQ(m.check(10.01), R::OK);
    EXPECT_EQ(m.check(10.01), R::DUPLICATE);    // dropped, not re-stamped
    EXPECT_EQ(m.check(10.005), R::OUT_OF_ORDER); // late sample dropped
    EXPECT_DOUBLE_EQ(m.last(), 10.01);           // the last accepted stamp is unchanged
    EXPECT_EQ(m.check(10.02), R::OK);
    EXPECT_EQ(m.check(10.50), R::GAP);           // accepted, but not integrated across
    EXPECT_EQ(m.check(8.50), R::OUT_OF_ORDER);   // 2 s late: still a late sample, not a clock reset
    EXPECT_DOUBLE_EQ(m.last(), 10.50);
    m.reset();                                   // the owner saw its own clock go back
    EXPECT_EQ(m.check(2.00), R::FIRST);
    EXPECT_EQ(m.check(2.01), R::OK);
    EXPECT_EQ(m.check(std::nan("")), R::INVALID);
    EXPECT_EQ(m.check(0.0), R::INVALID);
    EXPECT_EQ(m.check(-1.0), R::INVALID);
    EXPECT_EQ(m.duplicates, 1u);
    EXPECT_EQ(m.out_of_order, 2u);
    EXPECT_EQ(m.gaps, 1u);
    EXPECT_EQ(m.invalid, 3u);
}

TEST(OdometryIntegrator, IgnoresNonFiniteInput)
{
    OdometryIntegrator odom;
    odom.update({1.0, 0.0, 0.0}, 0.00);
    EXPECT_TRUE(odom.update({1.0, 0.0, 0.0}, 0.01));
    EXPECT_FALSE(odom.update({std::nan(""), 0.0, 0.0}, 0.02));
    EXPECT_FALSE(odom.update({1.0, 0.0, std::numeric_limits<double>::infinity()}, 0.03));
    EXPECT_TRUE(odom.update({1.0, 0.0, 0.0}, 0.04));
    EXPECT_TRUE(std::isfinite(odom.pose().x) && std::isfinite(odom.pose().yaw));
    EXPECT_NEAR(odom.pose().x, 0.04, 1e-12);
}
