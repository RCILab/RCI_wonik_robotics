// Gazebo-only stand-in for the wheel-ground contact of a steer-drive base.
// Every step the base link gets the planar twist that the measured module states (steer angle,
// wheel speed) imply, computed with qd_kinematics' own forward kinematics. The steer and drive
// joints are still commanded through ros2_control, so joint_states and the qd_kinematics node
// behave as on the real robot; only the tyre physics (slip, load distribution) is replaced.
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "qd_kinematics/kinematics_model.hpp"

namespace wonik_gazebo
{

template <typename T>
std::vector<T> parseList(const sdf::ElementPtr &sdf, const std::string &key)
{
    std::vector<T> out;
    if (!sdf->HasElement(key))
    {
        return out;
    }
    std::istringstream stream(sdf->Get<std::string>(key));
    T value;
    while (stream >> value)
    {
        out.push_back(value);
    }
    return out;
}

class QdKinematicBasePlugin : public gazebo::ModelPlugin
{
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        const auto xs = parseList<double>(sdf, "moduleX");
        const auto ys = parseList<double>(sdf, "moduleY");
        const auto steer_names = parseList<std::string>(sdf, "steerJoints");
        const auto drive_names = parseList<std::string>(sdf, "driveJoints");
        const std::string drive_type = sdf->Get<std::string>("driveType", "swerve").first;
        wheel_radius_ = sdf->Get<double>("wheelRadius", 0.0).first;

        if (xs.empty() || xs.size() != ys.size() || steer_names.size() != xs.size() ||
            drive_names.size() != xs.size() || wheel_radius_ <= 0.0)
        {
            gzerr << "qd_kinematic_base: moduleX/moduleY/steerJoints/driveJoints must have one entry per "
                     "module and wheelRadius must be > 0\n";
            return;
        }

        std::vector<qd_kinematics::ModulePosition> modules;
        for (size_t i = 0; i < xs.size(); ++i)
        {
            modules.push_back({xs[i], ys[i]});
        }
        try
        {
            if (drive_type == "swerve")
            {
                kinematics_ = std::make_unique<qd_kinematics::SwerveKinematics>(modules);
            }
            else if (drive_type == "tricycle")
            {
                kinematics_ = std::make_unique<qd_kinematics::TricycleKinematics>(
                    modules.at(0), sdf->Get<double>("fixedAxleX", 0.0).first);
            }
            else
            {
                gzerr << "qd_kinematic_base: unknown driveType '" << drive_type << "' (swerve | tricycle)\n";
                return;
            }
        }
        catch (const std::exception &e)
        {
            gzerr << "qd_kinematic_base: invalid geometry: " << e.what() << "\n";
            return;
        }

        for (size_t i = 0; i < xs.size(); ++i)
        {
            auto steer = model->GetJoint(steer_names[i]);
            auto drive = model->GetJoint(drive_names[i]);
            if (!steer || !drive)
            {
                gzerr << "qd_kinematic_base: joint '" << (!steer ? steer_names[i] : drive_names[i])
                      << "' not found\n";
                return;
            }
            steer_joints_.push_back(steer);
            drive_joints_.push_back(drive);
        }

        // Canonical link: base_footprint, with every fixed-joint child (base_link, sensors) lumped in.
        base_ = model->GetLink();
        update_connection_ =
            gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&QdKinematicBasePlugin::OnUpdate, this));
        gzmsg << "qd_kinematic_base: " << kinematics_->name() << " with " << xs.size()
              << " module(s) drives link '" << base_->GetName() << "'\n";
    }

  private:
    void OnUpdate()
    {
        std::vector<qd_kinematics::ModuleState> states(steer_joints_.size());
        for (size_t i = 0; i < states.size(); ++i)
        {
            states[i] = {steer_joints_[i]->Position(0), drive_joints_[i]->GetVelocity(0) * wheel_radius_};
        }
        const qd_kinematics::Twist2D twist = kinematics_->bodyTwist(states);

        // Twist at the base frame origin, rotated into the world. Roll/pitch and vertical motion are
        // left to the physics (the base rests on frictionless wheel and caster contacts).
        const ignition::math::Pose3d pose = base_->WorldPose();
        const double yaw = pose.Rot().Yaw();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const ignition::math::Vector3d omega(0.0, 0.0, twist.wz);
        ignition::math::Vector3d linear(c * twist.vx - s * twist.vy, s * twist.vx + c * twist.vy, 0.0);
        // SetLinearVel acts on the centre of mass, which is offset from the frame origin.
        linear += omega.Cross(pose.Rot().RotateVector(base_->GetInertial()->CoG()));
        linear.Z(base_->WorldCoGLinearVel().Z());

        ignition::math::Vector3d angular = base_->WorldAngularVel();
        angular.Z(twist.wz);

        base_->SetLinearVel(linear);
        base_->SetAngularVel(angular);
    }

    std::unique_ptr<qd_kinematics::KinematicsModel> kinematics_;
    std::vector<gazebo::physics::JointPtr> steer_joints_;
    std::vector<gazebo::physics::JointPtr> drive_joints_;
    gazebo::physics::LinkPtr base_;
    gazebo::event::ConnectionPtr update_connection_;
    double wheel_radius_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(QdKinematicBasePlugin)

} // namespace wonik_gazebo
