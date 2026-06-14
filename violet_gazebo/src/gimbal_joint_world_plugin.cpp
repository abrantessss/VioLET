#include <functional>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class GimbalJointWorldPlugin : public WorldPlugin
{
  public:
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
    {
      this->world_ = world;

      this->model_name_ = this->GetSdfString(sdf, "model", "drone1");
      this->link_name_ = this->GetSdfString(sdf, "link", "base_link");
      this->joint_name_ = this->GetSdfString(sdf, "joint_name", "world_gimbal_joint");
      this->center_of_mass_model_names_ = this->GetSdfStrings(sdf, "center_of_mass_model");
      if (this->center_of_mass_model_names_.empty()) {
        this->center_of_mass_model_names_.push_back(this->model_name_);
      }

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GimbalJointWorldPlugin::OnWorldUpdate, this, std::placeholders::_1));
    }

  private:
    std::string GetSdfString(
      const sdf::ElementPtr & sdf,
      const std::string & name,
      const std::string & fallback) const
    {
      if (sdf && sdf->HasElement(name)) {
        return sdf->Get<std::string>(name);
      }
      return fallback;
    }

    std::vector<std::string> GetSdfStrings(
      const sdf::ElementPtr & sdf,
      const std::string & name) const
    {
      std::vector<std::string> values;
      if (!sdf || !sdf->HasElement(name)) {
        return values;
      }

      auto element = sdf->GetElement(name);
      while (element) {
        values.push_back(element->Get<std::string>());
        element = element->GetNextElement(name);
      }
      return values;
    }

    bool CombinedCenterOfMass(ignition::math::Vector3d & center_of_mass) const
    {
      double total_mass = 0.0;
      center_of_mass.Set(0.0, 0.0, 0.0);

      for (const auto & model_name : this->center_of_mass_model_names_) {
        const auto model = this->world_->ModelByName(model_name);
        if (!model) {
          return false;
        }

        for (const auto & link : model->GetLinks()) {
          if (!link || !link->GetInertial()) {
            continue;
          }

          const double mass = link->GetInertial()->Mass();
          if (mass <= 0.0) {
            continue;
          }

          center_of_mass += link->WorldCoGPose().Pos() * mass;
          total_mass += mass;
        }
      }

      if (total_mass <= 0.0) {
        return false;
      }

      center_of_mass /= total_mass;
      return true;
    }

    void OnWorldUpdate(const common::UpdateInfo &)
    {
      if (this->attached_ || this->failed_ || !this->world_) {
        return;
      }

      const auto model = this->world_->ModelByName(this->model_name_);
      if (!model) {
        return;
      }

      const auto link = model->GetLink(this->link_name_);
      if (!link) {
        gzerr << "[GimbalJointWorldPlugin] Failed to find link "
              << this->model_name_ << "::" << this->link_name_ << ".\n";
        this->failed_ = true;
        return;
      }

      ignition::math::Vector3d anchor;
      if (!this->CombinedCenterOfMass(anchor)) {
        return;
      }

      const auto physics = this->world_->Physics();
      if (!physics) {
        gzerr << "[GimbalJointWorldPlugin] Physics engine is not available.\n";
        this->failed_ = true;
        return;
      }

      this->joint_ = physics->CreateJoint("ball", model);
      if (!this->joint_) {
        gzerr << "[GimbalJointWorldPlugin] Failed to create ball joint.\n";
        this->failed_ = true;
        return;
      }

      const ignition::math::Pose3d anchor_world_pose(
        anchor.X(), anchor.Y(), anchor.Z(), 0.0, 0.0, 0.0);
      const ignition::math::Vector3d anchor_in_link =
        anchor_world_pose.CoordPositionSub(link->WorldPose());
      const ignition::math::Pose3d anchor_link_pose(
        anchor_in_link.X(), anchor_in_link.Y(), anchor_in_link.Z(), 0.0, 0.0, 0.0);
      this->joint_->SetName(this->joint_name_);
      this->joint_->Attach(physics::LinkPtr(), link);
      this->joint_->Load(physics::LinkPtr(), link, anchor_link_pose);
      this->joint_->SetAnchor(0, anchor);
      this->joint_->SetModel(model);
      this->joint_->Init();

      this->attached_ = true;
      gzmsg << "[GimbalJointWorldPlugin] Attached world to "
            << this->model_name_ << "::" << this->link_name_
            << " with ball joint " << this->joint_name_
            << " at combined center of mass ["
            << anchor.X() << ", " << anchor.Y() << ", " << anchor.Z() << "]"
            << " world / ["
            << anchor_in_link.X() << ", " << anchor_in_link.Y() << ", "
            << anchor_in_link.Z() << "] link"
            << ". Translation is locked; roll, pitch, and yaw remain free.\n";
      this->update_connection_.reset();
    }

    physics::WorldPtr world_;
    physics::JointPtr joint_;
    event::ConnectionPtr update_connection_;
    std::string model_name_;
    std::string link_name_;
    std::string joint_name_;
    std::vector<std::string> center_of_mass_model_names_;
    bool attached_{false};
    bool failed_{false};
};

GZ_REGISTER_WORLD_PLUGIN(GimbalJointWorldPlugin)
}  // namespace gazebo
