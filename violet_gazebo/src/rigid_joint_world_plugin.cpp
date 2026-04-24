#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class RigidJointWorldPlugin : public WorldPlugin
{
  public:
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
    {
      this->world_ = world;

      this->parent_model_name_ = this->GetSdfString(sdf, "parent_model", "drone1");
      this->parent_link_name_ = this->GetSdfString(sdf, "parent_link", "base_link");
      this->child_model_name_ = this->GetSdfString(sdf, "child_model", "drone2");
      this->child_link_name_ = this->GetSdfString(sdf, "child_link", "base_link");
      this->joint_name_ = this->GetSdfString(sdf, "joint_name", "drone1_drone2_fixed_joint");

      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RigidJointWorldPlugin::OnWorldUpdate, this, std::placeholders::_1));
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

    void OnWorldUpdate(const common::UpdateInfo &)
    {
      if (this->attached_ || this->failed_ || !this->world_) {
        return;
      }

      const auto parent_model = this->world_->ModelByName(this->parent_model_name_);
      const auto child_model = this->world_->ModelByName(this->child_model_name_);
      if (!parent_model || !child_model) {
        return;
      }

      const auto parent_link = parent_model->GetLink(this->parent_link_name_);
      const auto child_link = child_model->GetLink(this->child_link_name_);
      if (!parent_link || !child_link) {
        gzerr << "[RigidJointWorldPlugin] Failed to find links "
              << this->parent_model_name_ << "::" << this->parent_link_name_
              << " or " << this->child_model_name_ << "::" << this->child_link_name_
              << ".\n";
        this->failed_ = true;
        return;
      }

      const auto physics = this->world_->Physics();
      if (!physics) {
        gzerr << "[RigidJointWorldPlugin] Physics engine is not available.\n";
        this->failed_ = true;
        return;
      }

      this->joint_ = physics->CreateJoint("fixed", parent_model);
      if (!this->joint_) {
        gzerr << "[RigidJointWorldPlugin] Failed to create fixed joint.\n";
        this->failed_ = true;
        return;
      }

      this->joint_->SetName(this->joint_name_);
      this->joint_->Attach(parent_link, child_link);
      this->joint_->Load(parent_link, child_link, ignition::math::Pose3d());
      this->joint_->SetModel(child_model);
      this->joint_->Init();

      this->attached_ = true;
      gzmsg << "[RigidJointWorldPlugin] Attached "
            << this->parent_model_name_ << "::" << this->parent_link_name_
            << " to " << this->child_model_name_ << "::" << this->child_link_name_
            << " with joint " << this->joint_name_ << ".\n";
      this->update_connection_.reset();
    }

    physics::WorldPtr world_;
    physics::JointPtr joint_;
    event::ConnectionPtr update_connection_;
    std::string parent_model_name_;
    std::string parent_link_name_;
    std::string child_model_name_;
    std::string child_link_name_;
    std::string joint_name_;
    bool attached_{false};
    bool failed_{false};
};

GZ_REGISTER_WORLD_PLUGIN(RigidJointWorldPlugin)
}  // namespace gazebo
