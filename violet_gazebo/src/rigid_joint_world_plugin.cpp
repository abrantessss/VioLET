#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

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
      this->child_cog_offset_from_parent_cog_ = this->GetSdfVector3(
        sdf, "child_cog_offset_from_parent_cog", ignition::math::Vector3d(0.0, 0.0, -0.5));
      this->center_of_mass_log_period_s_ =
        this->GetSdfDouble(sdf, "center_of_mass_log_period_s", 1.0);

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

    double GetSdfDouble(
      const sdf::ElementPtr & sdf,
      const std::string & name,
      const double fallback) const
    {
      if (sdf && sdf->HasElement(name)) {
        return sdf->Get<double>(name);
      }
      return fallback;
    }

    ignition::math::Vector3d GetSdfVector3(
      const sdf::ElementPtr & sdf,
      const std::string & name,
      const ignition::math::Vector3d & fallback) const
    {
      if (sdf && sdf->HasElement(name)) {
        return sdf->Get<ignition::math::Vector3d>(name);
      }
      return fallback;
    }

    bool ModelCenterOfMass(
      const physics::ModelPtr & model,
      ignition::math::Vector3d & center_of_mass,
      double & total_mass) const
    {
      if (!model) {
        return false;
      }

      total_mass = 0.0;
      center_of_mass.Set(0.0, 0.0, 0.0);

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

      if (total_mass <= 0.0) {
        return false;
      }

      center_of_mass /= total_mass;
      return true;
    }

    ignition::math::Pose3d ChildCogPose(const physics::LinkPtr & child_link) const
    {
      if (child_link->GetInertial()) {
        return child_link->GetInertial()->Pose();
      }

      gzerr << "[RigidJointWorldPlugin] Failed to find inertial pose for "
            << this->child_model_name_ << "::" << this->child_link_name_
            << "; using link origin as fixed joint anchor.\n";
      return ignition::math::Pose3d();
    }

    void ZeroModelMotion(const physics::ModelPtr & model) const
    {
      model->SetLinearVel(ignition::math::Vector3d::Zero);
      model->SetAngularVel(ignition::math::Vector3d::Zero);
    }

    bool PlaceChildModel(
      const physics::ModelPtr & parent_model,
      const physics::ModelPtr & child_model,
      const physics::LinkPtr & parent_link,
      const physics::LinkPtr & child_link,
      ignition::math::Pose3d & anchor_world_pose) const
    {
      ignition::math::Vector3d parent_com;
      double parent_mass = 0.0;
      if (!this->ModelCenterOfMass(parent_model, parent_com, parent_mass)) {
        gzerr << "[RigidJointWorldPlugin] Failed to compute parent center of mass.\n";
        return false;
      }

      const ignition::math::Pose3d child_link_in_model =
        child_link->WorldPose() - child_model->WorldPose();
      const ignition::math::Pose3d child_cog_in_link = this->ChildCogPose(child_link);
      const ignition::math::Vector3d desired_child_cog =
        parent_com + parent_link->WorldPose().Rot() * this->child_cog_offset_from_parent_cog_;

      anchor_world_pose = ignition::math::Pose3d(
        desired_child_cog,
        parent_link->WorldPose().Rot());
      const ignition::math::Pose3d desired_child_link_pose =
        anchor_world_pose * child_cog_in_link.Inverse();
      const ignition::math::Pose3d desired_child_model_pose =
        desired_child_link_pose * child_link_in_model.Inverse();

      child_model->SetWorldPose(desired_child_model_pose);
      this->ZeroModelMotion(parent_model);
      this->ZeroModelMotion(child_model);
      return true;
    }

    void LogCenterOfMassDistances(
      const physics::ModelPtr & parent_model,
      const physics::ModelPtr & child_model) const
    {
      ignition::math::Vector3d parent_com;
      ignition::math::Vector3d child_com;
      double parent_mass = 0.0;
      double child_mass = 0.0;

      if (!this->ModelCenterOfMass(parent_model, parent_com, parent_mass) ||
        !this->ModelCenterOfMass(child_model, child_com, child_mass))
      {
        gzerr << "[RigidJointWorldPlugin] Failed to compute centers of mass for "
              << this->parent_model_name_ << " and " << this->child_model_name_ << ".\n";
        return;
      }

      const ignition::math::Vector3d vehicle_delta = child_com - parent_com;
      const ignition::math::Vector3d combined_com =
        (parent_com * parent_mass + child_com * child_mass) / (parent_mass + child_mass);
      const ignition::math::Vector3d fixed_wing_delta = child_com - combined_com;
      /*
      gzmsg << "[RigidJointWorldPlugin] Vehicle COG distance "
            << this->parent_model_name_ << " -> " << this->child_model_name_
            << ": parent=(" << parent_com.X() << ", " << parent_com.Y() << ", "
            << parent_com.Z() << ") m, child=(" << child_com.X() << ", "
            << child_com.Y() << ", " << child_com.Z() << ") m, delta=("
            << vehicle_delta.X() << ", " << vehicle_delta.Y() << ", "
            << vehicle_delta.Z() << ") m, distance=" << vehicle_delta.Length()
            << " m.\n";

      gzmsg << "[RigidJointWorldPlugin] Combined COG to fixed-wing COG: combined=("
            << combined_com.X() << ", " << combined_com.Y() << ", "
            << combined_com.Z() << ") m, fixed_wing=(" << child_com.X() << ", "
            << child_com.Y() << ", " << child_com.Z() << ") m, delta=("
            << fixed_wing_delta.X() << ", " << fixed_wing_delta.Y() << ", "
            << fixed_wing_delta.Z() << ") m, distance=" << fixed_wing_delta.Length()
            << " m.\n";*/
    }

    void OnWorldUpdate(const common::UpdateInfo &)
    {
      if (this->failed_ || !this->world_) {
        return;
      }

      const auto parent_model = this->world_->ModelByName(this->parent_model_name_);
      const auto child_model = this->world_->ModelByName(this->child_model_name_);
      if (!parent_model || !child_model) {
        return;
      }

      if (this->attached_) {
        const double sim_time_s = this->world_->SimTime().Double();
        if (this->center_of_mass_log_period_s_ > 0.0 &&
          sim_time_s - this->last_center_of_mass_log_s_ >= this->center_of_mass_log_period_s_)
        {
          this->LogCenterOfMassDistances(parent_model, child_model);
          this->last_center_of_mass_log_s_ = sim_time_s;
        }
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

      ignition::math::Pose3d anchor_world_pose;
      if (!this->PlaceChildModel(parent_model, child_model, parent_link, child_link, anchor_world_pose)) {
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

      const ignition::math::Pose3d child_cog_in_link = this->ChildCogPose(child_link);
      this->ZeroModelMotion(parent_model);
      this->ZeroModelMotion(child_model);
      this->joint_->SetName(this->joint_name_);
      this->joint_->Attach(parent_link, child_link);
      this->joint_->Load(parent_link, child_link, child_cog_in_link);
      this->joint_->SetAnchor(0, anchor_world_pose.Pos());
      this->joint_->SetModel(child_model);
      this->joint_->Init();

      this->attached_ = true;
      gzmsg << "[RigidJointWorldPlugin] Attached "
            << this->parent_model_name_ << "::" << this->parent_link_name_
            << " to " << this->child_model_name_ << "::" << this->child_link_name_
            << " with joint " << this->joint_name_
            << " at child COG offset ["
            << this->child_cog_offset_from_parent_cog_.X() << ", "
            << this->child_cog_offset_from_parent_cog_.Y() << ", "
            << this->child_cog_offset_from_parent_cog_.Z() << "] m.\n";
      this->LogCenterOfMassDistances(parent_model, child_model);
      this->last_center_of_mass_log_s_ = this->world_->SimTime().Double();
    }

    physics::WorldPtr world_;
    physics::JointPtr joint_;
    event::ConnectionPtr update_connection_;
    std::string parent_model_name_;
    std::string parent_link_name_;
    std::string child_model_name_;
    std::string child_link_name_;
    std::string joint_name_;
    ignition::math::Vector3d child_cog_offset_from_parent_cog_{0.0, 0.0, -0.5};
    double center_of_mass_log_period_s_{1.0};
    double last_center_of_mass_log_s_{0.0};
    bool attached_{false};
    bool failed_{false};
};

GZ_REGISTER_WORLD_PLUGIN(RigidJointWorldPlugin)
}  // namespace gazebo
