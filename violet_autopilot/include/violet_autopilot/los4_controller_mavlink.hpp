#pragma once 

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include <mavsdk/plugins/offboard/offboard.h>
#include <utility>

#include <controller.hpp>

namespace autopilot{
  class LOS4ControllerMavlink : public autopilot::Controller {
    public:
      explicit LOS4ControllerMavlink(std::shared_ptr<mavsdk::Offboard> offboard)
        : offboard_(std::move(offboard)) {}
      ~LOS4ControllerMavlink();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;
    
    protected:
      std::shared_ptr<mavsdk::Offboard> offboard_;

      // Variables
      double gamma_{0.0};
      double m_;
      double Tmax_;
      double k1_;
      double k2_;
      double kpE_;
      double kiE_;
      double kpB_;
      double kiB_;
      double ka_;
      double kphi_;
      double ktheta_;
      double throttle_min_;
      double throttle_max_;
      double pitch_min_;
      double pitch_max_;
      double lateral_acceleration_min_;
      double lateral_acceleration_max_;
      double z_ref_;
      double previous_roll_cmd_{0.0};
      double previous_pitch_cmd_{0.0};
      double E_err_int_{0.0};
      double B_err_int_{0.0};
      bool z_ref_initialized_{false};
      bool attitude_cmd_initialized_{false};
  };
}
