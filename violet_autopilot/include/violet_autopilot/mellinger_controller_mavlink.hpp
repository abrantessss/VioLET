#pragma once

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"

#include <mavsdk/plugins/offboard/offboard.h>

#include <controller.hpp>

namespace autopilot {
  class MellingerControllerMavlink : public autopilot::Controller {
    public:
      explicit MellingerControllerMavlink(std::shared_ptr<mavsdk::Offboard> offboard)
        : offboard_(std::move(offboard)) {}
      ~MellingerControllerMavlink();

      void initialize() override;

      void set_position(const double dt, const Eigen::Vector3d& p) override;

      void set_attitude(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v) override;

      void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) override;

      void set_path(const int type, const double* path) override;

    protected:
      std::shared_ptr<mavsdk::Offboard> offboard_;

      // Variables
      double gamma_{0.0};
      double mass_;
      Eigen::Vector3d g_{Eigen::Vector3d(0.0, 0.0, 9.81)};
      Eigen::Matrix3d kp_;
      Eigen::Matrix3d ki_;
      Eigen::Matrix3d kd_;
      Eigen::Matrix3d kr_;
  };
}
