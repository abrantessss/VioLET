#pragma once

#include <Eigen/Dense>

#include <string>

namespace autopilot {
  class RateController {
    public:
      enum class GainForm {
        Parallel,
        Standard
      };

      struct AxisGains {
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
      };

      struct AxisLimit {
        double min{-1.0};
        double max{1.0};
      };

      struct Config {
        AxisGains roll;
        AxisGains pitch;
        AxisGains yaw;
        AxisLimit output_limits[3]{{-1.0, 1.0}, {-1.0, 1.0}, {-0.5, 0.5}};
        AxisLimit integral_limits[3]{{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};
        GainForm gain_form{GainForm::Parallel};
      };

      struct Debug {
        Eigen::Vector3d error{Eigen::Vector3d::Zero()};
        Eigen::Vector3d integral{Eigen::Vector3d::Zero()};
        Eigen::Vector3d derivative{Eigen::Vector3d::Zero()};
        Eigen::Vector3d torque_unsaturated_Nm{Eigen::Vector3d::Zero()};
        Eigen::Vector3d torque_saturated_Nm{Eigen::Vector3d::Zero()};
        Eigen::Vector3d omega_measured_filtered{Eigen::Vector3d::Zero()};
      };

      static GainForm gain_form_from_string(const std::string& value, bool* valid = nullptr);

      void configure(const Config& config);
      void reset();

      Eigen::Vector3d update(
        double dt,
        const Eigen::Vector3d& omega_sp,
        const Eigen::Vector3d& omega_measured);

      const Debug& debug() const;
      const Config& config() const;

    private:
      double axis_output(int axis, double error, double integral, double derivative) const;
      Eigen::Vector3d output_from_integral(
        const Eigen::Vector3d& error,
        const Eigen::Vector3d& integral,
        const Eigen::Vector3d& derivative) const;

      Config config_;
      Debug debug_;
      Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d previous_omega_{Eigen::Vector3d::Zero()};
      bool have_previous_omega_{false};
  };
}
