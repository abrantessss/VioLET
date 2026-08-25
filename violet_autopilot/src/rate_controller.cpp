#include "rate_controller.hpp"

#include <algorithm>
#include <cmath>

namespace autopilot {
  RateController::GainForm RateController::gain_form_from_string(
    const std::string& value,
    bool* valid) {
    if (value == "standard") {
      if (valid != nullptr) {
        *valid = true;
      }
      return GainForm::Standard;
    }
    if (valid != nullptr) {
      *valid = value == "parallel";
    }
    return GainForm::Parallel;
  }

  void RateController::configure(const Config& config) {
    config_ = config;
    reset();
  }

  void RateController::reset() {
    integral_.setZero();
    previous_omega_.setZero();
    debug_ = Debug{};
    have_previous_omega_ = false;
  }

  Eigen::Vector3d RateController::update(
    double dt,
    const Eigen::Vector3d& omega_sp,
    const Eigen::Vector3d& omega_measured) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
      dt = 0.01;
    }

    const Eigen::Vector3d error = omega_sp - omega_measured;
    Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
    if (have_previous_omega_) {
      derivative = -(omega_measured - previous_omega_) / dt;
    }
    previous_omega_ = omega_measured;
    have_previous_omega_ = true;

    const Eigen::Vector3d candidate_integral = integral_ + error * dt;
    Eigen::Vector3d clamped_candidate_integral = candidate_integral;
    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.integral_limits[axis];
      clamped_candidate_integral(axis) =
        std::clamp(clamped_candidate_integral(axis), limit.min, limit.max);
    }

    const Eigen::Vector3d unsaturated_with_candidate =
      output_from_integral(error, clamped_candidate_integral, derivative);
    Eigen::Vector3d saturated = unsaturated_with_candidate;
    Eigen::Vector3d accepted_integral = integral_;

    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.output_limits[axis];
      saturated(axis) = std::clamp(saturated(axis), limit.min, limit.max);

      const bool inside =
        unsaturated_with_candidate(axis) >= limit.min &&
        unsaturated_with_candidate(axis) <= limit.max;
      const bool high_and_reducing =
        unsaturated_with_candidate(axis) > limit.max && error(axis) < 0.0;
      const bool low_and_reducing =
        unsaturated_with_candidate(axis) < limit.min && error(axis) > 0.0;

      if (inside || high_and_reducing || low_and_reducing) {
        accepted_integral(axis) = clamped_candidate_integral(axis);
      }
    }
    integral_ = accepted_integral;

    const Eigen::Vector3d unsaturated = output_from_integral(error, integral_, derivative);
    for (int axis = 0; axis < 3; ++axis) {
      const auto& limit = config_.output_limits[axis];
      saturated(axis) = std::clamp(unsaturated(axis), limit.min, limit.max);
    }

    debug_.error = error;
    debug_.integral = integral_;
    debug_.derivative = derivative;
    debug_.torque_unsaturated_Nm = unsaturated;
    debug_.torque_saturated_Nm = saturated;
    debug_.omega_measured_filtered = omega_measured;

    return saturated;
  }

  const RateController::Debug& RateController::debug() const {
    return debug_;
  }

  const RateController::Config& RateController::config() const {
    return config_;
  }

  double RateController::axis_output(
    const int axis,
    const double error,
    const double integral,
    const double derivative) const {
    const AxisGains gains[3] = {config_.roll, config_.pitch, config_.yaw};
    const AxisGains& gain = gains[axis];

    if (config_.gain_form == GainForm::Standard) {
      return gain.kp * (error + gain.ki * integral + gain.kd * derivative);
    }
    return gain.kp * error + gain.ki * integral + gain.kd * derivative;
  }

  Eigen::Vector3d RateController::output_from_integral(
    const Eigen::Vector3d& error,
    const Eigen::Vector3d& integral,
    const Eigen::Vector3d& derivative) const {
    Eigen::Vector3d output;
    for (int axis = 0; axis < 3; ++axis) {
      output(axis) = axis_output(axis, error(axis), integral(axis), derivative(axis));
    }
    return output;
  }
} // namespace autopilot