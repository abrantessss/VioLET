#pragma once

#include <vector>

#include<Eigen/Dense>

#include"rclcpp/rclcpp.hpp"
#include "violet_msgs/msg/autopilot_plot.hpp"

struct Path {
  int type;
  
  // waypoint
  Eigen::Vector3d waypoint{Eigen::Vector3d::Zero()};

  // line
  Eigen::Vector3d line_p0{Eigen::Vector3d::Zero()};
  Eigen::Vector3d line_p1{Eigen::Vector3d::Zero()};
  double line_v{0};

  // circle
  Eigen::Vector3d circle_c{Eigen::Vector3d::Zero()};
  double circle_R{0};
  double circle_v{0};

  // lemniscate
  Eigen::Vector3d lemniscate_c{Eigen::Vector3d::Zero()};
  double lemniscate_a{0};
  double lemniscate_v{0};
};

namespace autopilot {
  class Controller {
    public:

    using SharedPtr = std::shared_ptr<Controller>;
    using UniquePtr = std::unique_ptr<Controller>;
    using WeakPtr = std::weak_ptr<Controller>;

      virtual ~Controller() = default;

      struct Config {
        rclcpp::Node::SharedPtr node;
        int vehicle_id{1};
      };
      
      inline void initialize_controller(const Controller::Config & config) {
        // init base class
        node_ = config.node;
        vehicle_id_ = config.vehicle_id;

        plot_pub_ = node_->create_publisher<violet_msgs::msg::AutopilotPlot>(
          "fmu/telemetry/autopilot_plot",
          rclcpp::SensorDataQoS());

        initialize();
      }

    virtual void initialize() = 0;

    virtual void reset() {};

    virtual void set_position(const double dt, const Eigen::Vector3d& p) = 0;
    
    virtual void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) = 0;

    virtual void set_path(const int type, const double* path) = 0;

    protected:
      inline void publish_plot_data(
        const double gamma,
        const double vd,
        const Eigen::Vector3d & pd,
        const Eigen::Vector3d & dpd_dgamma,
        const std::vector<float> & gains)
      {
        violet_msgs::msg::AutopilotPlot msg;
        msg.header.stamp = node_->get_clock()->now();
        msg.gamma = static_cast<float>(gamma);
        msg.vd = static_cast<float>(vd);
        msg.pd[0] = static_cast<float>(pd.x());
        msg.pd[1] = static_cast<float>(pd.y());
        msg.pd[2] = static_cast<float>(pd.z());
        msg.dpd_dgamma[0] = static_cast<float>(dpd_dgamma.x());
        msg.dpd_dgamma[1] = static_cast<float>(dpd_dgamma.y());
        msg.dpd_dgamma[2] = static_cast<float>(dpd_dgamma.z());
        msg.gains = gains;
        plot_pub_->publish(msg);
      }

      rclcpp::Node::SharedPtr node_{nullptr};
      int vehicle_id_{1};
      Path path_;
      rclcpp::Publisher<violet_msgs::msg::AutopilotPlot>::SharedPtr plot_pub_{nullptr};
  };
}
