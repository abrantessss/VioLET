#pragma once

#include<Eigen/Dense>

#include"rclcpp/rclcpp.hpp"

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

      struct Config {
        rclcpp::Node::SharedPtr node;
      };
      
      inline void initialize_controller(const Controller::Config & config) {
        // init base class
        node_ = config.node;

        initialize();
      }

    virtual void initialize() = 0;

    virtual void reset() {};

    virtual void set_position(const double dt, const Eigen::Vector3d& p) = 0;
    
    virtual void set_attitude_rate(const double dt, const Eigen::Vector3d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& eta) = 0;

    virtual void set_path(const int type, const double* path) = 0;

    protected:
      rclcpp::Node::SharedPtr node_{nullptr};
      Path path_;
  };
}
