#pragma once

#include <memory>
#include <sstream>
#include <map>
#include <iomanip>
#include <array>
#include <Eigen/Dense>
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"


// Maps
const std::map<uint8_t, std::string> mode_map = {
  {0, "ARM    "},
  {1, "DISARM "},
  {2, "TAKEOFF"},
  {3, "HOLD   "},
  {4, "LAND   "},
  {5, "KILL   "}
};

// Structures
struct State {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d attitude{Eigen::Vector3d::Zero()};
  Eigen::Vector3d inertial_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

struct Battery {
  float soc{0.0};
  float voltage{0.0};
  float current{0.0};
};

struct Status {
  uint8_t vehicle_id {0};
  bool armed {false};
  uint8_t mode {1}; // Default Disarm
};

struct Telemetry {
  Status status;
  Battery battery;
  State state;
};

struct Trajectories {
  // Waypoint UI data
  std::array<std::string, 3> waypoint_pos_input{"", "", ""};

  Eigen::Vector3d waypoint{Eigen::Vector3d::Zero()};

  // Line UI data
  std::array<std::string, 6> line_pos_input{"", "", "", "", "", ""};
  std::string line_speed_input{""};

  Eigen::Matrix<double, 6, 1> line{Eigen::Matrix<double, 6, 1>::Zero()};
  float line_speed{0.0};

  // Circle UI data
  std::array<std::string, 4> circle_pos_input{"", "", "", ""};
  std::string circle_speed_input{""};

  Eigen::Matrix<double, 4, 1> circle{Eigen::Matrix<double, 4, 1>::Zero()};
  float circle_speed{0.0};

  
  // Lemniscate UI data
  std::array<std::string, 4> lemniscate_pos_input{"", "", "", ""};
  std::string lemniscate_speed_input{""};

  Eigen::Matrix<double, 4, 1> lemniscate{Eigen::Matrix<double, 4, 1>::Zero()};
  float lemniscate_speed{0.0};
};

class ConsoleUI {
  
  public:

    using UniquePtr = std::unique_ptr<ConsoleUI>;
    
    // Structures
    struct Config {
      // Control Funtions
      std::function<void()> on_arm_click;
      std::function<void()> on_disarm_click;
      std::function<void()> on_takeoff_click;
      std::function<void()> on_hold_click;
      std::function<void()> on_land_click;
      std::function<void()> on_kill_click;

      // Trajectory Functions
      std::function<void()> on_waypoint_click;
      std::function<void()> on_line_click;
      std::function<void()> on_circle_click;
      std::function<void()> on_lemniscate_click;
    };

    ConsoleUI(const Config & config);
    ~ConsoleUI();

    void clear_terminal();
    void loop();

    // Data
    Telemetry telemetry_data_;
    Trajectories trajectory_data_;

  protected:

    // Aux Function float to string
    std::string f2s(float value, int precision=2) const;

    // Screen
    ftxui::ScreenInteractive screen_;

    // Components
    ftxui::Component telemetry();
    ftxui::Component control();

    // UI Config
    Config config_;
};
