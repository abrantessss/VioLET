#pragma once

#include <memory>
#include <sstream>
#include "ftxui/component/component.hpp"
#include "ftxui/component/screen_interactive.hpp"

class ConsoleUI {
  
  public:

    using UniquePtr = std::unique_ptr<ConsoleUI>;

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
    
    ConsoleUI();
    ~ConsoleUI();

    void clear_terminal();
    void loop();


  protected:

    // Aux Function float to string
    std::string f2s(float value, int precision=2) const;

    // Screen
    ftxui::ScreenInteractive screen_;

    // Components
    ftxui::Component telemetry();
    ftxui::Component control();

    
    std::string input_ = "0.0";
};
