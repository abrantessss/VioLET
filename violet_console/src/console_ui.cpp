#include "console_ui.hpp"
#include "ftxui/component/captured_mouse.hpp"  
#include "ftxui/component/component.hpp" 
#include "ftxui/component/component_base.hpp"      
#include "ftxui/component/component_options.hpp"   
#include "ftxui/component/screen_interactive.hpp"  
#include "ftxui/dom/elements.hpp"  
#include "ftxui/screen/color.hpp" 

ConsoleUI::ConsoleUI() : screen_(ftxui::ScreenInteractive::Fullscreen()) {
  // Clear terminal at build
  clear_terminal();
}

ConsoleUI::~ConsoleUI() {
  // Clear terminal at finish time
  clear_terminal();
}

void ConsoleUI::clear_terminal() {
  printf("\033[2J\033[1;1H");
}

std::string ConsoleUI::f2s(float value, int precision) const {
  // String stream to set the precision of the floats to represent on the UI
  std::stringstream stream;
  
  // Set the precision of the floats to represent in the UI to 2 decimal places
  stream.str("");
  stream.precision(precision);
  stream << value;
  return stream.str();
}

void ConsoleUI::loop() {
  
  // Console Title
  auto top_container = ftxui::Renderer([] {
    return ftxui::hbox({
             ftxui::text("Vio") | ftxui::color(ftxui::Color::White) | ftxui::bold,
             ftxui::text("LET") | ftxui::color(ftxui::Color::RGB(153, 67, 202)) | ftxui::bold,
             ftxui::text(" Console") | ftxui::color(ftxui::Color::White) | ftxui::bold,
           }) | ftxui::center | ftxui::border | ftxui::flex;
  }); 

  int left_size = 43;
  int top_size = 3;
  auto telemetry_component = telemetry();
  auto control_component = control();

  auto aux_container = ftxui::Container::Horizontal({
    telemetry_component,
    control_component,
  });

  auto document = ftxui::Container::Vertical({
    top_container,
    aux_container,
  });

  auto ui = ftxui::Renderer(document, [&] {
    return ftxui::vbox({
      top_container->Render() | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, top_size),
      ftxui::hbox({
        telemetry_component->Render() | ftxui::size(ftxui::WIDTH, ftxui::EQUAL, left_size),
        control_component->Render() | ftxui::flex,
      }) | ftxui::flex,
    });
  });

  screen_.Loop(ui);
}


ftxui::Component ConsoleUI::telemetry() {
  auto telemetry = ftxui::Renderer([this] {
    return ftxui::vbox({
      // Telemetry title
      ftxui::hbox({
        ftxui::text("Telemetry") | ftxui::color(ftxui::Color::RGB(153, 67, 202)) | ftxui::bold  
      }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3),

      // Status & Battery titles
      ftxui::hbox({
        ftxui::vbox({
          ftxui::text("Status") | ftxui::color(ftxui::Color::White) | ftxui::bold
        }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3) | ftxui::flex,

        ftxui::vbox({
          ftxui::text("Battery") | ftxui::color(ftxui::Color::White) | ftxui::bold
        }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3) | ftxui::flex
      }),

      // Status & Battery parameters
      ftxui::hbox({
        ftxui::vbox({
          ftxui::text("ID   : ") | ftxui::color(ftxui::Color::White),
          ftxui::text("Armed: ") | ftxui::color(ftxui::Color::White),
          ftxui::text("Mode : ") | ftxui::color(ftxui::Color::White)
        }) | ftxui::border | ftxui::xflex_grow | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 5),

        ftxui::vbox({
          ftxui::text("SoC  : ") | ftxui::color(ftxui::Color::White),
          ftxui::text("Volt.: ") | ftxui::color(ftxui::Color::White),
          ftxui::text("Curr.: ") | ftxui::color(ftxui::Color::White)
        }) | ftxui::border | ftxui::xflex_grow | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 5)
      }),

      // State title
      ftxui::hbox({
        ftxui::text("State") | ftxui::color(ftxui::Color::White) | ftxui::bold
      }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3),

      // State parameters
      ftxui::vbox({
        ftxui::text("Position     : " + input_) | ftxui::color(ftxui::Color::White),
        ftxui::text("Orientation  : ") | ftxui::color(ftxui::Color::White),
        ftxui::text("Inertial Vel.: ") | ftxui::color(ftxui::Color::White),
        ftxui::text("Angular Vel. : ") | ftxui::color(ftxui::Color::White),
        ftxui::vbox({
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text("                     ,---."),
          ftxui::text("                    /__./|"),
          ftxui::text("               ,---.;  ; | "),
          ftxui::text("              /___/ \\  | |"),
          ftxui::text("              \\   ;  \\ ' |"),
          ftxui::text("               \\   \\  \\: |"),
          ftxui::text("                ;   \\  ' ."),
          ftxui::text("                 \\   \\   '"),
          ftxui::text("                  \\   `  ;"),
          ftxui::text("                   :   \\ |"),
          ftxui::text("                    '---\""),
        }) | ftxui::color(ftxui::Color::White)  | ftxui::bold
      }) | ftxui::border | ftxui::flex
    });
    
  });

  return telemetry;
}



ftxui::Component ConsoleUI::control() {
  auto arm_button = ftxui::Button(
    "Arm",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto disarm_button = ftxui::Button(
    "Disarm",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(153, 67, 202))
  );
  auto takeoff_button = ftxui::Button(
    "Takeoff",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto hold_button = ftxui::Button(
    "Hold",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto land_button = ftxui::Button(
    "Land",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto kill_button = ftxui::Button(
    "Kill",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(153, 67, 202))
  );
  auto waypoint_button = ftxui::Button(
    "  Go  ",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto line_button = ftxui::Button(
    "  Go  ",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto circle_button = ftxui::Button(
    "  Go  ",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto lemniscate_button = ftxui::Button(
    "  Go  ",
    [] {},
    ftxui::ButtonOption::Animated(ftxui::Color::RGB(75, 155, 222))
  );
  auto waypoint_x_input = ftxui::Input(&input_, "0.0");
  auto waypoint_y_input = ftxui::Input(&input_, "0.0");
  auto waypoint_z_input = ftxui::Input(&input_, "0.0");
  auto line_x0_input = ftxui::Input(&input_, "0.0");
  auto line_y0_input = ftxui::Input(&input_, "0.0");
  auto line_z0_input = ftxui::Input(&input_, "0.0");
  auto line_x1_input = ftxui::Input(&input_, "0.0");
  auto line_y1_input = ftxui::Input(&input_, "0.0");
  auto line_z1_input = ftxui::Input(&input_, "0.0");
  auto line_v_input = ftxui::Input(&input_, "0.0");
  auto circle_x_input = ftxui::Input(&input_, "0.0");
  auto circle_y_input = ftxui::Input(&input_, "0.0");
  auto circle_z_input = ftxui::Input(&input_, "0.0");
  auto circle_r_input = ftxui::Input(&input_, "0.0");
  auto circle_v_input = ftxui::Input(&input_, "0.0");
  auto lemniscate_x_input = ftxui::Input(&input_, "0.0");
  auto lemniscate_y_input = ftxui::Input(&input_, "0.0");
  auto lemniscate_z_input = ftxui::Input(&input_, "0.0");
  auto lemniscate_r_input = ftxui::Input(&input_, "0.0");
  auto lemniscate_v_input = ftxui::Input(&input_, "0.0");

  auto container = ftxui::Container::Horizontal({
    arm_button,
    disarm_button,
    takeoff_button,
    hold_button,
    land_button,
    kill_button,
    waypoint_button,
    line_button,
    circle_button,
    lemniscate_button,
    waypoint_x_input,
    waypoint_y_input,
    waypoint_z_input,
    line_x0_input,
    line_y0_input,
    line_z0_input,
    line_x1_input,
    line_y1_input,
    line_z1_input,
    line_v_input,
    circle_x_input,
    circle_y_input,
    circle_z_input,
    circle_r_input,
    circle_v_input,
    lemniscate_x_input,
    lemniscate_y_input,
    lemniscate_z_input,
    lemniscate_r_input,
    lemniscate_v_input
  });

  auto control = ftxui::Renderer(container, [=] {
    return ftxui::vbox({
      // Control title
      ftxui::hbox({
        ftxui::text("Control") | ftxui::color(ftxui::Color::RGB(153, 67, 202)) | ftxui::bold
      }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3),
      ftxui::vbox({
        // Control Buttons
        ftxui::hbox({ 
          arm_button->Render(),
          disarm_button->Render(),
          takeoff_button->Render(),
          hold_button->Render(),
          land_button->Render(),
          kill_button->Render()
        }) | ftxui::center | ftxui::flex
      }) | ftxui::flex | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 5),
      // Trajectory title
      ftxui::hbox({
        ftxui::text("Trajectory") | ftxui::color(ftxui::Color::White) | ftxui::bold
      }) | ftxui::center | ftxui::border | ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 3),

      // State parameters
      ftxui::hbox({
        ftxui::vbox({
          ftxui::text("Waypoint") | ftxui::color(ftxui::Color::White) | ftxui::hcenter,
          ftxui::hbox({
            ftxui::text("x: ") | ftxui::color(ftxui::Color::White),
            waypoint_x_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("y: ") | ftxui::color(ftxui::Color::White),
            waypoint_y_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("z: ") | ftxui::color(ftxui::Color::White),
            waypoint_z_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text(""),
          waypoint_button->Render() | ftxui::center
        }) | ftxui::border | ftxui::flex,
        ftxui::vbox({
          ftxui::text("Line") | ftxui::color(ftxui::Color::White) | ftxui::hcenter,
          ftxui::hbox({
            ftxui::text("x0: ") | ftxui::color(ftxui::Color::White),
            line_x0_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("y0: ") | ftxui::color(ftxui::Color::White),
            line_y0_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("z0: ") | ftxui::color(ftxui::Color::White),
            line_z0_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("x1: ") | ftxui::color(ftxui::Color::White),
            line_x1_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("y1: ") | ftxui::color(ftxui::Color::White),
            line_y1_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("z1: ") | ftxui::color(ftxui::Color::White),
            line_z1_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("v : ") | ftxui::color(ftxui::Color::White),
            line_v_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::text(""),
          line_button->Render() | ftxui::center
        }) | ftxui::border | ftxui::flex,
        ftxui::vbox({
          ftxui::text("Circle") | ftxui::color(ftxui::Color::White) | ftxui::hcenter,
          ftxui::hbox({
            ftxui::text("x: ") | ftxui::color(ftxui::Color::White),
            circle_x_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("y: ") | ftxui::color(ftxui::Color::White),
            circle_y_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("z: ") | ftxui::color(ftxui::Color::White),
            circle_z_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("R: ") | ftxui::color(ftxui::Color::White),
            circle_r_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("v: ") | ftxui::color(ftxui::Color::White),
            circle_v_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text(""),
          circle_button->Render() | ftxui::center
        }) | ftxui::border | ftxui::flex,
        ftxui::vbox({
          ftxui::text("Lemniscate") | ftxui::color(ftxui::Color::White) | ftxui::hcenter,
          ftxui::hbox({
            ftxui::text("x: ") | ftxui::color(ftxui::Color::White),
            lemniscate_x_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("y: ") | ftxui::color(ftxui::Color::White),
            lemniscate_y_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("z: ") | ftxui::color(ftxui::Color::White),
            lemniscate_z_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("R: ") | ftxui::color(ftxui::Color::White),
            lemniscate_r_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::hbox({
            ftxui::text("v: ") | ftxui::color(ftxui::Color::White),
            lemniscate_v_input->Render() | ftxui::xflex_grow,
          }) | ftxui::xflex_grow,
          ftxui::text(""),
          ftxui::text(""),
          ftxui::text(""),
          lemniscate_button->Render() | ftxui::center
        }) | ftxui::border | ftxui::flex
      })
      | ftxui::flex
    });
    
  });

  return control;
}
