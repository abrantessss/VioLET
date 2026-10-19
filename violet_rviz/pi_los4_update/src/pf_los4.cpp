#include "los4_runtime.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Telemetry = mavsdk::Telemetry;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;
void stop(int) { stop_requested = 1; }
void usage(const char* executable) {
    const pf::Config defaults;
    std::cout << "Usage: " << executable << " [connection_url] [telemetry_rate_hz] [forward_udp_url] [--option value]\n"
              << "Connection defaults: serial:///dev/ttyAMA0:115200 10 (no forwarding)\n"
              << "--controller-enabled " << std::boolalpha << defaults.enabled << "  HARD INHIBIT when false. Never arms or takes off.\n"
              << "--test-gamma " << defaults.test << "  true: gamma test, NEVER sends control/mode commands\n"
              << "--offline " << defaults.offline << "  true: test gamma without an autopilot\n"
              << "--path circle               line | circle | lemniscate (Gerono)\n"
              << "--p0 0,0,-30 --p1 200,0,-30 --center 0,0,-30 (local NED metres)\n"
              << "--radius " << defaults.radius << " --scale " << defaults.scale << " --speed " << defaults.speed << " --gamma0 " << defaults.gamma0 << "\n"
              << "--rc-channel 7              RC mode channel, low/mid Manual, high Offboard\n"
              << "--return-mode manual        manual/stabilized/altitude/position\n"
              << "--rc-on 1700 --rc-off 1300   High >=rc-on; low/mid <rc-on\n"
              << "--bench-offboard false      Ground mode-switch test, zero thrust/rates, gamma frozen\n"
              << "--kill-channel 6 --kill-threshold 1700 --kill-active-high true\n"
              << "RC selects Offboard; script never requests entry, only fault handback.\n"
              << "--control-rate 50 --stale-timeout 0.5 --min-speed 5 --rate-limit 1.5 (rad/s)\n"
              << "--mass 1.48 --Tmax 104.720105 --throttle-model quadratic (simulation defaults)\n"
              << "--k1 1 --k2 20 --kpE 0.05 --kiE 0.005 --kpB 0.002 --kiB 0.00001\n"
              << "--ka 10 --kphi 10 --ktheta 10\n"
              << "--throttle-min 0 --throttle-max 1 --pitch-min -0.52 --pitch-max 0.52\n"
              << "--accel-min -10 --accel-max 10 (m/s^2)\n"
              << "Ctrl-C releases Offboard if active, then stops. Camera is a separate stream.\n";
}
template<typename T> struct Sample {
    std::optional<T> value;
    Clock::time_point received{};
};
struct TelemetryData {
    Sample<Telemetry::Position> position;
    Sample<Telemetry::VelocityNed> velocity;
    Sample<Telemetry::PositionVelocityNed> local;
    Sample<Telemetry::EulerAngle> attitude;
    Sample<Telemetry::ActuatorControlTarget> actuator_controls;
    Sample<Telemetry::ActuatorOutputStatus> actuator_outputs;
};
struct TelemetryCache {
    std::mutex mutex;
    TelemetryData data;
};
template<typename T>
void store(const std::shared_ptr<TelemetryCache>& cache,
           Sample<T> TelemetryData::* member, const T& value) {
    std::lock_guard<std::mutex> lock(cache->mutex);
    cache->data.*member = {value, Clock::now()};
}
template<typename T, typename Formatter>
void print_sample(const char* name, const Sample<T>& sample, Formatter format) {
    std::cout << name << ": ";
    if (!sample.value) {
        std::cout << "waiting for telemetry\n";
        return;
    }
    format(*sample.value);
    std::cout << " (age " << std::chrono::duration<double>(Clock::now()-sample.received).count()
              << " s)\n";
}
void print_data(const TelemetryData& data) {
    std::cout << std::fixed << std::setprecision(3);
    print_sample("GPS position", data.position, [](const auto& v) {
        std::cout << std::setprecision(7) << "lat=" << v.latitude_deg << " lon=" << v.longitude_deg
                  << std::setprecision(3) << " absolute_alt=" << v.absolute_altitude_m
                  << " relative_alt=" << v.relative_altitude_m << " m";
    });
    print_sample("NED velocity", data.velocity, [](const auto& v) {
        std::cout << "N=" << v.north_m_s << " E=" << v.east_m_s << " D=" << v.down_m_s << " m/s";
    });
    print_sample("Local NED", data.local, [](const auto& v) {
        std::cout << "position=[" << v.position.north_m << ", " << v.position.east_m
                  << ", " << v.position.down_m << "] m velocity=[" << v.velocity.north_m_s
                  << ", " << v.velocity.east_m_s << ", " << v.velocity.down_m_s << "] m/s";
    });
    print_sample("Euler attitude", data.attitude, [](const auto& v) {
        std::cout << "roll=" << v.roll_deg << " pitch=" << v.pitch_deg << " yaw=" << v.yaw_deg << " deg";
    });
    print_sample("Actuator control target", data.actuator_controls, [](const auto& v) {
        std::cout << "group=" << v.group << " controls=[";
        for (std::size_t i=0; i<v.controls.size(); ++i) {
            if (i) std::cout << ", ";
            std::cout << v.controls[i];
        }
        std::cout << "]";
    });
    print_sample("Actuator output status", data.actuator_outputs, [](const auto& v) {
        std::cout << "active=" << v.active << " outputs=[";
        for (std::size_t i=0; i<v.actuator.size(); ++i) {
            if (i) std::cout << ", ";
            std::cout << v.actuator[i];
        }
        std::cout << "]";
    });
    std::cout << std::endl;
}
void report_rate(const char* name, Telemetry::Result result) {
    if (result != Telemetry::Result::Success)
        std::cerr << "Rate request for " << name << " failed: " << result
                  << "; subscription remains active.\n";
}
} // namespace

int main(int argc, char** argv) {
    pf::Config config;
    std::vector<std::string> positional;
    try {
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") {usage(argv[0]);return 0;}
            if(arg.rfind("--",0)==0) {
                if(i+1>=argc)throw std::invalid_argument("Missing value for "+arg);
                pf::option(config,arg.substr(2),argv[++i]);
            } else positional.push_back(arg);
        }
        if(positional.size()>3)throw std::invalid_argument("Too many positional arguments");
        pf::validate(config);
    } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
    const std::string url=positional.empty()?"serial:///dev/ttyAMA0:115200":positional[0];
    double rate=10;
    try {if(positional.size()>1)rate=pf::number(positional[1]);
        if(rate<=0)throw std::invalid_argument("Telemetry rate must be positive");
    } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 1;}
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    const std::string forward_url=positional.size()>2?positional[2]:"";
    if(!forward_url.empty() && (forward_url.rfind("udp://",0)!=0 ||
       forward_url.rfind("udp://:",0)==0 || forward_url.rfind("udp://0.0.0.0:",0)==0)) {
        std::cerr<<"Forwarding URL must be udp://COMPUTER_IP:PORT.\n";return 1;
    }
    if(config.offline) {
        pf::Runtime controller(config);
        while(!stop_requested) {
            controller.tick();
            std::this_thread::sleep_for(std::chrono::duration<double>(1/config.rate));
        }
        return 0;
    }
    const auto cache = std::make_shared<TelemetryCache>();
    // Different component identity from the desktop GroundStation receiver:
    // otherwise MAVSDK may consume acknowledgements meant for that receiver.
    mavsdk::Mavsdk sdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::CompanionComputer}};
    const auto forwarding = forward_url.empty() ? mavsdk::ForwardingOption::ForwardingOff :
                                                mavsdk::ForwardingOption::ForwardingOn;
    const auto result = sdk.add_any_connection(url, forwarding);
    if (result!=mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << result << '\n'; return 1;
    }
    if (!forward_url.empty()) {
        const auto forward_result = sdk.add_any_connection(forward_url, mavsdk::ForwardingOption::ForwardingOn);
        if (forward_result != mavsdk::ConnectionResult::Success) {
            std::cerr << "Cannot open MAVLink forwarding endpoint: " << forward_result << '\n';
            return 1;
        }
        std::cout << "Forwarding MAVLink between " << url << " and " << forward_url << std::endl;
    }
    std::cout << "pf_los4 waiting for an autopilot on " << url << std::endl;
    std::shared_ptr<mavsdk::System> system;
    const auto deadline = Clock::now()+std::chrono::seconds(30);
    while (!stop_requested && Clock::now()<deadline) {
        const auto found = sdk.first_autopilot(0.2);
        if (found) { system=*found; break; }
    }
    if (stop_requested) return 0;
    if (!system) { std::cerr << "No autopilot discovered within 30 seconds.\n"; return 1; }
    std::cout << "Connected to system " << static_cast<int>(system->get_system_id())
              << "; requesting telemetry at " << rate << " Hz.\n";
    Telemetry telemetry{system};
    // Callback-owned shared cache outlives any queued callbacks at shutdown.
    const auto ph = telemetry.subscribe_position([cache](const auto& v) { store(cache, &TelemetryData::position, v); });
    const auto vh = telemetry.subscribe_velocity_ned([cache](const auto& v) { store(cache, &TelemetryData::velocity, v); });
    const auto lh = telemetry.subscribe_position_velocity_ned([cache](const auto& v) { store(cache, &TelemetryData::local, v); });
    const auto ah = telemetry.subscribe_attitude_euler([cache](const auto& v) { store(cache, &TelemetryData::attitude, v); });
    const auto ch = telemetry.subscribe_actuator_control_target([cache](const auto& v) { store(cache, &TelemetryData::actuator_controls, v); });
    const auto oh = telemetry.subscribe_actuator_output_status([cache](const auto& v) { store(cache, &TelemetryData::actuator_outputs, v); });
    report_rate("position", telemetry.set_rate_position(rate));
    report_rate("NED velocity", telemetry.set_rate_velocity_ned(rate));
    report_rate("local NED position/velocity", telemetry.set_rate_position_velocity_ned(rate));
    report_rate("Euler attitude", telemetry.set_rate_attitude_euler(rate));
    // MAVSDK 2.0.1's Euler callbacks depend on the cached quaternion stream.
    report_rate("quaternion", telemetry.set_rate_attitude_quaternion(rate));
    report_rate("actuator controls", telemetry.set_rate_actuator_control_target(rate));
    report_rate("actuator outputs", telemetry.set_rate_actuator_output_status(rate));
    pf::Runtime controller(config,system);
    int exit_code = 0;
    auto next_print = Clock::now();
    while (!stop_requested) {
        if (!system->is_connected()) { std::cerr << "Autopilot disconnected.\n"; exit_code=1; break; }
        controller.tick();
        if (Clock::now()>=next_print) {
            TelemetryData latest;
            { std::lock_guard<std::mutex> lock(cache->mutex); latest=cache->data; }
            print_data(latest);
            next_print=Clock::now()+std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(1/config.rate));
    }
    controller.shutdown();
    telemetry.unsubscribe_position(ph);
    telemetry.unsubscribe_velocity_ned(vh);
    telemetry.unsubscribe_position_velocity_ned(lh);
    telemetry.unsubscribe_attitude_euler(ah);
    telemetry.unsubscribe_actuator_control_target(ch);
    telemetry.unsubscribe_actuator_output_status(oh);
    std::cout << "pf_los4 stopped.\n";
    return exit_code;
}
