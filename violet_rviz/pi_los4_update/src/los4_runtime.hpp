#pragma once
#include "los4_core.hpp"
#include "los4_safety.hpp"
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
namespace pf {
class Runtime {
    using Clock=std::chrono::steady_clock;
    struct Data {
        bool hb=false,armed=false,air=false,px4=false,fixed_wing=false,rc_valid=false,rc_on=false,rc_off=false;
        bool rejected=false,kill=true,kill_valid=false;
        int mode=0;
        double hb_time=-100,air_time=-100,rc_time=-100,p_time=-100,a_time=-100;
        Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero(),eta=Eigen::Vector3d::Zero();
    };
    struct Cache {std::mutex mutex;Data data;};
    Config c;
    Los4 controller;
    Safety safety;
    std::shared_ptr<Cache> cache=std::make_shared<Cache>();
    std::unique_ptr<mavsdk::MavlinkPassthrough> link;
    std::vector<std::pair<uint16_t,mavsdk::MavlinkPassthrough::MessageHandle>> handles;
    Clock::time_point epoch=Clock::now(),last_tick=epoch;
    double last_return=-100,last_print=-100,handback_started=-100,last_diagnostic=-100,last_config=-100;
    Phase previous;
    bool have_output=false;
    Output latest;
    double time() const;
    bool mode_request(int mode);
    bool send_setpoint(const Output& output);
    void handback(double now);
    void diagnostics(double now);
public:
    explicit Runtime(Config config,std::shared_ptr<mavsdk::System> system=nullptr);
    ~Runtime();
    void tick();
    void shutdown();
};
}
