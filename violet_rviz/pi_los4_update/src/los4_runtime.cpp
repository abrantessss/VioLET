#include "los4_runtime.hpp"
#include "los4_diagnostics.hpp"
#include <mavsdk/system.h>
#include <iomanip>
#include <iostream>
#include <thread>
namespace pf {
namespace {
int rc_mode(const std::string& mode) {
    if(mode=="manual")return 1;
    if(mode=="altitude")return 2;
    if(mode=="position")return 3;
    if(mode=="stabilized")return 7;
    return 0;
}

}
double Runtime::time() const {return std::chrono::duration<double>(Clock::now()-epoch).count();}
Runtime::Runtime(Config config,std::shared_ptr<mavsdk::System> system):c(std::move(config)),controller(c),safety(c),previous(safety.phase) {
    std::cout<<"LOS4 "<<name(safety.phase)<<" path="<<c.path<<" rc-channel="<<c.rc_channel
             <<" return-mode="<<(c.return_mode.empty()?"UNSET":c.return_mode)<<"\n";
    if(c.bench_offboard)std::cout<<"BENCH OFFBOARD: no airborne/speed requirement; zero thrust and rates, gamma frozen."
        << (c.test ? " Gamma-test flag overrides bench: no control packets." : "") << "\n";
    if(!system)return;
    link=std::make_unique<mavsdk::MavlinkPassthrough>(system);
    const auto shared=cache;const auto origin=epoch;const auto config_copy=c;
    const auto target_sys=link->get_target_sysid(),target_comp=link->get_target_compid();
    auto callback=[shared,origin,config_copy,target_sys,target_comp](const mavlink_message_t& m) {
        if(m.sysid!=target_sys || m.compid!=target_comp)return;
        const double t=std::chrono::duration<double>(Clock::now()-origin).count();
        std::lock_guard<std::mutex> lock(shared->mutex);auto& d=shared->data;
        if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t v{};mavlink_msg_heartbeat_decode(&m,&v);
            d.hb=true;d.hb_time=t;d.armed=(v.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
            d.px4=v.autopilot==MAV_AUTOPILOT_PX4;d.fixed_wing=v.type==MAV_TYPE_FIXED_WING;
            d.mode=(v.custom_mode>>16)&0xff;
        } else if(m.msgid==MAVLINK_MSG_ID_EXTENDED_SYS_STATE) {
            mavlink_extended_sys_state_t v{};mavlink_msg_extended_sys_state_decode(&m,&v);
            d.air=v.landed_state==MAV_LANDED_STATE_IN_AIR;d.air_time=t;
        } else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
            mavlink_local_position_ned_t v{};mavlink_msg_local_position_ned_decode(&m,&v);
            d.p={v.x,v.y,v.z};d.v={v.vx,v.vy,v.vz};d.p_time=t;
        } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE) {
            mavlink_attitude_t v{};mavlink_msg_attitude_decode(&m,&v);
            d.eta={v.roll,v.pitch,v.yaw};d.a_time=t;
        } else if(m.msgid==MAVLINK_MSG_ID_RC_CHANNELS) {
            mavlink_rc_channels_t v{};mavlink_msg_rc_channels_decode(&m,&v);
            const uint16_t channels[]={v.chan1_raw,v.chan2_raw,v.chan3_raw,v.chan4_raw,v.chan5_raw,
                v.chan6_raw,v.chan7_raw,v.chan8_raw,v.chan9_raw,v.chan10_raw,v.chan11_raw,v.chan12_raw,
                v.chan13_raw,v.chan14_raw,v.chan15_raw,v.chan16_raw,v.chan17_raw,v.chan18_raw};
            const int ch=config_copy.rc_channel;
            const int pwm=ch>=1 && ch<=18?channels[ch-1]:0;
            d.rc_time=t;d.rc_off=pwm<config_copy.rc_on;d.rc_valid=ch>0 && ch<=v.chancount && pwm>=800 && pwm<=2200 && v.rssi!=0;
            // Only the configured high position permits control. The middle
            // band releases control but cannot reset the OFF->ON safety latch.
            d.rc_on=d.rc_valid && pwm>=config_copy.rc_on;
            const int k=config_copy.kill_channel;
            const int kill_pwm=channels[k-1];
            d.kill_valid=k<=v.chancount && kill_pwm>=800 && kill_pwm<=2200 && v.rssi!=0;
            d.kill=!d.kill_valid || (config_copy.kill_active_high ?
                kill_pwm>=config_copy.kill_threshold : kill_pwm<=config_copy.kill_threshold);
        } else if(m.msgid==MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t v{};mavlink_msg_command_ack_decode(&m,&v);
            if(v.command==MAV_CMD_DO_SET_MODE &&
               (v.result!=MAV_RESULT_ACCEPTED && v.result!=MAV_RESULT_IN_PROGRESS)) d.rejected=true;
        }
    };
    for(const uint16_t id : {MAVLINK_MSG_ID_HEARTBEAT,MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        MAVLINK_MSG_ID_LOCAL_POSITION_NED,MAVLINK_MSG_ID_ATTITUDE,MAVLINK_MSG_ID_RC_CHANNELS,MAVLINK_MSG_ID_COMMAND_ACK})
        handles.emplace_back(id,link->subscribe_message(id,callback));
    // Request safety streams once; no arm/takeoff command exists in this program.
    for(const auto& pair : {std::pair<int,double>{MAVLINK_MSG_ID_EXTENDED_SYS_STATE,5},
                          {MAVLINK_MSG_ID_RC_CHANNELS,10},{MAVLINK_MSG_ID_LOCAL_POSITION_NED,20},
                          {MAVLINK_MSG_ID_ATTITUDE,20}}) {
        link->queue_message([pair,target_sys,target_comp](auto address,uint8_t channel) {
            mavlink_message_t msg{};
            mavlink_msg_command_long_pack_chan(address.system_id,address.component_id,channel,&msg,
                target_sys,target_comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,
                static_cast<float>(pair.first),static_cast<float>(1000000/pair.second),0,0,0,0,0);
            return msg;
        });
    }
}
Runtime::~Runtime() {if(link)for(const auto& h:handles)link->unsubscribe_message(h.first,h.second);}
bool Runtime::mode_request(int mode) {
    if(!link || !c.enabled || c.test || mode==0 || mode==6)return false;
    const auto sys=link->get_target_sysid(),comp=link->get_target_compid();
    return link->queue_message([sys,comp,mode](auto address,uint8_t channel) {
        mavlink_message_t msg{};
        mavlink_msg_command_long_pack_chan(address.system_id,address.component_id,channel,&msg,
            sys,comp,MAV_CMD_DO_SET_MODE,0,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            static_cast<float>(mode),0,0,0,0,0);
        return msg;
    })==mavsdk::MavlinkPassthrough::Result::Success;
}
bool Runtime::send_setpoint(const Output& o) {
    if(!link || !c.enabled || c.test)return false;
    const auto sys=link->get_target_sysid(),comp=link->get_target_compid();
    const uint32_t stamp=static_cast<uint32_t>(time()*1000);
    // Enforce the bench command at the wire boundary as well as in tick().
    const Output command=c.bench_offboard ? Output{} : o;
    return link->queue_message([sys,comp,stamp,command](auto address,uint8_t channel) {
        mavlink_set_attitude_target_t target{};
        target.time_boot_ms=stamp;target.target_system=sys;target.target_component=comp;
        target.type_mask=ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE;target.q[0]=1;
        target.body_roll_rate=static_cast<float>(command.body_rates.x());
        target.body_pitch_rate=static_cast<float>(command.body_rates.y());
        target.body_yaw_rate=static_cast<float>(command.body_rates.z());
        target.thrust=static_cast<float>(command.throttle);
        mavlink_message_t msg{};
        mavlink_msg_set_attitude_target_encode_chan(address.system_id,address.component_id,channel,&msg,&target);
        return msg;
    })==mavsdk::MavlinkPassthrough::Result::Success;
}
void Runtime::handback(double t) {
    if(t-last_return<0.5)return;
    last_return=t;
    if(!mode_request(rc_mode(c.return_mode)))
        std::cerr<<"RC handback request could not be queued; PX4 Offboard-loss failsafe must recover\n";
}
void Runtime::diagnostics(double t) {
    if(!link || t-last_diagnostic<0.05)return; // <=20 Hz, independent of control.
    last_diagnostic=t;
    auto send=[this](const std::vector<float>& values) {
        const auto bytes=los4_wire::encode(values);
        const auto length=static_cast<uint8_t>(4+4*values.size());
        link->queue_message([bytes,length](auto address,uint8_t channel) {
            mavlink_message_t msg{};
            mavlink_msg_tunnel_pack_chan(address.system_id,address.component_id,channel,&msg,
                0,0,los4_wire::payload_type,length,bytes.data());
            return msg;
        });
    };
    if(t-last_config>=1) {
        last_config=t;
        send({1,1,c.path=="line"?1.0f:(c.path=="circle"?2.0f:3.0f),
              float(c.p0.x()),float(c.p0.y()),float(c.p0.z()),
              float(c.p1.x()),float(c.p1.y()),float(c.p1.z()),
              float(c.center.x()),float(c.center.y()),float(c.center.z()),
              float(c.radius),float(c.scale),float(c.speed)});
    }
    Data d;{std::lock_guard<std::mutex> lock(cache->mutex);d=cache->data;}
    Output output=latest;
    if(!have_output) {
        output.gamma=c.gamma0;const auto point=path_point(c,c.gamma0);
        output.pd=point.p;output.derivative=point.d1;
    }
    if(safety.phase!=Phase::Active && safety.phase!=Phase::Test)output.gamma_dot=0;
    unsigned flags=0;
    if(t-d.p_time<c.stale && d.p.allFinite() && d.v.allFinite())flags|=3;
    if(t-d.a_time<c.stale && d.eta.allFinite())flags|=4;
    if(safety.phase==Phase::Active && have_output && !c.bench_offboard)flags|=8;
    if(c.test && (flags&7)==7) {
        // Shadow desired attitude at the test gamma; never transmit setpoints.
        try {
            auto config=c;config.gamma0=output.gamma;Los4 shadow(config);
            const auto preview=shadow.step(1/c.rate,d.p,d.v,d.eta);
            output.roll=preview.roll;output.pitch=preview.pitch;output.heading=preview.heading;flags|=8;
        } catch(const std::exception&) {} // No reference if speed/geometry is unsuitable.
    }
    send({1,2,c.bench_offboard && safety.phase==Phase::Active ? 8.0f : float(static_cast<int>(safety.phase)),float(output.gamma),float(output.gamma_dot),float(c.speed),
          float(output.pd.x()),float(output.pd.y()),float(output.pd.z()),
          float(output.derivative.x()),float(output.derivative.y()),float(output.derivative.z()),
          float(d.p.x()),float(d.p.y()),float(d.p.z()),float(d.v.x()),float(d.v.y()),float(d.v.z()),
          float(d.eta.x()),float(d.eta.y()),float(d.eta.z()),
          float(output.roll),float(output.pitch),float(output.heading),float(flags)});
}

void Runtime::tick() {
    const auto stamp=Clock::now();const double dt=std::chrono::duration<double>(stamp-last_tick).count();
    last_tick=stamp;const double t=time();
    if(c.test) {latest=controller.test_step(dt);have_output=true;}
    else if(c.enabled) {
        Data d;{std::lock_guard<std::mutex> lock(cache->mutex);d=cache->data;cache->data.rejected=false;}
        const bool heartbeat_fresh=d.hb && t-d.hb_time<3;
        const bool attitude_fresh=t-d.a_time<c.stale && d.eta.allFinite();
        const bool flight_ready=t-d.air_time<1.5 && d.air && t-d.p_time<c.stale &&
                                d.p.allFinite() && d.v.allFinite() && d.v.head<2>().norm()>=c.min_speed;
        SafetyInput in;
        in.offboard=d.mode==6;
        in.ready=heartbeat_fresh && attitude_fresh && d.px4 && d.fixed_wing && d.armed &&
                 (c.bench_offboard || flight_ready) && (in.offboard || d.mode==1);
        in.switch_valid=d.rc_valid && d.kill_valid && t-d.rc_time<c.stale;
        in.kill=d.kill;in.manual=d.mode==1;
        in.switch_on=d.rc_on;in.switch_off=d.rc_off;
        in.return_confirmed=d.hb && t-d.hb_time<3 && d.hb_time>handback_started && handback_started>=0 && d.mode!=6;
        in.request_denied=d.rejected;
        const auto before=safety.phase;
        const auto decision=safety.tick(in,t);
        if(before!=Phase::Returning && safety.phase==Phase::Returning)handback_started=t;
        try {
            if(decision.reset)controller.reset();
            if(decision.setpoint) {
                // Gamma and integrators only advance after confirmed Offboard.
                if(c.bench_offboard) {
                    latest=Output{};latest.gamma=c.gamma0;
                    const auto point=path_point(c,c.gamma0);
                    latest.pd=point.p;latest.derivative=point.d1;
                } else if(safety.phase==Phase::Active)latest=controller.step(dt,d.p,d.v,d.eta);
                else {auto preview=controller;latest=preview.step(1/c.rate,d.p,d.v,d.eta);latest.gamma_dot=0;}
                have_output=true;
                if(!send_setpoint(latest))throw std::runtime_error("Setpoint could not be queued");
            }
            if(decision.handback && d.mode==6)handback(t);
        } catch(const std::exception& e) {
            std::cerr<<"LOS4 fault: "<<e.what()<<"; stop setpoints and request RC mode\n";
            safety.fault(t);handback_started=t;
            if(d.mode==6)handback(t);
        }
    }
    diagnostics(t);
    if(previous!=safety.phase) {
        std::cout<<"LOS4 state: "<<(c.bench_offboard && previous==Phase::Active ? "BENCH_OFFBOARD_ACTIVE" : name(previous))<<" -> "<<(c.bench_offboard && safety.phase==Phase::Active ? "BENCH_OFFBOARD_ACTIVE" : name(safety.phase))<<"\n";
        if(previous==Phase::Returning) {
            Data d;{std::lock_guard<std::mutex> lock(cache->mutex);d=cache->data;}
            if(t-d.hb_time<3 && d.mode!=6)std::cout<<"Pixhawk reports mode outside Offboard; RC/failsafe has control\n";
            else std::cerr<<"RC handback NOT confirmed; setpoints remain stopped\n";
        }
        previous=safety.phase;
    }
    if(t-last_print>=1) {
        last_print=t;std::cout<<std::fixed<<std::setprecision(4)<<"LOS4 state="<<(c.bench_offboard && safety.phase==Phase::Active ? "BENCH_OFFBOARD_ACTIVE" : name(safety.phase));
        if(have_output)std::cout<<" path="<<c.path<<" gamma="<<latest.gamma<<" gamma_dot="<<latest.gamma_dot
            <<" pd_NED=["<<latest.pd.transpose()<<"] ep_norm="<<latest.ep_norm;
        std::cout<<"\n";
    }
}
void Runtime::shutdown() {
    if(!c.enabled || c.test || !link)return;
    if(safety.phase!=Phase::Active && safety.phase!=Phase::Starting && safety.phase!=Phase::Priming && safety.phase!=Phase::Returning)return;
    std::cerr<<"Stopping LOS4 setpoints; requesting RC handback\n";
    const double release_time=time();
    const double end=release_time+3;
    while(time()<end) {
        handback(time());
        {std::lock_guard<std::mutex> lock(cache->mutex);
            if(cache->data.hb && time()-cache->data.hb_time<3 && cache->data.hb_time>release_time && cache->data.mode!=6) {
                std::cerr<<"Mode outside Offboard confirmed\n";return;
            }}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cerr<<"RC handback not confirmed; check PX4 Offboard-loss failsafe\n";
}
}
