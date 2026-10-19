#pragma once
#include "los4_config.hpp"
namespace pf {
// Numeric values retained for the diagnostic wire protocol.
enum class Phase { Disabled,Test,Waiting,Priming,Starting,Active,Returning,Fault };
inline const char* name(Phase p) {
    switch(p) {
    case Phase::Disabled:return "DISABLED";case Phase::Test:return "TEST_NO_CONTROL";
    case Phase::Waiting:return "WAIT_RC_MODE";case Phase::Priming:return "PRESTREAM_MANUAL";
    case Phase::Starting:return "WAIT_OFFBOARD";case Phase::Active:return "LOS4_ACTIVE";
    case Phase::Returning:return "RETURN_MANUAL";case Phase::Fault:return "FAULT_LATCHED";
    }
    return "UNKNOWN";
}
struct SafetyInput {
    bool ready=false,switch_valid=false,switch_on=false,switch_off=false,offboard=false;
    bool return_confirmed=false,request_denied=false;
    bool manual=false,kill=true;
};
struct Decision { bool setpoint=false,start=false,handback=false,reset=false; };
class Safety {
    bool enabled,test,manual_seen=false;
    double since=0;
public:
    Phase phase;
    Safety(const Config& c):enabled(c.enabled),test(c.test),phase(test?Phase::Test:(enabled?Phase::Waiting:Phase::Disabled)){}
    Decision tick(const SafetyInput& in,double now) {
        Decision d;
        if(!enabled || test) {phase=test?Phase::Test:Phase::Disabled;return d;}
        if(phase==Phase::Returning) {
            if(in.return_confirmed || !in.offboard || now-since>3)phase=Phase::Fault;
            else d.handback=true;
            return d;
        }
        if(phase==Phase::Fault) {
            if(in.ready && in.switch_valid && in.switch_off && in.manual && !in.kill) {
                phase=Phase::Waiting;manual_seen=true;
            }
            return d;
        }
        if(!in.ready || !in.switch_valid || in.kill || in.request_denied) {
            manual_seen=false;
            if(phase==Phase::Active || phase==Phase::Priming || in.offboard) {
                phase=in.offboard?Phase::Returning:Phase::Fault;
                since=now;d.handback=in.offboard;
            }
            return d;
        }
        if(in.manual && in.switch_off) {
            if(phase==Phase::Active) {phase=Phase::Waiting;manual_seen=false;return d;}
            // Prestream while Manual is actually reported. PX4 ignores these
            // setpoints until Offboard; gamma and controller integrators freeze.
            if(!manual_seen) {manual_seen=true;since=now;d.reset=true;}
            phase=Phase::Priming;d.setpoint=true;return d;
        }
        if(!manual_seen) {
            // Starting with the switch high is not permission to take control.
            if(in.offboard) {phase=Phase::Returning;since=now;d.handback=true;}
            return d;
        }
        if(in.switch_on && in.offboard && now-since>=1.2) {
            if(phase!=Phase::Active)d.reset=true;
            phase=Phase::Active;d.setpoint=true;return d;
        }
        if(phase==Phase::Active || in.offboard) {
            // RC has switched away (including mid), or the mode is unexpected.
            manual_seen=false;phase=in.offboard?Phase::Returning:Phase::Waiting;
            since=now;d.handback=in.offboard;return d;
        }
        // Channel high but PX4 still in Manual: maintain proof-of-life and
        // wait for the RC's actual mode transition, never request Offboard.
        if(in.switch_on && in.manual && now-since>=1.2) {
            phase=Phase::Priming;d.setpoint=true;
        } else {phase=Phase::Waiting;manual_seen=false;}
        return d;
    }
    void fault(double now) {if(phase!=Phase::Disabled && phase!=Phase::Test) {phase=Phase::Returning;since=now;manual_seen=false;}}
};
}
