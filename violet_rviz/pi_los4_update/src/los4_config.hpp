#pragma once
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pf {
struct Config {
    std::string path="circle", return_mode="manual", throttle_model="quadratic";
    bool enabled=false, test=true, offline=true;
    bool bench_offboard=false;
    // rc_off is retained for old CLI compatibility; low/mid is <rc_on.
    int rc_channel=7, rc_on=1700, rc_off=1300;
    int kill_channel=6, kill_threshold=1700;
    bool kill_active_high=true;
    double radius=50, scale=50, speed=22, gamma0=0, rate=50, stale=0.5;
    Eigen::Vector3d p0{0,0,-30}, p1{200,0,-30}, center{0,0,-30};
    double mass=1.48, tmax=104.720105;
    double k1=1, k2=20, kpE=0.05, kiE=0.005, kpB=0.002, kiB=0.00001;
    double ka=10, kphi=10, ktheta=10;
    double throttle_min=0, throttle_max=1, pitch_min=-0.52, pitch_max=0.52;
    double accel_min=-10, accel_max=10, rate_limit=1.5, min_speed=5;
};
inline double number(const std::string& s) {
    std::size_t used=0;
    const double v=std::stod(s,&used);
    if (used!=s.size() || !std::isfinite(v)) throw std::invalid_argument("Invalid number: "+s);
    return v;
}
inline Eigen::Vector3d vector3(const std::string& s) {
    const auto a=s.find(','), b=s.find(',', a==std::string::npos ? s.size() : a+1);
    if(a==std::string::npos || b==std::string::npos || s.find(',',b+1)!=std::string::npos)
        throw std::invalid_argument("Expected N,E,D vector: "+s);
    return {number(s.substr(0,a)),number(s.substr(a+1,b-a-1)),number(s.substr(b+1))};
}
inline bool boolean(const std::string& s) {
    if(s=="true" || s=="1") return true;
    if(s=="false" || s=="0") return false;
    throw std::invalid_argument("Expected true or false: "+s);
}
inline void option(Config& c,const std::string& key,const std::string& v) {
    if(key=="path") c.path=v;
    else if(key=="controller-enabled") c.enabled=boolean(v);
    else if(key=="bench-offboard") c.bench_offboard=boolean(v);
    else if(key=="kill-active-high") c.kill_active_high=boolean(v);
    else if(key=="test-gamma") c.test=boolean(v);
    else if(key=="offline") c.offline=boolean(v);
    else if(key=="return-mode") c.return_mode=v;
    else if(key=="throttle-model") c.throttle_model=v;
    else if(key=="p0") c.p0=vector3(v);
    else if(key=="p1") c.p1=vector3(v);
    else if(key=="center") c.center=vector3(v);
    else {
        const double n=number(v);
        if(key=="rc-channel" || key=="rc-on" || key=="rc-off" || key=="kill-channel" || key=="kill-threshold") {
            if(std::floor(n)!=n || n<0 || n>2200) throw std::invalid_argument("Invalid RC integer");
            if(key=="kill-channel") c.kill_channel=static_cast<int>(n);
            if(key=="kill-threshold") c.kill_threshold=static_cast<int>(n);
            if(key=="rc-channel") c.rc_channel=static_cast<int>(n);
            if(key=="rc-on") c.rc_on=static_cast<int>(n);
            if(key=="rc-off") c.rc_off=static_cast<int>(n);
        }
#define PF_OPTION(name, field) else if(key==name) c.field=n
        PF_OPTION("radius",radius); PF_OPTION("scale",scale); PF_OPTION("speed",speed);
        PF_OPTION("gamma0",gamma0); PF_OPTION("control-rate",rate); PF_OPTION("stale-timeout",stale);
        PF_OPTION("mass",mass); PF_OPTION("Tmax",tmax);
        PF_OPTION("k1",k1); PF_OPTION("k2",k2); PF_OPTION("kpE",kpE); PF_OPTION("kiE",kiE);
        PF_OPTION("kpB",kpB); PF_OPTION("kiB",kiB); PF_OPTION("ka",ka);
        PF_OPTION("kphi",kphi); PF_OPTION("ktheta",ktheta);
        PF_OPTION("throttle-min",throttle_min); PF_OPTION("throttle-max",throttle_max);
        PF_OPTION("pitch-min",pitch_min); PF_OPTION("pitch-max",pitch_max);
        PF_OPTION("accel-min",accel_min); PF_OPTION("accel-max",accel_max);
        PF_OPTION("rate-limit",rate_limit); PF_OPTION("min-speed",min_speed);
#undef PF_OPTION
        else throw std::invalid_argument("Unknown option --"+key);
    }
}
inline void validate(const Config& c) {
    if(c.path!="line" && c.path!="circle" && c.path!="lemniscate")
        throw std::invalid_argument("Path must be line, circle or lemniscate");
    if(!c.p0.allFinite() || !c.p1.allFinite() || !c.center.allFinite() ||
       (c.p1-c.p0).norm()<1e-6 || c.radius<=0 || c.scale<=0 || c.speed<=0 ||
       c.mass<=0 || c.tmax<=0 || c.k1<=0 || c.k2<=0 || c.rate<10 || c.rate>100 ||
       c.stale<0.1 || c.stale>2 || c.min_speed<=0 || c.rate_limit<=0 ||
       c.throttle_min<0 || c.throttle_max>1 || c.throttle_min>=c.throttle_max ||
       c.pitch_min>=c.pitch_max || c.pitch_min<=-1.5 || c.pitch_max>=1.5 ||
       c.accel_min>=c.accel_max || c.rc_channel<0 || c.rc_channel>18 ||
       c.kill_channel<1 || c.kill_channel>18 || c.kill_channel==c.rc_channel ||
       c.kill_threshold<800 || c.kill_threshold>2200 || c.rc_off<800 || c.rc_on>2200 || c.rc_off>=c.rc_on)
        throw std::invalid_argument("Invalid path, gains, rates, limits or RC thresholds");
    if(c.kpE<0 || c.kiE<0 || c.kpB<0 || c.kiB<0 || c.ka<0 || c.kphi<0 || c.ktheta<0)
        throw std::invalid_argument("Feedback gains must be nonnegative");
    if(c.throttle_model!="linear" && c.throttle_model!="quadratic")
        throw std::invalid_argument("Throttle model must be linear or quadratic");
    if(!c.return_mode.empty() && c.return_mode!="manual" && c.return_mode!="stabilized" &&
       c.return_mode!="altitude" && c.return_mode!="position")
        throw std::invalid_argument("Return mode: manual/stabilized/altitude/position");
    if(c.offline && !c.test) throw std::invalid_argument("--offline requires --test-gamma true");
    // Reject missing mappings even when explicitly enabling live control.
    if(c.enabled && !c.test && (c.rc_channel==0 || c.return_mode.empty()))
        throw std::invalid_argument("Live control requires a configured --rc-channel and valid return-mode");
}
}
