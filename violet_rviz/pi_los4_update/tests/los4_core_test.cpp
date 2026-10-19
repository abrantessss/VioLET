#include "los4_core.hpp"
#include "los4_safety.hpp"
#include <iostream>
void check(bool ok,const char* msg) {if(!ok)throw std::runtime_error(msg);}
int main() {
    pf::Config c;c.test=false;c.offline=false;pf::validate(c);
    auto p=pf::path_point(c,0);check((p.p-Eigen::Vector3d(50,0,-30)).norm()<1e-9,"circle start");
    check(std::abs(p.d1.norm()-50)<1e-9,"circle derivative");
    c.path="line";p=pf::path_point(c,0.5);check((p.p-Eigen::Vector3d(100,0,-30)).norm()<1e-9,"line midpoint");
    c.path="lemniscate";p=pf::path_point(c,1.5707963267948966);
    check((p.p-c.center).norm()<1e-9,"Gerono centre crossing");
    const double x=0.73,h=1e-5;
    const auto l=pf::path_point(c,x-h),r=pf::path_point(c,x+h),m=pf::path_point(c,x);
    check(((r.p-l.p)/(2*h)-m.d1).norm()<1e-6,"derivative finite difference");
    check(((r.d1-l.d1)/(2*h)-m.d2).norm()<1e-6,"second derivative finite difference");
    pf::Los4 dry(c);auto o=dry.test_step(0.02);check(o.gamma==0 && o.gamma_dot==1,"test gamma derivative");
    o=dry.test_step(0.02);check(std::abs(o.gamma-0.02)<1e-9,"test gamma integrates");
    c.path="circle";pf::Los4 core(c);o=core.step(0.02,{50,0,-30},{0,c.speed,0},{0,0,1.5707963267948966});
    check(o.body_rates.allFinite() && o.throttle>=0 && o.throttle<=1,"bounded output");
    check(std::abs(o.gamma_dot-c.speed/50)<1e-9,"on-path gamma speed");
    bool threw=false;try{core.step(0.02,{0,0,0},{0,0,0},{0,0,0});}catch(...){threw=true;}
    check(threw,"zero speed rejected");
    pf::SafetyInput in;in.ready=true;in.switch_valid=true;in.switch_on=true;in.kill=false;in.manual=true;
    c.test=false;c.enabled=false;pf::Safety disabled(c);check(!disabled.tick(in,0).setpoint,"disabled inhibits");
    c.enabled=true;c.test=true;pf::Safety test(c);auto d=test.tick(in,0);
    check(!d.setpoint && !d.start && !d.handback,"test overrides live enable");
    c.test=false;pf::validate(c);
    pf::Safety gate(c);check(!gate.tick(in,0).setpoint,"switch high at startup blocked");
    in.switch_on=false;in.switch_off=true;d=gate.tick(in,0.1);
    check(d.setpoint && d.reset && gate.phase==pf::Phase::Priming,"prestream in Manual");
    in.switch_on=true;in.switch_off=false;d=gate.tick(in,1.4);
    check(d.setpoint && !d.start,"script never requests Offboard entry");
    in.manual=false;in.offboard=true;d=gate.tick(in,1.5);
    check(gate.phase==pf::Phase::Active && d.reset,"RC-confirmed Offboard activates");
    in.kill=true;d=gate.tick(in,1.6);
    check(!d.setpoint && d.handback,"kill stops setpoints");
    in.kill=false;in.offboard=false;in.manual=true;in.return_confirmed=true;
    gate.tick(in,1.7);check(gate.phase==pf::Phase::Fault,"fault latched");
    gate.tick(in,1.8);check(gate.phase==pf::Phase::Fault,"held-high does not reset fault");
    in.switch_on=false;in.switch_off=true;gate.tick(in,1.9);
    check(gate.phase==pf::Phase::Waiting,"Manual low/mid clears latch");
    gate.tick(in,2.0);in.switch_on=true;in.switch_off=false;in.manual=false;in.offboard=true;
    gate.tick(in,3.3);check(gate.phase==pf::Phase::Active,"reactivates after RC cycle");
    in.ready=false;d=gate.tick(in,3.4);check(!d.setpoint && d.handback,"stale stops control");
    std::cout<<"LOS4 geometry, numerical guards, hard inhibit, RC mode entry and kill checks passed\n";
}
