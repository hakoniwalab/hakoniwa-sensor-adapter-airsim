#include "hakoniwa/sensor_adapter/airsim/projectairsim_gps_adapter.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
namespace sa=hakoniwa::sensor_adapter;
namespace {
int failed=0;
void check(bool pass,const char* message) {
    if(!pass) {std::fprintf(stderr,"FAIL: %s\n",message);++failed;}
}
bool near(double a,double b,double eps=1e-5) {return std::fabs(a-b)<=eps;}
sa::GpsAdapterSetup setup(const char* file) {
    sa::GpsAdapterSetup s{};
    s.config_path=std::string(HAKO_AIRSIM_CONFIG_DIR)+"/"+file;
    s.physics_step_ns=1000000;
    s.has_random_seed=true;
    s.random_seed=9201;
    return s;
}
sa::GpsTruth truth(int k) {
    sa::GpsTruth t{};
    t.sim_time_ns=static_cast<std::uint64_t>(k)*1000000;
    t.position={35.6895+k*1e-7,139.6917-k*1e-7,40.0+k*0.02};
    t.position_nwu={k*0.02,k*0.03,k*0.02};
    t.velocity_nwu={2.0,-3.0,0.25};
    return t;
}
void passthrough_and_sample_hold() {
    std::string error;
    auto gps=sa::airsim::create_projectairsim_gps_adapter(setup("gps-immediate.json"),error);
    check(bool(gps),"immediate GPS adapter created");if(!gps)return;
    const auto caps=gps->capabilities();
    check(!caps.position_noise && !caps.velocity_noise && !caps.drift,
          "native GPS adds no position or velocity noise");
    check(!caps.receiver_status && !caps.course && caps.deterministic_with_seed,
          "partial receiver status is honestly declared");
    for(int k=0;k<250;++k) {
        sa::GpsSample s{};
        check(gps->step(truth(k),s,error),"GPS step succeeds");
        int sample=k-k%100;
        const auto t=truth(sample);
        check(s.updated==(k%100==0),"samples at 10Hz");
        check(s.sample_time_ns==t.sim_time_ns,"timestamp held");
        check(s.position.latitude_deg==t.position.latitude_deg &&
              s.position.longitude_deg==t.position.longitude_deg &&
              s.position.altitude_m==t.position.altitude_m,
              "geodetic truth is held without noise");
        check(s.velocity_nwu.x==2 && s.velocity_nwu.y==-3 && s.velocity_nwu.z==0.25,
              "NWU velocity unchanged");
        check(s.fix_type==3,"zero time constant yields immediate 3D Fix");
        check(s.horizontal_accuracy_m==0 && s.vertical_accuracy_m==0 &&
              s.speed_accuracy_mps==0 && s.satellites_visible==0 && !s.course_valid,
              "no fabricated metric accuracy, satellites or course");
    }
    std::vector<double> logs;
    gps->log_values(logs);
    check(logs.size()==2 && gps->log_columns().size()==2 &&
          near(logs[0],0.1) && near(logs[1],0.1),
          "dimensionless EPH/EPV diagnostics");
}
void convergence_and_reset() {
    std::string error;
    auto a=sa::airsim::create_projectairsim_gps_adapter(setup("gps-default.json"),error);
    auto b=sa::airsim::create_projectairsim_gps_adapter(setup("gps-default.json"),error);
    check(bool(a)&&bool(b),"default GPS adapters created");if(!a||!b)return;
    float expected_eph=100.0f, expected_epv=100.0f;
    bool seen0=false,seen2=false,seen3=false;
    for(int k=0;k<4000;++k) {
        sa::GpsSample s{};
        check(a->step(truth(k),s,error),"filter step");
        if(k%100==0) {
            const double alpha=std::exp(-0.1/0.9f);
            expected_eph=static_cast<float>(expected_eph*alpha+0.1f*(1.0-alpha));
            expected_epv=static_cast<float>(expected_epv*alpha+0.1f*(1.0-alpha));
            std::vector<double> logs;
            a->log_values(logs);
            check(logs.size()==2 && near(logs[0],expected_eph) &&
                  near(logs[1],expected_epv),"native float first-order reference");
            const unsigned fix=expected_eph<=3?3:expected_eph<=4?2:0;
            check(s.fix_type==fix,"native Fix thresholds");
            seen0|=fix==0;seen2|=fix==2;seen3|=fix==3;
        }
    }
    check(seen0 && seen2 && seen3,"No Fix to 2D to 3D transition");
    check(a->reset(error),"reset");
    for(int k=0;k<250;++k) {
        sa::GpsSample x{},y{};
        check(a->step(truth(k),x,error) && b->step(truth(k),y,error),
              "independent instances step");
        check(x.fix_type==y.fix_type && x.position.latitude_deg==y.position.latitude_deg,
              "reset reproduces first 250 steps");
    }
}
void rejects_bad_inputs() {
    std::string error;
    check(!sa::airsim::create_projectairsim_gps_adapter(setup("missing.json"),error) &&
          !error.empty(),"missing config");
    auto s=setup("gps-default.json");s.physics_step_ns=3000000;
    check(!sa::airsim::create_projectairsim_gps_adapter(s,error),"non-integer sample period");
    s=setup("gps-default.json");s.physics_step_ns=0;
    check(!sa::airsim::create_projectairsim_gps_adapter(s,error),"zero physics step");
    auto gps=sa::airsim::create_projectairsim_gps_adapter(setup("gps-default.json"),error);
    check(bool(gps),"valid adapter for input tests");if(!gps)return;
    sa::GpsSample sample{};
    auto t=truth(0);t.sim_time_ns=13;
    check(!gps->step(t,sample,error) && !error.empty(),"time jump fails");
    t=truth(0);t.position.latitude_deg=91;
    check(!gps->step(t,sample,error) && !error.empty(),"invalid latitude fails");
    check(gps->step(truth(0),sample,error) && error.empty(),"recovery after invalid input");
}
}
int main() {
    passthrough_and_sample_hold();
    convergence_and_reset();
    rejects_bad_inputs();
    if(failed==0)std::puts("projectairsim_gps_adapter_test: OK");
    return failed?EXIT_FAILURE:EXIT_SUCCESS;
}
