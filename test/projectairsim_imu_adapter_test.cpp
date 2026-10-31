#include "hakoniwa/sensor_adapter/airsim/projectairsim_imu_adapter.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace {
int failures = 0;
void check(bool ok, const char* message) { if (!ok) { std::fprintf(stderr,"FAIL: %s\n",message); ++failures; } }
bool near(double a, double b, double eps=1e-12) { return std::fabs(a-b)<=eps; }

sa::ImuAdapterSetup setup(const char* filename, bool seeded=true) {
    sa::ImuAdapterSetup s{};
    s.config_path = std::string(HAKO_AIRSIM_CONFIG_DIR) + "/" + filename;
    s.physics_step_ns=1000000;
    s.has_random_seed=seeded;
    s.random_seed=123456;
    s.vehicle_name="X500";
    return s;
}
sa::ImuTruth input(std::uint64_t k) {
    sa::ImuTruth t{};
    t.sim_time_ns=k*1000000;
    t.angular_velocity={0.1+0.01*k,-0.2,0.3};
    t.specific_force={0.5,-0.7,9.80665};
    return t;
}
std::vector<sa::ImuSample> run(sa::IImuSensorAdapter& imu, int steps) {
    std::vector<sa::ImuSample> samples;
    std::string e;
    for(int k=0;k<steps;++k){sa::ImuSample out{};check(imu.step(input(k),out,e),"step succeeds");samples.push_back(out);}
    return samples;
}
void ideal_and_time() {
    std::string e;
    auto a=sa::airsim::create_projectairsim_imu_adapter(setup("imu-ideal.json"),e);
    check(a!=nullptr,"ideal adapter created"); if(!a){std::fprintf(stderr,"%s\n",e.c_str());return;}
    const auto values=run(*a,10);
    for(int i=0;i<10;++i) {
        int sampled=i-i%4;
        check(values[i].gyro.updated==(i%4==0),"gyro updated at 250 Hz");
        check(values[i].accel.updated==(i%4==0),"accel updated at 250 Hz");
        check(values[i].gyro.sample_time_ns==(std::uint64_t)sampled*1000000,"sample timestamp held");
        check(near(values[i].gyro.value.x,input(sampled).angular_velocity.x),"gyro true x held");
        check(near(values[i].accel.value.z,9.80665),"specific force in FLU");
        check(values[i].gyro.flags==0,"no clipping flag");
    }
    check(a->capabilities().noise && a->capabilities().drift,"model capabilities even when disabled by config");
    sa::ImuSample out{};auto t=input(99);t.sim_time_ns=9;
    check(!a->step(t,out,e) && !e.empty(),"time jump rejected");
    check(a->reset(e),"reset successful");
    t=input(0);check(a->step(t,out,e)&&out.gyro.updated,"time restarts at zero");
}
void noisy_reset_and_independence() {
    std::string e;
    auto first=sa::airsim::create_projectairsim_imu_adapter(setup("imu-noisy.json"),e);
    auto second=sa::airsim::create_projectairsim_imu_adapter(setup("imu-noisy.json"),e);
    check(first && second,"noisy instances created");if(!first || !second) return;
    const auto x=run(*first,128);
    check(first->reset(e),"seeded reset succeeds");
    const auto y=run(*first,128), z=run(*second,128);
    for(size_t i=0;i<x.size();++i) {
        check(x[i].gyro.value.x==y[i].gyro.value.x && x[i].accel.value.z==y[i].accel.value.z,"reset bit-for-bit");
        check(x[i].gyro.value.x==z[i].gyro.value.x && x[i].accel.value.z==z[i].accel.value.z,"independent instances reproducible");
    }
    sa::ImuCalibration cal{};
    check(first->capabilities().noise && first->capabilities().drift && first->capabilities().thread_safe_across_instances,"noise/drift/instance capability");
    check(!first->capabilities().delay && !first->capabilities().range_clamp,"unsupported features declared false");
    check(first->calibration(cal) && near(cal.accel_zero_g_offset.x,0.01),"initial calibration");
    std::vector<double> vals;
    first->log_values(vals);
    check(vals.size()==first->log_columns().size() && vals.size()==6,"diagnostic log columns align");
}
void upstream_equation_and_default_rng() {
    // Reference computation follows the four draws in ProjectAirSim/imu.cpp.
    auto s=setup("imu-noisy.json",false);
    std::string e;
    auto a=sa::airsim::create_projectairsim_imu_adapter(s,e);
    if(!a){check(false,"model reference created");return;}
    sa::ImuSample actual{};
    check(a->step(input(0),actual,e),"first measurement");
    const double sqrt_dt=static_cast<float>(std::sqrt(0.004));
    std::array<std::mt19937,3> gens{std::mt19937{1},std::mt19937{2},std::mt19937{3}};
    std::array<std::normal_distribution<double>,3> normal{};
    auto next=[&](){return sa::Vector3{normal[0](gens[0]),normal[1](gens[1]),normal[2](gens[2])};};
    const auto acc_noise=next();(void)next(); // acc white, then acc bias increment
    const auto gyro_noise=next();(void)next();
    check(near(actual.accel.value.x,0.5+acc_noise.x*(0.002353/sqrt_dt)+0.01),"Project AirSim accel equation/reference draw");
    check(near(actual.gyro.value.x,0.1+gyro_noise.x*(0.0000872665/sqrt_dt)+0.001),"Project AirSim gyro equation/reference draw");
}
void invalid_config() {
    auto s=setup("does-not-exist.json");std::string e;
    check(!sa::airsim::create_projectairsim_imu_adapter(s,e) && !e.empty(),"missing config fails closed");
    s=setup("imu-ideal.json");s.physics_step_ns=3000000;
    check(!sa::airsim::create_projectairsim_imu_adapter(s,e) && !e.empty(),"non-integer sensor period fails closed");
}
}
int main() {
    ideal_and_time(); noisy_reset_and_independence(); upstream_equation_and_default_rng(); invalid_config();
    if(failures==0)std::puts("projectairsim_imu_adapter_test: OK");
    return failures?EXIT_FAILURE:EXIT_SUCCESS;
}
