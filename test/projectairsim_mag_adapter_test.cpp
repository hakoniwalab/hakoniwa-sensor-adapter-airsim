#include "hakoniwa/sensor_adapter/airsim/projectairsim_mag_adapter.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace sa=hakoniwa::sensor_adapter;
namespace {
int failures=0;
void check(bool yes, const char* name) {
    if(!yes) {std::fprintf(stderr,"FAIL: %s\n",name);++failures;}
}
bool near(double a, double b, double tol=1e-6) {return std::fabs(a-b)<=tol;}
sa::MagAdapterSetup setup(const char* filename, bool has_seed=true, unsigned vehicle_index=0) {
    sa::MagAdapterSetup s{};
    s.config_path=std::string(HAKO_AIRSIM_CONFIG_DIR)+"/"+filename;
    s.physics_step_ns=1000000;
    s.vehicle_name="test-drone";
    s.vehicle_index=vehicle_index;
    s.has_random_seed=has_seed;
    s.random_seed=9201;
    return s;
}
sa::MagTruth truth(int k) {
    sa::MagTruth t{};
    t.sim_time_ns=static_cast<std::uint64_t>(k)*1000000ULL;
    // Host applies a +90 deg rotation (world->sensor) to the world NWU field.
    t.magnetic_field_world={32000.0-4*k, -15000.0-10*k, -40000.0+2*k};
    t.magnetic_field={-t.magnetic_field_world.y,t.magnetic_field_world.x,t.magnetic_field_world.z};
    const double half=3.14159265358979323846/4;
    t.orientation_world_to_sensor={std::cos(half),0,0,std::sin(half)};
    return t;
}
void ideal_and_hold() {
    std::string e;
    auto mag=sa::airsim::create_projectairsim_mag_adapter(setup("mag-ideal.json"),e);
    check(bool(mag),"ideal created");
    if(!mag){std::fprintf(stderr,"%s\n",e.c_str());return;}
    const auto caps=mag->capabilities();
    check(caps.noise && caps.hard_iron && !caps.soft_iron && !caps.drift,
          "capabilities distinguish constant bias from drift and soft-iron");
    check(!caps.delay && !caps.resolution && !caps.range_clamp,
          "unsupported features are false");
    check(caps.deterministic_with_seed && caps.thread_safe_across_instances,
          "per-instance deterministic random streams");
    for(int k=0;k<65;++k) {
        sa::MagSample out{};
        check(mag->step(truth(k),out,e),"ideal step");
        const int last=k-k%10;
        const auto expect=truth(last);
        check(out.updated==(k%10==0),"100 Hz updates");
        check(out.sample_time_ns==static_cast<std::uint64_t>(last)*1000000,
              "time held at previous sample");
        check(near(out.value.x,expect.magnetic_field.x) &&
              near(out.value.y,expect.magnetic_field.y) &&
              near(out.value.z,expect.magnetic_field.z),
              "sensor-frame nT truth held, without repeating rotation");
        check(out.flags==0,"no saturation");
    }
    std::vector<double> vals;
    mag->log_values(vals);
    check(mag->log_columns().size()==6 && vals.size()==6,
          "six transparent diagnostics");
    bool zeros=true;
    for(double v:vals) zeros &= (v==0.0);
    check(zeros,"ideal random bias and noise are all zero");
}
void scale_is_isotropic() {
    std::string e;
    auto mag=sa::airsim::create_projectairsim_mag_adapter(setup("mag-scaled.json"),e);
    check(bool(mag),"scaled created");if(!mag)return;
    sa::MagSample s{};
    check(mag->step(truth(0),s,e),"scaled step");
    const auto v=truth(0).magnetic_field;
    check(near(s.value.x,1.25*v.x) &&
          near(s.value.y,1.25*v.y) &&
          near(s.value.z,1.25*v.z),
          "scalar scale-factor multiplies all components, not a soft-iron matrix");
}
void native_default_gauss_and_bias_distribution() {
    std::string e;
    auto mag=sa::airsim::create_projectairsim_mag_adapter(setup("mag-noise.json",false),e);
    check(bool(mag),"native default-seed created");if(!mag)return;
    sa::MagSample s{};
    check(mag->step(truth(0),s,e),"first native-style noise draw");
    // Project AirSim uses two distinct RandomVectorGaussianR objects.
    // Both have x/y/z mt19937 seeds 1/2/3, and the startup bias has
    // Gaussian(mean=-noise_bias, stddev=noise_bias), not a uniform offset.
    std::array<std::mt19937,3> noise_rng{std::mt19937{1u},std::mt19937{2u},std::mt19937{3u}};
    std::array<std::mt19937,3> bias_rng{std::mt19937{1u},std::mt19937{2u},std::mt19937{3u}};
    const double sigma[3]={0.005,0.004,0.006};
    const double bias_mag[3]={0.001,0.002,0.003};
    const double sensor_nt[3]={truth(0).magnetic_field.x,
                               truth(0).magnetic_field.y,truth(0).magnetic_field.z};
    const double actual_nt[3]={s.value.x,s.value.y,s.value.z};
    std::vector<double> log;
    mag->log_values(log);
    check(log.size()==6,"bias and white-noise logs exist");
    for(int axis=0;axis<3;++axis) {
        std::normal_distribution<double> bias(-bias_mag[axis],bias_mag[axis]);
        std::normal_distribution<double> noise(0.0,sigma[axis]);
        const double expected_bias_g=bias(bias_rng[axis]);
        const double expected_white_g=noise(noise_rng[axis]);
        // The draws are on Project AirSim's body axes (FRD): y and z change sign in FLU.
        const double frd_to_flu[3]={1.0,-1.0,-1.0};
        const double expected=sensor_nt[axis]+frd_to_flu[axis]*(expected_bias_g+expected_white_g)*100000.0;
        check(near(actual_nt[axis],expected,1e-5),
              "Gauss->nT startup bias + Gaussian white noise equation");
        check(near(log[axis],frd_to_flu[axis]*expected_bias_g*100000.0,1e-6),
              "startup bias diagnostic matches independent RNG");
        check(near(log[3+axis],frd_to_flu[axis]*expected_white_g*100000.0,1e-6),
              "white noise diagnostic matches independent RNG");
    }
}
void seeded_reset_and_independent_instances() {
    std::string e;
    auto first=sa::airsim::create_projectairsim_mag_adapter(setup("mag-noise.json",true,4),e);
    auto second=sa::airsim::create_projectairsim_mag_adapter(setup("mag-noise.json",true,4),e);
    auto third=sa::airsim::create_projectairsim_mag_adapter(setup("mag-noise.json",true,5),e);
    check(bool(first) && bool(second) && bool(third),"seeded three instances");
    if(!first||!second||!third)return;
    std::vector<sa::MagSample> baseline;
    for(int k=0;k<110;++k){
        sa::MagSample s{};
        check(first->step(truth(k),s,e),"first seeded steps");
        baseline.push_back(s);
    }
    check(first->reset(e),"reset");
    bool different=false;
    for(int k=0;k<110;++k) {
        sa::MagSample a{},b{},c{};
        check(first->step(truth(k),a,e) &&
              second->step(truth(k),b,e) &&
              third->step(truth(k),c,e),"parallel-instances sequential stepping");
        check(a.value.x==baseline[k].value.x &&
              a.value.y==baseline[k].value.y &&
              a.value.z==baseline[k].value.z &&
              a.updated==baseline[k].updated,
              "reset reproduces measurements");
        check(a.value.x==b.value.x && a.value.y==b.value.y &&
              a.value.z==b.value.z,"identical identity and seed reproduce independently");
        if(k%10==0 && a.value.x!=c.value.x)different=true;
    }
    check(different,"distinct vehicle IDs isolate random sequences");
}
void invalid_config_and_inputs() {
    std::string e;
    check(!sa::airsim::create_projectairsim_mag_adapter(setup("mag-does-not-exist.json"),e) &&
          !e.empty(),"missing config fails closed");
    check(!sa::airsim::create_projectairsim_mag_adapter(setup("mag-bad-sigma.json"),e) &&
          !e.empty(),"negative standard deviation fails closed");
    check(!sa::airsim::create_projectairsim_mag_adapter(setup("mag-unsupported-origin.json"),e) &&
          !e.empty(),"mounting pose cannot be silently double-applied");
    auto wrong_step=setup("mag-ideal.json");
    wrong_step.physics_step_ns=3000000; // 10ms is not divisible by 3ms
    check(!sa::airsim::create_projectairsim_mag_adapter(wrong_step,e) &&
          !e.empty(),"incompatible sampling interval fails closed");
    auto zero_step=setup("mag-ideal.json");zero_step.physics_step_ns=0;
    check(!sa::airsim::create_projectairsim_mag_adapter(zero_step,e),
          "zero physics step rejected");
    auto mag=sa::airsim::create_projectairsim_mag_adapter(setup("mag-ideal.json"),e);
    check(bool(mag),"valid adapter exists");if(!mag)return;
    sa::MagSample sample{};
    auto wrong_time=truth(0);wrong_time.sim_time_ns=17;
    check(!mag->step(wrong_time,sample,e) && !e.empty(),"unexpected time rejected");
    auto nan_truth=truth(0);
    nan_truth.magnetic_field.x=std::numeric_limits<double>::quiet_NaN();
    check(!mag->step(nan_truth,sample,e) && !e.empty(),"non-finite field rejected");
    check(mag->step(truth(0),sample,e) && e.empty(),
          "valid input succeeds after failures without advancing clock");
}
} // namespace

int main() {
    ideal_and_hold();
    scale_is_isotropic();
    native_default_gauss_and_bias_distribution();
    seeded_reset_and_independent_instances();
    invalid_config_and_inputs();
    if(failures==0)std::puts("projectairsim_mag_adapter_test: OK");
    return failures?EXIT_FAILURE:EXIT_SUCCESS;
}
