#include "hakoniwa/sensor_adapter/airsim/projectairsim_baro_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace {
int failures = 0;
void check(bool pass, const char* message) {
    if (!pass) { std::fprintf(stderr, "FAIL: %s\n", message); ++failures; }
}
bool near(double a, double b, double eps = 1e-4) { return std::fabs(a-b) <= eps; }

sa::BaroAdapterSetup setup(const char* file, bool seeded=true, std::uint32_t vehicle=0) {
    sa::BaroAdapterSetup s{};
    s.config_path = std::string(HAKO_AIRSIM_CONFIG_DIR) + "/" + file;
    s.physics_step_ns = 1000000;
    s.vehicle_index = vehicle;
    s.vehicle_name = "test-drone";
    s.random_seed = 9201;
    s.has_random_seed = seeded;
    return s;
}

sa::BaroTruth truth(int k, double altitude=0.0) {
    sa::BaroTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000ULL;
    t.altitude_m = altitude;
    t.sea_level_pressure_pa = 89000.0; // intentionally NOT the standard 101325 Pa
    t.sea_level_temperature_c = 31.0; // ignored by native atmosphere model
    return t;
}

void ideal_standard_atmosphere_and_hold() {
    std::string error;
    auto baro = sa::airsim::create_projectairsim_baro_adapter(setup("baro-ideal.json"), error);
    check(bool(baro), "ideal barometer created");
    if (!baro) { std::fprintf(stderr, "%s\n", error.c_str()); return; }

    const auto caps = baro->capabilities();
    check(!caps.sea_level_weather && !caps.temperature && caps.pressure_altitude,
          "standard atmosphere and pressure altitude capabilities truthful");
    check(caps.noise && caps.drift && caps.offset &&
          !caps.delay && !caps.range_clamp && !caps.resolution,
          "modeled features and unsupported features declared");
    check(caps.thread_safe_across_instances && caps.deterministic_with_seed,
          "independent deterministic streams");

    // The reference uses independent double-precision standard troposphere
    // formula (close, not bit-identical to native float EarthUtils).
    const double altitude0 = 1000.0;
    double last_pressure=0, last_altitude=0;
    for (int k=0; k<65; ++k) {
        sa::BaroSample sample{};
        auto t = truth(k, altitude0 + 20*k);
        check(baro->step(t, sample, error), "ideal step succeeds");
        if (k%20 == 0) {
            const double h_m = 6378137.0 * (altitude0 + 20*k) / (6378137.0 + altitude0 + 20*k);
            const double expect = 101325.0*std::pow(1.0-0.0065*h_m/288.15, 5.255877);
            check(near(sample.pressure_pa, expect, 3.0),
                  "standard atmosphere computed from altitude (not host weather)");
            last_pressure = sample.pressure_pa;
            last_altitude = sample.pressure_altitude_m;
        }
        check(sample.updated == (k%20==0), "50Hz measurement updates");
        check(sample.sample_time_ns ==
              static_cast<std::uint64_t>(k-k%20)*1000000ULL, "sample time held");
        check(sample.pressure_pa == last_pressure &&
              sample.pressure_altitude_m == last_altitude,
              "pressure and pressure altitude held between measurements");
        check(sample.temperature_c == 0.0,
              "upstream BarometerMessage has no ambient temperature");
    }
    std::vector<double> logs;
    baro->log_values(logs);
    check(baro->log_columns().size()==4 && logs.size()==4,
          "four diagnostics: baseline, drift, white noise, QNH");
    check(near(logs[1],0.0) && near(logs[2],0.0),
          "ideal noise and drift are zero");
}

void qnh_correction_and_pressure_altitude() {
    std::string error;
    auto baro=sa::airsim::create_projectairsim_baro_adapter(setup("baro-qnh.json"),error);
    check(bool(baro),"QNH barometer created");if(!baro)return;
    sa::BaroSample sample{};
    check(baro->step(truth(0,0),sample,error),"QNH measurement succeeds");
    check(near(sample.pressure_pa,102000.0,1e-4),
          "QNH changes measured pressure: 101325-101325+1020*100");
    // Independent pressure altitude calculation with published constants.
    const double expected=(1.0-std::pow(102000.0/101325.0,0.190284))*145366.45*0.3048;
    check(near(sample.pressure_altitude_m, expected, 0.03),
          "pressure altitude is calculated AFTER QNH pressure correction");
    check(sample.pressure_altitude_m < 0.0,
          "QNH above standard pressure yields negative indicated pressure altitude");
}

void default_native_random_equations() {
    std::string error;
    // No host seed: both upstream RandomGeneratorGaussianF instances start
    // from mt19937 seed=42, with independent normal<float> distributions.
    auto baro=sa::airsim::create_projectairsim_baro_adapter(setup("baro-noise.json",false),error);
    check(bool(baro),"native-seed barometer created");if(!baro)return;
    std::mt19937 drift_rng{42u}, white_rng{42u};
    std::normal_distribution<float> drift_dist(0.0f,1.0f);
    std::normal_distribution<float> white_dist(0.0f,2.7f);
    float factor=0.0f;
    int observations=0;
    for (int k=0;k<105;++k) {
        sa::BaroSample sample{};
        check(baro->step(truth(k,0.0),sample,error),"native equation step succeeds");
        if (k%20!=0) continue;
        ++observations;
        const double alpha=std::exp(-0.02/3600.0f);
        factor=static_cast<float>(alpha*factor+(1.0-alpha)*drift_dist(drift_rng)*0.001825f);
        const float white=white_dist(white_rng);
        float pressure=101325.0f;
        pressure+=pressure*factor;
        pressure+=white;
        pressure=pressure-101325.0f+1013.25f*100.0f;
        check(near(sample.pressure_pa,pressure,0.01),
              "pressure follows native GaussianMarkov + float noise + QNH equations");
        std::vector<double> log;
        baro->log_values(log);
        check(log.size()==4 && near(log[0],101325,0.01) &&
              near(log[1],factor,1e-8) && near(log[2],white,1e-5),
              "diagnostics agree with independent native-style reference");
    }
    check(observations==6, "stochastic 50Hz sequence has six samples");
}

void seeded_reset_and_instance_independence() {
    std::string error;
    auto a=sa::airsim::create_projectairsim_baro_adapter(setup("baro-noise.json",true,2),error);
    auto b=sa::airsim::create_projectairsim_baro_adapter(setup("baro-noise.json",true,2),error);
    auto c=sa::airsim::create_projectairsim_baro_adapter(setup("baro-noise.json",true,3),error);
    check(bool(a)&&bool(b)&&bool(c),"three barometers created");
    if (!a||!b||!c)return;
    std::vector<sa::BaroSample> first;
    for (int k=0;k<120;++k) {
        sa::BaroSample sample{};
        check(a->step(truth(k),sample,error),"first instance step");
        first.push_back(sample);
    }
    check(a->reset(error),"reset succeeds");
    bool different=false;
    for (int k=0;k<120;++k) {
        sa::BaroSample sa{},sb{},sc{};
        check(a->step(truth(k),sa,error) &&
              b->step(truth(k),sb,error) &&
              c->step(truth(k),sc,error), "reset/independent instances step");
        check(sa.pressure_pa==first[k].pressure_pa &&
              sa.pressure_altitude_m==first[k].pressure_altitude_m &&
              sa.updated==first[k].updated,
              "reset reproducibility and hold semantics");
        check(sa.pressure_pa==sb.pressure_pa,
              "same seed and identity produces identical results independently");
        if (k%20==0 && sa.pressure_pa!=sc.pressure_pa) different=true;
    }
    check(different,"different vehicle indices produce independent streams");
}

void errors_are_fail_closed() {
    std::string error;
    check(!sa::airsim::create_projectairsim_baro_adapter(setup("missing-baro.json"),error) &&
          !error.empty(),"missing file rejected");
    check(!sa::airsim::create_projectairsim_baro_adapter(setup("baro-bad-alias.json"),error) &&
          !error.empty(),"hyphenated noise aliases rejected: native actually uses underscores");
    check(!sa::airsim::create_projectairsim_baro_adapter(setup("baro-unsupported-delay.json"),error) &&
          !error.empty(),"unimplemented latency rejected");
    auto incompatible=setup("baro-ideal.json");
    incompatible.physics_step_ns=3000000; // 20ms is not an integer multiple
    check(!sa::airsim::create_projectairsim_baro_adapter(incompatible,error) &&
          !error.empty(),"non-integral update period rejected");
    auto zero=setup("baro-ideal.json");zero.physics_step_ns=0;
    check(!sa::airsim::create_projectairsim_baro_adapter(zero,error) &&
          !error.empty(),"zero physics dt rejected");

    auto baro=sa::airsim::create_projectairsim_baro_adapter(setup("baro-ideal.json"),error);
    check(bool(baro),"valid barometer available");if(!baro)return;
    sa::BaroSample sample{};
    auto bad=truth(0);bad.sim_time_ns=5;
    check(!baro->step(bad,sample,error)&&!error.empty(),"time jump rejected");
    bad=truth(0);bad.altitude_m=std::numeric_limits<double>::quiet_NaN();
    check(!baro->step(bad,sample,error)&&!error.empty(),"non-finite altitude rejected");
    bad=truth(0);bad.sea_level_pressure_pa=-1;
    check(!baro->step(bad,sample,error)&&!error.empty(),"invalid host pressure rejected");
    check(baro->step(truth(0),sample,error)&&error.empty(),
          "adapter recovers from invalid input without advancing its clock");
}
}
int main() {
    ideal_standard_atmosphere_and_hold();
    qnh_correction_and_pressure_altitude();
    default_native_random_equations();
    seeded_reset_and_instance_independence();
    errors_are_fail_closed();
    if (failures==0)std::puts("projectairsim_baro_adapter_test: OK");
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
