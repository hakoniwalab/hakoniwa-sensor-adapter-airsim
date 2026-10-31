// Tests of the magnetometer, barometer and GPS adapters that run Project AirSim's own core_sim
// sensors, with comparisons to the reference implementation where it models the same thing.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_baro_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_gps_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_mag_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_baro_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_gps_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_mag_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace as = hakoniwa::sensor_adapter::airsim;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

template <typename SetupT>
SetupT setup_for(const char* file)
{
    SetupT setup{};
    setup.config_path = std::string(HAKO_AIRSIM_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;  // 1 ms
    return setup;                     // no host seed: core_sim's own seeds
}

sa::MagTruth mag_truth(int k)
{
    sa::MagTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    const double s = 0.001 * k;
    t.magnetic_field = {30000.0 * std::cos(s), -4000.0 + 2000.0 * std::sin(3 * s), 35000.0};
    return t;
}

double largest(const sa::Vector3& a, const sa::Vector3& b)
{
    return std::max({std::fabs(a.x - b.x), std::fabs(a.y - b.y), std::fabs(a.z - b.z)});
}

void mag_ideal_and_scaled()
{
    std::string error;
    for (const char* file : {"mag-ideal.json", "mag-scaled.json"}) {
        auto setup = setup_for<sa::MagAdapterSetup>(file);
        auto native = as::create_projectairsim_native_mag_adapter(setup, error);
        auto reference = as::create_projectairsim_mag_adapter(setup, error);
        check(native && reference, "native and reference magnetometers are created");
        if (!native || !reference) {
            std::fprintf(stderr, "  %s\n", error.c_str());
            return;
        }
        double worst = 0.0;
        int samples = 0;
        for (int k = 0; k < 1000; ++k) {
            sa::MagSample a{}, b{};
            check(native->step(mag_truth(k), a, error) && reference->step(mag_truth(k), b, error), "mag step");
            check(a.updated == b.updated, "same sample times as the reference");
            if (a.updated) {
                ++samples;
                worst = std::max(worst, largest(a.value, b.value));
            }
        }
        std::printf("native mag %s: %d samples, largest difference from the reference %.3g nT\n", file, samples,
                    worst);
        check(worst < 1e-6, "without noise the native magnetometer gives scale times the host's field");
    }
}

void mag_noise_matches_reference()
{
    std::string error;
    auto setup = setup_for<sa::MagAdapterSetup>("mag-noise.json");
    auto native = as::create_projectairsim_native_mag_adapter(setup, error);
    auto reference = as::create_projectairsim_mag_adapter(setup, error);
    check(native && reference, "noisy magnetometers are created");
    if (!native || !reference) return;
    double worst = 0.0;
    std::vector<double> first;
    for (int k = 0; k < 20000; ++k) {
        sa::MagSample a{}, b{};
        native->step(mag_truth(k), a, error);
        reference->step(mag_truth(k), b, error);
        if (a.updated) {
            worst = std::max(worst, largest(a.value, b.value));
            if (first.size() < 50) first.push_back(a.value.y);
        }
    }
    std::printf("native vs reference mag over 20 s: largest difference %.3g nT (noise sigma 400-600 nT)\n", worst);
    check(worst < 0.05, "core_sim's noise and startup bias equal the reference implementation (float precision)");

    check(native->reset(error), "mag reset");
    bool same = true;
    for (int k = 0, n = 0; n < 50; ++k) {
        sa::MagSample a{};
        native->step(mag_truth(k), a, error);
        if (a.updated) same &= a.value.y == first[static_cast<std::size_t>(n++)];
    }
    check(same, "a mag reset starts core_sim's sequence again");
    sa::MagAdapterSetup bad = setup_for<sa::MagAdapterSetup>("mag-unsupported-origin.json");
    check(!as::create_projectairsim_native_mag_adapter(bad, error), "a mount rotation in the config is rejected");
}

sa::BaroTruth baro_truth(int k)
{
    sa::BaroTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    t.altitude_m = 120.0 + 30.0 * std::sin(0.0005 * k);
    return t;
}

void baro_matches_reference()
{
    std::string error;
    for (const char* file : {"baro-ideal.json", "baro-qnh.json", "baro-noise.json"}) {
        auto setup = setup_for<sa::BaroAdapterSetup>(file);
        auto native = as::create_projectairsim_native_baro_adapter(setup, error);
        auto reference = as::create_projectairsim_baro_adapter(setup, error);
        check(native && reference, "native and reference barometers are created");
        if (!native || !reference) {
            std::fprintf(stderr, "  %s\n", error.c_str());
            return;
        }
        double worst_pa = 0.0, worst_m = 0.0;
        int samples = 0;
        for (int k = 0; k < 20000; ++k) {
            sa::BaroSample a{}, b{};
            check(native->step(baro_truth(k), a, error) && reference->step(baro_truth(k), b, error), "baro step");
            check(a.updated == b.updated, "same sample times as the reference");
            if (a.updated) {
                ++samples;
                worst_pa = std::max(worst_pa, std::fabs(a.pressure_pa - b.pressure_pa));
                worst_m = std::max(worst_m, std::fabs(a.pressure_altitude_m - b.pressure_altitude_m));
            }
        }
        std::printf("native vs reference baro %s over 20 s (%d samples): pressure %.3g Pa, pressure altitude %.3g m\n",
                    file, samples, worst_pa, worst_m);
        check(worst_pa < 0.05 && worst_m < 0.01, "core_sim's barometer equals the reference implementation (float precision)");
    }
    auto bad = setup_for<sa::BaroAdapterSetup>("baro-bad-alias.json");
    check(!as::create_projectairsim_native_baro_adapter(bad, error), "hyphenated noise keys are rejected");
    bad = setup_for<sa::BaroAdapterSetup>("baro-unsupported-delay.json");
    check(!as::create_projectairsim_native_baro_adapter(bad, error), "a latency core_sim does not apply is rejected");
}

void gps_maps_and_filters()
{
    std::string error;
    auto setup = setup_for<sa::GpsAdapterSetup>("gps-default.json");
    setup.origin = {35.681236, 139.767125, 40.0};
    auto native = as::create_projectairsim_native_gps_adapter(setup, error);
    auto reference = as::create_projectairsim_gps_adapter(setup, error);
    check(native && reference, "native and reference GPS are created");
    if (!native || !reference) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double radius = 6378137.0;  // core_sim's EarthUtils::kEarthRadius
    double worst_lat_m = 0.0, worst_lon_m = 0.0, worst_alt = 0.0, worst_v = 0.0, worst_eph = 0.0;
    double worst_steps = 0.0;  // in float steps of the expected degrees
    bool same_fix = true;
    for (int k = 0; k < 20000; ++k) {
        sa::GpsTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        const double s = 0.001 * k;
        t.position_nwu = {50.0 * s, -20.0 * s, 10.0 + 2.0 * s};  // north, west, up
        t.velocity_nwu = {50.0, -20.0, 2.0};
        // Independent expectation for these short distances: a sphere of core_sim's radius.
        t.position.latitude_deg = setup.origin.latitude_deg + t.position_nwu.x / radius * 180.0 / kPi;
        t.position.longitude_deg = setup.origin.longitude_deg +
            (-t.position_nwu.y) / (radius * std::cos(setup.origin.latitude_deg * kPi / 180.0)) * 180.0 / kPi;
        t.position.altitude_m = setup.origin.altitude_m + t.position_nwu.z;
        sa::GpsSample a{}, b{};
        check(native->step(t, a, error) && reference->step(t, b, error), "gps step");
        check(a.updated == b.updated, "same sample times as the reference");
        if (!a.updated) continue;
        worst_lat_m = std::max(worst_lat_m, std::fabs(a.position.latitude_deg - t.position.latitude_deg) * kPi / 180.0 * radius);
        worst_lon_m = std::max(worst_lon_m, std::fabs(a.position.longitude_deg - t.position.longitude_deg) * kPi / 180.0 *
                                                radius * std::cos(setup.origin.latitude_deg * kPi / 180.0));
        auto steps = [](double actual, double expected) {
            const float f = static_cast<float>(expected);
            return std::fabs(actual - expected) / (std::nextafter(f, 1e9f) - f);
        };
        worst_steps = std::max({worst_steps, steps(a.position.latitude_deg, t.position.latitude_deg),
                                steps(a.position.longitude_deg, t.position.longitude_deg)});
        worst_alt = std::max(worst_alt, std::fabs(a.position.altitude_m - t.position.altitude_m));
        worst_v = std::max(worst_v, largest(a.velocity_nwu, t.velocity_nwu));
        same_fix &= a.fix_type == b.fix_type;
        std::vector<double> la, lb;
        native->log_values(la);
        reference->log_values(lb);
        worst_eph = std::max({worst_eph, std::fabs(la[0] - lb[0]), std::fabs(la[1] - lb[1])});
    }
    std::printf("native GPS over 20 s (up to 1 km): position error north %.3g m, east %.3g m, up %.3g m;"
                " velocity %.3g m/s; EPH/EPV vs reference %.3g; degrees within %.2f float steps\n",
                worst_lat_m, worst_lon_m, worst_alt, worst_v, worst_eph, worst_steps);
    // core_sim's GPS message carries degrees as 32-bit floats: here a step is about 0.4 m north and
    // 1.4 m east.
    check(worst_steps <= 1.0, "latitude/longitude follow the local position within one float step");
    check(worst_alt < 0.01 && worst_v < 1e-5, "altitude and velocity pass through");
    check(same_fix && worst_eph < 1e-4, "fix type and EPH/EPV equal the reference implementation");
}

void time_rule()
{
    std::string error;
    auto mag = as::create_projectairsim_native_mag_adapter(setup_for<sa::MagAdapterSetup>("mag-ideal.json"), error);
    sa::MagTruth t = mag_truth(0);
    t.sim_time_ns = 7;
    sa::MagSample s{};
    check(mag && !mag->step(t, s, error), "a time jump is an error");
    auto odd = setup_for<sa::BaroAdapterSetup>("baro-ideal.json");
    odd.physics_step_ns = 3000000;  // 50 Hz is not a whole number of 3 ms steps
    check(!as::create_projectairsim_native_baro_adapter(odd, error), "a period that is not whole steps is rejected");
}

}  // namespace

int main()
{
    mag_ideal_and_scaled();
    mag_noise_matches_reference();
    baro_matches_reference();
    gps_maps_and_filters();
    time_rule();
    if (failures == 0) std::printf("projectairsim_native_mag_baro_gps_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
