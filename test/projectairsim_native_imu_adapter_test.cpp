// Tests of the IMU adapter that runs Project AirSim's own core_sim Imu, and a comparison with the
// reference implementation (create_projectairsim_imu_adapter), which re-writes the same equations.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_imu_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_imu_adapter.hpp"

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

sa::ImuAdapterSetup setup_for(const char* file)
{
    sa::ImuAdapterSetup setup{};
    setup.config_path = std::string(HAKO_AIRSIM_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;  // 1 ms
    return setup;                     // no host seed: core_sim's own seeds
}

sa::ImuTruth truth_at(int k)
{
    sa::ImuTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    const double s = 0.001 * k;
    t.angular_velocity = {0.5 * std::sin(7 * s), -0.3 * std::cos(3 * s), 0.2 + 0.1 * std::sin(40 * s)};
    t.specific_force = {0.8 * std::sin(2 * s), -0.4 * std::cos(5 * s), 9.80665 + 0.5 * std::sin(9 * s)};
    return t;
}

double rel(const sa::Vector3& a, const sa::Vector3& b)
{
    const double d = std::max({std::fabs(a.x - b.x), std::fabs(a.y - b.y), std::fabs(a.z - b.z)});
    const double m = std::max({std::fabs(b.x), std::fabs(b.y), std::fabs(b.z), 1.0});
    return d / m;
}

void ideal_passes_the_truth_at_250_hz()
{
    std::string error;
    auto imu = as::create_projectairsim_native_imu_adapter(setup_for("imu-ideal.json"), error);
    check(imu != nullptr, "native IMU is created");
    if (!imu) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
        sa::ImuSample s{};
        check(imu->step(truth_at(k), s, error), "step");
        check(s.gyro.updated == (k % 4 == 0), "measures every 4 ms from time 0");
        const sa::ImuTruth t = truth_at(k - k % 4);
        worst = std::max({worst, rel(s.gyro.value, t.angular_velocity), rel(s.accel.value, t.specific_force)});
    }
    std::printf("native ideal: largest relative difference from the truth %.3g (32-bit floats)\n", worst);
    check(worst < 1e-6, "native IMU passes the truth (FLU in, FLU out) within float precision");
}

void native_and_reference_agree()
{
    std::string error;
    auto native = as::create_projectairsim_native_imu_adapter(setup_for("imu-noisy.json"), error);
    auto reference = as::create_projectairsim_imu_adapter(setup_for("imu-noisy.json"), error);
    check(native && reference, "native and reference IMU are created");
    if (!native || !reference) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    // Both add the noise and the turn-on bias on Project AirSim's body axes (FRD).
    auto largest = [](const sa::Vector3& a, const sa::Vector3& b) {
        return std::max({std::fabs(a.x - b.x), std::fabs(a.y - b.y), std::fabs(a.z - b.z)});
    };
    double worst_gyro = 0.0, worst_accel = 0.0;
    for (int k = 0; k < 20000; ++k) {
        sa::ImuSample a{}, b{};
        const sa::ImuTruth t = truth_at(k);
        check(native->step(t, a, error) && reference->step(t, b, error), "step both");
        if (a.gyro.updated) {
            worst_gyro = std::max(worst_gyro, largest(a.gyro.value, b.gyro.value));
            worst_accel = std::max(worst_accel, largest(a.accel.value, b.accel.value));
        }
    }
    std::printf("native vs reference over 20 s: gyro %.3g rad/s, accel %.3g m/s2 (largest difference)\n",
                worst_gyro, worst_accel);
    check(worst_gyro < 1e-6 && worst_accel < 1e-5,
          "the reference implementation follows core_sim (same seeds, draw order and axes) within float precision");
}

void reset_starts_the_same_sequence()
{
    std::string error;
    auto imu = as::create_projectairsim_native_imu_adapter(setup_for("imu-noisy.json"), error);
    std::vector<double> first;
    for (int pass = 0; pass < 2; ++pass) {
        bool same = true;
        for (int k = 0; k < 400; ++k) {
            sa::ImuSample s{};
            imu->step(truth_at(k), s, error);
            if (pass == 0) first.push_back(s.gyro.value.x);
            else same &= s.gyro.value.x == first[static_cast<std::size_t>(k)];
        }
        if (pass == 1) check(same, "a reset starts core_sim's sequence again");
        check(imu->reset(error), "reset");
    }
}

void time_jump_and_bad_period()
{
    std::string error;
    auto imu = as::create_projectairsim_native_imu_adapter(setup_for("imu-ideal.json"), error);
    sa::ImuTruth t = truth_at(0);
    t.sim_time_ns = 5;
    sa::ImuSample s{};
    check(imu && !imu->step(t, s, error) && !error.empty(), "a time jump is an error");
    auto odd = setup_for("imu-ideal.json");
    odd.physics_step_ns = 3000000;  // 250 Hz is not a whole number of 3 ms steps
    check(!as::create_projectairsim_native_imu_adapter(odd, error), "a period that is not whole steps is rejected");
}

}  // namespace

int main()
{
    ideal_passes_the_truth_at_250_hz();
    native_and_reference_agree();
    reset_starts_the_same_sequence();
    time_jump_and_bad_period();
    if (failures == 0) std::printf("projectairsim_native_imu_adapter_test: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
