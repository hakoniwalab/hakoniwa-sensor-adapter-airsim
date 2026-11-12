// Contract test of the "airsim" descriptor: the sensor types it provides, and
// adapters created through it from definition files that use the host's type
// names (baro, mag) as well as the Project AirSim ones.
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(airsim);

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

template <class Setup>
Setup setup_for(const char* file)
{
    Setup setup;
    setup.config_path = std::string(HAKO_AIRSIM_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;
    setup.has_random_seed = true;
    setup.random_seed = 1;
    return setup;
}

}  // namespace

int main()
{
    const sa::SensorAdapterImplementation* airsim = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(airsim)();
    check(airsim != nullptr, "the entry point returns the descriptor");
    if (airsim == nullptr) {
        return 1;
    }
    check(airsim->version == sa::kSensorAdapterImplementationVersion, "descriptor version");
    check(std::strcmp(airsim->name, "airsim") == 0, "descriptor name");
    const std::vector<std::string> expected {"imu", "mag", "baro", "gps"};
    check(sa::provided_sensor_types(*airsim) == expected, "provides imu, mag, baro and gps only");

    std::string error;
    check(airsim->create_imu(setup_for<sa::ImuAdapterSetup>("imu-ideal.json"), error) != nullptr,
          "IMU through the descriptor");
    check(airsim->create_gps(setup_for<sa::GpsAdapterSetup>("gps-default.json"), error) != nullptr,
          "GPS through the descriptor");
    check(airsim->create_baro(setup_for<sa::BaroAdapterSetup>("baro-ideal.json"), error) != nullptr,
          "barometer (type barometer) through the descriptor");
    check(airsim->create_baro(setup_for<sa::BaroAdapterSetup>("baro-host-type.json"), error) != nullptr,
          "barometer (type baro) through the descriptor");
    check(airsim->create_mag(setup_for<sa::MagAdapterSetup>("mag-ideal.json"), error) != nullptr,
          "magnetometer (type magnetometer) through the descriptor");
    check(airsim->create_mag(setup_for<sa::MagAdapterSetup>("mag-host-type.json"), error) != nullptr,
          "magnetometer (type mag) through the descriptor");

    if (failures != 0) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("airsim_implementation_test: OK\n");
    return 0;
}
