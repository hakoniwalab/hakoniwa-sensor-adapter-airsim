// The "airsim" implementation descriptor: hosts that choose sensor
// implementations by name (hakoniwa-sensor-adapter
// sensor_adapter_implementation.hpp) get the reference adapters through it.
// The native adapters are not part of it.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_baro_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_gps_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_imu_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_mag_adapter.hpp"
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(airsim)
{
    namespace airsim = hakoniwa::sensor_adapter::airsim;
    static const hakoniwa::sensor_adapter::SensorAdapterImplementation implementation = [] {
        hakoniwa::sensor_adapter::SensorAdapterImplementation value;
        value.name = "airsim";
        value.create_imu = &airsim::create_projectairsim_imu_adapter;
        value.create_mag = &airsim::create_projectairsim_mag_adapter;
        value.create_baro = &airsim::create_projectairsim_baro_adapter;
        value.create_gps = &airsim::create_projectairsim_gps_adapter;
        return value;
    }();
    return &implementation;
}
