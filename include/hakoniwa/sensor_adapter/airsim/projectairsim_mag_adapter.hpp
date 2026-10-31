#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

// Lightweight model-equation compatibility adapter, NOT native core_sim::Magnetometer.
// Takes a host-provided magnetic field already resolved in the sensor FLU axes [nT].
// Native Project AirSim EarthUtils dipole/constant field sources are not embedded.
// config_path: a single Project AirSim-style "magnetometer" sensor JSON object.
std::unique_ptr<IMagSensorAdapter> create_projectairsim_mag_adapter(
    const MagAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
