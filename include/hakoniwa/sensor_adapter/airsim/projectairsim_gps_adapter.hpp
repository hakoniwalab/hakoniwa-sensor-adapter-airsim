#pragma once

#include <memory>
#include <string>
#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

// Standalone, Project AirSim GPS model-equation compatibility prototype.
// This is NOT the native core_sim::Gps implementation.
// config_path is a single Project AirSim-style GPS sensor JSON object.
std::unique_ptr<IGpsSensorAdapter> create_projectairsim_gps_adapter(
    const GpsAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
