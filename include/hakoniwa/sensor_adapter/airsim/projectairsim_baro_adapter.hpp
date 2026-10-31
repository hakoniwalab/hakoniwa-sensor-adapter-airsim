#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

// Standalone compatibility prototype for Project AirSim's barometer model.
// NOT the native Project AirSim core_sim::Barometer class.
// config_path accepts a single Project AirSim-style barometer JSON object.
// See README for scheduler assumptions, atmosphere model, and non-native behavior.
std::unique_ptr<IBaroSensorAdapter> create_projectairsim_baro_adapter(
    const BaroAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
