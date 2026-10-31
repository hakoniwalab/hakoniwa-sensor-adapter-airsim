#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

/**
 * @brief Barometer adapter running Project AirSim's own core_sim code (not a re-implementation).
 *
 * Built only with HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE=ON, from the core_sim files pinned in
 * third_party/projectairsim/lock.json. setup.config_path is a JSON object with core_sim's barometer
 * settings ("qnh", "pressure_factor_sigma", "pressure-factor-tau", "uncorrelated_noise_sigma",
 * "update-frequency"). Returns nullptr and sets error on failure.
 */
std::unique_ptr<IBaroSensorAdapter> create_projectairsim_native_baro_adapter(const BaroAdapterSetup& setup,
                                                                          std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
