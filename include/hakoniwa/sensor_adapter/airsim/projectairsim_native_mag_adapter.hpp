#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

/**
 * @brief Magnetometer adapter running Project AirSim's own core_sim code (not a re-implementation).
 *
 * Built only with HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE=ON, from the core_sim files pinned in
 * third_party/projectairsim/lock.json. The field is the host's (MagTruth::magnetic_field); core_sim's
 * Magnetometer adds its noise and startup bias. setup.config_path is a JSON object with core_sim's
 * "scale-factor", "noise-sigma" and "noise-bias" and the adapter's "update-rate-hz". Returns nullptr
 * and sets error on failure.
 */
std::unique_ptr<IMagSensorAdapter> create_projectairsim_native_mag_adapter(const MagAdapterSetup& setup,
                                                                          std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
