#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

/**
 * @brief GPS adapter running Project AirSim's own core_sim code (not a re-implementation).
 *
 * Built only with HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE=ON, from the core_sim files pinned in
 * third_party/projectairsim/lock.json. setup.config_path is a JSON object with core_sim's GPS
 * settings ("eph-*", "epv-*") and the adapter's "update-rate-hz". Returns nullptr and sets error
 * on failure.
 */
std::unique_ptr<IGpsSensorAdapter> create_projectairsim_native_gps_adapter(const GpsAdapterSetup& setup,
                                                                          std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
