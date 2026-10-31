#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

/**
 * @brief IMU adapter running Project AirSim's own core_sim Imu (not a re-implementation).
 *
 * Built only with HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE=ON, from the core_sim files pinned in
 * third_party/projectairsim/lock.json. setup.config_path is a JSON object with the core_sim IMU
 * settings ("accelerometer", "gyroscope") and the adapter's "update-rate-hz". Returns nullptr and
 * sets error on failure.
 */
std::unique_ptr<IImuSensorAdapter> create_projectairsim_native_imu_adapter(const ImuAdapterSetup& setup,
                                                                           std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
