#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::airsim {

// Lightweight Project AirSim IMU *model* compatibility prototype.
// This is not the native core_sim::Imu class; see README.md for scope.
// config_path: a Project AirSim IMU JSON object with optional adapter-only
// "update-rate-hz" (must divide the host's physics step exactly).
// Returns nullptr and sets error on a configuration error.
std::unique_ptr<IImuSensorAdapter> create_projectairsim_imu_adapter(
    const ImuAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::airsim
