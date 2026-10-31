# hakoniwa-sensor-adapter-airsim

A small, standalone **Project AirSim IMU error-model compatibility prototype**
implementing the public [hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter)
`IImuSensorAdapter` contract. This is **not** a binary wrapper for Project AirSim's
native `core_sim` library, and runtime equivalence is **not yet proven**.

## Scope and limitations

Implemented: IMU gyro and accelerometer; NWU/FLU already-mounted truth input;
fixed host physics step; sample-and-hold; Project AirSim-style white noise,
turn-on bias, bias random walk, seed/reset; capabilities; bias logs; tests.

Not implemented: the original `SensorImpl`, `TopicManager`, `ServiceManager`,
runtime or rendering, GPS, barometer, magnetometer, dynamic plugin ABI,
filter/delay/clipping, separate gyro and accel periods.

Source consulted: [ProjectAirSim imu.cpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/src/sensors/imu.cpp),
[imu.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/imu.hpp),
[noise_model_utils.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/noise_model_utils.hpp).
The upstream core is MIT licensed. No original source file or class is copied.
This is **a model-equation compatibility implementation**, not a native
Project AirSim sensor. The native constructor is private and requires
simulation managers; using the full `core_sim` library also brings unrelated
dependencies.

## Build and test

Requires CMake 3.16+, C++17 and Boost headers (PropertyTree JSON parser).
The shared public adapter interface remains header-only without Boost.

`git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter.git`
`git clone https://github.com/hakoniwalab/hakoniwa-sensor-adapter-airsim.git`
`cmake -S hakoniwa-sensor-adapter-airsim -B build-airsim -DHAKONIWA_SENSOR_ADAPTER_AIRSIM_BUILD_TESTS=ON`
`cmake --build build-airsim -j`
`ctest --test-dir build-airsim --output-on-failure`

Or set `-DHAKONIWA_SENSOR_ADAPTER_ROOT=/path/to/hakoniwa-sensor-adapter`.
No Project AirSim runtime, Gazebo, MuJoCo, Unreal, network or background
thread is required for this PoC.

## Configuration and timing

`ImuAdapterSetup::config_path` accepts an individual JSON IMU sensor object
with Project AirSim's original `accelerometer` and `gyroscope` blocks:
`velocity-random-walk`, `angle-random-walk`, `bias-stability`, `tau`, and
`turn-on-bias` as an `"x y z"` string. Defaults match upstream `ImuParams`.
See [config/imu-noisy.json](config/imu-noisy.json).

Two **adapter-only** JSON keys are supported: `update-rate-hz` (default every
physics step) and `min-sample-time` (default 0.001 seconds). The sensor period
must be a positive integer multiple of `physics_step_ns`. The host passes
`sim_time_ns` = 0, dt, 2dt, ... . Values are held between measurements,
with `updated=false` and the original `sample_time_ns`.
Full robot JSONC and the native Project AirSim scheduler are not parsed.

At each sample, the IMU applies the upstream-style equations:

- `measurement = truth + Gaussian(0,1) * random_walk / sqrt(max(dt, min_dt)) + bias`
- `bias_next = bias + Gaussian(0,1) * bias_stability / sqrt(tau) * sqrt(max(dt, min_dt))`

The default, unseeded streams reproduce the upstream per-axis RNG seeds 1/2/3
and draw order **within the same C++ toolchain**. With a host seed, streams
are intentionally derived from the seed plus vehicle name/index and sensor
name so that two instances do not share random state. This differs from the
native Project AirSim random sequence. Standard-library normal distributions
can also vary between OS/compiler environments.

The host applies mounting pose, vibration and NWU/FLU conversion. The input
`specific_force` has already had gravity subtracted. The adapter does not
apply gravity again.

## Next validation gate

1. Run native Project AirSim and this prototype on the same recorded truth.
   Check time-step semantics, units, noise and bias statistics before making
   a claim about fidelity.
2. Decide whether to bridge the native `core_sim` implementation (with its
   management dependencies) or retain this standalone equation-compatible
   evaluation model.
3. Add GPS, barometer and magnetometer only after IMU equivalence is evaluated.
   PRO integration and shared-library plugin ABI remain separate work.

Licensed MIT; see LICENSE.
