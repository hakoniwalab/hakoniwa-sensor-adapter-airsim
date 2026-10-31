# hakoniwa-sensor-adapter-airsim

Standalone **Project AirSim IMU, GPS and barometer model-equation compatibility prototypes**
implementing the public [hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter)
`IImuSensorAdapter`, `IGpsSensorAdapter` and `IBaroSensorAdapter` contracts. This is **not** a binary wrapper for Project AirSim's
native `core_sim` library, and runtime equivalence is **not yet proven**.

## Scope and limitations

Implemented: IMU gyro/accel, GPS and barometer compatibility models;
IMU uses NWU/FLU already-mounted truth input;
fixed host physics step; sample-and-hold; Project AirSim-style white noise,
turn-on bias, bias random walk, seed/reset; capabilities; bias logs; tests.

Not implemented: the original `SensorImpl`, `TopicManager`, `ServiceManager`,
runtime or rendering, magnetometer, dynamic plugin ABI,
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

## GPS model compatibility (PoC)

The native [Project AirSim GPS](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/src/sensors/gps.cpp)
passes geodetic **truth** and linear velocity through unchanged, **without position
or velocity noise**. It changes EPH/EPV using its
[FirstOrderFilter<float>](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/first_order_filter.hpp)
and derives Fix type (no fix / 2D / 3D) from EPH. Upstream describes EPH
and EPV as **dimensionless dilution-of-precision**, not metric uncertainty.

The new `create_projectairsim_gps_adapter()` implements `IGpsSensorAdapter`.
It accepts a **single GPS sensor JSON object**, e.g. [`gps-default.json`](config/gps-default.json),
using the upstream keys `eph-time-constant`, `epv-time-constant`,
`eph-initial`, `epv-initial`, `eph-final`, `epv-final`,
`eph-min-3d`, `eph-min-2d`.

- The **adapter-only** `update-rate-hz` controls measurement frequency and
  must be aligned with the host's fixed `physics_step_ns`. This adapter
  advances the filter once at t=0 with dt equal to the sensor interval.
  Native scene scheduler and first-sample dt equivalence are **not verified**.
- Coordinates (WGS-84) and NWU velocity are forwarded without noise at each
  measurement; values are held between samples. `fix_type` follows EPH.
- Public `horizontal_accuracy_m`, `vertical_accuracy_m`, `speed_accuracy_mps`
  are **not filled**, since EPH/EPV are not metres. Raw EPH and EPV instead
  appear in diagnostic `log_values()` columns with unit `dimensionless`.
- The native model offers partial Fix state, but does not provide satellite
  count, metre-valued accuracy or course. `receiver_status=false` therefore
  reflects the complete public API capability; no values are invented.
- No runtime, physics engine, topic/service managers, thread or networking
  are required. This is a mathematical counterpart, **not** the original
  Project AirSim `core_sim::Gps` implementation.

Tests check 10Hz sample-and-hold, truthful position and velocity, EPH/EPV
convergence, Fix 0→2D→3D, reset and independent instances, and bad inputs.
Native Runtime output equivalence remains an explicit future gate.

## Barometer model compatibility (PoC)

This adapter reproduces equations from [Project AirSim barometer.cpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/src/sensors/barometer.cpp),
[barometer.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/barometer.hpp),
[noise_model_utils.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/noise_model_utils.hpp),
[EarthUtils](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/earth_utils.hpp)
and [Environment::SetPosition](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/environment.hpp).
It does **not** instantiate the upstream Project AirSim class.

The native-style operations occur in this order:

1. Fixed standard atmosphere: native Environment computes pressure using
   `EarthUtils::GetGeopotential/Temperature/Pressure` from altitude.
   `BaroTruth` weather is **ignored** as native Project AirSim does.
2. Native GaussianMarkov pressure factor:
   `factor = float(alpha*old + (1-alpha)*normal(0,1)*sigma)`,
   `alpha=exp(-dt/tau)`. Do **not** replace with a conventional OU
   `sqrt(1-alpha`2)` factor: upstream does not implement that.
3. Multiply standard pressure by `(1+factor)`, add Gaussian white noise
   in Pa, then apply QNH: `pressure = pressure - 101325 + 100*qnh_hpa`.
4. Derive `pressure_altitude_m` from corrected pressure using native
   `EarthUtils::GetPressureAltitude`, not raw geometric altitude.
   Ambient temperature is not included in native BarometerMessage:
   `temperature_c=0` and `capabilities.temperature=false`.

`BaroTruth::altitude_m` is used as the upstream environment altitude.
Altitude datum differences (geodetic versus mean sea level) must be
resolved by the caller when comparing models. The provided sea-level
weather is validated but intentionally **not used**.

**Configuration caveat:** upstream `core_sim/src/constant.hpp` spells
the two parsed noise keys `pressure_factor_sigma` and
`uncorrelated_noise_sigma` (underscores), while sample robot JSONC
files mistakenly use `pressure-factor-sigma` and
`uncorrelated-noise-sigma` (hyphens). For auditable comparisons, this
adapter accepts the actual native names and **rejects** the misleading
hyphenated noise aliases. `pressure-factor-tau`, `update-frequency`,
`update-latency`, `startup-delay` and `qnh` retain the upstream naming.
Examples: [ideal](config/baro-ideal.json) and [noise](config/baro-noise.json).

The adapter uses `update-frequency` (50 Hz by default) as its sample
rate; samples are held between measurements, beginning at t=0.
The GaussianMarkov update uses `dt=1/update-frequency`. However, native
`Barometer::Update` receives dt from the Scene, so identical scheduler
semantics are **not yet demonstrated**. Nonzero `update-latency` or
`startup-delay` is explicitly rejected: the native sensor class parses
these but does not apply them in its own `Update` implementation.

Without a caller-provided random seed, both independent
`std::mt19937` streams use the upstream native default seed 42.
When seeded by the host, the adapter intentionally derives
per-vehicle/per-stream seeds. Normal-distribution outputs can differ
across platforms. No Gazebo, Project AirSim Runtime, MuJoCo, network
or background thread is required for this evaluation model.

`capabilities.pressure_altitude/noise/offset/drift` are true
(QNH adjustment qualifies as configured offset); `sea_level_weather`,
`temperature`, `delay`, `range_clamp` and `resolution` are false.
Diagnostic log values expose baseline pressure, multiplicative
drift factor, white noise, and QNH separately.

Tests: standard troposphere against an independent equation, 50 Hz
sample-and-hold, QNH correction before pressure altitude, native-style
float stochastic sequence, seed/reset and multi-instance isolation,
and fail-closed invalid inputs. **Upstream Runtime output equivalence
is not yet established.**

## Next validation gate

1. Run native Project AirSim and this prototype on the same recorded truth.
   Check time-step semantics, units, noise and bias statistics before making
   a claim about fidelity.
2. Decide whether to bridge the native `core_sim` implementation (with its
   management dependencies) or retain this standalone equation-compatible
   evaluation model.
3. Add magnetometer after native-runtime equivalence is evaluated;
   retain the barometer PoC until native runtime equivalence is tested.
   PRO integration and shared-library plugin ABI remain separate work.

Licensed MIT; see LICENSE.
