# hakoniwa-sensor-adapter-airsim

This repository has two kinds of Project AirSim adapters behind the public
[hakoniwa-sensor-adapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter) contracts:

| Kind | What runs | Sensors | Factory |
|---|---|---|---|
| Native | Project AirSim's own `core_sim` sensor classes, compiled unchanged from a pinned commit | IMU, GPS, barometer, magnetometer | `create_projectairsim_native_imu_adapter`, `..._gps_...`, `..._baro_...`, `..._mag_...` |
| Reference implementation | The same model equations written again in this repository (no Project AirSim code) | IMU, GPS, barometer, magnetometer | `create_projectairsim_imu_adapter`, `..._gps_...`, `..._baro_...`, `..._mag_...` |

The reference implementation needs only Boost headers. The native adapters need the pinned
`core_sim` files and core_sim's third-party libraries; see [Native adapters](#native-adapters).

## Scope and limitations

Implemented: IMU gyro/accel, GPS, barometer and magnetometer compatibility models;
IMU uses NWU/FLU already-mounted truth input;
fixed host physics step; sample-and-hold; Project AirSim-style white noise,
turn-on bias, bias random walk, seed/reset; capabilities; bias logs; tests.

Not implemented: the original `SensorImpl`, `TopicManager`, `ServiceManager`,
runtime or rendering, dynamic plugin ABI,
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

Noise, turn-on bias and bias random walk are added on Project AirSim's body axes (FRD), as
upstream does: `turn-on-bias` is an FRD vector, and in the FLU output its y and z, like the
noise, have the opposite sign. Bias logs and `calibration()` report FLU values.

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

## Magnetometer model compatibility (PoC)

The magnetometer adapter implements the public
[IMagSensorAdapter](https://github.com/hakoniwalab/hakoniwa-sensor-adapter/blob/main/include/hakoniwa/sensor_adapter/mag_sensor_adapter.hpp)
interface, reproducing the *error-model equations* from
[Project AirSim magnetometer.cpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/src/sensors/magnetometer.cpp),
[magnetometer.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/magnetometer.hpp)
and [noise_model_utils.hpp](https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/noise_model_utils.hpp).

**Deliberate separation of physics truth and sensor errors.** Native
Project AirSim normally uses EarthUtils' low-order geomagnetic dipole
field (or a compiled-in constant field) and rotates it by the combined
body/mounting pose, then scales it and adds noise. In this controlled
comparison adapter, the host supplies the field **already rotated
into the sensor FLU axes**, as `MagTruth::magnetic_field` in **nT**.
No magnetic world-field computation or rotation is done here.
Consequently this is **NOT** the original Project AirSim field model
and cannot be used as evidence of native EarthUtils equivalence.

The native magnetometer reports **Gauss**, whereas the public Hakoniwa
adapter reports **nanotesla** (1 G = 100,000 nT). The model uses
`scale-factor` (scalar), `noise-sigma` and `noise-bias`, with the latter
two configured as strings of three numbers in **Gauss**.
Examples: [ideal](config/mag-ideal.json),
[scaled](config/mag-scaled.json),
[noisy](config/mag-noise.json).

At initialization, the original native implementation calls
`RandomVectorGaussianR(-noise_bias,noise_bias).next()` to draw a
**constant per-axis startup bias**, then adds new white Gaussian noise
every measurement. This is a Gaussian with **mean = -noise_bias** and
**standard deviation = noise_bias**, not a uniform distribution and
not a zero-mean Gaussian. This unusual behavior is deliberately
preserved, not normalized away.

The observation equation at each sample is:

`sample_nT = 100000 * (scale_factor * (truth_nT / 100000) +
                      N(0, noise_sigma_G) + startup_bias_G)`

Each axis has a separate `std::mt19937` generator. Without a host seed
the white-noise and startup-bias generators each start with seeds
1/2/3, as in the native `RandomVectorGaussianR` implementation.
If `has_random_seed=true` the host seed and sensor/vehicle identity
are deliberately mixed into **independent per-instance streams**.
Such seeded streams are a Hakoniwa extension, not a native behavior.
As with other adapters, Gaussian samples can differ between C++
standard libraries; do not claim cross-platform bit-for-bit matches.

`update-rate-hz` is an **adapter-only** sampling control (100Hz in
the example configs); native Project AirSim updates when its Scene
scheduler calls `Magnetometer::Update`, rather than following this
standalone config key. The interface's 0,dt,2dt,... monotonic rule,
sample-and-hold, `reset()` and diagnostic logs apply.

The adapter rejects `origin`, `ref-source` and
`dynamic-reference-source` rather than silently ignoring native
mounting/dipole options which cannot be honored without the full
runtime. The host remains responsible for field generation,
NWU/FLU conversion, and mounting pose, once only. The scale factor applies to the FLU
field; the white noise and the startup bias are drawn on Project AirSim's body axes (FRD),
as upstream does, so `noise-sigma` and `noise-bias` are FRD vectors and their y and z
appear with the opposite sign in the FLU output and in the logs.

Capabilities report `noise=true` and `hard_iron=true` for the fixed
additive startup bias (not a hard-iron calibration model). An
isotropic `scale-factor` does **not** implement soft-iron matrix
distortion, so `soft_iron=false`. The native magnetometer does not
evolve the bias after initialization, so `drift=false`; delay,
resolution and saturation are also unsupported.

Contract tests cover 100Hz sampling, independently supplied and
pre-rotated magnetic truth, scale in Gauss, independent normal RNG
reference and bias calculation, reset, multiple-instance isolation,
fail-closed malformed settings and invalid timestamps. **Native
Project AirSim runtime equivalence is not established.**


## Native adapters

### Pinned Project AirSim sources

This repository does not contain Project AirSim code. [third_party/projectairsim/lock.json](third_party/projectairsim/lock.json)
records the upstream repository, one commit SHA, and the path and sha256 of every file used:
the `core_sim` sources listed under `sources`, every file they include, and Project AirSim's
`LICENSE` and `NOTICE.txt` (MIT). [tools/projectairsim_lock.py](tools/projectairsim_lock.py) manages them:

- `fetch` downloads each file from that commit (`raw.githubusercontent.com`) into
  `.cache/projectairsim/<commit>/` and checks its sha256. The cache is not committed.
- `verify` checks the cache against the lock file. CMake runs it at configure time and stops
  if a file is missing or changed.
- `lock --clone DIR` writes a new lock file from an unmodified Project AirSim checkout. Run it
  only to move to another upstream commit; the lock file change is then reviewed like code.

Nothing from upstream is patched. Warnings in upstream code are silenced, not fixed.

### Build and test

With [pixi](https://pixi.sh) (conda-forge packages; macOS, Linux, Windows):

```bash
pixi run test
```

This fetches the pinned sources, configures with `-DHAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE=ON`,
builds, and runs all tests. Without pixi, provide the same libraries that core_sim builds with:
Eigen 3, nlohmann/json (`json.hpp`), msgpack-cxx (header-only, compiled with `MSGPACK_NO_BOOST`) and
nng 1.x (core_sim pins v1.11; nng 2.x changed its headers). CMake 3.19 or later.

### What is compiled

core_sim's sensors are created by its `Robot` inside a scene. A standalone adapter needs only the
sensor, its Kinematics/Environment inputs and the topic/service/state managers the sensor
registers with. The subset is the sensors, those managers, every message type, and the
utilities they use, and core_sim's clock (a transform takes its time stamp from it when it is
built; nothing steps it). The managers reference a few functions of the scene, robot and
authorization classes that a standalone sensor never calls; [src/native/core_sim_stubs.cpp](src/native/core_sim_stubs.cpp)
defines them so the subset links, and each one aborts if it is called.

The sensor constructors are private and name `Robot` as a friend. The adapter defines its own
small `microsoft::projectairsim::Robot` that only creates, loads and initializes a sensor, as
core_sim's own unit tests do. Do not link this library into a program that also links
core_sim's real `Robot`.

Results are read from each sensor's `getOutput()` message, so they carry the message's types
(32-bit floats). core_sim declares `GetMagnetometerSettings()` and `GetBarometerSettings()` but
does not define them; the adapters read those settings from the config as core_sim reads them.

### Native IMU

The host gives truth in the sensor axes (FLU). The adapter sets core_sim's Kinematics with the
identity orientation, the angular velocity in FRD, and the linear acceleration as the specific
force plus core_sim's gravity (core_sim's Imu subtracts it again). The Environment is at
latitude/longitude/altitude 0, where core_sim's gravity is 9.80665 m/s². The output is turned
back into FLU.

- Config: a JSON object with core_sim's `accelerometer` and `gyroscope` blocks (as in
  [config/imu-noisy.json](config/imu-noisy.json)), plus the adapter's `update-rate-hz`. The
  period must be a whole number of `physics_step_ns`; `Imu::Update` is called at each sample
  with that period as its dt. Values are held between samples.
- Noise and turn-on bias are on core_sim's body axes (FRD): the config's `turn-on-bias`
  is an FRD vector, so its y and z appear with the opposite sign in the FLU output. The
  reference implementation does the same.
- core_sim seeds its generators with 1, 2, 3 in every instance; a host seed is not used.
  `reset()` creates a new Imu, which starts the same sequence again.
- core_sim computes in 32-bit floats.

Tests ([test/projectairsim_native_imu_adapter_test.cpp](test/projectairsim_native_imu_adapter_test.cpp)):
without noise the output equals the truth within float precision at 250 Hz; with noise, 20 s
of output equals the reference implementation within float precision (same seeds, draw
order and axes); reset repeats the sequence; time jumps and periods that
are not whole steps are rejected.

### Native magnetometer

core_sim's Magnetometer computes the field itself (a dipole model at its position, rotated by the
body orientation) and then adds its white noise and startup bias. The interface gives the field
(`MagTruth::magnetic_field`, sensor axes). The adapter runs two core_sim Magnetometers on the
same position and orientation, the configured one and a copy with zero `noise-sigma` and
`noise-bias`. Their difference is exactly what core_sim adds; the sample is core_sim's
`scale-factor` times the host's field plus that difference, on core_sim's body axes (FRD) turned
into FLU, in nT.

- Config: core_sim's `scale-factor`, `noise-sigma`, `noise-bias` (Gauss) and the adapter's
  `update-rate-hz`. A non-empty `origin` (mount rotation) is rejected: the host applies the
  mount pose.
- `reset()` creates new Magnetometers, which draw the same startup bias again.

### Native barometer

core_sim's Barometer reads the pressure of its Environment, which core_sim computes from the
position with its standard atmosphere. The adapter puts the Environment at
`BaroTruth::altitude_m` above a sea-level origin and returns core_sim's pressure and pressure
altitude. The sea-level weather of `BaroTruth` is not used and the temperature is 0, as in core_sim.

- Config: core_sim's `qnh`, `pressure_factor_sigma`, `pressure-factor-tau`,
  `uncorrelated_noise_sigma` and `update-frequency` (the sample rate, 50 Hz by default).
- Rejected: the hyphenated `pressure-factor-sigma` and `uncorrelated-noise-sigma` (core_sim
  reads these names with underscores and would ignore them silently), and non-zero
  `update-latency` or `startup-delay` (core_sim parses them but does not apply them).

### Native GPS

core_sim's Gps reports the geodetic position of its Environment, which core_sim maps from the
local NED position and the home point. The adapter puts the home at `GpsAdapterSetup::origin`
and the Environment at `GpsTruth::position_nwu`; `GpsTruth::position` is not used. Velocity
passes through. core_sim has no position or velocity noise.

- Latitude, longitude and altitude come from core_sim's GPS message as 32-bit floats: near
  latitude 35.7° and longitude 139.8°, one step is about 0.4 m north and 1.4 m east.
- Fix type (0, 2, 3) is core_sim's, from EPH. EPH and EPV are dilutions of precision, not metres;
  they are logged (`eph`, `epv`) and the accuracy fields are not filled.
- Config: core_sim's `eph-*` and `epv-*` and the adapter's `update-rate-hz`.

Tests ([test/projectairsim_native_mag_baro_gps_adapter_test.cpp](test/projectairsim_native_mag_baro_gps_adapter_test.cpp)):
the magnetometer and barometer outputs equal the reference implementation within float
precision with and without noise (20 s); the GPS latitude and longitude are within one float
step of an independent mapping up to 1 km, and fix type and EPH/EPV equal the reference
implementation; resets, time jumps, periods that are not whole steps and the rejected keys.

## Next validation gate

1. Run native Project AirSim and this prototype on the same recorded truth.
   Check time-step semantics, units, noise and bias statistics before making
   a claim about fidelity.
2. Decide whether to bridge the native `core_sim` implementation (with its
   management dependencies) or retain this standalone equation-compatible
   evaluation model.
3. Evaluate IMU, GPS, barometer and magnetometer against the actual native
   Project AirSim Runtime before claiming native equivalence.
   PRO integration and shared-library plugin ABI remain separate work.

Licensed MIT; see LICENSE.
