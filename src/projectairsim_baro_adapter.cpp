// Independent, standalone Project AirSim barometer model-equation compatibility prototype.
// This is NOT the native core_sim::Barometer class and has not been cross-validated
// against the upstream runtime. Original formulas and constants consulted from:
//   iamaisim/ProjectAirSim/core_sim/src/sensors/barometer.cpp
//   iamaisim/ProjectAirSim/core_sim/include/core_sim/sensors/barometer.hpp
//   iamaisim/ProjectAirSim/core_sim/include/core_sim/sensors/noise_model_utils.hpp
//   iamaisim/ProjectAirSim/core_sim/include/core_sim/earth_utils.hpp
//   iamaisim/ProjectAirSim/core_sim/include/core_sim/environment.hpp
// Upstream license MIT. Copyright (c) 2026 Hakoniwa Lab, LLC. MIT License.

#include "hakoniwa/sensor_adapter/airsim/projectairsim_baro_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace hakoniwa::sensor_adapter::airsim {
namespace {

constexpr float kSeaLevelPressurePa = 101325.0f;
constexpr float kSeaLevelTemperatureK = 288.15f;
constexpr float kEarthRadiusM = 6378137.0f;

struct Config {
    // Keep native BarometerParams float defaults.
    float qnh_hpa{kSeaLevelPressurePa / 100.0f};
    float pressure_factor_sigma{0.0365f / 20.0f};
    float pressure_factor_tau{3600.0f};
    float uncorrelated_noise_sigma{0.027f * 100.0f};  // Pa
    std::uint64_t period_ns{20000000};                // 50 Hz
};

Config load_config(const BaroAdapterSetup& setup)
{
    if (setup.physics_step_ns == 0)
        throw std::runtime_error("physics_step_ns must be positive");
    std::ifstream input(setup.config_path);
    if (!input) throw std::runtime_error("cannot open barometer config: " + setup.config_path);
    boost::property_tree::ptree json;
    boost::property_tree::read_json(input, json);
    if (json.get<std::string>("type", "barometer") != "barometer")
        throw std::runtime_error("barometer config type must be 'barometer'");

    Config cfg{};
    cfg.qnh_hpa = json.get<float>("qnh", cfg.qnh_hpa);
    cfg.pressure_factor_tau = json.get<float>("pressure-factor-tau", cfg.pressure_factor_tau);

    // Upstream constant.hpp actually spells these two keys with '_' even
    // though example robot configs use '-'. Use the keys the native parser
    // reads. Reject the misleading aliases instead of silently disregarding
    // requested noise, to keep comparisons auditable.
    if (json.get_optional<float>("pressure-factor-sigma") ||
        json.get_optional<float>("uncorrelated-noise-sigma"))
        throw std::runtime_error(
            "Project AirSim barometer parser expects pressure_factor_sigma and "
            "uncorrelated_noise_sigma (underscores); hyphen aliases are not native keys");
    cfg.pressure_factor_sigma = json.get<float>("pressure_factor_sigma", cfg.pressure_factor_sigma);
    cfg.uncorrelated_noise_sigma = json.get<float>("uncorrelated_noise_sigma", cfg.uncorrelated_noise_sigma);

    const float frequency = json.get<float>("update-frequency", 50.0f);
    const float update_latency = json.get<float>("update-latency", 0.0f);
    const float startup_delay = json.get<float>("startup-delay", 0.0f);
    for (float v : {cfg.qnh_hpa, cfg.pressure_factor_tau, cfg.pressure_factor_sigma,
                    cfg.uncorrelated_noise_sigma, frequency, update_latency, startup_delay}) {
        if (!std::isfinite(v)) throw std::runtime_error("non-finite barometer setting");
    }
    if (cfg.qnh_hpa <= 0 || cfg.pressure_factor_tau <= 0 ||
        cfg.pressure_factor_sigma < 0 || cfg.uncorrelated_noise_sigma < 0 || frequency <= 0)
        throw std::runtime_error("invalid barometer QNH, noise, tau, or update frequency");
    // Native barometer.cpp parses these parameters but does not use them in
    // Barometer::Update. The prototype does not fabricate a delay model.
    if (update_latency != 0 || startup_delay != 0)
        throw std::runtime_error("non-zero update-latency/startup-delay is unsupported in the standalone model");

    const double period_d = 1e9 / static_cast<double>(frequency);
    if (!std::isfinite(period_d) ||
        period_d < static_cast<double>(setup.physics_step_ns) ||
        period_d > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("barometer period must be >= physics_step_ns");
    cfg.period_ns = static_cast<std::uint64_t>(std::llround(period_d));
    if (cfg.period_ns == 0 || cfg.period_ns % setup.physics_step_ns != 0 ||
        std::fabs(period_d - static_cast<double>(cfg.period_ns)) > 0.001)
        throw std::runtime_error("barometer period must be an integer multiple of physics_step_ns");
    return cfg;
}

bool valid_truth(const BaroTruth& truth)
{
    return std::isfinite(truth.altitude_m) &&
           truth.altitude_m > -static_cast<double>(kEarthRadiusM) &&
           std::isfinite(truth.sea_level_pressure_pa) && truth.sea_level_pressure_pa > 0 &&
           std::isfinite(truth.sea_level_temperature_c) &&
           truth.sea_level_temperature_c > -273.15;
}

// Project AirSim Environment::SetPosition uses EarthUtils standard atmosphere
// and does not incorporate sea-level weather from the host's BaroTruth.
// All the following calculations intentionally use float intermediates.
// Units: geometric height [m] -> geopotential height [km] -> pressure [Pa].
float standard_pressure_pa(double altitude_m)
{
    const float h_km = static_cast<float>(altitude_m / 1000.0);
    const float radius_km = kEarthRadiusM / 1000.0f;
    const float h = radius_km * h_km / (radius_km + h_km);
    float temperature_k;
    if (h <= 11.0f) temperature_k = kSeaLevelTemperatureK - 6.5f * h;
    else if (h <= 20.0f) temperature_k = 216.65f;
    else if (h <= 32.0f) temperature_k = 196.65f + h;
    else if (h <= 47.0f) temperature_k = 228.65f + 2.8f * (h - 32.0f);
    else if (h <= 51.0f) temperature_k = 270.65f;
    else if (h <= 71.0f) temperature_k = 270.65f - 2.8f * (h - 51.0f);
    else if (h <= 84.85f) temperature_k = 214.65f - 2.0f * (h - 71.0f);
    else temperature_k = 3.0f;

    if (h <= 11.0f)
        return kSeaLevelPressurePa * std::pow(kSeaLevelTemperatureK / temperature_k, -5.255877f);
    if (h <= 20.0f) return 22632.06f * std::exp(-0.1577f * (h - 11.0f));
    if (h <= 32.0f) return 5474.889f * std::pow(216.65f / temperature_k, 34.16319f);
    if (h <= 47.0f) return 868.0187f * std::pow(228.65f / temperature_k, 12.2011f);
    if (h <= 51.0f) return 110.9063f * std::exp(-0.1262f * (h - 47.0f));
    if (h <= 71.0f) return 66.93887f * std::pow(270.65f / temperature_k, -12.2011f);
    if (h <= 84.85f) return 3.956420f * std::pow(214.65f / temperature_k, -17.0816f);
    return 1.0e-3f;
}

float native_pressure_altitude_m(float pressure_pa)
{
    // EarthUtils::GetPressureAltitude. The constants include feet -> metres.
    return (1.0f - std::pow(pressure_pa / kSeaLevelPressurePa, 0.190284f)) *
           145366.45f * 0.3048f;
}

std::uint32_t seed_for(const BaroAdapterSetup& setup, std::uint32_t stream)
{
    if (!setup.has_random_seed) return 42u; // native RandomGeneratorGaussianF default
    // Per-adapter, per-stream FNV-1a seed; unlike the native static seed 42.
    std::uint64_t hash = 14695981039346656037ULL;
    auto mix = [&](std::uint8_t b) { hash = (hash ^ b) * 1099511628211ULL; };
    for (int i = 0; i < 8; ++i) mix(static_cast<std::uint8_t>(setup.random_seed >> (8 * i)));
    for (int i = 0; i < 4; ++i) mix(static_cast<std::uint8_t>(setup.vehicle_index >> (8 * i)));
    for (unsigned char c : setup.vehicle_name) mix(c);
    mix(0);
    for (unsigned char c : setup.sensor_name) mix(c);
    mix(0);
    for (int i = 0; i < 4; ++i) mix(static_cast<std::uint8_t>(stream >> (8 * i)));
    return static_cast<std::uint32_t>(hash ^ (hash >> 32));
}

class ProjectAirSimBaroAdapter final : public IBaroSensorAdapter {
public:
    ProjectAirSimBaroAdapter(BaroAdapterSetup setup, Config cfg)
        : setup_(std::move(setup)), cfg_(cfg),
          markov_dist_(0.0f, 1.0f),
          white_dist_(0.0f, cfg_.uncorrelated_noise_sigma)
    {
        columns_ = {{"standard_pressure", "Pa"}, {"pressure_factor", "1"},
                    {"white_noise", "Pa"}, {"qnh", "hPa"}};
        reset_state();
    }

    bool step(const BaroTruth& truth, BaroSample& out, std::string& error) override
    {
        try {
            if (truth.sim_time_ns != next_ns_)
                throw std::runtime_error("sim_time_ns must advance by physics_step_ns from zero");
            if (!valid_truth(truth)) throw std::runtime_error("invalid barometer truth");
            if (next_ns_ > std::numeric_limits<std::uint64_t>::max() - setup_.physics_step_ns)
                throw std::runtime_error("simulation time overflow");

            if (truth.sim_time_ns % cfg_.period_ns == 0) {
                // Upstream Barometer::Update takes sim_dt_nanos from Scene.
                // The standalone adapter assumes one update each sensor period,
                // starting at t=0, with dt = period.
                const double dt = cfg_.period_ns / 1.0e9;
                const double alpha = std::exp(-dt / cfg_.pressure_factor_tau);
                // Upstream GaussianMarkov: alpha*old + (1-alpha)*normal*sigma.
                // Deliberately NOT the conventional sqrt(1-alpha^2) OU update.
                const float markov_random = markov_dist_(markov_rng_);
                pressure_factor_ = static_cast<float>(
                    alpha * pressure_factor_ +
                    (1.0 - alpha) * markov_random * cfg_.pressure_factor_sigma);
                const float white_noise = white_dist_(white_rng_);

                const float baseline = standard_pressure_pa(truth.altitude_m);
                float measured = baseline;
                measured += measured * pressure_factor_;
                measured += white_noise;
                measured = measured - kSeaLevelPressurePa + cfg_.qnh_hpa * 100.0f;
                if (!std::isfinite(measured) || measured <= 0.0f)
                    throw std::runtime_error("barometer produced non-positive or non-finite pressure");

                last_ = {};
                last_.updated = true;
                last_.sample_time_ns = truth.sim_time_ns;
                last_.pressure_pa = measured;
                // No observed ambient temperature in upstream BarometerMessage.
                last_.temperature_c = 0.0;
                last_.pressure_altitude_m = native_pressure_altitude_m(measured);
                standard_pressure_last_ = baseline;
                white_noise_last_ = white_noise;
            } else {
                last_.updated = false;
            }
            out = last_;
            next_ns_ += setup_.physics_step_ns;
            error.clear();
            return true;
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
    }

    bool reset(std::string& error) override
    {
        reset_state();
        error.clear();
        return true;
    }

    BaroAdapterCapabilities capabilities() const override
    {
        BaroAdapterCapabilities caps{};
        caps.sea_level_weather = false; // upstream uses a fixed standard atmosphere
        caps.temperature = false;       // no temperature in upstream BarometerMessage
        caps.pressure_altitude = true;
        caps.noise = true;
        caps.offset = true;            // pressure adjustment via configured QNH
        caps.drift = true;             // Gaussian-Markov pressure factor
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override
    {
        values = {standard_pressure_last_, pressure_factor_, white_noise_last_, cfg_.qnh_hpa};
    }

private:
    void reset_state()
    {
        markov_rng_.seed(seed_for(setup_, 1u));
        white_rng_.seed(seed_for(setup_, 2u));
        markov_dist_.reset();
        white_dist_.reset();
        next_ns_ = 0;
        last_ = {};
        pressure_factor_ = 0.0f;
        standard_pressure_last_ = 0.0f;
        white_noise_last_ = 0.0f;
    }

    BaroAdapterSetup setup_;
    Config cfg_;
    std::mt19937 markov_rng_{};
    std::mt19937 white_rng_{};
    std::normal_distribution<float> markov_dist_;
    std::normal_distribution<float> white_dist_;
    std::uint64_t next_ns_{0};
    BaroSample last_{};
    float pressure_factor_{0};
    float standard_pressure_last_{0};
    float white_noise_last_{0};
    std::vector<LogColumn> columns_;
};
}  // namespace

std::unique_ptr<IBaroSensorAdapter> create_projectairsim_baro_adapter(
    const BaroAdapterSetup& setup, std::string& error)
{
    try {
        auto adapter = std::make_unique<ProjectAirSimBaroAdapter>(setup, load_config(setup));
        error.clear();
        return adapter;
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}
}  // namespace hakoniwa::sensor_adapter::airsim
