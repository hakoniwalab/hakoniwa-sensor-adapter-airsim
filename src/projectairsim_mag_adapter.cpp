// Standalone, math-model compatibility adapter for Project AirSim's magnetometer.
// NOT the native core_sim::Magnetometer class. Native Runtime equivalence unverified.
// Source of the noise/scale/bias equations and defaults:
// https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/src/sensors/magnetometer.cpp
// https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/magnetometer.hpp
// https://github.com/iamaisim/ProjectAirSim/blob/main/core_sim/include/core_sim/sensors/noise_model_utils.hpp
// Upstream is MIT. Copyright (c) 2026 Hakoniwa Lab, LLC. MIT License.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_mag_adapter.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace hakoniwa::sensor_adapter::airsim {
namespace {

// Project AirSim stores magnetometer values in Gauss; public Hakoniwa API uses nT.
constexpr double kNanoteslaPerGauss = 100000.0;
constexpr double kTimeNsPerSecond = 1e9;

struct Config {
    double scale_factor{1.0};  // scalar; not a soft-iron distortion matrix
    Vector3 noise_sigma_g{0.005, 0.005, 0.005};
    Vector3 noise_bias_g{}; // upstream startup: Gaussian(mean=-b, stddev=b)
    std::uint64_t period_ns{};
};

Vector3 parse_vector(const std::string& source, const char* name)
{
    std::istringstream input(source);
    Vector3 v{};
    std::string trailing;
    if (!(input >> v.x >> v.y >> v.z) || (input >> trailing) ||
        !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
        throw std::runtime_error(std::string("invalid vector for ") + name + ": expected 'x y z'");
    return v;
}
void check_nonnegative(const Vector3& v, const char* name)
{
    if (v.x < 0 || v.y < 0 || v.z < 0)
        throw std::runtime_error(std::string(name) + " must be nonnegative on every axis");
}
Config load_config(const MagAdapterSetup& setup)
{
    if (setup.physics_step_ns == 0)
        throw std::runtime_error("physics_step_ns must be positive");

    std::ifstream file(setup.config_path);
    if (!file) throw std::runtime_error("cannot open magnetometer config: " + setup.config_path);
    boost::property_tree::ptree json;
    boost::property_tree::read_json(file, json);
    if (json.get<std::string>("type", "magnetometer") != "magnetometer")
        throw std::runtime_error("magnetometer config type must be 'magnetometer'");

    // Host has applied both the magnetic world-field model and mounting pose.
    // Silently accepting native ref-source/origin configuration would claim
    // semantics that this common-truth evaluation model does not implement.
    if (json.get_child_optional("origin") || json.get_optional<std::string>("ref-source") ||
        json.get_optional<std::string>("dynamic-reference-source"))
        throw std::runtime_error(
            "origin/ref-source are native field/mounting options; this adapter requires "
            "an already-rotated common MagTruth.magnetic_field in nT");

    Config cfg{};
    cfg.scale_factor = json.get<double>("scale-factor", 1.0);
    cfg.noise_sigma_g = parse_vector(
        json.get<std::string>("noise-sigma", "0.005 0.005 0.005"), "noise-sigma");
    cfg.noise_bias_g = parse_vector(
        json.get<std::string>("noise-bias", "0 0 0"), "noise-bias");
    if (!std::isfinite(cfg.scale_factor) || cfg.scale_factor <= 0)
        throw std::runtime_error("scale-factor must be positive and finite");
    check_nonnegative(cfg.noise_sigma_g, "noise-sigma");
    check_nonnegative(cfg.noise_bias_g, "noise-bias");

    // Native Update() is driven by the Scene; update-rate-hz is adapter-only.
    const double hz = json.get<double>("update-rate-hz",
        kTimeNsPerSecond / static_cast<double>(setup.physics_step_ns));
    if (!std::isfinite(hz) || hz <= 0)
        throw std::runtime_error("update-rate-hz must be positive and finite");
    const double ns = kTimeNsPerSecond / hz;
    if (!std::isfinite(ns) || ns < static_cast<double>(setup.physics_step_ns) ||
        ns > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("magnetometer period must be >= physics_step_ns");
    cfg.period_ns = static_cast<std::uint64_t>(std::llround(ns));
    if (cfg.period_ns == 0 || cfg.period_ns % setup.physics_step_ns != 0 ||
        std::fabs(ns - static_cast<double>(cfg.period_ns)) > 0.001)
        throw std::runtime_error("magnetometer period must be an integer multiple of physics_step_ns");
    return cfg;
}
bool finite(const Vector3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
Vector3 add(const Vector3& a, const Vector3& b)
{
    return {a.x+b.x, a.y+b.y, a.z+b.z};
}
Vector3 mul(const Vector3& v, double k)
{
    return {v.x*k, v.y*k, v.z*k};
}

// A separate mt19937 per axis, as in upstream RandomVectorGaussianR,
// which seeds x/y/z to 1/2/3. Two such generators exist upstream: white
// measurement noise and startup bias. Host seeds intentionally replace those
// streams with independent per-adapter streams (not bit-identical to native).
std::uint32_t seed_for(const MagAdapterSetup& setup, std::uint32_t axis, std::uint32_t stream)
{
    if (!setup.has_random_seed) return axis+1u;
    std::uint64_t hash = 14695981039346656037ULL;
    const auto mix = [&hash](std::uint8_t byte) {
        hash = (hash ^ static_cast<std::uint64_t>(byte)) * 1099511628211ULL;
    };
    for (int i=0; i<8; ++i) mix(static_cast<std::uint8_t>(setup.random_seed >> (8*i)));
    for (int i=0; i<4; ++i) mix(static_cast<std::uint8_t>(setup.vehicle_index >> (8*i)));
    for (unsigned char c : setup.vehicle_name) mix(c);
    mix(0);
    for (unsigned char c : setup.sensor_name) mix(c);
    mix(0);
    mix(static_cast<std::uint8_t>(axis));
    mix(static_cast<std::uint8_t>(stream));
    return static_cast<std::uint32_t>(hash ^ (hash >> 32));
}

class GaussianVector {
public:
    void reset(const MagAdapterSetup& setup, const Vector3& mean,
               const Vector3& sigma, std::uint32_t stream)
    {
        const double m[3]={mean.x, mean.y, mean.z};
        const double s[3]={sigma.x, sigma.y, sigma.z};
        for (std::size_t i=0; i<3; ++i) {
            gen_[i].seed(seed_for(setup, static_cast<std::uint32_t>(i), stream));
            dist_[i]=std::normal_distribution<double>(m[i],s[i]);
            dist_[i].reset();
        }
    }
    Vector3 next()
    {
        return {dist_[0](gen_[0]), dist_[1](gen_[1]), dist_[2](gen_[2])};
    }
private:
    std::array<std::mt19937,3> gen_{};
    std::array<std::normal_distribution<double>,3> dist_{};
};

class ProjectAirSimMagAdapter final : public IMagSensorAdapter {
public:
    ProjectAirSimMagAdapter(MagAdapterSetup setup, Config cfg)
        : setup_(std::move(setup)), cfg_(cfg)
    {
        columns_={
            {"startup_bias_x","nT"},{"startup_bias_y","nT"},{"startup_bias_z","nT"},
            {"white_noise_x","nT"},{"white_noise_y","nT"},{"white_noise_z","nT"}
        };
        reset_state();
    }

    bool step(const MagTruth& truth, MagSample& out, std::string& error) override
    {
        try {
            if (truth.sim_time_ns != next_ns_)
                throw std::runtime_error("sim_time_ns must advance by physics_step_ns from zero");
            if (!finite(truth.magnetic_field))
                throw std::runtime_error("magnetic_field (sensor axes nT) must be finite");
            if (next_ns_ > std::numeric_limits<std::uint64_t>::max() - setup_.physics_step_ns)
                throw std::runtime_error("simulation time overflow");

            if (truth.sim_time_ns % cfg_.period_ns == 0) {
                // The host supplies the field already rotated into the sensor
                // frame. We apply upstream scale/noise/bias, but do not call
                // Project AirSim EarthUtils or re-apply the mounting rotation.
                const Vector3 white_g=noise_.next();
                const Vector3 true_g=mul(truth.magnetic_field, 1.0 / kNanoteslaPerGauss);
                const Vector3 observed_g=add(add(mul(true_g, cfg_.scale_factor), white_g), bias_g_);
                const Vector3 result_nt=mul(observed_g,kNanoteslaPerGauss);
                if (!finite(result_nt))
                    throw std::runtime_error("magnetometer measurement is non-finite");
                last_={true, truth.sim_time_ns, result_nt, 0};
                white_last_nt_=mul(white_g,kNanoteslaPerGauss);
            } else {
                last_.updated=false;
            }
            out=last_;
            next_ns_+=setup_.physics_step_ns;
            error.clear();
            return true;
        } catch (const std::exception& e) {
            error=e.what();
            return false;
        }
    }

    bool reset(std::string& error) override
    {
        reset_state();
        error.clear();
        return true;
    }
    MagAdapterCapabilities capabilities() const override
    {
        MagAdapterCapabilities c{};
        c.noise=true;
        c.hard_iron=true;       // additive fixed startup bias; not a calibration model
        c.soft_iron=false;      // uniform scalar scale, no anisotropic matrix
        c.drift=false;          // bias does NOT evolve after initialization
        c.deterministic_with_seed=true;
        c.thread_safe_across_instances=true;
        return c;
    }
    const std::vector<LogColumn>& log_columns() const override {return columns_;}
    void log_values(std::vector<double>& values) const override
    {
        const Vector3 bias_nt=mul(bias_g_,kNanoteslaPerGauss);
        values={bias_nt.x,bias_nt.y,bias_nt.z,
                white_last_nt_.x,white_last_nt_.y,white_last_nt_.z};
    }

private:
    void reset_state()
    {
        // Native initialization draws constant bias from Gaussian(-b,b),
        // not uniform(-b,b) and not Gaussian(0,b). This unusual choice
        // must be retained for an honest model comparison.
        const Vector3 negative_bias=mul(cfg_.noise_bias_g,-1.0);
        bias_.reset(setup_,negative_bias,cfg_.noise_bias_g,1u);
        noise_.reset(setup_,{0,0,0},cfg_.noise_sigma_g,2u);
        bias_g_=bias_.next();
        white_last_nt_={};
        last_={};
        next_ns_=0;
    }

    MagAdapterSetup setup_;
    Config cfg_;
    GaussianVector bias_{};
    GaussianVector noise_{};
    Vector3 bias_g_{};
    Vector3 white_last_nt_{};
    MagSample last_{};
    std::uint64_t next_ns_{0};
    std::vector<LogColumn> columns_;
};

} // namespace

std::unique_ptr<IMagSensorAdapter> create_projectairsim_mag_adapter(
    const MagAdapterSetup& setup, std::string& error)
{
    try {
        auto instance=std::make_unique<ProjectAirSimMagAdapter>(setup,load_config(setup));
        error.clear();
        return instance;
    } catch(const std::exception& e) {
        error=e.what();
        return nullptr;
    }
}

} // namespace hakoniwa::sensor_adapter::airsim
