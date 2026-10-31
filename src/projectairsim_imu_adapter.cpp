// Independent, minimal compatibility implementation of the Project AirSim IMU
// error equations. Source consulted: iamaisim/ProjectAirSim,
// core_sim/src/sensors/imu.cpp and core_sim/include/core_sim/sensors/noise_model_utils.hpp
// Copyright (c) 2026 Hakoniwa Lab, LLC. MIT License.
// This is NOT a binary wrapper around Project AirSim's SensorImpl/TopicManager.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_imu_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace hakoniwa::sensor_adapter::airsim {
namespace {

using boost::property_tree::ptree;

struct AxisParams {
    double random_walk{};    // velocity/angle random walk, per sqrt(second)
    double tau{800.0};
    double bias_stability{};
    Vector3 turn_on_bias{};
};

struct Config {
    AxisParams acc, gyro;
    double min_sample_time{0.001};
    std::uint64_t period_ns{};
};

Vector3 parse_vector(const std::string& s, const char* field)
{
    std::istringstream input(s);
    Vector3 result;
    std::string remainder;
    if (!(input >> result.x >> result.y >> result.z) || (input >> remainder) ||
        !std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) {
        throw std::runtime_error(std::string("invalid vector 'x y z' for ") + field);
    }
    return result;
}

void check_axis(const AxisParams& p, const char* name)
{
    if (!std::isfinite(p.random_walk) || p.random_walk < 0 ||
        !std::isfinite(p.bias_stability) || p.bias_stability < 0 ||
        !std::isfinite(p.tau) || p.tau <= 0) {
        throw std::runtime_error(std::string("invalid ") + name + " random walk, bias stability or tau");
    }
}

Config load_config(const ImuAdapterSetup& setup)
{
    if (setup.physics_step_ns == 0) throw std::runtime_error("physics_step_ns must be positive");
    std::ifstream in(setup.config_path);
    if (!in) throw std::runtime_error("cannot open IMU config: " + setup.config_path);
    ptree root;
    boost::property_tree::read_json(in, root);
    Config cfg;
    cfg.acc.random_walk = root.get<double>("accelerometer.velocity-random-walk", 0.24 * 9.80665 / 1000.0);
    cfg.acc.tau = root.get<double>("accelerometer.tau", 800.0);
    cfg.acc.bias_stability = root.get<double>("accelerometer.bias-stability", 36e-6 * 9.80665);
    cfg.acc.turn_on_bias = parse_vector(root.get<std::string>("accelerometer.turn-on-bias", "0 0 0"),
                                        "accelerometer.turn-on-bias");
    cfg.gyro.random_walk = root.get<double>("gyroscope.angle-random-walk", 0.3 / 60.0 * 3.14159265358979323846 / 180.0);
    cfg.gyro.tau = root.get<double>("gyroscope.tau", 500.0);
    cfg.gyro.bias_stability = root.get<double>("gyroscope.bias-stability", 4.6 / 3600.0 * 3.14159265358979323846 / 180.0);
    cfg.gyro.turn_on_bias = parse_vector(root.get<std::string>("gyroscope.turn-on-bias", "0 0 0"),
                                         "gyroscope.turn-on-bias");
    cfg.min_sample_time = root.get<double>("min-sample-time", 0.001);
    const double hz = root.get<double>("update-rate-hz", 1e9 / static_cast<double>(setup.physics_step_ns));
    check_axis(cfg.acc, "accelerometer");
    check_axis(cfg.gyro, "gyroscope");
    if (!std::isfinite(cfg.min_sample_time) || cfg.min_sample_time <= 0 ||
        !std::isfinite(hz) || hz <= 0) throw std::runtime_error("invalid min-sample-time or update-rate-hz");
    const double ns = 1.0e9 / hz;
    if (!std::isfinite(ns) || ns < setup.physics_step_ns ||
        ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
        throw std::runtime_error("sensor update period must be finite and >= physics step");
    }
    cfg.period_ns = static_cast<std::uint64_t>(std::llround(ns));
    if (cfg.period_ns % setup.physics_step_ns != 0 ||
        std::fabs(ns - static_cast<double>(cfg.period_ns)) > 0.001) {
        throw std::runtime_error("sensor period must be an integer multiple of physics_step_ns");
    }
    return cfg;
}

Vector3 add(const Vector3& a, const Vector3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 scale(const Vector3& a, double k) { return {a.x * k, a.y * k, a.z * k}; }
bool finite(const Vector3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// Project AirSim's RandomVectorGaussianR uses mt19937 seeds 1,2,3 for
// x,y,z, and reuses each stream for white noise and both bias increments.
// An explicit host seed produces independent per-adapter streams instead.
class Gaussian3 {
public:
    void reset(const ImuAdapterSetup& setup)
    {
        std::uint64_t seed = setup.random_seed;
        if (setup.has_random_seed) {
            // Stable FNV-1a mixing avoids std::hash implementation differences.
            seed = 14695981039346656037ULL;
            auto mix = [&seed](std::uint8_t b) { seed = (seed ^ b) * 1099511628211ULL; };
            for (unsigned i = 0; i < 8; ++i) mix(static_cast<std::uint8_t>(setup.random_seed >> (8 * i)));
            for (unsigned i = 0; i < 4; ++i) mix(static_cast<std::uint8_t>(setup.vehicle_index >> (8 * i)));
            for (unsigned char c : setup.vehicle_name) mix(c);
            mix(0);
            for (unsigned char c : setup.sensor_name) mix(c);
        }
        for (std::size_t i = 0; i < 3; ++i) {
            const auto axis_seed = setup.has_random_seed ? static_cast<std::uint32_t>(seed + 0x9E3779B97F4A7C15ULL * (i + 1))
                                                         : static_cast<std::uint32_t>(i + 1);
            generators_[i].seed(axis_seed);
            distributions_[i].reset();
        }
    }
    Vector3 next()
    {
        return {distributions_[0](generators_[0]), distributions_[1](generators_[1]),
                distributions_[2](generators_[2])};
    }
private:
    std::array<std::mt19937, 3> generators_{};
    std::array<std::normal_distribution<double>, 3> distributions_{};
};

class ProjectAirSimImuAdapter final : public IImuSensorAdapter {
public:
    ProjectAirSimImuAdapter(ImuAdapterSetup setup, Config cfg)
        : setup_(std::move(setup)), cfg_(cfg)
    {
        columns_ = {{"accel_bias_x", "m/s^2"}, {"accel_bias_y", "m/s^2"}, {"accel_bias_z", "m/s^2"},
                    {"gyro_bias_x", "rad/s"}, {"gyro_bias_y", "rad/s"}, {"gyro_bias_z", "rad/s"}};
        reset_state();
    }

    bool step(const ImuTruth& truth, ImuSample& out, std::string& error) override
    {
        try {
            if (truth.sim_time_ns != next_time_ns_)
                throw std::runtime_error("sim_time_ns must advance by physics_step_ns starting at zero");
            if (!finite(truth.angular_velocity) || !finite(truth.specific_force))
                throw std::runtime_error("IMU truth contains a non-finite value");
            if (next_time_ns_ > std::numeric_limits<std::uint64_t>::max() - setup_.physics_step_ns)
                throw std::runtime_error("simulation time overflow");

            if (truth.sim_time_ns % cfg_.period_ns == 0) {
                const double dt = std::max(cfg_.period_ns / 1e9, cfg_.min_sample_time);
                const double root_dt = static_cast<double>(static_cast<float>(std::sqrt(dt)));
                const Vector3 acc_white = gaussian_.next();
                const Vector3 acc_measure = add(add(truth.specific_force, scale(acc_white, cfg_.acc.random_walk / root_dt)), acc_bias_);
                const Vector3 acc_drift = gaussian_.next();
                acc_bias_ = add(acc_bias_, scale(acc_drift, (cfg_.acc.bias_stability / std::sqrt(cfg_.acc.tau)) * root_dt));

                const Vector3 gyro_white = gaussian_.next();
                const Vector3 gyro_measure = add(add(truth.angular_velocity, scale(gyro_white, cfg_.gyro.random_walk / root_dt)), gyro_bias_);
                const Vector3 gyro_drift = gaussian_.next();
                gyro_bias_ = add(gyro_bias_, scale(gyro_drift, (cfg_.gyro.bias_stability / std::sqrt(cfg_.gyro.tau)) * root_dt));

                last_.gyro = {true, truth.sim_time_ns, gyro_measure, 0};
                last_.accel = {true, truth.sim_time_ns, acc_measure, 0};
            } else {
                last_.gyro.updated = false;
                last_.accel.updated = false;
            }
            out = last_;
            next_time_ns_ += setup_.physics_step_ns;
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

    ImuAdapterCapabilities capabilities() const override
    {
        ImuAdapterCapabilities c{};
        c.noise = true;
        c.bias = true;
        c.drift = true;
        c.calibration_values = true;  // startup accelerometer offset; scale=1
        c.deterministic_with_seed = true;
        c.thread_safe_across_instances = true;
        return c;
    }

    bool calibration(ImuCalibration& out) const override
    {
        out.accel_zero_g_offset = cfg_.acc.turn_on_bias;
        out.accel_scale = {1, 1, 1};
        return true;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override
    {
        values = {acc_bias_.x, acc_bias_.y, acc_bias_.z, gyro_bias_.x, gyro_bias_.y, gyro_bias_.z};
    }
private:
    void reset_state()
    {
        gaussian_.reset(setup_);
        next_time_ns_ = 0;
        last_ = {};
        acc_bias_ = cfg_.acc.turn_on_bias;
        gyro_bias_ = cfg_.gyro.turn_on_bias;
    }
    ImuAdapterSetup setup_;
    Config cfg_;
    std::uint64_t next_time_ns_{0};
    ImuSample last_{};
    Vector3 acc_bias_{}, gyro_bias_{};
    Gaussian3 gaussian_{};
    std::vector<LogColumn> columns_;
};
}  // namespace

std::unique_ptr<IImuSensorAdapter> create_projectairsim_imu_adapter(
    const ImuAdapterSetup& setup, std::string& error)
{
    try {
        auto result = std::make_unique<ProjectAirSimImuAdapter>(setup, load_config(setup));
        error.clear();
        return result;
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}
}  // namespace hakoniwa::sensor_adapter::airsim
