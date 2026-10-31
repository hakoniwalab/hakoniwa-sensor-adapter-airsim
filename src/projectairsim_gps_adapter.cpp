// Independent compatibility implementation, not native core_sim::Gps.
// References: iamaisim/ProjectAirSim, core_sim/src/sensors/gps.cpp,
// core_sim/include/core_sim/sensors/gps.hpp, core_sim/include/core_sim/first_order_filter.hpp.
// Original Project AirSim sources MIT; this implementation (c) 2026 Hakoniwa Lab, LLC (MIT).
#include "hakoniwa/sensor_adapter/airsim/projectairsim_gps_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace hakoniwa::sensor_adapter::airsim {
namespace {

struct Config {
    float eph_time_constant{0.9f}, epv_time_constant{0.9f};
    float eph_initial{100.0f}, epv_initial{100.0f};
    float eph_final{0.1f}, epv_final{0.1f};
    float eph_min_3d{3.0f}, eph_min_2d{4.0f};
    std::uint64_t period_ns{};
};

Config load_config(const GpsAdapterSetup& setup)
{
    if (setup.physics_step_ns == 0)
        throw std::runtime_error("physics_step_ns must be positive");
    std::ifstream file(setup.config_path);
    if (!file) throw std::runtime_error("cannot open GPS config: " + setup.config_path);
    boost::property_tree::ptree root;
    boost::property_tree::read_json(file, root);
    if (root.get<std::string>("type", "gps") != "gps")
        throw std::runtime_error("GPS config type must be 'gps'");

    Config c{};
    c.eph_time_constant = root.get<float>("eph-time-constant", c.eph_time_constant);
    c.epv_time_constant = root.get<float>("epv-time-constant", c.epv_time_constant);
    c.eph_initial = root.get<float>("eph-initial", c.eph_initial);
    c.epv_initial = root.get<float>("epv-initial", c.epv_initial);
    c.eph_final = root.get<float>("eph-final", c.eph_final);
    c.epv_final = root.get<float>("epv-final", c.epv_final);
    c.eph_min_3d = root.get<float>("eph-min-3d", c.eph_min_3d);
    c.eph_min_2d = root.get<float>("eph-min-2d", c.eph_min_2d);
    for (float v : {c.eph_time_constant,c.epv_time_constant,c.eph_initial,c.epv_initial,
                    c.eph_final,c.epv_final,c.eph_min_3d,c.eph_min_2d}) {
        if (!std::isfinite(v) || v < 0)
            throw std::runtime_error("GPS parameters must be finite and nonnegative");
    }
    if (c.eph_min_3d > c.eph_min_2d)
        throw std::runtime_error("eph-min-3d cannot exceed eph-min-2d");

    // Adapter-only timing: native Scene scheduler is deliberately not reproduced.
    const double hz = root.get<double>("update-rate-hz", 1e9 / static_cast<double>(setup.physics_step_ns));
    if (!std::isfinite(hz) || hz <= 0)
        throw std::runtime_error("update-rate-hz must be positive and finite");
    const double ns = 1e9 / hz;
    if (!std::isfinite(ns) || ns < static_cast<double>(setup.physics_step_ns) ||
        ns > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        throw std::runtime_error("GPS period must be >= physics_step_ns");
    c.period_ns = static_cast<std::uint64_t>(std::llround(ns));
    if (c.period_ns == 0 || c.period_ns % setup.physics_step_ns != 0 ||
        std::fabs(ns - static_cast<double>(c.period_ns)) > 0.001)
        throw std::runtime_error("GPS period must be an integer multiple of physics_step_ns");
    return c;
}

bool valid_truth(const GpsTruth& t)
{
    const auto& p = t.position;
    return std::isfinite(p.latitude_deg) && p.latitude_deg >= -90 && p.latitude_deg <= 90 &&
           std::isfinite(p.longitude_deg) && p.longitude_deg >= -180 && p.longitude_deg <= 180 &&
           std::isfinite(p.altitude_m) &&
           std::isfinite(t.velocity_nwu.x) && std::isfinite(t.velocity_nwu.y) &&
           std::isfinite(t.velocity_nwu.z) &&
           std::isfinite(t.position_nwu.x) && std::isfinite(t.position_nwu.y) &&
           std::isfinite(t.position_nwu.z);
}

// Mirrors Project AirSim FirstOrderFilter<float>, including a float state.
class FirstOrderFilter {
public:
    FirstOrderFilter(float tau, float initial, float target)
        : tau_(tau), initial_(initial), target_(target), value_(initial) {}
    void reset() { value_ = initial_; }
    void update(double dt) {
        if (dt <= 0) return;
        if (tau_ == 0) { value_ = target_; return; }
        const double alpha = std::exp(-dt / tau_);
        value_ = static_cast<float>(value_ * alpha + target_ * (1.0 - alpha));
    }
    float value() const { return value_; }
private:
    float tau_, initial_, target_, value_;
};

class ProjectAirSimGpsAdapter final : public IGpsSensorAdapter {
public:
    ProjectAirSimGpsAdapter(GpsAdapterSetup setup, Config cfg)
        : setup_(std::move(setup)), cfg_(cfg),
          eph_(cfg.eph_time_constant, cfg.eph_initial, cfg.eph_final),
          epv_(cfg.epv_time_constant, cfg.epv_initial, cfg.epv_final)
    {
        columns_ = {{"eph","dimensionless"},{"epv","dimensionless"}};
    }
    bool step(const GpsTruth& truth, GpsSample& out, std::string& error) override {
        try {
            if (truth.sim_time_ns != next_ns_)
                throw std::runtime_error("sim_time_ns must advance by physics_step_ns starting at zero");
            if (!valid_truth(truth))
                throw std::runtime_error("invalid GPS truth: coordinates or vector must be finite");
            if (next_ns_ > std::numeric_limits<std::uint64_t>::max() - setup_.physics_step_ns)
                throw std::runtime_error("simulation time overflow");

            if (truth.sim_time_ns % cfg_.period_ns == 0) {
                // Assumption for comparison: one GPS update per sample, dt=period.
                // The first sample at t=0 advances the filter once.
                const double dt = static_cast<double>(cfg_.period_ns) * 1e-9;
                eph_.update(dt);
                epv_.update(dt);
                GpsSample sample{};
                sample.updated = true;
                sample.sample_time_ns = truth.sim_time_ns;
                sample.position = truth.position;    // no native GPS position noise
                sample.velocity_nwu = truth.velocity_nwu; // no native velocity noise
                const float eph = eph_.value();
                sample.fix_type = eph <= cfg_.eph_min_3d ? 3u :
                                  eph <= cfg_.eph_min_2d ? 2u : 0u;
                // Do not invent metric accuracy: upstream EPH/EPV are dimensionless.
                last_ = sample;
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
    bool reset(std::string& error) override {
        next_ns_ = 0;
        last_ = {};
        eph_.reset();
        epv_.reset();
        error.clear();
        return true;
    }
    GpsAdapterCapabilities capabilities() const override {
        GpsAdapterCapabilities caps{};
        // Only fix_type is available. receiver_status ALSO requires reported
        // metric accuracy and satellite count, so false is mandatory.
        caps.receiver_status = false;
        caps.deterministic_with_seed = true; // native GPS has no RNG
        caps.thread_safe_across_instances = true;
        return caps;
    }
    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override {
        values = {static_cast<double>(eph_.value()), static_cast<double>(epv_.value())};
    }
private:
    GpsAdapterSetup setup_;
    Config cfg_;
    FirstOrderFilter eph_, epv_;
    std::uint64_t next_ns_{0};
    GpsSample last_{};
    std::vector<LogColumn> columns_;
};
}

std::unique_ptr<IGpsSensorAdapter> create_projectairsim_gps_adapter(
    const GpsAdapterSetup& setup, std::string& error)
{
    try {
        auto result = std::make_unique<ProjectAirSimGpsAdapter>(setup, load_config(setup));
        error.clear();
        return result;
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}
} // namespace hakoniwa::sensor_adapter::airsim
