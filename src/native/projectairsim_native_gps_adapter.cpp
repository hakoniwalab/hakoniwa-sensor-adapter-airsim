// GPS adapter running Project AirSim's own core_sim Gps, compiled unchanged from the files pinned in
// third_party/projectairsim/lock.json.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_gps_adapter.hpp"

#include "core_sim/sensors/gps.hpp"
#include "native_common.hpp"

namespace hakoniwa::sensor_adapter::airsim {
namespace {

using namespace native;

// core_sim's Gps reports the geodetic position of its Environment, which core_sim maps from the local
// NED position and the home point (EarthUtils::NedToGeodetic). The adapter puts the home at the
// setup's origin and the Environment at the truth's local position; GpsTruth::position is not used.
// core_sim's GPS message carries latitude, longitude and altitude as 32-bit floats.
class NativeGpsAdapter : public IGpsSensorAdapter {
public:
    ~NativeGpsAdapter() override
    {
        if (gps_) gps_->EndUpdate();
    }

    bool initialize(const GpsAdapterSetup& setup, std::string& error)
    {
        try {
            nlohmann::json json = read_config(setup.config_path);
            const double every_step_hz = 1e9 / static_cast<double>(setup.physics_step_ns ? setup.physics_step_ns : 1);
            clock_.set(setup.physics_step_ns, period_ns(take_rate(json, every_step_hz), setup.physics_step_ns));
            set_sensor_keys(json, "gps", "gps");
            config_ = json;
            origin_ = setup.origin;
            create();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        return true;
    }

    bool step(const GpsTruth& truth, GpsSample& out, std::string& error) override
    {
        bool sample = false;
        if (!clock_.advance(truth.sim_time_ns, sample, error)) return false;
        if (sample) {
            environment_.SetPosition(to_pas_axes(truth.position_nwu));
            kinematics_.twist.linear = to_pas_axes(truth.velocity_nwu);
            gps_->Update(static_cast<TimeNano>(truth.sim_time_ns), static_cast<TimeNano>(clock_.period_ns()));
            const auto data = gps_->getOutput().getData();
            last_ = {};
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.position = {data["latitude"].get<float>(), data["longitude"].get<float>(),
                              data["altitude"].get<float>()};
            const auto& v = data["velocity"];
            last_.velocity_nwu = from_pas_axes(pas::Vector3(v["x"].get<float>(), v["y"].get<float>(), v["z"].get<float>()));
            last_.fix_type = data["fix_type"].get<std::uint32_t>();
            // EPH and EPV are dilutions of precision (no unit), not metres; they are logged only.
            eph_ = data["eph"].get<float>();
            epv_ = data["epv"].get<float>();
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        try {
            gps_->EndUpdate();
            create();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        clock_.reset();
        last_ = {};
        eph_ = epv_ = 0.0;
        return true;
    }

    GpsAdapterCapabilities capabilities() const override
    {
        GpsAdapterCapabilities caps{};
        // core_sim's Gps passes the position and velocity through; it has no noise.
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values = {eph_, epv_}; }

private:
    void create()
    {
        environment_ = pas::Environment(
            pas::Vector3::Zero(),
            pas::HomeGeoPoint(pas::GeoPoint(origin_.latitude_deg, origin_.longitude_deg, origin_.altitude_m)));
        kinematics_ = pas::Kinematics();
        gps_ = pas::Robot::create<pas::Gps>("gps", managers_.logger, managers_.topics, managers_.services,
                                            managers_.states);
        pas::Robot::load(*gps_, config_);
        pas::Robot::initialize(*gps_, kinematics_, environment_);
        gps_->BeginUpdate();
    }

    Managers managers_;
    nlohmann::json config_;
    GeodeticPosition origin_{};
    pas::Kinematics kinematics_;
    pas::Environment environment_;
    std::unique_ptr<pas::Gps> gps_;
    SampleClock clock_;
    GpsSample last_{};
    double eph_{0.0}, epv_{0.0};
    std::vector<LogColumn> columns_{{"eph", "dimensionless"}, {"epv", "dimensionless"}};
};

}  // namespace

std::unique_ptr<IGpsSensorAdapter> create_projectairsim_native_gps_adapter(const GpsAdapterSetup& setup,
                                                                           std::string& error)
{
    auto adapter = std::make_unique<NativeGpsAdapter>();
    if (!adapter->initialize(setup, error)) return nullptr;
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::airsim
