// Barometer adapter running Project AirSim's own core_sim Barometer, compiled unchanged from the files
// pinned in third_party/projectairsim/lock.json.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_baro_adapter.hpp"

#include "core_sim/sensors/barometer.hpp"
#include "native_common.hpp"

namespace hakoniwa::sensor_adapter::airsim {
namespace {

using namespace native;

// core_sim's Barometer reads the pressure of its Environment, which core_sim computes from the
// position (its standard atmosphere). The adapter places the Environment at the truth's altitude
// above a sea-level origin; the sea-level weather of the truth is not used, as in core_sim.
class NativeBaroAdapter : public IBaroSensorAdapter {
public:
    ~NativeBaroAdapter() override
    {
        if (baro_) baro_->EndUpdate();
    }

    bool initialize(const BaroAdapterSetup& setup, std::string& error)
    {
        try {
            nlohmann::json json = read_config(setup.config_path);
            // core_sim parses these names with underscores; the hyphenated spellings found in some
            // sample robot configs would be ignored silently.
            for (const char* alias : {"pressure-factor-sigma", "uncorrelated-noise-sigma"})
                if (json.contains(alias))
                    throw std::runtime_error(std::string(alias) + " is not a core_sim key (core_sim reads it with underscores)");
            // core_sim's Barometer parses these but its Update does not apply them.
            for (const char* key : {"update-latency", "startup-delay"})
                if (json.contains(key) && json[key].get<double>() != 0.0)
                    throw std::runtime_error(std::string(key) + " is not applied by core_sim's Barometer");
            set_sensor_keys(json, "baro", "barometer");
            config_ = json;
            create();
            // The sample rate is core_sim's own "update-frequency". core_sim declares
            // GetBarometerSettings() but does not define it: the value is read as core_sim reads it.
            const float hz = json.contains("update-frequency") ? json["update-frequency"].get<float>()
                                                               : pas::BarometerParams{}.update_frequency;
            clock_.set(setup.physics_step_ns, period_ns(hz, setup.physics_step_ns));
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        return true;
    }

    bool step(const BaroTruth& truth, BaroSample& out, std::string& error) override
    {
        bool sample = false;
        if (!clock_.advance(truth.sim_time_ns, sample, error)) return false;
        if (sample) {
            environment_.SetPosition(pas::Vector3(0.0f, 0.0f, static_cast<float>(-truth.altitude_m)));
            baro_->Update(static_cast<TimeNano>(truth.sim_time_ns), static_cast<TimeNano>(clock_.period_ns()));
            const auto data = baro_->getOutput().getData();
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.pressure_pa = data["pressure"].get<float>();
            last_.pressure_altitude_m = data["altitude"].get<float>();
            last_.temperature_c = 0.0;  // core_sim's barometer message has no temperature
            last_.flags = 0;
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        try {
            baro_->EndUpdate();
            create();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        clock_.reset();
        last_ = {};
        return true;
    }

    BaroAdapterCapabilities capabilities() const override
    {
        BaroAdapterCapabilities caps{};
        caps.pressure_altitude = true;
        caps.noise = true;
        caps.offset = true;  // qnh
        caps.drift = true;   // pressure factor (Gauss-Markov)
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    void create()
    {
        environment_ = pas::Environment(pas::Vector3::Zero(), pas::HomeGeoPoint(pas::GeoPoint(0.0, 0.0, 0.0)));
        kinematics_ = pas::Kinematics();
        baro_ = pas::Robot::create<pas::Barometer>("baro", managers_.logger, managers_.topics, managers_.services,
                                                   managers_.states);
        pas::Robot::load(*baro_, config_);
        pas::Robot::initialize(*baro_, kinematics_, environment_);
        baro_->BeginUpdate();
    }

    Managers managers_;
    nlohmann::json config_;
    pas::Kinematics kinematics_;
    pas::Environment environment_;
    std::unique_ptr<pas::Barometer> baro_;
    SampleClock clock_;
    BaroSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IBaroSensorAdapter> create_projectairsim_native_baro_adapter(const BaroAdapterSetup& setup,
                                                                             std::string& error)
{
    auto adapter = std::make_unique<NativeBaroAdapter>();
    if (!adapter->initialize(setup, error)) return nullptr;
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::airsim
