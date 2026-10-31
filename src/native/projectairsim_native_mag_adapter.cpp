// Magnetometer adapter running Project AirSim's own core_sim Magnetometer, compiled unchanged from the
// files pinned in third_party/projectairsim/lock.json.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_mag_adapter.hpp"

#include "core_sim/sensors/magnetometer.hpp"
#include "native_common.hpp"

namespace hakoniwa::sensor_adapter::airsim {
namespace {

using namespace native;

constexpr double kNanoteslaPerGauss = 100000.0;

// core_sim's Magnetometer computes the field itself (a dipole model at the environment's position,
// rotated by the body orientation) and then adds its noise and startup bias. The interface gives the
// field. The adapter runs two core_sim Magnetometers on the same position and orientation: the
// configured one and a copy without noise and bias. Their difference is exactly what core_sim adds,
// and it is added to the host's field (times core_sim's scale factor).
class NativeMagAdapter : public IMagSensorAdapter {
public:
    ~NativeMagAdapter() override { end(); }

    bool initialize(const MagAdapterSetup& setup, std::string& error)
    {
        try {
            nlohmann::json json = read_config(setup.config_path);
            const double every_step_hz = 1e9 / static_cast<double>(setup.physics_step_ns ? setup.physics_step_ns : 1);
            clock_.set(setup.physics_step_ns, period_ns(take_rate(json, every_step_hz), setup.physics_step_ns));
            // The host applies the mount pose; a rotation here would be applied a second time.
            if (json.contains("origin") && !json["origin"].empty())
                throw std::runtime_error("origin is not supported: the host gives the field in the sensor axes");
            set_sensor_keys(json, "mag", "magnetometer");
            // core_sim declares GetMagnetometerSettings() but does not define it: the scale factor is
            // read as core_sim reads it (a float, core_sim's default when absent).
            scale_ = json.contains("scale-factor") ? json["scale-factor"].get<float>()
                                                   : pas::MagnetometerParams{}.scale_factor;
            config_ = json;
            clean_config_ = json;
            clean_config_["noise-sigma"] = "0 0 0";
            clean_config_["noise-bias"] = "0 0 0";
            create();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        return true;
    }

    bool step(const MagTruth& truth, MagSample& out, std::string& error) override
    {
        bool sample = false;
        if (!clock_.advance(truth.sim_time_ns, sample, error)) return false;
        if (sample) {
            const auto t = static_cast<TimeNano>(truth.sim_time_ns);
            const auto dt = static_cast<TimeNano>(clock_.period_ns());
            mag_->Update(t, dt);
            clean_->Update(t, dt);
            const pas::Vector3 added = body_field(*mag_) - body_field(*clean_);  // Gauss, FRD
            const Vector3 noise_nt = from_pas_axes(added);
            last_ = {true, truth.sim_time_ns,
                     {scale_ * truth.magnetic_field.x + noise_nt.x * kNanoteslaPerGauss,
                      scale_ * truth.magnetic_field.y + noise_nt.y * kNanoteslaPerGauss,
                      scale_ * truth.magnetic_field.z + noise_nt.z * kNanoteslaPerGauss},
                     0};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        // A new Magnetometer draws its startup bias again from its fixed seeds.
        try {
            end();
            create();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        clock_.reset();
        last_ = {};
        return true;
    }

    MagAdapterCapabilities capabilities() const override
    {
        MagAdapterCapabilities caps{};
        caps.noise = true;
        caps.hard_iron = true;  // the startup bias, a fixed vector per instance
        // core_sim seeds its generators with fixed values: every instance gives the same sequence
        // whatever the host's seed, and instances share no state.
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    static pas::Vector3 body_field(const pas::Magnetometer& mag)
    {
        const auto field = mag.getOutput().getData()["magnetic_field_body"];
        return pas::Vector3(field["x"].get<float>(), field["y"].get<float>(), field["z"].get<float>());
    }

    void create()
    {
        environment_ = pas::Environment(pas::Vector3::Zero(), pas::HomeGeoPoint(pas::GeoPoint(0.0, 0.0, 0.0)));
        kinematics_ = pas::Kinematics();
        kinematics_.pose.orientation = pas::Quaternion::Identity();
        mag_ = make("mag", config_);
        clean_ = make("mag-clean", clean_config_);
    }
    std::unique_ptr<pas::Magnetometer> make(const char* id, nlohmann::json config)
    {
        config["id"] = id;
        auto mag = pas::Robot::create<pas::Magnetometer>(id, managers_.logger, managers_.topics,
                                                         managers_.services, managers_.states);
        pas::Robot::load(*mag, config);
        pas::Robot::initialize(*mag, kinematics_, environment_);
        mag->BeginUpdate();
        return mag;
    }
    void end()
    {
        if (mag_) mag_->EndUpdate();
        if (clean_) clean_->EndUpdate();
    }

    Managers managers_;
    nlohmann::json config_, clean_config_;
    double scale_{1.0};
    pas::Kinematics kinematics_;
    pas::Environment environment_;
    std::unique_ptr<pas::Magnetometer> mag_, clean_;
    SampleClock clock_;
    MagSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IMagSensorAdapter> create_projectairsim_native_mag_adapter(const MagAdapterSetup& setup,
                                                                           std::string& error)
{
    auto adapter = std::make_unique<NativeMagAdapter>();
    if (!adapter->initialize(setup, error)) return nullptr;
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::airsim
