// IMU adapter running Project AirSim's own core_sim Imu, compiled unchanged from the files pinned in
// third_party/projectairsim/lock.json. Only the host side (truth in, sample out, timing) is written here.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_imu_adapter.hpp"

#include "core_sim/sensors/imu.hpp"
#include "native_common.hpp"

namespace hakoniwa::sensor_adapter::airsim {
namespace {

using namespace native;

class NativeImuAdapter : public IImuSensorAdapter {
public:
    ~NativeImuAdapter() override
    {
        if (imu_) imu_->EndUpdate();
    }

    bool initialize(const ImuAdapterSetup& setup, std::string& error)
    {
        try {
            nlohmann::json json = read_config(setup.config_path);
            // The adapter's own key: how often the scene would update the sensor (default every
            // physics step). core_sim's Imu measures on every Update call and uses the given dt.
            const double every_step_hz = 1e9 / static_cast<double>(setup.physics_step_ns ? setup.physics_step_ns : 1);
            clock_.set(setup.physics_step_ns, period_ns(take_rate(json, every_step_hz), setup.physics_step_ns));
            set_sensor_keys(json, "imu", "imu");
            config_ = json;
            create_imu();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        return true;
    }

    bool step(const ImuTruth& truth, ImuSample& out, std::string& error) override
    {
        bool sample = false;
        if (!clock_.advance(truth.sim_time_ns, sample, error)) return false;
        if (sample) {
            // The truth is already in the sensor axes, so the body orientation is the identity and the
            // linear acceleration is the specific force plus core_sim's own gravity (its Imu subtracts it).
            kinematics_.pose.orientation = pas::Quaternion::Identity();
            kinematics_.twist.angular = to_pas_axes(truth.angular_velocity);
            kinematics_.accels.linear = to_pas_axes(truth.specific_force) + environment_.env_info.gravity;
            imu_->Update(static_cast<TimeNano>(truth.sim_time_ns), static_cast<TimeNano>(clock_.period_ns()));
            const pas::ImuMessage message = imu_->getOutput();
            last_.gyro = {true, truth.sim_time_ns, from_pas_axes(message.GetAngularVelocity()), 0};
            last_.accel = {true, truth.sim_time_ns, from_pas_axes(message.GetLinearAcceleration()), 0};
        } else {
            last_.gyro.updated = false;
            last_.accel.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        // core_sim's Imu has no reset: a new one starts from its fixed random seeds and turn-on bias.
        try {
            imu_->EndUpdate();
            create_imu();
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
        clock_.reset();
        last_ = {};
        return true;
    }

    ImuAdapterCapabilities capabilities() const override
    {
        ImuAdapterCapabilities caps{};
        caps.noise = true;
        caps.bias = true;   // turn-on bias (a fixed vector)
        caps.drift = true;  // bias random walk (bias-stability)
        // core_sim seeds its generators with fixed values (1, 2, 3): every instance gives the same
        // sequence whatever the host's seed, and instances share no state.
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    bool calibration(ImuCalibration&) const override { return false; }
    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    void create_imu()
    {
        imu_ = pas::Robot::create<pas::Imu>("imu", managers_.logger, managers_.topics, managers_.services,
                                            managers_.states);
        pas::Robot::load(*imu_, config_);
        // Origin at sea level: core_sim's gravity is the standard 9.80665 below 10 km.
        environment_ = pas::Environment(pas::Vector3::Zero(), pas::HomeGeoPoint(pas::GeoPoint(0.0, 0.0, 0.0)));
        kinematics_ = pas::Kinematics();
        pas::Robot::initialize(*imu_, kinematics_, environment_);
        imu_->BeginUpdate();
    }

    Managers managers_;
    nlohmann::json config_;
    pas::Kinematics kinematics_;
    pas::Environment environment_;
    std::unique_ptr<pas::Imu> imu_;
    SampleClock clock_;
    ImuSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IImuSensorAdapter> create_projectairsim_native_imu_adapter(const ImuAdapterSetup& setup,
                                                                           std::string& error)
{
    auto adapter = std::make_unique<NativeImuAdapter>();
    if (!adapter->initialize(setup, error)) return nullptr;
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::airsim
