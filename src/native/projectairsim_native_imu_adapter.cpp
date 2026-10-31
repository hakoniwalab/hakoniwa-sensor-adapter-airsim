// IMU adapter running Project AirSim's own core_sim Imu, compiled unchanged from the files pinned in
// third_party/projectairsim/lock.json. Only the host side (truth in, sample out, timing) is written here.
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_imu_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "core_sim/config_json.hpp"
#include "core_sim/environment.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/physics_common_types.hpp"
#include "core_sim/sensors/imu.hpp"
#include "core_sim/service_manager.hpp"
#include "json.hpp"
#include "state_manager.hpp"
#include "topic_manager.hpp"

namespace microsoft::projectairsim {

// core_sim's sensors have private constructors and name Robot as a friend; core_sim's own unit tests
// define this helper the same way to create a sensor without a scene. This translation unit does not
// include core_sim's real Robot.
class Robot {
 public:
  static std::unique_ptr<Imu> create_imu(const Logger& logger, const TopicManager& topics,
                                         const ServiceManager& services, const StateManager& states)
  {
    return std::unique_ptr<Imu>(new Imu("imu", true, "body", logger, topics, "/hakoniwa", services, states));
  }
  static void load(Imu& imu, const nlohmann::json& config) { imu.Load(ConfigJson(config)); }
  static void initialize(Imu& imu, const Kinematics& kinematics, const Environment& environment)
  {
    imu.Initialize(kinematics, environment);
  }
};

}  // namespace microsoft::projectairsim

namespace hakoniwa::sensor_adapter::airsim {
namespace {

namespace pas = microsoft::projectairsim;

// The interface's sensor axes are FLU; core_sim's body axes are FRD: y and z change sign both ways.
pas::Vector3 flu_to_frd(const Vector3& v)
{
    return pas::Vector3(static_cast<float>(v.x), static_cast<float>(-v.y), static_cast<float>(-v.z));
}

Vector3 frd_to_flu(const pas::Vector3& v)
{
    return {static_cast<double>(v.x()), -static_cast<double>(v.y()), -static_cast<double>(v.z())};
}

class NativeImuAdapter : public IImuSensorAdapter {
public:
    NativeImuAdapter()
        : logger_([](const std::string&, pas::LogLevel, const std::string&) {}),
          topics_(logger_), services_(logger_), states_(logger_)
    {
    }

    ~NativeImuAdapter() override
    {
        if (imu_) imu_->EndUpdate();
    }

    bool initialize(const ImuAdapterSetup& setup, std::string& error)
    {
        try {
            setup_ = setup;
            if (setup.physics_step_ns == 0) throw std::runtime_error("physics_step_ns must be positive");
            std::ifstream in(setup.config_path);
            if (!in) throw std::runtime_error("cannot open " + setup.config_path);
            nlohmann::json json = nlohmann::json::parse(in);
            // The adapter's own key: how often the scene would update the sensor. core_sim's Imu measures
            // on every Update call and uses the given dt.
            period_ns_ = setup.physics_step_ns;
            if (json.contains("update-rate-hz")) {
                const double hz = json["update-rate-hz"].get<double>();
                if (!(hz > 0)) throw std::runtime_error("update-rate-hz must be positive");
                const double period = 1e9 / hz;
                const auto steps = static_cast<std::uint64_t>(period / setup.physics_step_ns + 0.5);
                if (steps == 0 || std::abs(static_cast<double>(steps * setup.physics_step_ns) - period) > 1.0)
                    throw std::runtime_error("sensor period must be an integer multiple of physics_step_ns");
                period_ns_ = steps * setup.physics_step_ns;
                json.erase("update-rate-hz");
            }
            json["id"] = "imu";
            json["type"] = "imu";
            json["enabled"] = true;
            json["parent-link"] = "body";
            if (!json.contains("origin")) json["origin"] = nlohmann::json::object();
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
        if (truth.sim_time_ns != next_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_ns_ += setup_.physics_step_ns;
        if (truth.sim_time_ns % period_ns_ == 0) {
            // The truth is already in the sensor axes, so the body orientation is the identity and the
            // linear acceleration is the specific force plus core_sim's own gravity (its Imu subtracts it).
            kinematics_.pose.orientation = pas::Quaternion::Identity();
            kinematics_.twist.angular = flu_to_frd(truth.angular_velocity);
            kinematics_.accels.linear = flu_to_frd(truth.specific_force) + environment_.env_info.gravity;
            imu_->Update(static_cast<TimeNano>(truth.sim_time_ns), static_cast<TimeNano>(period_ns_));
            const pas::ImuMessage message = imu_->getOutput();
            last_.gyro = {true, truth.sim_time_ns, frd_to_flu(message.GetAngularVelocity()), 0};
            last_.accel = {true, truth.sim_time_ns, frd_to_flu(message.GetLinearAcceleration()), 0};
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
        next_ns_ = 0;
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
        imu_ = pas::Robot::create_imu(logger_, topics_, services_, states_);
        pas::Robot::load(*imu_, config_);
        // Origin at sea level: core_sim's gravity is the standard 9.80665 below 10 km.
        environment_ = pas::Environment(pas::Vector3::Zero(), pas::HomeGeoPoint(pas::GeoPoint(0.0, 0.0, 0.0)));
        kinematics_ = pas::Kinematics();
        pas::Robot::initialize(*imu_, kinematics_, environment_);
        imu_->BeginUpdate();
    }

    ImuAdapterSetup setup_{};
    pas::Logger logger_;
    pas::TopicManager topics_;
    pas::ServiceManager services_;
    pas::StateManager states_;
    nlohmann::json config_;
    pas::Kinematics kinematics_;
    pas::Environment environment_;
    std::unique_ptr<pas::Imu> imu_;
    std::uint64_t period_ns_{1};
    std::uint64_t next_ns_{0};
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
