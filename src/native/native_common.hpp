// Host side shared by the native adapters: creating a core_sim sensor without a scene, the
// adapter's sample period, and the time rule of the interface. Nothing here models a sensor.
#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "core_sim/config_json.hpp"
#include "core_sim/environment.hpp"
#include "core_sim/logger.hpp"
#include "core_sim/physics_common_types.hpp"
#include "core_sim/service_manager.hpp"
#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"
#include "json.hpp"
#include "state_manager.hpp"
#include "topic_manager.hpp"

namespace microsoft::projectairsim {

// core_sim's sensors have private constructors and name Robot as a friend; core_sim's own unit tests
// define this helper the same way to create a sensor without a scene. The native adapters do not
// include core_sim's real Robot.
class Robot {
 public:
  template <typename SensorT>
  static std::unique_ptr<SensorT> create(const std::string& id, const Logger& logger, const TopicManager& topics,
                                         const ServiceManager& services, const StateManager& states)
  {
    return std::unique_ptr<SensorT>(new SensorT(id, true, "body", logger, topics, "/hakoniwa", services, states));
  }
  template <typename SensorT>
  static void load(SensorT& sensor, const nlohmann::json& config)
  {
    sensor.Load(ConfigJson(config));
  }
  template <typename SensorT>
  static void initialize(SensorT& sensor, const Kinematics& kinematics, const Environment& environment)
  {
    sensor.Initialize(kinematics, environment);
  }
};

}  // namespace microsoft::projectairsim

namespace hakoniwa::sensor_adapter::airsim::native {

namespace pas = microsoft::projectairsim;

// The interface's axes are NWU (world) and FLU (sensor); core_sim's are NED and FRD. Both turns
// change the sign of y and z.
inline pas::Vector3 to_pas_axes(const Vector3& v)
{
    return pas::Vector3(static_cast<float>(v.x), static_cast<float>(-v.y), static_cast<float>(-v.z));
}
inline Vector3 from_pas_axes(const pas::Vector3& v)
{
    return {static_cast<double>(v.x()), -static_cast<double>(v.y()), -static_cast<double>(v.z())};
}

// Managers a sensor registers its topics with. Nothing subscribes; the adapter reads getOutput().
struct Managers {
    Managers()
        : logger([](const std::string&, pas::LogLevel, const std::string&) {}), topics(logger), services(logger),
          states(logger)
    {
    }
    pas::Logger logger;
    pas::TopicManager topics;
    pas::ServiceManager services;
    pas::StateManager states;
};

inline nlohmann::json read_config(const std::string& path)
{
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    return nlohmann::json::parse(in);
}

// The sample period: rate_hz turned into a whole number of physics steps (otherwise an error).
inline std::uint64_t period_ns(double rate_hz, std::uint64_t physics_step_ns)
{
    if (physics_step_ns == 0) throw std::runtime_error("physics_step_ns must be positive");
    if (!(rate_hz > 0)) throw std::runtime_error("the sensor rate must be positive");
    const double period = 1e9 / rate_hz;
    const auto steps = static_cast<std::uint64_t>(period / physics_step_ns + 0.5);
    if (steps == 0 || std::abs(static_cast<double>(steps * physics_step_ns) - period) > 1.0)
        throw std::runtime_error("sensor period must be an integer multiple of physics_step_ns");
    return steps * physics_step_ns;
}

// Takes the adapter's own "update-rate-hz" out of a core_sim sensor config and sets the keys
// core_sim needs. Returns the rate, or fallback_hz when the key is absent.
inline double take_rate(nlohmann::json& json, double fallback_hz)
{
    double hz = fallback_hz;
    if (json.contains("update-rate-hz")) {
        hz = json["update-rate-hz"].get<double>();
        json.erase("update-rate-hz");
    }
    return hz;
}
inline void set_sensor_keys(nlohmann::json& json, const char* id, const char* type)
{
    json["id"] = id;
    json["type"] = type;
    json["enabled"] = true;
    json["parent-link"] = "body";
    if (!json.contains("origin")) json["origin"] = nlohmann::json::object();
}

// The interface's time rule: sim_time_ns = 0, dt, 2dt, ... ; true at a sample boundary.
class SampleClock {
public:
    void set(std::uint64_t step_ns, std::uint64_t period_ns)
    {
        step_ns_ = step_ns;
        period_ns_ = period_ns;
        next_ns_ = 0;
    }
    bool advance(std::uint64_t sim_time_ns, bool& sample, std::string& error)
    {
        if (sim_time_ns != next_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_ns_ += step_ns_;
        sample = sim_time_ns % period_ns_ == 0;
        return true;
    }
    void reset() { next_ns_ = 0; }
    std::uint64_t period_ns() const { return period_ns_; }

private:
    std::uint64_t step_ns_{1};
    std::uint64_t period_ns_{1};
    std::uint64_t next_ns_{0};
};

}  // namespace hakoniwa::sensor_adapter::airsim::native
