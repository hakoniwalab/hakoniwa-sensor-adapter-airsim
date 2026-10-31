// Link stand-ins for core_sim functions that the managers (TopicManager, ServiceManager, StateManager)
// reference but a standalone sensor never calls: robot and scene state sync, rotors, the simulation clock
// and client authorization. They are not part of Project AirSim. Each aborts if it is ever called.
#include <cstdio>
#include <cstdlib>

#include "core_sim/actor.hpp"
#include "core_sim/actor/robot.hpp"
#include "core_sim/actuators/actuator.hpp"
#include "core_sim/actuators/rotor.hpp"
#include "core_sim/client_authorization.hpp"
#include "core_sim/clock.hpp"
#include "core_sim/scene.hpp"

namespace microsoft::projectairsim {
[[noreturn]] static void not_in_standalone(const char* what) {
  std::fprintf(stderr, "core_sim standalone: %s is not available\n", what);
  std::abort();
}
const std::string& Actor::GetID() const { not_in_standalone("Actor::GetID"); }
ActorType Actor::GetType() const { not_in_standalone("Actor::GetType"); }
ActuatorType Actuator::GetType() const { not_in_standalone("Actuator::GetType"); }
ClientAuthorization::ClientAuthorization() {}
bool ClientAuthorization::IsAuthorized(void) const { return true; }
bool ClientAuthorization::IsAuthorizedMethod(const std::string&) const { return true; }
uint64_t ClientAuthorization::SetToken(const char*, size_t) { not_in_standalone("ClientAuthorization::SetToken"); }
const std::vector<std::reference_wrapper<Actuator>>& Robot::GetActuators() const { not_in_standalone("Robot::GetActuators"); }
const Kinematics& Robot::GetKinematics() const { not_in_standalone("Robot::GetKinematics"); }
const std::vector<std::reference_wrapper<Sensor>>& Robot::GetSensors() const { not_in_standalone("Robot::GetSensors"); }
void Robot::SetActuatedRotations(const ActuatedRotations&, TimeNano) { not_in_standalone("Robot::SetActuatedRotations"); }
void Robot::SetPhysicsType(const PhysicsType&) { not_in_standalone("Robot::SetPhysicsType"); }
void Robot::UpdateKinematics(const Kinematics&, TimeNano) { not_in_standalone("Robot::UpdateKinematics"); }
const ActuatedRotations& Rotor::GetActuatedRotations() const { not_in_standalone("Rotor::GetActuatedRotations"); }
const std::vector<std::reference_wrapper<Actor>>& Scene::GetActors() const { not_in_standalone("Scene::GetActors"); }
const std::string& Scene::GetID() const { not_in_standalone("Scene::GetID"); }
ClockBase* SimClock::Get(std::shared_ptr<ClockBase>) { not_in_standalone("SimClock::Get"); }
}  // namespace microsoft::projectairsim
