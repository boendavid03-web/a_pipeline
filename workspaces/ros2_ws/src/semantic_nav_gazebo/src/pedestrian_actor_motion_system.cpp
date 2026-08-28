#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/gazebo/Actor.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace semantic_nav_gazebo
{
namespace
{
struct PoseCommand
{
  std::string name;
  ignition::math::Pose3d pose;
};

double YawFromQuaternion(const ignition::msgs::Quaternion &_orientation)
{
  const double sin_yaw = 2.0 * (
    _orientation.w() * _orientation.z() +
    _orientation.x() * _orientation.y());
  const double cos_yaw = 1.0 - 2.0 * (
    _orientation.y() * _orientation.y() +
    _orientation.z() * _orientation.z());
  return std::atan2(sin_yaw, cos_yaw);
}
}  // namespace

class PedestrianActorMotionSystem final : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &) override
  {
    const auto *world_name =
      _ecm.Component<ignition::gazebo::components::Name>(_entity);
    if (world_name == nullptr)
    {
      ignerr << "PedestrianActorMotionSystem requires a named world entity"
             << std::endl;
      return;
    }

    this->service_name_ = "/world/" + world_name->Data() +
      "/pedestrian_actor_motion/set_pose_vector";
    this->pose_topic_ = "/world/" + world_name->Data() +
      "/pedestrian_actor_motion/pose_cmd";
    this->status_service_name_ = "/world/" + world_name->Data() +
      "/pedestrian_actor_motion/status";
    if (!this->node_.Advertise(
        this->service_name_,
        &PedestrianActorMotionSystem::OnPoseVector,
        this))
    {
      ignerr << "Failed to advertise actor motion service ["
             << this->service_name_ << "]" << std::endl;
      return;
    }
    if (!this->node_.Subscribe(
        this->pose_topic_, &PedestrianActorMotionSystem::OnPoseTopic, this))
    {
      ignerr << "Failed to subscribe to actor motion topic ["
             << this->pose_topic_ << "]" << std::endl;
    }
    if (!this->node_.Advertise(
        this->status_service_name_,
        &PedestrianActorMotionSystem::OnStatus,
        this))
    {
      ignerr << "Failed to advertise actor motion status service ["
             << this->status_service_name_ << "]" << std::endl;
    }

    igndbg << "Pedestrian actor motion service ready on ["
           << this->service_name_ << "]" << std::endl;
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::vector<PoseCommand> commands;
    {
      std::lock_guard<std::mutex> lock(this->commands_mutex_);
      commands.swap(this->pending_commands_);
    }

    for (const auto &command : commands)
    {
      const auto entity = _ecm.EntityByComponents(
        ignition::gazebo::components::Name(command.name));
      if (entity == ignition::gazebo::kNullEntity)
        continue;

      ignition::gazebo::Actor actor(entity);
      if (actor.Valid(_ecm))
      {
        this->UpdateActor(actor, command, _ecm);
        continue;
      }

      ignition::gazebo::Model proxy(entity);
      if (proxy.Valid(_ecm))
        proxy.SetWorldPoseCmd(_ecm, command.pose);
    }
  }

private:
  void OnPoseTopic(const ignition::msgs::Pose_V &_request)
  {
    ignition::msgs::Boolean response;
    this->OnPoseVector(_request, response);
  }

  bool OnPoseVector(
    const ignition::msgs::Pose_V &_request,
    ignition::msgs::Boolean &_response)
  {
    std::vector<PoseCommand> commands;
    commands.reserve(_request.pose_size());
    for (const auto &pose : _request.pose())
    {
      if (pose.name().empty())
        continue;

      commands.push_back({
        pose.name(),
        ignition::math::Pose3d(
          pose.position().x(), pose.position().y(), pose.position().z(),
          0.0, 0.0, YawFromQuaternion(pose.orientation()))});
    }

    {
      std::lock_guard<std::mutex> lock(this->commands_mutex_);
      this->pending_commands_ = std::move(commands);
    }
    _response.set_data(true);
    return true;
  }

  bool OnStatus(
    const ignition::msgs::Empty &,
    ignition::msgs::StringMsg_V &_response)
  {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    for (const auto &[name, seconds] : this->animation_seconds_)
    {
      const auto position = this->last_positions_.at(name);
      const auto world_position = this->actor_world_positions_.find(name);
      _response.add_data(
        name + " animation_seconds=" + std::to_string(seconds) +
        " x=" + std::to_string(position.X()) +
        " y=" + std::to_string(position.Y()) +
        (world_position == this->actor_world_positions_.end() ? "" :
          " actor_world_x=" + std::to_string(world_position->second.X()) +
          " actor_world_y=" + std::to_string(world_position->second.Y()) +
          " actor_world_z=" + std::to_string(world_position->second.Z())));
    }
    return true;
  }

  void UpdateActor(
    ignition::gazebo::Actor &_actor,
    const PoseCommand &_command,
    ignition::gazebo::EntityComponentManager &_ecm)
  {
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    const auto last = this->last_positions_.find(_command.name);
    if (last == this->last_positions_.end())
    {
      this->last_positions_.emplace(_command.name, _command.pose.Pos());
    }
    else
    {
      const double distance = last->second.Distance(_command.pose.Pos());
      if (distance > this->movement_threshold_)
      {
        this->animation_seconds_[ _command.name ] +=
          distance * this->animation_seconds_per_meter_;
        last->second = _command.pose.Pos();
      }
    }

    _actor.SetTrajectoryPose(_ecm, _command.pose);
    _actor.SetAnimationName(_ecm, "walk");
    const auto animation_time = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(this->animation_seconds_[ _command.name ]));
    _actor.SetAnimationTime(_ecm, animation_time);
    const auto world_pose = _actor.WorldPose(_ecm);
    if (world_pose)
      this->actor_world_positions_[_command.name] = world_pose->Pos();
  }

  ignition::transport::Node node_;
  std::string service_name_;
  std::string pose_topic_;
  std::string status_service_name_;
  std::mutex commands_mutex_;
  std::mutex state_mutex_;
  std::vector<PoseCommand> pending_commands_;
  std::unordered_map<std::string, ignition::math::Vector3d> last_positions_;
  std::unordered_map<std::string, ignition::math::Vector3d> actor_world_positions_;
  std::unordered_map<std::string, double> animation_seconds_;
  const double movement_threshold_{1e-4};
  const double animation_seconds_per_meter_{1.35};
};
}  // namespace semantic_nav_gazebo

IGNITION_ADD_PLUGIN(
  semantic_nav_gazebo::PedestrianActorMotionSystem,
  ignition::gazebo::System,
  semantic_nav_gazebo::PedestrianActorMotionSystem::ISystemConfigure,
  semantic_nav_gazebo::PedestrianActorMotionSystem::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  semantic_nav_gazebo::PedestrianActorMotionSystem,
  "semantic_nav_gazebo::PedestrianActorMotionSystem")
