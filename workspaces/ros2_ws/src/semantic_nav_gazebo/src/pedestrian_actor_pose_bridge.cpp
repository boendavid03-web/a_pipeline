#include <cstddef>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ignition/msgs/pose_v.pb.h>
#include <ignition/transport/Node.hh>

namespace
{
class PedestrianActorPoseBridge final : public rclcpp::Node
{
public:
  PedestrianActorPoseBridge()
  : Node("pedestrian_actor_pose_bridge")
  {
    const auto world_name = this->declare_parameter<std::string>(
      "world_name", "default");
    this->pose_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/pedestrian_actor_pose_commands");
    this->actor_z_offset_ = this->declare_parameter<double>(
      "actor_z_offset", 1.0);
    this->use_actors_ = this->declare_parameter<bool>("use_actors", true);
    const auto ign_topic = "/world/" + world_name +
      "/pedestrian_actor_motion/pose_cmd";
    this->publisher_ = this->ign_node_.Advertise<ignition::msgs::Pose_V>(
      ign_topic);
    if (!this->publisher_)
      throw std::runtime_error("Unable to advertise Ignition actor pose topic");

    this->subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      this->pose_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message)
      {
        ignition::msgs::Pose_V pose_vector;
        for (std::size_t index = 0; index < message->poses.size(); ++index)
        {
          const auto pedestrian_index = index / 2 + 1;
          const bool is_primary = index % 2 == 0;
          const auto &source = message->poses[index];
          auto *target = pose_vector.add_pose();
          const auto base_name =
            "pedestrian_" + std::to_string(pedestrian_index);
          if (this->use_actors_)
            target->set_name(
              base_name + (is_primary ? "_actor" : "_collision_proxy"));
          else
            target->set_name(
              base_name + (is_primary ? "" : "_collision_proxy"));
          target->mutable_position()->set_x(source.position.x);
          target->mutable_position()->set_y(source.position.y);
          // The walking DAE's feet are 1 m below its actor origin (matching
          // Gazebo Fortress's actor example).  Keep the LiDAR proxy on the
          // ground and lift only the visible skeletal Actor.
          target->mutable_position()->set_z(
            source.position.z +
            (this->use_actors_ && is_primary ? this->actor_z_offset_ : 0.0));
          target->mutable_orientation()->set_x(source.orientation.x);
          target->mutable_orientation()->set_y(source.orientation.y);
          target->mutable_orientation()->set_z(source.orientation.z);
          target->mutable_orientation()->set_w(source.orientation.w);
        }
        this->publisher_.Publish(pose_vector);
      });

    RCLCPP_INFO(
      this->get_logger(), "Forwarding %s to Gazebo actor motion in world %s",
      this->pose_topic_.c_str(), world_name.c_str());
  }

private:
  std::string pose_topic_;
  double actor_z_offset_;
  bool use_actors_;
  ignition::transport::Node ign_node_;
  ignition::transport::Node::Publisher publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PedestrianActorPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
