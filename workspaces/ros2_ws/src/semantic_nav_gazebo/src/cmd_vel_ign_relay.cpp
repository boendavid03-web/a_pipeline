#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <ignition/msgs/twist.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

class CmdVelIgnRelay : public rclcpp::Node
{
public:
  CmdVelIgnRelay() : Node("cmd_vel_ign_relay")
  {
    const auto ros_topic =
      this->declare_parameter<std::string>("ros_topic", "/cmd_vel");
    const auto ign_topic =
      this->declare_parameter<std::string>("ign_topic", "/cmd_vel");
    this->angular_z_scale_ =
      this->declare_parameter<double>("angular_z_scale", 1.0);

    this->ign_pub_ = this->ign_node_.Advertise<ignition::msgs::Twist>(
      ign_topic);

    this->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      ros_topic,
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        ignition::msgs::Twist ign_msg;
        ign_msg.mutable_linear()->set_x(msg->linear.x);
        ign_msg.mutable_linear()->set_y(msg->linear.y);
        ign_msg.mutable_linear()->set_z(msg->linear.z);
        ign_msg.mutable_angular()->set_x(msg->angular.x);
        ign_msg.mutable_angular()->set_y(msg->angular.y);
        ign_msg.mutable_angular()->set_z(
          msg->angular.z * this->angular_z_scale_);
        this->ign_pub_.Publish(ign_msg);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Relaying ROS 2 [%s] to Ignition [%s] as ignition::msgs::Twist "
      "(angular_z_scale=%.3f)",
      ros_topic.c_str(),
      ign_topic.c_str(),
      this->angular_z_scale_);
  }

private:
  ignition::transport::Node ign_node_;
  ignition::transport::Node::Publisher ign_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  double angular_z_scale_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelIgnRelay>());
  rclcpp::shutdown();
  return 0;
}
