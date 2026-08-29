/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Template Controller for nav2py
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2py_drl_vo_controller/drl_vo_controller.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace nav2py_drl_vo_controller
{

  void DRL_VO_Controller::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Set transform tolerance parameter
    double transform_tolerance;
    nav2_util::declare_parameter_if_not_declared(
        node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

    // Initialize nav2py
    std::string nav2py_script = ament_index_cpp::get_package_share_directory("nav2py_drl_vo_controller") + "/../../lib/nav2py_drl_vo_controller/nav2py_run";
    nav2py_bootstrap(nav2py_script + " --host 127.0.0.1" + " --port 0");

    // Create publisher for global plan
    global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);

    nav2py::utils::Costmap costmap(
        std::string(node->get_namespace()) + "/local_costmap/local_costmap",
        node->get_namespace());

    auto laserscan_observation = costmap.findObservationByType("LaserScan");

    if (laserscan_observation.has_value())
    {
      std::string topic = laserscan_observation.value().topic();
      RCLCPP_INFO(
          logger_,
          "Laser scan topic: %s",
          topic.c_str());
      scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
          topic,
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
          std::bind(&DRL_VO_Controller::sendScanData, this, std::placeholders::_1)
        );

    }
    else
    {
      RCLCPP_WARN(
          logger_,
          "no laser scan found");
    }

    RCLCPP_INFO(
        logger_,
        "Configured controller: %s of type nav2py_drl_vo_controller::DRL_VO_Controller",
        plugin_name_.c_str());
  }

  void DRL_VO_Controller::sendOdomData(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity)
  {
    std::stringstream ss;

    ss << "pose: ";
    ss << geometry_msgs::msg::to_yaml(pose, true);
    ss << "\n";
    ss << "velocity: ";
    ss << geometry_msgs::msg::to_yaml(velocity, true);
    ss << "\n";

    nav2py_send("odom", {ss.str()});
  }

  void DRL_VO_Controller::sendScanData(
      const sensor_msgs::msg::LaserScan &scan)
  {
    nav2py_send("scan", {sensor_msgs::msg::to_yaml(scan, true)});
  }

  void DRL_VO_Controller::cleanup()
  {
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s",
        plugin_name_.c_str());
    nav2py_cleanup();
    global_pub_.reset();
  }

  void DRL_VO_Controller::activate()
  {
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s",
        plugin_name_.c_str());
    global_pub_->on_activate();
  }

  void DRL_VO_Controller::deactivate()
  {
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s",
        plugin_name_.c_str());
    global_pub_->on_deactivate();
  }

  void DRL_VO_Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
  {
    nav2py_send("speed_limit", {std::to_string(speed_limit), percentage ? "true" : "false"});
  }

  geometry_msgs::msg::TwistStamped DRL_VO_Controller::computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker)
  {
    (void)goal_checker;

    // Simple implementation that just sends data to Python and waits for response
    try
    {
      sendOdomData(pose, velocity);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          logger_,
          "Error sending data: %s", e.what());
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    try
    {
      RCLCPP_INFO(logger_, "Waiting for velocity command from Python...");
      cmd_vel.twist = wait_for_cmd_vel();

      RCLCPP_INFO(
          logger_,
          "Received velocity command: linear_x=%.2f, angular_z=%.2f",
          cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          logger_,
          "Error receiving velocity command: %s", e.what());

      // Default to stop if there's an error
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
    }

    return cmd_vel;
  }

  void DRL_VO_Controller::setPlan(const nav_msgs::msg::Path &path)
  {
    global_plan_ = path;
    global_pub_->publish(path);

    try
    {
      nav2py_send("path", {nav_msgs::msg::to_yaml(path, true)});
      RCLCPP_INFO(logger_, "Sent path data to Python controller");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          logger_,
          "Error sending path: %s", e.what());
    }
  }

} // namespace nav2py_drl_vo_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2py_drl_vo_controller::DRL_VO_Controller, nav2_core::Controller)