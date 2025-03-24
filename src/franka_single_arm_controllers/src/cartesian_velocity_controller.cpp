// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_single_arm_controllers/cartesian_velocity_controller.hpp>
#include <franka_single_arm_controllers/default_robot_behavior_utils.hpp>
#include <franka_single_arm_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_single_arm_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type CartesianVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  auto control_signal_linear = ramp_up_controller_linear_->computeControlSignal(desired_linear_velocity_);
  auto control_signal_angular = ramp_up_controller_angular_->computeControlSignal(desired_angular_velocity_);

  if (franka_cartesian_velocity_->setCommand(control_signal_linear, control_signal_angular)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianVelocityController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  ramp_up_controller_linear_ = std::make_unique<LinearRampUpController>(max_step_size_linear_);
  ramp_up_controller_angular_ = std::make_unique<LinearRampUpController>(max_step_size_angular_);

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  target_velocity_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/franka_controller/target_cartesian_velocity", 10, std::bind(&CartesianVelocityController::target_velocity_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to franka_controller/target_cartesian_velocity.");


  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianVelocityController::target_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  const double v_max_linear = 0.10; 
  const double v_max_angular = 0.065; 
  
  desired_linear_velocity_ = v_max_linear * Eigen::Vector3d(msg->linear.x, msg->linear.y, msg->linear.z);
  desired_angular_velocity_ = v_max_angular * Eigen::Vector3d(msg->angular.x, msg->angular.y, msg->angular.z);

  RCLCPP_DEBUG(get_node()->get_logger(), "Received target velocity input: Linear [%f, %f, %f], Angular [%f, %f, %f]",
               desired_linear_velocity_.x(), desired_linear_velocity_.y(), desired_linear_velocity_.z(),
               desired_angular_velocity_.x(), desired_angular_velocity_.y(), desired_angular_velocity_.z());
}


}  // namespace franka_single_arm_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_single_arm_controllers::CartesianVelocityController,
                       controller_interface::ControllerInterface)



                       
