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

#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_single_arm_controllers/linear_ramp_up_controller.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_single_arm_controllers {

/**
 * @brief A controller that allows to control the robot's end-effector velocity in Cartesian space.
 *
 * This controller subscribes to a geometry_msgs::msg::Twist message and uses the received
 * linear and angular velocities to control the robot's end-effector velocity in Cartesian space.
 * The controller uses a linear ramp-up controller to smoothly ramp up the control signal to the desired
 * velocity and limits the rate of change of the control signal to ensure smooth transitions.
 */
class CartesianVelocityController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private: 
  void target_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_velocity_sub_;
  Eigen::Vector3d desired_linear_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d desired_angular_velocity_{0.0, 0.0, 0.0};

  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;

  const bool k_elbow_activated_{false};
  const int num_joints = 7;
  std::string arm_id_;

  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);

  std::unique_ptr<LinearRampUpController> ramp_up_controller_linear_;
  std::unique_ptr<LinearRampUpController> ramp_up_controller_angular_;
  const double max_step_size_linear_{0.00020}; 
  const double max_step_size_angular_{0.00050}; 
};

} // namespace franka_single_arm_controllers

