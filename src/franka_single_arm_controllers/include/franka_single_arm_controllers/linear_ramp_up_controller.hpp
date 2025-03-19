// Copyright (c) 2025 Franka Robotics GmbH
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

#include <Eigen/Dense>

namespace franka_single_arm_controllers {

/**
 * @class LinearRampUpController
 * @brief A controller that smoothly ramps up the control signal to a desired velocity.
 *
 * This class implements a linear ramp-up controller that filters the desired velocity
 * and limits the rate of change of the control signal to ensure smooth transitions.
 *
 * @param max_step_size The maximum allowable change in the control signal per step.
 * @param k_alpha The smoothing factor for the exponential moving average filter.
 *                A value closer to 1 gives more weight to the desired velocity,
 *                while a value closer to 0 gives more weight to the previous filtered velocity.
 */
class LinearRampUpController {

  public:
   LinearRampUpController(double max_step_size, double k_alpha=0.1) 
       : max_step_size_(max_step_size), k_alpha_(k_alpha), last_control_signal_(Eigen::Vector3d::Zero()), filtered_velocity_(Eigen::Vector3d::Zero()) {}
 
   Eigen::Vector3d computeControlSignal(const Eigen::Vector3d& desired_velocity) {
     filtered_velocity_ = (1 - k_alpha_) * filtered_velocity_ + k_alpha_ * desired_velocity;
 
     Eigen::Vector3d step_increment = filtered_velocity_ - last_control_signal_;
 
     for (int i = 0; i < step_increment.size(); ++i) {
       if (step_increment[i] > max_step_size_) {
         step_increment[i] = max_step_size_;
       } else if (step_increment[i] < -max_step_size_) {
         step_increment[i] = -max_step_size_;
       }
     }
 
     last_control_signal_ += step_increment;
     return last_control_signal_;
   }
 
  private:
   double max_step_size_;
   double k_alpha_;
   Eigen::Vector3d last_control_signal_;
   Eigen::Vector3d filtered_velocity_;
 };

} // namespace franka_single_arm_controllers