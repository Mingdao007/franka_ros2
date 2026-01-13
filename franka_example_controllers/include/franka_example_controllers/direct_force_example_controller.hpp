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

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include "franka_example_controllers/motion_generator.hpp"
#include "franka_example_controllers/robot_utils.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * DirectForceExampleController (simple version)
 *
 * Phase 1: Move-to-start using MotionGenerator + joint PD
 * Phase 2: 1-axis force PID
 *   - Force estimated from joint torque via damped least squares:
 *     F_hat = (J*J^T + lambda^2 I)^(-1) * J * tau
 *   - Wrench command only has one component (axis 0..5), tau_cmd = J^T * F_cmd
 *
 * Command interface: 7x effort
 * State interfaces : 7x position, 7x velocity, 7x effort
 *   - Real Franka: semantic interfaces <arm_id>/robot_model & <arm_id>/robot_state available
 *   - Gazebo: typically NOT available -> use URDF + Pinocchio to compute Jacobian
 *
 * Publishes:
 *   - ~/estimated_wrench (geometry_msgs/WrenchStamped): F_hat (6D)
 *   - ~/commanded_wrench (geometry_msgs/WrenchStamped): F_cmd (6D)
 */
class DirectForceExampleController : public controller_interface::ControllerInterface {
 public:
  static constexpr int num_joints = 7;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix67d = Eigen::Matrix<double, 6, 7>;

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
  void updateJointStates();

  // -------- Move-to-start --------
  std::string arm_id_{"fr3"};
  bool is_gazebo_{false};

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d tau_meas_{Vector7d::Zero()};

  Vector7d q_goal_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};
  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};
  double speed_factor_{0.2};

  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time start_time_;
  bool motion_finished_{false};

  // -------- Force PID --------
  int force_axis_{0};          // 0..5 => Fx,Fy,Fz,Mx,My,Mz
  double force_desired_{10.0}; // target in that axis
  double kp_{0.5};
  double ki_{0.0};
  double kd_{0.0};
  double i_limit_{50.0};
  double max_command_{50.0};
  double command_sign_{-1.0};
  double lambda_{1e-3};

  double e_prev_{0.0};
  double i_state_{0.0};

  Vector7d joint_damping_{(Vector7d() << 2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 0.5).finished()};

  // -------- Real robot semantic model --------
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  // -------- Gazebo Jacobian path (Pinocchio from URDF) --------
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_{pin_model_};
  std::array<pinocchio::JointIndex, num_joints> pin_joint_ids_{};
  std::array<int, num_joints> pin_v_idx_{};
  pinocchio::FrameIndex ee_frame_id_{0};
  std::string ee_link_name_{};

  // -------- Publishers for monitoring wrench --------
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_est_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_cmd_;
  std::string wrench_frame_id_{"world"};
  int pub_decimation_{1};
  int pub_count_{0};
};

}  // namespace franka_example_controllers
