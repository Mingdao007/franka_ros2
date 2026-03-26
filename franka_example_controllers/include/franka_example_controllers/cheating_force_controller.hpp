// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <array>
#include <fstream>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/// Oracle upper-bound controller for free-space XY tracking precision.
///
/// Uses Pinocchio model-based compensation (gravity, Coriolis, feedforward)
/// with high-gain Cartesian PD. No contact, no force control.
/// Each compensation term can be toggled independently for ablation.
class CheatingForceController : public controller_interface::ControllerInterface {
 public:
  static constexpr int num_joints = 7;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
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
  void update_joint_states();
  void update_pinocchio_model(Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin);
  Matrix67d compute_jacobian(const Eigen::VectorXd& q_pin);
  Vector3d compute_ee_position(const Eigen::VectorXd& q_pin);
  Vector7d saturate_torque_rate(const Vector7d& tau_d, const Vector7d& tau_d_last);

  // Robot identity.
  std::string arm_id_{"fr3"};
  std::string ee_link_name_{};

  // Joint states.
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d tau_meas_{Vector7d::Zero()};

  // Pinocchio model.
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_{pin_model_};
  Eigen::Matrix<double, 6, Eigen::Dynamic> j_full_;
  std::array<pinocchio::JointIndex, num_joints> pin_joint_ids_{};
  std::array<int, num_joints> pin_v_idx_{};
  pinocchio::FrameIndex ee_frame_id_{0};

  // Circle trajectory.
  double circle_radius_{0.03};
  double circle_frequency_{0.1};
  bool use_current_pose_as_center_{true};
  double circle_center_offset_x_{0.0};
  double circle_center_offset_y_{0.0};
  bool center_initialized_{false};
  double circle_center_x_{0.0};
  double circle_center_y_{0.0};
  double hold_z_{0.0};  // Hold Z at initial height (free-space).

  // Cartesian PD gains.
  double kp_xy_{6000.0};
  double kd_xy_{200.0};
  double kp_z_{6000.0};
  double kd_z_{200.0};

  // Compensation toggles (for ablation study).
  bool enable_gravity_comp_{true};
  bool enable_coriolis_comp_{false};
  bool enable_feedforward_accel_{false};

  // Null-space damping.
  Vector7d joint_damping_{(Vector7d() << 15.0, 15.0, 15.0, 15.0, 10.0, 8.0, 5.0).finished()};
  double lambda_{0.01};

  // Torque rate limit.
  double delta_tau_max_{1.0};

  // Timing.
  double elapsed_time_{0.0};
  double soft_start_duration_{2.0};
  Vector7d tau_cmd_prev_{Vector7d::Zero()};

  // Ink trail.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ink_meas_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ink_des_;
  std::vector<geometry_msgs::msg::Point> trail_meas_;
  std::vector<geometry_msgs::msg::Point> trail_des_;
  bool ink_enabled_{true};
  int trail_decimation_{50};
  double ink_line_width_{0.001};
  int ink_max_points_{5000};
  int pub_count_{0};

  // CSV logging.
  std::string log_file_path_{};
  std::ofstream log_file_;
};

}  // namespace franka_example_controllers
