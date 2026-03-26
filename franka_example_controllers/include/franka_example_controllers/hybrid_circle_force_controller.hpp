// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class HybridCircleForceController : public controller_interface::ControllerInterface {
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
  Matrix67d compute_jacobian();
  Vector7d compute_external_torque();
  Vector3d compute_ee_position(const Eigen::VectorXd& q_pin);
  Vector6d estimate_cartesian_force(const Matrix67d& J, const Vector7d& tau_ext);
  Vector7d saturate_torque_rate(const Vector7d& tau_d_calculated, const Vector7d& tau_d_last);

  // Interfaces and robot identity.
  std::string arm_id_{"fr3"};
  std::string ee_link_name_{};
  bool is_gazebo_{true};

  // Joint states.
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector7d tau_meas_{Vector7d::Zero()};

  // Pinocchio model objects.
  pinocchio::Model pin_model_;
  pinocchio::Data pin_data_{pin_model_};
  Eigen::Matrix<double, 6, Eigen::Dynamic> j_full_;
  std::array<pinocchio::JointIndex, num_joints> pin_joint_ids_{};
  std::array<int, num_joints> pin_v_idx_{};
  pinocchio::FrameIndex ee_frame_id_{0};

  // Circular XY trajectory settings.
  double circle_radius_{0.05};
  double circle_frequency_{0.2};
  double circle_center_offset_x_{0.0};
  double circle_center_offset_y_{0.0};
  bool use_current_pose_as_center_{true};

  // XY Cartesian position PID gains.
  double kp_xy_{250.0};
  double kd_xy_{35.0};
  double ki_xy_{0.0};
  double i_limit_xy_{0.5};
  double d_filter_alpha_xy_{0.5};

  // Z-axis force PID settings.
  double force_desired_{5.0};
  double force_kp_{0.03};
  double force_ki_{0.005};
  double force_kd_{0.006};
  double i_limit_{15.0};
  double max_force_command_{60.0};
  double command_sign_{-1.0};
  double force_filter_alpha_{0.8};
  double d_filter_alpha_{0.5};
  double lambda_{0.01};

  // Safety and damping.
  Vector7d joint_damping_{(Vector7d() << 15.0, 15.0, 15.0, 15.0, 10.0, 8.0, 5.0).finished()};
  static constexpr double k_delta_tau_max{1.0};

  // Force PID state (Z-axis).
  double elapsed_time_{0.0};
  double e_prev_{0.0};
  double i_state_{0.0};
  double f_meas_filtered_{0.0};
  double de_filtered_{0.0};

  // XY position PID state (circle phase).
  double ix_state_{0.0};
  double iy_state_{0.0};
  double ex_prev_{0.0};
  double ey_prev_{0.0};
  double dex_filtered_{0.0};
  double dey_filtered_{0.0};

  // Bias capture at activation.
  bool use_bias_calibration_{false};
  Vector7d tau_ext_initial_{Vector7d::Zero()};
  Vector7d tau_cmd_prev_{Vector7d::Zero()};

  // Phase state machine: descent → circle.
  enum class Phase { kDescent, kCircle };
  Phase phase_{Phase::kDescent};
  double descent_kp_xy_{4000.0};
  double descent_kd_xy_{160.0};
  double descent_kp_z_{1000.0};
  double descent_kd_z_{50.0};
  double descent_ki_xy_{0.0};
  double descent_ki_z_{0.0};
  double descent_speed_{0.02};           // m/s downward
  double descent_contact_force_{2.0};    // |Fz| threshold to transition
  double descent_start_z_{0.0};

  // Descent PID state.
  double ix_descent_{0.0};
  double iy_descent_{0.0};
  double iz_descent_{0.0};
  double ex_prev_descent_{0.0};
  double ey_prev_descent_{0.0};
  double ez_prev_descent_{0.0};

  // Circle center is initialized from first valid EE pose.
  bool center_initialized_{false};
  double circle_center_x_{0.0};
  double circle_center_y_{0.0};

  // FT sensor integration: switch between Jacobian estimation and direct sensor.
  bool use_ft_sensor_{false};
  std::string ft_sensor_topic_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sensor_sub_;
  geometry_msgs::msg::WrenchStamped latest_ft_msg_;
  std::mutex ft_mutex_;
  std::atomic<bool> ft_data_received_{false};

  // Monitoring publishers.
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_est_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_wrench_cmd_;
  std::string wrench_frame_id_{"world"};
  int pub_decimation_{1};
  int pub_count_{0};

  // Ink trail state.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ink_marker_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ink_desired_;
  std::vector<geometry_msgs::msg::Point> trail_points_;
  std::vector<geometry_msgs::msg::Point> trail_desired_points_;
  bool ink_enabled_{true};
  int trail_decimation_{50};  // Add a point every N controller updates.
  double ink_force_threshold_{1.0};
  double ink_line_width_{0.006};
  bool ink_project_to_surface_{true};
  double ink_surface_z_{0.4160};
  double ink_surface_epsilon_{0.0005};
  double ink_contact_offset_z_{-0.03};
  int ink_max_points_{5000};

  // Model-based compensation toggles (cheating terms).
  bool enable_coriolis_comp_{false};

  // Soft start: ramp radius from 0 to target over this duration (seconds).
  double soft_start_duration_{2.0};

  // Optional CSV logging.
  std::string log_file_path_{};
  std::ofstream log_file_;
};

}  // namespace franka_example_controllers
