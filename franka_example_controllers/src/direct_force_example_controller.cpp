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

#include <franka_example_controllers/direct_force_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
DirectForceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
DirectForceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // position/velocity/effort for 7 joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  // Real Franka only. Gazebo typically does NOT export these semantic interfaces.
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/" + k_robot_model_interface_name);
    config.names.push_back(arm_id_ + "/" + k_robot_state_interface_name);
  }
  return config;
}

CallbackReturn DirectForceExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("ee_link", "");

    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("speed_factor", 0.2);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>(
        "start_joint_configuration",
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});

    auto_declare<int>("force_axis", 0);
    auto_declare<double>("force_desired", 10.0);
    auto_declare<double>("kp", 0.5);
    auto_declare<double>("ki", 0.0);
    auto_declare<double>("kd", 0.0);
    auto_declare<double>("i_limit", 50.0);
    auto_declare<double>("max_command", 50.0);
    auto_declare<double>("command_sign", -1.0);
    auto_declare<double>("lambda", 1e-3);

    auto_declare<std::vector<double>>("joint_damping",
                                      {2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 0.5});

    // publishing
    auto_declare<std::string>("wrench_frame_id", "world");
    auto_declare<int>("pub_decimation", 1);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DirectForceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  speed_factor_ = get_node()->get_parameter("speed_factor").as_double();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto start_q = get_node()->get_parameter("start_joint_configuration").as_double_array();

  if (k_gains.empty() || d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains / d_gains must be set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<size_t>(num_joints) ||
      d_gains.size() != static_cast<size_t>(num_joints) ||
      start_q.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "k_gains/d_gains/start_joint_configuration must have size %d", num_joints);
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    k_gains_(i) = k_gains[i];
    d_gains_(i) = d_gains[i];
    q_goal_(i) = start_q[i];
  }
  dq_filtered_.setZero();

  force_axis_ = get_node()->get_parameter("force_axis").as_int();
  force_desired_ = get_node()->get_parameter("force_desired").as_double();
  kp_ = get_node()->get_parameter("kp").as_double();
  ki_ = get_node()->get_parameter("ki").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();
  i_limit_ = get_node()->get_parameter("i_limit").as_double();
  max_command_ = get_node()->get_parameter("max_command").as_double();
  command_sign_ = get_node()->get_parameter("command_sign").as_double();
  lambda_ = get_node()->get_parameter("lambda").as_double();

  auto jd = get_node()->get_parameter("joint_damping").as_double_array();
  if (jd.size() == static_cast<size_t>(num_joints)) {
    for (int i = 0; i < num_joints; ++i) {
      joint_damping_(i) = jd[i];
    }
  }

  if (force_axis_ < 0 || force_axis_ > 5) {
    RCLCPP_FATAL(get_node()->get_logger(), "force_axis must be in [0,5]");
    return CallbackReturn::FAILURE;
  }

  // publishers
  wrench_frame_id_ = get_node()->get_parameter("wrench_frame_id").as_string();
  pub_decimation_ = get_node()->get_parameter("pub_decimation").as_int();
  if (pub_decimation_ < 1) pub_decimation_ = 1;
  pub_count_ = 0;

  // pub_wrench_est_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
  //     "~/estimated_wrench", rclcpp::SensorDataQoS());
  // pub_wrench_cmd_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
  //     "~/commanded_wrench", rclcpp::SensorDataQoS());

  pub_wrench_est_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/estimated_wrench", 10);
  pub_wrench_cmd_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/commanded_wrench", 10);

  if (is_gazebo_) {
    // In Gazebo: use URDF + Pinocchio
    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
    parameters_client->wait_for_service();

    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (result.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Failed to get robot_description from robot_state_publisher.");
      return CallbackReturn::FAILURE;
    }
    const std::string robot_description = result[0].as_string();

    const std::string parsed_name =
        robot_utils::getRobotNameFromDescription(robot_description, get_node()->get_logger());
    if (!parsed_name.empty() && parsed_name != arm_id_) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "arm_id param='%s' but robot_description name='%s' -> using '%s'",
                  arm_id_.c_str(), parsed_name.c_str(), parsed_name.c_str());
      arm_id_ = parsed_name;
    }

    try {
      pinocchio::urdf::buildModelFromXML(robot_description, pin_model_);
      pin_data_ = pinocchio::Data(pin_model_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_node()->get_logger(), "Pinocchio URDF parse/build failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }

    // Map 7 joints to Pinocchio indices
    for (int i = 0; i < num_joints; ++i) {
      const std::string jname = arm_id_ + "_joint" + std::to_string(i + 1);
      const auto jid = pin_model_.getJointId(jname);
      if (jid == 0) {
        RCLCPP_FATAL(get_node()->get_logger(), "Pinocchio: joint '%s' not found in URDF", jname.c_str());
        return CallbackReturn::FAILURE;
      }
      pin_joint_ids_[i] = jid;
      pin_v_idx_[i] = pin_model_.joints[jid].idx_v();
    }

    // End-effector frame
    ee_link_name_ = get_node()->get_parameter("ee_link").as_string();
    if (ee_link_name_.empty()) {
      ee_link_name_ = arm_id_ + "_hand_tcp";
    }
    ee_frame_id_ = pin_model_.getFrameId(ee_link_name_);
    if (ee_frame_id_ >= pin_model_.frames.size()) {
      const std::string fallback = arm_id_ + "_link8";
      RCLCPP_WARN(get_node()->get_logger(),
                  "Pinocchio: frame '%s' not found, fallback to '%s'",
                  ee_link_name_.c_str(), fallback.c_str());
      ee_link_name_ = fallback;
      ee_frame_id_ = pin_model_.getFrameId(ee_link_name_);
      if (ee_frame_id_ >= pin_model_.frames.size()) {
        RCLCPP_FATAL(get_node()->get_logger(),
                     "Pinocchio: neither '%s' nor '%s' frame exists in URDF",
                     ee_link_name_.c_str(), fallback.c_str());
        return CallbackReturn::FAILURE;
      }
    }
  } else {
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                     arm_id_ + "/" + k_robot_state_interface_name));
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured DirectForceExampleController: arm_id=%s, axis=%d, Fd=%.3f, kp=%.3f ki=%.3f kd=%.3f",
              arm_id_.c_str(), force_axis_, force_desired_, kp_, ki_, kd_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Wrench topics: %s, %s (frame_id=%s, decimation=%d)",
              (get_node()->get_name() + std::string("/estimated_wrench")).c_str(),
              (get_node()->get_name() + std::string("/commanded_wrench")).c_str(),
              wrench_frame_id_.c_str(), pub_decimation_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DirectForceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!is_gazebo_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }

  updateJointStates();
  motion_generator_ = std::make_unique<MotionGenerator>(speed_factor_, q_, q_goal_);
  start_time_ = this->get_node()->now();
  motion_finished_ = false;

  e_prev_ = 0.0;
  i_state_ = 0.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn DirectForceExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!is_gazebo_ && franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void DirectForceExampleController::updateJointStates() {
  // Requested: position, velocity, effort -> 3 per joint
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    assert(effort_interface.get_interface_name() == "effort");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
    tau_meas_(i) = effort_interface.get_value();
  }
}

controller_interface::return_type DirectForceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  const double dt = period.seconds();
  updateJointStates();

  // ---------------- Phase 1: move-to-start ----------------
  if (!motion_finished_) {
    auto trajectory_time = this->get_node()->now() - start_time_;
    auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
    Vector7d q_desired = motion_generator_output.first;
    motion_finished_ = motion_generator_output.second;

    if (!motion_finished_) {
      const double kAlpha = 0.99;
      dq_filtered_ = (1.0 - kAlpha) * dq_filtered_ + kAlpha * dq_;
      Vector7d tau_cmd =
          k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
      for (int i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(tau_cmd(i));
      }
      return controller_interface::return_type::OK;
    }

    e_prev_ = 0.0;
    i_state_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Move-to-start finished, entering force PID mode.");
  }

  // ---------------- Phase 2: 1-axis force PID ----------------
  Matrix67d J = Matrix67d::Zero();

  if (is_gazebo_) {
    // Pinocchio Jacobian at ee_link_name_ in LOCAL_WORLD_ALIGNED frame
    Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(pin_model_.nq);
    for (int i = 0; i < num_joints; ++i) {
      const auto jid = pin_joint_ids_[i];
      const int idx_q = static_cast<int>(pin_model_.joints[jid].idx_q());
      if (idx_q >= 0 && idx_q < q_pin.size()) {
        q_pin(idx_q) = q_(i);
      }
    }

    pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);

    Eigen::Matrix<double, 6, Eigen::Dynamic> J_full(6, pin_model_.nv);
    J_full.setZero();
    pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_pin, ee_frame_id_,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < J_full.cols()) {
        J.col(i) = J_full.col(idx_v);
      }
    }
  } else {
    // Jacobian from Franka semantic model
    const std::array<double, 42> jac_array =
        franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<const Matrix67d> J_map(jac_array.data());
    J = J_map;
  }

  // DLS force estimate: F_hat = (J J^T + lambda^2 I)^(-1) J tau
  const Matrix6d A = (J * J.transpose()) + (lambda_ * lambda_) * Matrix6d::Identity();
  const Vector6d b = J * tau_meas_;
  const Vector6d F_hat = A.ldlt().solve(b);

  const double f_meas = F_hat(force_axis_);
  const double e = force_desired_ - f_meas;

  i_state_ += e * dt;
  if (i_state_ > i_limit_) i_state_ = i_limit_;
  if (i_state_ < -i_limit_) i_state_ = -i_limit_;

  const double de = (dt > 1e-9) ? (e - e_prev_) / dt : 0.0;
  e_prev_ = e;

  const double u = kp_ * e + ki_ * i_state_ + kd_ * de;

  Vector6d F_cmd = Vector6d::Zero();
  double f_cmd_axis = command_sign_ * (force_desired_ + u);
  if (f_cmd_axis > max_command_) f_cmd_axis = max_command_;
  if (f_cmd_axis < -max_command_) f_cmd_axis = -max_command_;
  F_cmd(force_axis_) = f_cmd_axis;

  // tau_cmd = J^T F_cmd + damping(-dq)
  const Vector7d tau_cmd = J.transpose() * F_cmd + joint_damping_.cwiseProduct(-dq_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  // publish wrench (decimated)
  pub_count_++;
  if ((pub_count_ % pub_decimation_) == 0) {
    geometry_msgs::msg::WrenchStamped msg_est;
    msg_est.header.stamp = get_node()->now();
    msg_est.header.frame_id = wrench_frame_id_;
    msg_est.wrench.force.x = F_hat(0);
    msg_est.wrench.force.y = F_hat(1);
    msg_est.wrench.force.z = F_hat(2);
    msg_est.wrench.torque.x = F_hat(3);
    msg_est.wrench.torque.y = F_hat(4);
    msg_est.wrench.torque.z = F_hat(5);
    pub_wrench_est_->publish(msg_est);

    geometry_msgs::msg::WrenchStamped msg_cmd;
    msg_cmd.header = msg_est.header;
    msg_cmd.wrench.force.x = F_cmd(0);
    msg_cmd.wrench.force.y = F_cmd(1);
    msg_cmd.wrench.force.z = F_cmd(2);
    msg_cmd.wrench.torque.x = F_cmd(3);
    msg_cmd.wrench.torque.y = F_cmd(4);
    msg_cmd.wrench.torque.z = F_cmd(5);
    pub_wrench_cmd_->publish(msg_cmd);
  }

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 200,
                       "axis=%d  f_meas=%.3f  f_des=%.3f  e=%.3f  u=%.3f  f_cmd=%.3f",
                       force_axis_, f_meas, force_desired_, e, u, f_cmd_axis);

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DirectForceExampleController,
                       controller_interface::ControllerInterface)
