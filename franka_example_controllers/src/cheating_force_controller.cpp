#include <franka_example_controllers/cheating_force_controller.hpp>

#include <cmath>
#include <exception>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "franka_example_controllers/robot_utils.hpp"

namespace franka_example_controllers {

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
CheatingForceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CheatingForceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_init
// ---------------------------------------------------------------------------

CallbackReturn CheatingForceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("ee_link", "");

    auto_declare<double>("circle_radius", 0.03);
    auto_declare<double>("circle_frequency", 0.1);
    auto_declare<double>("circle_center_offset_x", 0.0);
    auto_declare<double>("circle_center_offset_y", 0.0);
    auto_declare<bool>("use_current_pose_as_center", true);

    auto_declare<double>("kp_xy", 6000.0);
    auto_declare<double>("kd_xy", 200.0);
    auto_declare<double>("kp_z", 6000.0);
    auto_declare<double>("kd_z", 200.0);

    auto_declare<bool>("enable_gravity_comp", true);
    auto_declare<bool>("enable_coriolis_comp", false);
    auto_declare<bool>("enable_feedforward_accel", false);

    auto_declare<std::vector<double>>("joint_damping", {15.0, 15.0, 15.0, 15.0, 10.0, 8.0, 5.0});
    auto_declare<double>("lambda", 0.01);
    auto_declare<double>("delta_tau_max", 1.0);
    auto_declare<double>("soft_start_duration", 2.0);

    auto_declare<std::string>("log_file_path", "");
    auto_declare<bool>("ink_enabled", true);
    auto_declare<int>("ink_trail_decimation", 50);
    auto_declare<double>("ink_line_width", 0.001);
    auto_declare<int>("ink_max_points", 5000);
  } catch (const std::exception& e) {
    fprintf(stderr, "CheatingForceController init exception: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_configure
// ---------------------------------------------------------------------------

CallbackReturn CheatingForceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  ee_link_name_ = get_node()->get_parameter("ee_link").as_string();
  if (ee_link_name_.empty()) {
    ee_link_name_ = arm_id_ + "_hand_tcp";
  }

  circle_radius_ = get_node()->get_parameter("circle_radius").as_double();
  circle_frequency_ = get_node()->get_parameter("circle_frequency").as_double();
  circle_center_offset_x_ = get_node()->get_parameter("circle_center_offset_x").as_double();
  circle_center_offset_y_ = get_node()->get_parameter("circle_center_offset_y").as_double();
  use_current_pose_as_center_ = get_node()->get_parameter("use_current_pose_as_center").as_bool();

  kp_xy_ = get_node()->get_parameter("kp_xy").as_double();
  kd_xy_ = get_node()->get_parameter("kd_xy").as_double();
  kp_z_ = get_node()->get_parameter("kp_z").as_double();
  kd_z_ = get_node()->get_parameter("kd_z").as_double();

  enable_gravity_comp_ = get_node()->get_parameter("enable_gravity_comp").as_bool();
  enable_coriolis_comp_ = get_node()->get_parameter("enable_coriolis_comp").as_bool();
  enable_feedforward_accel_ = get_node()->get_parameter("enable_feedforward_accel").as_bool();

  lambda_ = get_node()->get_parameter("lambda").as_double();
  delta_tau_max_ = get_node()->get_parameter("delta_tau_max").as_double();
  soft_start_duration_ = get_node()->get_parameter("soft_start_duration").as_double();

  auto jd = get_node()->get_parameter("joint_damping").as_double_array();
  if (jd.size() == static_cast<size_t>(num_joints)) {
    for (int i = 0; i < num_joints; ++i) joint_damping_(i) = jd[i];
  }

  log_file_path_ = get_node()->get_parameter("log_file_path").as_string();

  ink_enabled_ = get_node()->get_parameter("ink_enabled").as_bool();
  trail_decimation_ = get_node()->get_parameter("ink_trail_decimation").as_int();
  ink_line_width_ = get_node()->get_parameter("ink_line_width").as_double();
  ink_max_points_ = get_node()->get_parameter("ink_max_points").as_int();
  if (trail_decimation_ < 1) trail_decimation_ = 1;
  if (ink_max_points_ < 2) ink_max_points_ = 2;

  if (ink_enabled_) {
    pub_ink_meas_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
        "~/ink_trail", rclcpp::QoS(1).transient_local());
    pub_ink_des_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
        "~/ink_desired", rclcpp::QoS(1).transient_local());
  }

  // Load robot description from robot_state_publisher.
  auto params_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  params_client->wait_for_service();
  auto future = params_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (result.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to get robot_description");
    return CallbackReturn::FAILURE;
  }
  const std::string robot_description = result[0].as_string();

  const std::string parsed_name =
      robot_utils::getRobotNameFromDescription(robot_description, get_node()->get_logger());
  if (!parsed_name.empty() && parsed_name != arm_id_) {
    RCLCPP_WARN(get_node()->get_logger(), "arm_id='%s' but URDF name='%s' -> using '%s'",
                arm_id_.c_str(), parsed_name.c_str(), parsed_name.c_str());
    arm_id_ = parsed_name;
  }

  try {
    pinocchio::urdf::buildModelFromXML(robot_description, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);
    j_full_.setZero(6, pin_model_.nv);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Pinocchio build failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_joints; ++i) {
    const std::string jname = arm_id_ + "_joint" + std::to_string(i + 1);
    const auto jid = pin_model_.getJointId(jname);
    if (jid == 0) {
      RCLCPP_FATAL(get_node()->get_logger(), "Joint '%s' not found", jname.c_str());
      return CallbackReturn::FAILURE;
    }
    pin_joint_ids_[i] = jid;
    pin_v_idx_[i] = pin_model_.joints[jid].idx_v();
  }

  ee_frame_id_ = pin_model_.getFrameId(ee_link_name_);
  if (ee_frame_id_ >= pin_model_.frames.size()) {
    const std::string fb = arm_id_ + "_link8";
    RCLCPP_WARN(get_node()->get_logger(), "Frame '%s' not found, fallback '%s'",
                ee_link_name_.c_str(), fb.c_str());
    ee_link_name_ = fb;
    ee_frame_id_ = pin_model_.getFrameId(ee_link_name_);
    if (ee_frame_id_ >= pin_model_.frames.size()) {
      RCLCPP_FATAL(get_node()->get_logger(), "Frame '%s' also not found", fb.c_str());
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "CheatingForceController configured: r=%.3fm, f=%.3fHz, "
              "Kp_xy=%.0f, Kd_xy=%.0f, gravity=%s, coriolis=%s, ff_accel=%s",
              circle_radius_, circle_frequency_, kp_xy_, kd_xy_,
              enable_gravity_comp_ ? "ON" : "OFF",
              enable_coriolis_comp_ ? "ON" : "OFF",
              enable_feedforward_accel_ ? "ON" : "OFF");

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: on_activate
// ---------------------------------------------------------------------------

CallbackReturn CheatingForceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  update_joint_states();

  elapsed_time_ = 0.0;
  pub_count_ = 0;
  center_initialized_ = false;

  // Seed tau_cmd_prev_ with gravity torques so the rate limiter doesn't
  // clip the first step from zero → full gravity compensation (~30 Nm).
  {
    Eigen::VectorXd q_pin, v_pin;
    update_pinocchio_model(q_pin, v_pin);
    const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd tau_g = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);
    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < tau_g.size()) {
        tau_cmd_prev_(i) = tau_g(idx_v);
      } else {
        tau_cmd_prev_(i) = 0.0;
      }
    }
  }
  trail_meas_.clear();
  trail_des_.clear();

  if (!log_file_path_.empty()) {
    log_file_.open(log_file_path_);
    if (log_file_.is_open()) {
      log_file_ << "time,x_des,y_des,z_des,x_meas,y_meas,z_meas,"
                << "ex,ey,ez,exy,vx,vy,vz,"
                << "tau0,tau1,tau2,tau3,tau4,tau5,tau6,"
                << "rate_sat\n";
      RCLCPP_INFO(get_node()->get_logger(), "Logging to: %s", log_file_path_.c_str());
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CheatingForceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (log_file_.is_open()) log_file_.close();
  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Joint state and Pinocchio helpers
// ---------------------------------------------------------------------------

void CheatingForceController::update_joint_states() {
  for (int i = 0; i < num_joints; ++i) {
    q_(i) = state_interfaces_.at(3 * i).get_value();
    dq_(i) = state_interfaces_.at(3 * i + 1).get_value();
    tau_meas_(i) = state_interfaces_.at(3 * i + 2).get_value();
  }
}

void CheatingForceController::update_pinocchio_model(
    Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin) {
  q_pin = Eigen::VectorXd::Zero(pin_model_.nq);
  v_pin = Eigen::VectorXd::Zero(pin_model_.nv);
  for (int i = 0; i < num_joints; ++i) {
    const auto jid = pin_joint_ids_[i];
    const int idx_q = static_cast<int>(pin_model_.joints[jid].idx_q());
    const int idx_v = pin_v_idx_[i];
    if (idx_q >= 0 && idx_q < q_pin.size()) q_pin(idx_q) = q_(i);
    if (idx_v >= 0 && idx_v < v_pin.size()) v_pin(idx_v) = dq_(i);
  }
}

CheatingForceController::Matrix67d CheatingForceController::compute_jacobian(
    const Eigen::VectorXd& q_pin) {
  Matrix67d J = Matrix67d::Zero();
  j_full_.setZero();
  pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_pin, ee_frame_id_,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, j_full_);
  for (int i = 0; i < num_joints; ++i) {
    const int idx_v = pin_v_idx_[i];
    if (idx_v >= 0 && idx_v < j_full_.cols()) J.col(i) = j_full_.col(idx_v);
  }
  return J;
}

CheatingForceController::Vector3d CheatingForceController::compute_ee_position(
    const Eigen::VectorXd& q_pin) {
  return pin_data_.oMf[ee_frame_id_].translation();
}

CheatingForceController::Vector7d CheatingForceController::saturate_torque_rate(
    const Vector7d& tau_d, const Vector7d& tau_d_last) {
  Vector7d out;
  for (int i = 0; i < num_joints; ++i) {
    const double diff = tau_d(i) - tau_d_last(i);
    out(i) = tau_d_last(i) + std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
  }
  return out;
}

// ---------------------------------------------------------------------------
// Core update loop
// ---------------------------------------------------------------------------

controller_interface::return_type CheatingForceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  update_joint_states();

  Eigen::VectorXd q_pin, v_pin;
  update_pinocchio_model(q_pin, v_pin);

  // Forward kinematics (needed for Jacobian and EE position).
  pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin, v_pin);
  pinocchio::updateFramePlacements(pin_model_, pin_data_);

  const Matrix67d J = compute_jacobian(q_pin);
  const Vector3d p_ee = compute_ee_position(q_pin);
  const Vector3d v_ee = J.topRows<3>() * dq_;

  elapsed_time_ += dt;

  // Initialize circle center on first call.
  if (!center_initialized_) {
    if (use_current_pose_as_center_) {
      circle_center_x_ = p_ee(0) + circle_center_offset_x_;
      circle_center_y_ = p_ee(1) + circle_center_offset_y_;
    } else {
      circle_center_x_ = circle_center_offset_x_;
      circle_center_y_ = circle_center_offset_y_;
    }
    hold_z_ = p_ee(2);
    center_initialized_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
                "Center initialized: (%.4f, %.4f), hold_z=%.4f",
                circle_center_x_, circle_center_y_, hold_z_);
  }

  // --- Smooth gain ramp (0→1 over soft_start_duration_) ---
  // Ramps gravity comp AND PD gains together so there's no discontinuity.
  const double gain_ramp = (elapsed_time_ < soft_start_duration_)
      ? 0.5 * (1.0 - std::cos(M_PI * elapsed_time_ / soft_start_duration_))
      : 1.0;

  // Circle radius ramp starts AFTER gains reach ~50% (half of soft_start).
  const double circle_delay = soft_start_duration_ * 0.5;
  const double circle_time = std::max(0.0, elapsed_time_ - circle_delay);
  const double circle_ramp_dur = soft_start_duration_;
  const double circle_ramp = (circle_time < circle_ramp_dur)
      ? 0.5 * (1.0 - std::cos(M_PI * circle_time / circle_ramp_dur))
      : 1.0;

  // --- Desired trajectory ---
  const double omega = 2.0 * M_PI * circle_frequency_;
  const double phi = omega * circle_time;
  const double r = circle_radius_ * circle_ramp;

  const double x_des = circle_center_x_ + r * std::cos(phi);
  const double y_des = circle_center_y_ + r * std::sin(phi);
  const double z_des = hold_z_;

  // Desired velocity (analytical derivative).
  const double dramp_dt = (circle_time < circle_ramp_dur)
      ? 0.5 * M_PI / circle_ramp_dur * std::sin(M_PI * circle_time / circle_ramp_dur)
      : 0.0;
  const double dr = circle_radius_ * dramp_dt;
  const double vx_des = (circle_time > 0.0) ? (dr * std::cos(phi) - r * omega * std::sin(phi)) : 0.0;
  const double vy_des = (circle_time > 0.0) ? (dr * std::sin(phi) + r * omega * std::cos(phi)) : 0.0;
  const double vz_des = 0.0;

  // Desired acceleration (for feedforward).
  double ax_des = 0.0, ay_des = 0.0, az_des = 0.0;
  if (enable_feedforward_accel_ && circle_ramp >= 1.0) {
    ax_des = -r * omega * omega * std::cos(phi);
    ay_des = -r * omega * omega * std::sin(phi);
  }

  // --- Position and velocity errors ---
  const double ex = x_des - p_ee(0);
  const double ey = y_des - p_ee(1);
  const double ez = z_des - p_ee(2);
  const double evx = vx_des - v_ee(0);
  const double evy = vy_des - v_ee(1);
  const double evz = vz_des - v_ee(2);

  // --- Cartesian PD (scaled by gain_ramp) ---
  Vector6d F_cmd = Vector6d::Zero();
  const double kp_xy_eff = kp_xy_ * gain_ramp;
  const double kd_xy_eff = kd_xy_ * gain_ramp;
  const double kp_z_eff = kp_z_ * gain_ramp;
  const double kd_z_eff = kd_z_ * gain_ramp;
  F_cmd(0) = kp_xy_eff * ex + kd_xy_eff * evx + (enable_feedforward_accel_ ? ax_des : 0.0);
  F_cmd(1) = kp_xy_eff * ey + kd_xy_eff * evy + (enable_feedforward_accel_ ? ay_des : 0.0);
  F_cmd(2) = kp_z_eff * ez + kd_z_eff * evz + (enable_feedforward_accel_ ? az_des : 0.0);

  // --- Null-space projector ---
  const Eigen::Matrix<double, 7, 6> J_T = J.transpose();
  const Matrix6d JJt = J * J_T + lambda_ * lambda_ * Matrix6d::Identity();
  const Eigen::Matrix<double, 7, 6> J_pinv = J_T * JJt.ldlt().solve(Matrix6d::Identity());
  const Eigen::Matrix<double, 7, 7> N =
      Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

  // --- Joint torque computation ---
  Vector7d tau_cmd = J_T * F_cmd + N * joint_damping_.cwiseProduct(-dq_);

  // Gravity compensation (also ramped).
  if (enable_gravity_comp_) {
    const Eigen::VectorXd v_zero_g = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd a_zero_g = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd tau_g_full = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero_g, a_zero_g);
    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < tau_g_full.size()) {
        tau_cmd(i) += gain_ramp * tau_g_full(idx_v);
      }
    }
  }

  // Coriolis compensation (also ramped).
  if (enable_coriolis_comp_) {
    const Eigen::VectorXd a_zero_c = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd v_zero_c = Eigen::VectorXd::Zero(pin_model_.nv);
    const Eigen::VectorXd nle_full = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_pin, a_zero_c);
    const Eigen::VectorXd tau_g_only = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero_c, a_zero_c);
    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < nle_full.size()) {
        tau_cmd(i) += gain_ramp * (nle_full(idx_v) - tau_g_only(idx_v));
      }
    }
  }

  // No tight rate saturation — let the controller respond freely.
  bool rate_sat = false;
  tau_cmd_prev_ = tau_cmd;

  // Send torques.
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  // --- Logging ---
  const double exy = std::sqrt(ex * ex + ey * ey);

  if (log_file_.is_open()) {
    log_file_ << elapsed_time_ << ","
              << x_des << "," << y_des << "," << z_des << ","
              << p_ee(0) << "," << p_ee(1) << "," << p_ee(2) << ","
              << ex << "," << ey << "," << ez << "," << exy << ","
              << v_ee(0) << "," << v_ee(1) << "," << v_ee(2) << ","
              << tau_cmd(0) << "," << tau_cmd(1) << "," << tau_cmd(2) << ","
              << tau_cmd(3) << "," << tau_cmd(4) << "," << tau_cmd(5) << ","
              << tau_cmd(6) << ","
              << (rate_sat ? 1 : 0) << "\n";
  }

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                       "exy=%.4fmm ez=%.4fmm | vx=%.4f vy=%.4f | rate_sat=%d",
                       exy * 1000.0, std::abs(ez) * 1000.0,
                       v_ee(0), v_ee(1), rate_sat ? 1 : 0);

  // --- Ink trail (measured vs desired) ---
  pub_count_++;
  if (ink_enabled_ && (pub_count_ % trail_decimation_) == 0) {
    geometry_msgs::msg::Point pt_m, pt_d;
    pt_m.x = p_ee(0); pt_m.y = p_ee(1); pt_m.z = p_ee(2);
    pt_d.x = x_des;   pt_d.y = y_des;   pt_d.z = z_des;

    trail_meas_.push_back(pt_m);
    trail_des_.push_back(pt_d);
    if (static_cast<int>(trail_meas_.size()) > ink_max_points_)
      trail_meas_.erase(trail_meas_.begin(), trail_meas_.begin() + (trail_meas_.size() - ink_max_points_));
    if (static_cast<int>(trail_des_.size()) > ink_max_points_)
      trail_des_.erase(trail_des_.begin(), trail_des_.begin() + (trail_des_.size() - ink_max_points_));

    const auto now = get_node()->now();
    if (pub_ink_meas_ && !trail_meas_.empty()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now;
      m.ns = "cheat_meas"; m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = ink_line_width_;
      m.color.b = 0.8f; m.color.g = 0.1f; m.color.r = 0.1f; m.color.a = 1.0f;
      m.points = trail_meas_;
      pub_ink_meas_->publish(m);
    }
    if (pub_ink_des_ && !trail_des_.empty()) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world"; m.header.stamp = now;
      m.ns = "cheat_des"; m.id = 0;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = ink_line_width_;
      m.color.r = 0.8f; m.color.g = 0.1f; m.color.b = 0.1f; m.color.a = 1.0f;
      m.points = trail_des_;
      pub_ink_des_->publish(m);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CheatingForceController,
                       controller_interface::ControllerInterface)
