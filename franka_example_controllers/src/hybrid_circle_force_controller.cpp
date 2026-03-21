#include <franka_example_controllers/hybrid_circle_force_controller.hpp>

#include <cassert>
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

controller_interface::InterfaceConfiguration
HybridCircleForceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
HybridCircleForceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn HybridCircleForceController::on_init() {
  try {
    auto_declare<bool>("gazebo", true);
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("ee_link", "");

    auto_declare<double>("circle_radius", 0.05);
    auto_declare<double>("circle_frequency", 0.2);
    auto_declare<double>("circle_center_offset_x", 0.0);
    auto_declare<double>("circle_center_offset_y", 0.0);
    auto_declare<bool>("use_current_pose_as_center", true);

    auto_declare<double>("kp_xy", 250.0);
    auto_declare<double>("kd_xy", 35.0);
    auto_declare<double>("ki_xy", 0.0);
    auto_declare<double>("i_limit_xy", 0.5);
    auto_declare<double>("d_filter_alpha_xy", 0.5);

    auto_declare<double>("force_desired", 5.0);
    auto_declare<double>("force_kp", 0.03);
    auto_declare<double>("force_ki", 0.005);
    auto_declare<double>("force_kd", 0.006);
    auto_declare<double>("i_limit", 15.0);
    auto_declare<double>("max_force_command", 60.0);
    auto_declare<double>("command_sign", -1.0);
    auto_declare<double>("force_filter_alpha", 0.8);
    auto_declare<double>("d_filter_alpha", 0.5);
    auto_declare<double>("lambda", 0.01);

    auto_declare<bool>("use_bias_calibration", false);
    auto_declare<std::vector<double>>("joint_damping", {15.0, 15.0, 15.0, 15.0, 10.0, 8.0, 5.0});

    auto_declare<double>("descent_kp_xy", 4000.0);
    auto_declare<double>("descent_kd_xy", 160.0);
    auto_declare<double>("descent_kp_z", 1000.0);
    auto_declare<double>("descent_kd_z", 50.0);
    auto_declare<double>("descent_ki_xy", 0.0);
    auto_declare<double>("descent_ki_z", 0.0);
    auto_declare<double>("descent_speed", 0.02);
    auto_declare<double>("descent_contact_force", 2.0);

    auto_declare<bool>("use_ft_sensor", false);
    auto_declare<std::string>("ft_sensor_topic", "");

    auto_declare<std::string>("wrench_frame_id", "world");
    auto_declare<int>("pub_decimation", 1);
    auto_declare<std::string>("log_file_path", "");

    auto_declare<bool>("ink_enabled", true);
    auto_declare<int>("ink_trail_decimation", 50);
    auto_declare<double>("ink_force_threshold", 1.0);
    auto_declare<double>("ink_line_width", 0.006);
    auto_declare<bool>("ink_project_to_surface", true);
    auto_declare<double>("ink_surface_z", 0.4160);
    auto_declare<double>("ink_surface_epsilon", 0.0005);
    auto_declare<double>("ink_contact_offset_z", -0.03);
    auto_declare<int>("ink_max_points", 5000);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridCircleForceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  if (!is_gazebo_) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "HybridCircleForceController currently supports gazebo=true only.");
    return CallbackReturn::FAILURE;
  }

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
  ki_xy_ = get_node()->get_parameter("ki_xy").as_double();
  i_limit_xy_ = get_node()->get_parameter("i_limit_xy").as_double();
  d_filter_alpha_xy_ = get_node()->get_parameter("d_filter_alpha_xy").as_double();

  force_desired_ = get_node()->get_parameter("force_desired").as_double();
  force_kp_ = get_node()->get_parameter("force_kp").as_double();
  force_ki_ = get_node()->get_parameter("force_ki").as_double();
  force_kd_ = get_node()->get_parameter("force_kd").as_double();
  i_limit_ = get_node()->get_parameter("i_limit").as_double();
  max_force_command_ = get_node()->get_parameter("max_force_command").as_double();
  command_sign_ = get_node()->get_parameter("command_sign").as_double();
  force_filter_alpha_ = get_node()->get_parameter("force_filter_alpha").as_double();
  d_filter_alpha_ = get_node()->get_parameter("d_filter_alpha").as_double();
  lambda_ = get_node()->get_parameter("lambda").as_double();

  use_bias_calibration_ = get_node()->get_parameter("use_bias_calibration").as_bool();

  descent_kp_xy_ = get_node()->get_parameter("descent_kp_xy").as_double();
  descent_kd_xy_ = get_node()->get_parameter("descent_kd_xy").as_double();
  descent_kp_z_ = get_node()->get_parameter("descent_kp_z").as_double();
  descent_kd_z_ = get_node()->get_parameter("descent_kd_z").as_double();
  descent_ki_xy_ = get_node()->get_parameter("descent_ki_xy").as_double();
  descent_ki_z_ = get_node()->get_parameter("descent_ki_z").as_double();
  descent_speed_ = get_node()->get_parameter("descent_speed").as_double();
  descent_contact_force_ = get_node()->get_parameter("descent_contact_force").as_double();

  auto jd = get_node()->get_parameter("joint_damping").as_double_array();
  if (jd.size() == static_cast<size_t>(num_joints)) {
    for (int i = 0; i < num_joints; ++i) {
      joint_damping_(i) = jd[i];
    }
  }

  wrench_frame_id_ = get_node()->get_parameter("wrench_frame_id").as_string();
  pub_decimation_ = get_node()->get_parameter("pub_decimation").as_int();
  if (pub_decimation_ < 1) {
    pub_decimation_ = 1;
  }

  log_file_path_ = get_node()->get_parameter("log_file_path").as_string();

  ink_enabled_ = get_node()->get_parameter("ink_enabled").as_bool();
  trail_decimation_ = get_node()->get_parameter("ink_trail_decimation").as_int();
  ink_force_threshold_ = get_node()->get_parameter("ink_force_threshold").as_double();
  ink_line_width_ = get_node()->get_parameter("ink_line_width").as_double();
  ink_project_to_surface_ = get_node()->get_parameter("ink_project_to_surface").as_bool();
  ink_surface_z_ = get_node()->get_parameter("ink_surface_z").as_double();
  ink_surface_epsilon_ = get_node()->get_parameter("ink_surface_epsilon").as_double();
  ink_contact_offset_z_ = get_node()->get_parameter("ink_contact_offset_z").as_double();
  ink_max_points_ = get_node()->get_parameter("ink_max_points").as_int();

  if (trail_decimation_ < 1) {
    trail_decimation_ = 1;
  }
  if (ink_force_threshold_ < 0.0) {
    ink_force_threshold_ = 0.0;
  }
  if (ink_line_width_ <= 0.0) {
    ink_line_width_ = 0.006;
  }
  if (ink_max_points_ < 2) {
    ink_max_points_ = 2;
  }

  pub_wrench_est_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/estimated_wrench", 10);
  pub_wrench_cmd_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/commanded_wrench", 10);

  // FT sensor: subscribe to bridged Gazebo ForceTorqueSensor topic.
  use_ft_sensor_ = get_node()->get_parameter("use_ft_sensor").as_bool();
  ft_sensor_topic_ = get_node()->get_parameter("ft_sensor_topic").as_string();
  if (use_ft_sensor_) {
    if (ft_sensor_topic_.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "use_ft_sensor=true but ft_sensor_topic is empty.");
      return CallbackReturn::FAILURE;
    }
    ft_sensor_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        ft_sensor_topic_, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ft_mutex_);
          latest_ft_msg_ = *msg;
          ft_data_received_.store(true);
        });
    RCLCPP_INFO(get_node()->get_logger(),
                "FT sensor mode: subscribing to '%s'", ft_sensor_topic_.c_str());
  } else {
    RCLCPP_INFO(get_node()->get_logger(),
                "Jacobian force estimation mode (use_ft_sensor=false).");
  }

  // RViz marker publishers for ink trails.
  if (ink_enabled_) {
    pub_ink_marker_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
        "~/ink_trail", rclcpp::QoS(1).transient_local());
    pub_ink_desired_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
        "~/ink_desired", rclcpp::QoS(1).transient_local());
  }
  trail_points_.clear();
  trail_desired_points_.clear();

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
    j_full_.setZero(6, pin_model_.nv);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Pinocchio URDF parse/build failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

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

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured HybridCircleForceController: radius=%.3fm, freq=%.3fHz, Fz=%.3fN",
              circle_radius_, circle_frequency_, force_desired_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Ink config: enabled=%s, decimation=%d, width=%.4f, "
              "project_surface=%s (z=%.4f, eps=%.4f), tcp_offset_z=%.4f",
              ink_enabled_ ? "true" : "false", trail_decimation_, ink_line_width_,
              ink_project_to_surface_ ? "true" : "false",
              ink_surface_z_, ink_surface_epsilon_, ink_contact_offset_z_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridCircleForceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  update_joint_states();

  elapsed_time_ = 0.0;
  e_prev_ = 0.0;
  i_state_ = 0.0;
  f_meas_filtered_ = 0.0;
  de_filtered_ = 0.0;
  pub_count_ = 0;
  tau_cmd_prev_.setZero();
  trail_points_.clear();
  trail_desired_points_.clear();

  // XY PID state (circle phase).
  ix_state_ = 0.0;
  iy_state_ = 0.0;
  ex_prev_ = 0.0;
  ey_prev_ = 0.0;
  dex_filtered_ = 0.0;
  dey_filtered_ = 0.0;

  // Descent PID state.
  ix_descent_ = 0.0;
  iy_descent_ = 0.0;
  iz_descent_ = 0.0;
  ex_prev_descent_ = 0.0;
  ey_prev_descent_ = 0.0;
  ez_prev_descent_ = 0.0;

  center_initialized_ = false;
  phase_ = Phase::kDescent;
  descent_start_z_ = 0.0;

  tau_ext_initial_.setZero();
  Eigen::VectorXd q_pin, v_pin;
  update_pinocchio_model(q_pin, v_pin);
  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
  const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
  const Eigen::VectorXd tau_gravity = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);
  for (int i = 0; i < num_joints; ++i) {
    const int idx_v = pin_v_idx_[i];
    if (idx_v >= 0 && idx_v < tau_gravity.size()) {
      tau_ext_initial_(i) = tau_meas_(i) - tau_gravity(idx_v);
    }
  }

  if (!log_file_path_.empty()) {
    log_file_.open(log_file_path_);
    if (log_file_.is_open()) {
      log_file_ << "time,x_des,y_des,x_meas,y_meas,z_meas,ex,ey,vx,vy,vz,fz_des,fz_meas,fz_cmd,phase\n";
      RCLCPP_INFO(get_node()->get_logger(), "Logging started: %s", log_file_path_.c_str());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn HybridCircleForceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (log_file_.is_open()) {
    log_file_.close();
  }
  return CallbackReturn::SUCCESS;
}

void HybridCircleForceController::update_joint_states() {
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

void HybridCircleForceController::update_pinocchio_model(Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin) {
  q_pin = Eigen::VectorXd::Zero(pin_model_.nq);
  v_pin = Eigen::VectorXd::Zero(pin_model_.nv);
  for (int i = 0; i < num_joints; ++i) {
    const auto jid = pin_joint_ids_[i];
    const int idx_q = static_cast<int>(pin_model_.joints[jid].idx_q());
    const int idx_v = pin_v_idx_[i];
    if (idx_q >= 0 && idx_q < q_pin.size()) {
      q_pin(idx_q) = q_(i);
    }
    if (idx_v >= 0 && idx_v < v_pin.size()) {
      v_pin(idx_v) = dq_(i);
    }
  }
}

HybridCircleForceController::Matrix67d HybridCircleForceController::compute_jacobian() {
  Matrix67d J = Matrix67d::Zero();

  Eigen::VectorXd q_pin, v_pin;
  update_pinocchio_model(q_pin, v_pin);

  pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin);
  pinocchio::updateFramePlacements(pin_model_, pin_data_);

  j_full_.setZero();
  pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_pin, ee_frame_id_,
                                  pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, j_full_);

  for (int i = 0; i < num_joints; ++i) {
    const int idx_v = pin_v_idx_[i];
    if (idx_v >= 0 && idx_v < j_full_.cols()) {
      J.col(i) = j_full_.col(idx_v);
    }
  }

  return J;
}

HybridCircleForceController::Vector3d HybridCircleForceController::compute_ee_position(
    const Eigen::VectorXd& q_pin) {
  pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin);
  pinocchio::updateFramePlacements(pin_model_, pin_data_);
  return pin_data_.oMf[ee_frame_id_].translation();
}

HybridCircleForceController::Vector7d HybridCircleForceController::compute_external_torque() {
  Vector7d tau_ext = Vector7d::Zero();
  const double bias_factor = use_bias_calibration_ ? 1.0 : 0.0;

  Eigen::VectorXd q_pin, v_pin;
  update_pinocchio_model(q_pin, v_pin);

  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
  const Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
  const Eigen::VectorXd tau_gravity = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);

  for (int i = 0; i < num_joints; ++i) {
    const int idx_v = pin_v_idx_[i];
    if (idx_v >= 0 && idx_v < tau_gravity.size()) {
      tau_ext(i) = tau_meas_(i) - tau_gravity(idx_v) - bias_factor * tau_ext_initial_(i);
    }
  }

  return tau_ext;
}

HybridCircleForceController::Vector6d HybridCircleForceController::estimate_cartesian_force(
    const Matrix67d& J, const Vector7d& tau_ext) {
  const Matrix6d A = (J * J.transpose()) + (lambda_ * lambda_) * Matrix6d::Identity();
  const Vector6d b = J * tau_ext;
  return A.ldlt().solve(b);
}

controller_interface::return_type HybridCircleForceController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  update_joint_states();

  Eigen::VectorXd q_pin, v_pin;
  update_pinocchio_model(q_pin, v_pin);

  const Matrix67d J = compute_jacobian();
  const Vector7d tau_ext = compute_external_torque();

  Vector6d F_hat;
  if (use_ft_sensor_ && ft_data_received_.load()) {
    std::lock_guard<std::mutex> lock(ft_mutex_);
    F_hat << latest_ft_msg_.wrench.force.x,
             latest_ft_msg_.wrench.force.y,
             latest_ft_msg_.wrench.force.z,
             latest_ft_msg_.wrench.torque.x,
             latest_ft_msg_.wrench.torque.y,
             latest_ft_msg_.wrench.torque.z;
  } else {
    F_hat = estimate_cartesian_force(J, tau_ext);
  }

  const Vector3d p_ee = compute_ee_position(q_pin);
  const Vector3d v_ee = J.topRows<3>() * dq_;

  // Initialize reference positions on first call.
  if (!center_initialized_) {
    if (use_current_pose_as_center_) {
      circle_center_x_ = p_ee(0) + circle_center_offset_x_;
      circle_center_y_ = p_ee(1) + circle_center_offset_y_;
    } else {
      circle_center_x_ = circle_center_offset_x_;
      circle_center_y_ = circle_center_offset_y_;
    }
    descent_start_z_ = p_ee(2);
    center_initialized_ = true;
  }

  // Force estimation (needed by both phases).
  // EMA: alpha=1.0 means no filtering (use raw), alpha=0.0 means frozen (max smoothing).
  const double fz_meas_raw = F_hat(2);
  f_meas_filtered_ = force_filter_alpha_ * fz_meas_raw + (1.0 - force_filter_alpha_) * f_meas_filtered_;
  const double fz_meas = command_sign_ * f_meas_filtered_;

  double ex = 0.0, ey = 0.0;
  double x_des = circle_center_x_, y_des = circle_center_y_;
  double fz_cmd = 0.0;
  Vector6d F_cmd = Vector6d::Zero();

  if (phase_ == Phase::kDescent) {
    // --- Descent phase: high-stiffness impedance control, descend in Z ---
    elapsed_time_ += dt;
    const double z_des = descent_start_z_ - descent_speed_ * elapsed_time_;

    // Descent XY PID: hold at circle center.
    const double dex = circle_center_x_ - p_ee(0);
    const double dey = circle_center_y_ - p_ee(1);
    const double dez = z_des - p_ee(2);

    ix_descent_ += dex * dt;
    iy_descent_ += dey * dt;
    iz_descent_ += dez * dt;

    const double dex_dt = (dt > 1e-9) ? (dex - ex_prev_descent_) / dt : 0.0;
    const double dey_dt = (dt > 1e-9) ? (dey - ey_prev_descent_) / dt : 0.0;
    const double dez_dt = (dt > 1e-9) ? (dez - ez_prev_descent_) / dt : 0.0;
    ex_prev_descent_ = dex;
    ey_prev_descent_ = dey;
    ez_prev_descent_ = dez;

    F_cmd(0) = descent_kp_xy_ * dex + descent_ki_xy_ * ix_descent_ + descent_kd_xy_ * dex_dt;
    F_cmd(1) = descent_kp_xy_ * dey + descent_ki_xy_ * iy_descent_ + descent_kd_xy_ * dey_dt;
    F_cmd(2) = descent_kp_z_ * dez + descent_ki_z_ * iz_descent_ + descent_kd_z_ * dez_dt;

    // Clamp descent force command.
    if (F_cmd(2) > max_force_command_) F_cmd(2) = max_force_command_;
    if (F_cmd(2) < -max_force_command_) F_cmd(2) = -max_force_command_;

    fz_cmd = F_cmd(2);
    ex = circle_center_x_ - p_ee(0);
    ey = circle_center_y_ - p_ee(1);

    // Transition to circle phase when contact force exceeds threshold.
    if (std::abs(fz_meas) > descent_contact_force_) {
      phase_ = Phase::kCircle;
      elapsed_time_ = 0.0;  // Reset for circle soft-start.
      e_prev_ = 0.0;
      i_state_ = 0.0;
      de_filtered_ = 0.0;
      ix_state_ = 0.0;
      iy_state_ = 0.0;
      ex_prev_ = 0.0;
      ey_prev_ = 0.0;
      dex_filtered_ = 0.0;
      dey_filtered_ = 0.0;
      RCLCPP_INFO(get_node()->get_logger(),
                   "Contact detected (Fz=%.2f), switching to circle phase", fz_meas);
    }

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                         "DESCENT z_des=%.4f z=%.4f | fz_meas=%.3f threshold=%.1f",
                         z_des, p_ee(2), fz_meas, descent_contact_force_);
  } else {
    // --- Circle phase: hybrid force/position control ---
    elapsed_time_ += dt;
    const double omega = 2.0 * M_PI * circle_frequency_;
    const double phi = omega * elapsed_time_;

    // Cosine ramp: C1-continuous (zero derivative at both endpoints).
    const double ramp = (elapsed_time_ < soft_start_duration_)
                            ? 0.5 * (1.0 - std::cos(M_PI * elapsed_time_ / soft_start_duration_))
                            : 1.0;
    const double r = circle_radius_ * ramp;

    x_des = circle_center_x_ + r * std::cos(phi);
    y_des = circle_center_y_ + r * std::sin(phi);

    ex = x_des - p_ee(0);
    ey = y_des - p_ee(1);

    // XY PID: integral with anti-windup clamp.
    ix_state_ += ex * dt;
    iy_state_ += ey * dt;
    if (ix_state_ > i_limit_xy_) ix_state_ = i_limit_xy_;
    if (ix_state_ < -i_limit_xy_) ix_state_ = -i_limit_xy_;
    if (iy_state_ > i_limit_xy_) iy_state_ = i_limit_xy_;
    if (iy_state_ < -i_limit_xy_) iy_state_ = -i_limit_xy_;

    // XY PID: filtered derivative of position error.
    const double dex_raw = (dt > 1e-9) ? (ex - ex_prev_) / dt : 0.0;
    const double dey_raw = (dt > 1e-9) ? (ey - ey_prev_) / dt : 0.0;
    dex_filtered_ = (1.0 - d_filter_alpha_xy_) * dex_filtered_ + d_filter_alpha_xy_ * dex_raw;
    dey_filtered_ = (1.0 - d_filter_alpha_xy_) * dey_filtered_ + d_filter_alpha_xy_ * dey_raw;
    ex_prev_ = ex;
    ey_prev_ = ey;

    const double fx_cmd = kp_xy_ * ex + ki_xy_ * ix_state_ + kd_xy_ * dex_filtered_;
    const double fy_cmd = kp_xy_ * ey + ki_xy_ * iy_state_ + kd_xy_ * dey_filtered_;

    const double e_force = force_desired_ - fz_meas;

    i_state_ += e_force * dt;
    if (i_state_ > i_limit_) i_state_ = i_limit_;
    if (i_state_ < -i_limit_) i_state_ = -i_limit_;

    const double de_raw = (dt > 1e-9) ? (e_force - e_prev_) / dt : 0.0;
    de_filtered_ = (1.0 - d_filter_alpha_) * de_filtered_ + d_filter_alpha_ * de_raw;
    e_prev_ = e_force;

    const double u_force = force_kp_ * e_force + force_ki_ * i_state_ + force_kd_ * de_filtered_;
    fz_cmd = command_sign_ * (force_desired_ + u_force);
    if (fz_cmd > max_force_command_) fz_cmd = max_force_command_;
    if (fz_cmd < -max_force_command_) fz_cmd = -max_force_command_;

    F_cmd(0) = fx_cmd;
    F_cmd(1) = fy_cmd;
    F_cmd(2) = fz_cmd;
  }

  // Null-space projector: N = I - J_pinv * J (damping only in null space,
  // so it does not fight the Cartesian position/force commands).
  const Eigen::Matrix<double, 7, 6> J_T = J.transpose();
  const Matrix6d JJt = J * J_T + lambda_ * lambda_ * Matrix6d::Identity();
  const Eigen::Matrix<double, 7, 6> J_pinv = J_T * JJt.ldlt().solve(Matrix6d::Identity());
  const Eigen::Matrix<double, 7, 7> N =
      Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;

  // Hybrid force/position control law:
  //   tau = J^T * F_cmd + N * (-D * dq)
  // Note: no gravity compensation — Gazebo physics engine handles gravity.
  Vector7d tau_cmd = J_T * F_cmd + N * joint_damping_.cwiseProduct(-dq_);
  tau_cmd = saturate_torque_rate(tau_cmd, tau_cmd_prev_);
  tau_cmd_prev_ = tau_cmd;

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

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

  // Ink trails during circle phase.
  if (ink_enabled_ && phase_ == Phase::kCircle &&
      (pub_count_ % trail_decimation_) == 0) {
    double ink_z = p_ee(2) + ink_contact_offset_z_;
    if (ink_project_to_surface_) {
      ink_z = ink_surface_z_ + ink_surface_epsilon_;
    }

    // Measured trail: only when in contact (force above threshold).
    if (std::abs(fz_meas) > ink_force_threshold_) {
      geometry_msgs::msg::Point pt;
      pt.x = p_ee(0);
      pt.y = p_ee(1);
      pt.z = ink_z;
      trail_points_.push_back(pt);
      if (static_cast<int>(trail_points_.size()) > ink_max_points_) {
        trail_points_.erase(
            trail_points_.begin(),
            trail_points_.begin() + (trail_points_.size() - static_cast<size_t>(ink_max_points_)));
      }
    }

    // Desired trail: always (planned trajectory, independent of contact).
    geometry_msgs::msg::Point pt_des;
    pt_des.x = x_des;
    pt_des.y = y_des;
    pt_des.z = ink_z;
    trail_desired_points_.push_back(pt_des);
    if (static_cast<int>(trail_desired_points_.size()) > ink_max_points_) {
      trail_desired_points_.erase(
          trail_desired_points_.begin(),
          trail_desired_points_.begin() +
              (trail_desired_points_.size() - static_cast<size_t>(ink_max_points_)));
    }

    const auto now = get_node()->now();
    const double half_width = ink_line_width_ * 0.5;

    // RViz: measured trail (blue).
    if (pub_ink_marker_ && !trail_points_.empty()) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = now;
      marker.ns = "ink_trail";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = half_width;
      marker.color.r = 0.1f;
      marker.color.g = 0.1f;
      marker.color.b = 0.8f;
      marker.color.a = 1.0f;
      marker.points = trail_points_;
      pub_ink_marker_->publish(marker);
    }

    // RViz: desired trail (red).
    if (pub_ink_desired_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = now;
      marker.ns = "ink_desired";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = half_width;
      marker.color.r = 0.8f;
      marker.color.g = 0.1f;
      marker.color.b = 0.1f;
      marker.color.a = 1.0f;
      marker.points = trail_desired_points_;
      pub_ink_desired_->publish(marker);
    }

  }

  if (phase_ == Phase::kCircle) {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                         "circle ex=%.4f ey=%.4f | fz_des=%.2f fz_meas=%.3f",
                         ex, ey, force_desired_, fz_meas);
  }

  if (log_file_.is_open()) {
    log_file_ << elapsed_time_ << "," << x_des << "," << y_des << ","
              << p_ee(0) << "," << p_ee(1) << "," << p_ee(2) << ","
              << ex << "," << ey << ","
              << v_ee(0) << "," << v_ee(1) << "," << v_ee(2) << ","
              << force_desired_ << "," << fz_meas << "," << fz_cmd << ","
              << (phase_ == Phase::kDescent ? 0 : 1) << "\n";
  }

  return controller_interface::return_type::OK;
}

HybridCircleForceController::Vector7d HybridCircleForceController::saturate_torque_rate(
    const Vector7d& tau_d_calculated, const Vector7d& tau_d_last) {
  Vector7d tau_d_saturated;
  for (int i = 0; i < num_joints; ++i) {
    const double difference = tau_d_calculated(i) - tau_d_last(i);
    tau_d_saturated(i) =
        tau_d_last(i) + std::max(std::min(difference, k_delta_tau_max), -k_delta_tau_max);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HybridCircleForceController,
                       controller_interface::ControllerInterface)
