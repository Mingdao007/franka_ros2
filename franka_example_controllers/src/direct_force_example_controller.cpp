#include <franka_example_controllers/direct_force_example_controller.hpp>  // 控制器自己的头文件

#include <cassert>      // assert() 断言宏
#include <cmath>        // 数学函数 (sin, cos, M_PI 等)
#include <exception>    // std::exception 异常处理

// Pinocchio 刚体动力学库 - 用于 Gazebo 仿真中的运动学/动力学计算
#include <pinocchio/algorithm/frames.hpp>      // 坐标系位姿计算
#include <pinocchio/algorithm/jacobian.hpp>    // 雅可比矩阵计算
#include <pinocchio/algorithm/kinematics.hpp>  // 正运动学
#include <pinocchio/algorithm/rnea.hpp>        // 递归牛顿-欧拉算法 (重力补偿)
#include <pinocchio/parsers/urdf.hpp>          // 从URDF解析机器人模型

// Eigen 线性代数库 - 矩阵/向量运算
#include <Eigen/Core>   // 基础矩阵类型 (Matrix, Vector)
#include <Eigen/Dense>  // 稠密矩阵运算 (求逆, 分解等)

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

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

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
    auto_declare<double>("force_amplitude", 0.0);    // New: sine wave amplitude
    auto_declare<double>("force_frequency", 0.0);    // New: sine wave frequency (Hz)
    
    // 3rd/4th order reference generator: waypoints and durations
    // waypoint_forces: [f0, f1, f2, ...] force values at each waypoint
    // waypoint_durations: [d0, d1, ...] time to reach each waypoint from previous
    // If empty, use force_desired as constant reference
    auto_declare<std::vector<double>>("waypoint_forces", {});
    auto_declare<std::vector<double>>("waypoint_durations", {});
    auto_declare<bool>("waypoint_loop", false);  // Loop back to first waypoint after last
    
    auto_declare<double>("kp", 0.5);
    auto_declare<double>("ki", 0.0);
    auto_declare<double>("kd", 0.0);
    auto_declare<double>("d_filter_alpha", 0.8);  // D term filter (higher = more smoothing)
    auto_declare<double>("coriolis_factor", 1.0); // Coriolis compensation (0=off, 1=full)
    auto_declare<bool>("use_bias_calibration", true);  // Whether to subtract tau_ext_initial
    auto_declare<double>("i_limit", 50.0);
    auto_declare<double>("max_command", 50.0);
    auto_declare<double>("command_sign", -1.0);
    auto_declare<double>("lambda", 1e-3);

    auto_declare<std::vector<double>>("joint_damping",
                                      {2.0, 2.0, 2.0, 2.0, 1.0, 1.0, 0.5});

    auto_declare<bool>("start_in_force_mode", false);

    auto_declare<std::string>("wrench_frame_id", "world");
    auto_declare<int>("pub_decimation", 1);

    // F/T sensor options (Gazebo direct measurement)
    auto_declare<bool>("use_ft_sensor", false);
    auto_declare<std::string>("ft_sensor_topic", "/ft_sensor/wrench");

    // Configurable parameters (previously hardcoded)
    auto_declare<double>("force_filter_alpha", 0.95);  // Force measurement low-pass filter
    auto_declare<std::string>("log_file_path", "");    // Empty = no logging
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
  force_amplitude_ = get_node()->get_parameter("force_amplitude").as_double();
  force_frequency_ = get_node()->get_parameter("force_frequency").as_double();
  
  // Load waypoint trajectory (3rd order reference generator)
  waypoint_forces_ = get_node()->get_parameter("waypoint_forces").as_double_array();
  waypoint_durations_ = get_node()->get_parameter("waypoint_durations").as_double_array();
  waypoint_loop_ = get_node()->get_parameter("waypoint_loop").as_bool();
  
  if (!waypoint_forces_.empty() && waypoint_durations_.size() == waypoint_forces_.size() - 1) {
    use_waypoint_trajectory_ = true;
    // Compute cumulative times for each waypoint
    waypoint_times_.clear();
    waypoint_times_.push_back(0.0);
    for (size_t i = 0; i < waypoint_durations_.size(); ++i) {
      waypoint_times_.push_back(waypoint_times_.back() + waypoint_durations_[i]);
    }
    total_trajectory_time_ = waypoint_times_.back();
    RCLCPP_INFO(get_node()->get_logger(), 
                "Waypoint trajectory enabled: %zu waypoints, total time=%.2fs, loop=%s",
                waypoint_forces_.size(), total_trajectory_time_, waypoint_loop_ ? "true" : "false");
  } else {
    use_waypoint_trajectory_ = false;
  }
  
  kp_ = get_node()->get_parameter("kp").as_double();
  ki_ = get_node()->get_parameter("ki").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();
  d_filter_alpha_ = get_node()->get_parameter("d_filter_alpha").as_double();
  coriolis_factor_ = get_node()->get_parameter("coriolis_factor").as_double();
  use_bias_calibration_ = get_node()->get_parameter("use_bias_calibration").as_bool();
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

  wrench_frame_id_ = get_node()->get_parameter("wrench_frame_id").as_string();
  pub_decimation_ = get_node()->get_parameter("pub_decimation").as_int();
  if (pub_decimation_ < 1) pub_decimation_ = 1;
  pub_count_ = 0;

  start_in_force_mode_ = get_node()->get_parameter("start_in_force_mode").as_bool();

  pub_wrench_est_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/estimated_wrench", 10);
  pub_wrench_cmd_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/commanded_wrench", 10);

  // F/T sensor subscriber (Gazebo direct measurement)
  // Configurable parameters (previously hardcoded)
  force_filter_alpha_ = get_node()->get_parameter("force_filter_alpha").as_double();
  log_file_path_ = get_node()->get_parameter("log_file_path").as_string();

  use_ft_sensor_ = get_node()->get_parameter("use_ft_sensor").as_bool();
  if (use_ft_sensor_) {
    std::string ft_topic = get_node()->get_parameter("ft_sensor_topic").as_string();
    ft_sensor_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        ft_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(ft_sensor_mutex_);
          ft_sensor_msg_ = *msg;
          ft_sensor_received_ = true;
        });
    RCLCPP_INFO(get_node()->get_logger(), "Subscribing to F/T sensor topic: %s", ft_topic.c_str());
  }

  if (is_gazebo_) {
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
      J_full_.setZero(6, pin_model_.nv);
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
              "Advanced: coriolis_factor=%.2f, d_filter_alpha=%.2f, use_bias_calibration=%s",
              coriolis_factor_, d_filter_alpha_, use_bias_calibration_ ? "true" : "false");
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
  motion_finished_ = start_in_force_mode_;
  if (start_in_force_mode_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "start_in_force_mode=true -> skipping move-to-start, entering force PID immediately.");
  }

  e_prev_ = 0.0;
  i_state_ = 0.0;
  elapsed_time_ = 0.0;
  f_meas_filtered_ = 0.0;
  de_filtered_ = 0.0;       // Reset filtered derivative state
  tau_cmd_prev_.setZero();  // Reset torque rate limiter state

  // ===== Official Bias Calibration Pattern =====
  // Capture initial external torque: tau_ext_initial = tau_measured - tau_gravity
  // This must be done when robot is stationary with no external contact
  // Reference: frankarobotics/franka_ros/force_example_controller.cpp
  tau_ext_initial_.setZero();
  
  if (is_gazebo_) {
    // Gazebo: compute gravity torques via Pinocchio RNEA
    Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(pin_model_.nq);
    for (int i = 0; i < num_joints; ++i) {
      const auto jid = pin_joint_ids_[i];
      const int idx_q = static_cast<int>(pin_model_.joints[jid].idx_q());
      if (idx_q >= 0 && idx_q < q_pin.size()) q_pin(idx_q) = q_(i);
    }
    
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd tau_gravity = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);
    
    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < tau_gravity.size()) {
        tau_ext_initial_(i) = tau_meas_(i) - tau_gravity(idx_v);
      }
    }
  } else {
    // Real robot: use libfranka model for gravity
    const std::array<double, 7> gravity_array = franka_robot_model_->getGravityForceVector();
    for (int i = 0; i < num_joints; ++i) {
      tau_ext_initial_(i) = tau_meas_(i) - gravity_array[i];
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(),
              "Initial bias captured (tau_ext_initial): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              tau_ext_initial_(0), tau_ext_initial_(1), tau_ext_initial_(2),
              tau_ext_initial_(3), tau_ext_initial_(4), tau_ext_initial_(5), tau_ext_initial_(6));

  // Open log file (overwrite mode) - only if path is configured
  if (!log_file_path_.empty()) {
    log_file_.open(log_file_path_);
    if (log_file_.is_open()) {
      log_file_ << "time,f_des,f_meas,f_cmd\n";
      RCLCPP_INFO(get_node()->get_logger(), "Logging started: %s", log_file_path_.c_str());
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DirectForceExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(get_node()->get_logger(), "Internal Logging Stopped.");
  }

  if (!is_gazebo_ && franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void DirectForceExampleController::updateJointStates() {
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

void DirectForceExampleController::updatePinocchioModel(Eigen::VectorXd& q_pin, Eigen::VectorXd& v_pin) {
  // Consolidates repeated Pinocchio model state update code
  // Maps our 7-joint state to Pinocchio's generalized coordinates
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

Eigen::Matrix<double, 6, 7> DirectForceExampleController::computeJacobian() {
  Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();

  if (is_gazebo_) {
    Eigen::VectorXd q_pin, v_pin;
    updatePinocchioModel(q_pin, v_pin);

    pinocchio::forwardKinematics(pin_model_, pin_data_, q_pin);
    pinocchio::updateFramePlacements(pin_model_, pin_data_);

    J_full_.setZero();
    pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_pin, ee_frame_id_,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full_);

    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < J_full_.cols()) J.col(i) = J_full_.col(idx_v);
    }
  } else {
    const std::array<double, 42> jac_array =
        franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J_map(jac_array.data());
    J = J_map;
  }
  return J;
}

DirectForceExampleController::Vector7d DirectForceExampleController::computeExternalTorque() {
  // tau_ext = tau_measured - tau_gravity - tau_ext_initial
  Vector7d tau_ext;
  double bias_factor = use_bias_calibration_ ? 1.0 : 0.0;

  if (is_gazebo_) {
    Eigen::VectorXd q_pin, v_pin;
    updatePinocchioModel(q_pin, v_pin);

    // RNEA with zero velocity and acceleration returns gravity torques
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd tau_gravity = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);

    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < tau_gravity.size()) {
        tau_ext(i) = tau_meas_(i) - tau_gravity(idx_v) - bias_factor * tau_ext_initial_(i);
      } else {
        tau_ext(i) = tau_meas_(i) - bias_factor * tau_ext_initial_(i);
      }
    }
  } else {
    // Real robot: libfranka already compensates gravity
    tau_ext = tau_meas_ - bias_factor * tau_ext_initial_;
  }
  return tau_ext;
}

DirectForceExampleController::Vector6d DirectForceExampleController::estimateCartesianForce(
    const Eigen::Matrix<double, 6, 7>& J, const Vector7d& tau_ext) {
  Vector6d F_hat;

  if (use_ft_sensor_ && ft_sensor_received_) {
    // Direct F/T sensor measurement (Gazebo only)
    std::lock_guard<std::mutex> lock(ft_sensor_mutex_);
    F_hat(0) = ft_sensor_msg_.wrench.force.x;
    F_hat(1) = ft_sensor_msg_.wrench.force.y;
    F_hat(2) = ft_sensor_msg_.wrench.force.z;
    F_hat(3) = ft_sensor_msg_.wrench.torque.x;
    F_hat(4) = ft_sensor_msg_.wrench.torque.y;
    F_hat(5) = ft_sensor_msg_.wrench.torque.z;
  } else {
    // Jacobian-based: F = (J*J^T + λ²I)^{-1} * J * tau_ext
    const Matrix6d A = (J * J.transpose()) + (lambda_ * lambda_) * Matrix6d::Identity();
    const Vector6d b = J * tau_ext;
    F_hat = A.ldlt().solve(b);
  }
  return F_hat;
}

DirectForceExampleController::Vector7d DirectForceExampleController::computeCoriolisCompensation() {
  Vector7d coriolis = Vector7d::Zero();
  if (coriolis_factor_ < 1e-6) return coriolis;

  if (is_gazebo_) {
    Eigen::VectorXd q_pin, v_pin;
    updatePinocchioModel(q_pin, v_pin);

    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(pin_model_.nv);
    Eigen::VectorXd tau_rnea = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_pin, a_zero);
    Eigen::VectorXd tau_gravity = pinocchio::rnea(pin_model_, pin_data_, q_pin, v_zero, a_zero);
    Eigen::VectorXd tau_coriolis = tau_rnea - tau_gravity;

    for (int i = 0; i < num_joints; ++i) {
      const int idx_v = pin_v_idx_[i];
      if (idx_v >= 0 && idx_v < tau_coriolis.size()) {
        coriolis(i) = coriolis_factor_ * tau_coriolis(idx_v);
      }
    }
  } else {
    const std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
    for (int i = 0; i < num_joints; ++i) {
      coriolis(i) = coriolis_factor_ * coriolis_array[i];
    }
  }
  return coriolis;
}

controller_interface::return_type DirectForceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  const double dt = period.seconds();
  updateJointStates();

  // publish in BOTH phases
  pub_count_++;
  auto publish_wrenches = [&](const Vector6d& F_est, const Vector6d& F_cmd) {
    if ((pub_count_ % pub_decimation_) != 0) return;
    geometry_msgs::msg::WrenchStamped msg_est;
    msg_est.header.stamp = get_node()->now();
    msg_est.header.frame_id = wrench_frame_id_;
    msg_est.wrench.force.x = F_est(0);
    msg_est.wrench.force.y = F_est(1);
    msg_est.wrench.force.z = F_est(2);
    msg_est.wrench.torque.x = F_est(3);
    msg_est.wrench.torque.y = F_est(4);
    msg_est.wrench.torque.z = F_est(5);
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
  };

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

      publish_wrenches(Vector6d::Zero(), Vector6d::Zero());
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Move-to-start running... (no force control yet)");
      return controller_interface::return_type::OK;
    }

    e_prev_ = 0.0;
    i_state_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Move-to-start finished, entering force PID mode.");
  }

  // Compute Jacobian, external torque, and Cartesian force (refactored)
  const Eigen::Matrix<double, 6, 7> J = computeJacobian();
  const Vector7d tau_ext = computeExternalTorque();
  const Vector6d F_hat = estimateCartesianForce(J, tau_ext);

  // Low-pass filter on force measurement
  const double f_meas_raw = F_hat(force_axis_);
  f_meas_filtered_ = force_filter_alpha_ * f_meas_filtered_ + (1.0 - force_filter_alpha_) * f_meas_raw;
  
  elapsed_time_ += dt;
  
  // Generate force reference signal
  double f_des_now;
  if (use_waypoint_trajectory_) {
    // 3rd order polynomial trajectory between waypoints
    f_des_now = computeWaypointReference(elapsed_time_);
  } else if (force_amplitude_ > 1e-6 && force_frequency_ > 1e-6) {
    // Sine wave reference
    f_des_now = force_desired_ + force_amplitude_ * std::sin(2.0 * M_PI * force_frequency_ * elapsed_time_);
  } else {
    // Constant reference
    f_des_now = force_desired_;
  }

  // Convert measured force to the same sign convention as desired force
  // Since command_sign_ = -1 means pressing down (negative Z), and we want f_meas positive
  // when pressing down with force equal to f_des, we multiply by command_sign_
  const double f_meas = command_sign_ * f_meas_filtered_;
  const double e = f_des_now - f_meas;

  i_state_ += e * dt;
  if (i_state_ > i_limit_) i_state_ = i_limit_;
  if (i_state_ < -i_limit_) i_state_ = -i_limit_;

  // ===== Filtered Derivative (D term) =====
  // Raw derivative amplifies noise; use low-pass filter to smooth it
  // Reference: standard practice in industrial PID controllers
  const double de_raw = (dt > 1e-9) ? (e - e_prev_) / dt : 0.0;
  de_filtered_ = d_filter_alpha_ * de_filtered_ + (1.0 - d_filter_alpha_) * de_raw;
  e_prev_ = e;

  const double u = kp_ * e + ki_ * i_state_ + kd_ * de_filtered_;

  Eigen::Matrix<double, 6, 1> F_cmd = Eigen::Matrix<double, 6, 1>::Zero();
  double f_cmd_axis = command_sign_ * (f_des_now + u);
  if (f_cmd_axis > max_command_) f_cmd_axis = max_command_;
  if (f_cmd_axis < -max_command_) f_cmd_axis = -max_command_;
  F_cmd(force_axis_) = f_cmd_axis;

  // Compute final torque command: tau = J^T * F + coriolis + damping
  const Vector7d coriolis = computeCoriolisCompensation();
  Vector7d tau_cmd = J.transpose() * F_cmd + coriolis + joint_damping_.cwiseProduct(-dq_);
  
  // ===== Official Torque Rate Saturation =====
  // Prevents discontinuities and protects robot from sudden torque changes
  // Reference: frankarobotics/franka_ros/force_example_controller.cpp#L137-L147
  tau_cmd = saturateTorqueRate(tau_cmd, tau_cmd_prev_);
  tau_cmd_prev_ = tau_cmd;
  
  for (int i = 0; i < num_joints; ++i) command_interfaces_[i].set_value(tau_cmd(i));

  publish_wrenches(F_hat, F_cmd);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                       "f_des=%.2f f_meas=%.3f err=%.3f u=%.3f (P=%.3f I=%.3f D=%.3f)",
                       f_des_now, f_meas, e, u, kp_*e, ki_*i_state_, kd_*de_filtered_);

  if (log_file_.is_open()) {
      log_file_ << elapsed_time_ << "," << f_des_now << "," << f_meas << "," << f_cmd_axis << "\n";
  }

  return controller_interface::return_type::OK;
}

DirectForceExampleController::Vector7d DirectForceExampleController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_d_last) {
  // Official implementation from frankarobotics/franka_ros
  // Limits the rate of change of commanded torque to kDeltaTauMax per sample
  Vector7d tau_d_saturated;
  for (int i = 0; i < num_joints; ++i) {
    double difference = tau_d_calculated(i) - tau_d_last(i);
    tau_d_saturated(i) = tau_d_last(i) + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

double DirectForceExampleController::computeWaypointReference(double t) {
  // Handle looping
  if (waypoint_loop_ && t > total_trajectory_time_) {
    t = std::fmod(t, total_trajectory_time_);
  }
  
  // Find current segment
  size_t seg = 0;
  for (size_t i = 0; i < waypoint_times_.size() - 1; ++i) {
    if (t >= waypoint_times_[i] && t < waypoint_times_[i + 1]) {
      seg = i;
      break;
    }
    if (i == waypoint_times_.size() - 2) {
      seg = i;  // Last segment
    }
  }
  
  // If past the end and not looping, hold last value
  if (!waypoint_loop_ && t >= total_trajectory_time_) {
    return waypoint_forces_.back();
  }
  
  // Cubic polynomial interpolation (3rd order) with zero velocity at waypoints
  // This ensures smooth transitions (C1 continuity)
  const double t0 = waypoint_times_[seg];
  const double t1 = waypoint_times_[seg + 1];
  const double f0 = waypoint_forces_[seg];
  const double f1 = waypoint_forces_[seg + 1];
  const double T = t1 - t0;
  
  if (T < 1e-9) return f0;  // Avoid division by zero
  
  // Normalized time [0, 1]
  const double s = (t - t0) / T;
  
  // Cubic polynomial: f(s) = a0 + a1*s + a2*s^2 + a3*s^3
  // Boundary conditions: f(0)=f0, f(1)=f1, f'(0)=0, f'(1)=0
  // Solution: f(s) = f0 + (f1-f0) * (3*s^2 - 2*s^3)
  // This is the "smoothstep" function
  const double blend = 3.0 * s * s - 2.0 * s * s * s;
  
  return f0 + (f1 - f0) * blend;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DirectForceExampleController,
                       controller_interface::ControllerInterface)
