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

// =============================================================================
// FILE: gravity_compensation_example_controller.cpp
// PURPOSE: Simplest possible ROS2 controller - sends zero torque to all joints
// EFFECT: Robot floats in place (gravity compensated), can be hand-guided
// =============================================================================

// =============================================================================
// SECTION 1: INCLUDE DIRECTIVES (头文件包含)
// =============================================================================
// #include <header> searches system/library paths first
// #include "header" searches current directory first

// Include our own header file - declares the class interface
// Convention: .hpp for C++ headers, .h for C headers
// This header contains:
//   - Class declaration: GravityCompensationExampleController
//   - Member variables: arm_id_, num_joints
//   - Method declarations: on_init(), on_configure(), update(), etc.
#include <franka_example_controllers/gravity_compensation_example_controller.hpp>

// Standard C++ exception handling library
// Provides: std::exception (base class for all exceptions)
// Usage: catch (const std::exception& e) { e.what(); }
#include <exception>

// Standard C++ string library
// Provides: std::string class for text manipulation
// Much safer than C-style char* strings (automatic memory management)
#include <string>

// =============================================================================
// SECTION 2: NAMESPACE (命名空间)
// =============================================================================
// Namespace = a container that groups related code together
// Purpose: Avoid naming conflicts (e.g., two libraries both have "Controller" class)
// 
// Without namespace: GravityCompensationExampleController
// With namespace: franka_example_controllers::GravityCompensationExampleController
//
// All code between { } belongs to this namespace
namespace franka_example_controllers {

// =============================================================================
// SECTION 3: COMMAND INTERFACE CONFIGURATION (命令接口配置)
// =============================================================================
// WHAT: Tells ros2_control which hardware interfaces this controller will WRITE to
// WHEN: Called once during controller loading (not real-time)
// WHY: ros2_control needs to know which joints this controller will command
//
// Syntax breakdown:
//   controller_interface::InterfaceConfiguration  <- Return type (a struct)
//   GravityCompensationExampleController::        <- Class name (scope resolution)
//   command_interface_configuration()             <- Method name
//   const                                         <- This method doesn't modify member variables
//
controller_interface::InterfaceConfiguration
GravityCompensationExampleController::command_interface_configuration() const {
  
  // Create a configuration object to hold our interface requirements
  // InterfaceConfiguration is a struct with two members:
  //   .type  = INDIVIDUAL, ALL, or NONE
  //   .names = vector of interface names (only used if type=INDIVIDUAL)
  controller_interface::InterfaceConfiguration config;
  
  // INDIVIDUAL = we will specify exact interface names one by one
  // Alternatives:
  //   ALL = claim all available interfaces of this type
  //   NONE = don't need any interfaces
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Loop through all 7 joints of the Franka robot
  // Note: joint numbering starts from 1 (not 0) in ROS convention
  // 
  // Iteration 1: i=1, builds "fr3_joint1/effort"
  // Iteration 2: i=2, builds "fr3_joint2/effort"
  // ...
  // Iteration 7: i=7, builds "fr3_joint7/effort"
  //
  // ++i vs i++:
  //   ++i (pre-increment): increment first, then use value
  //   i++ (post-increment): use value first, then increment
  //   ++i is slightly more efficient (no temp copy needed)
  for (int i = 1; i <= num_joints; ++i) {
    // arm_id_ = "fr3" (from parameter)
    // "_joint" = literal string
    // std::to_string(i) = converts int to string ("1", "2", etc.)
    // "/effort" = interface type (torque command)
    //
    // String concatenation with + operator:
    // "fr3" + "_joint" + "1" + "/effort" = "fr3_joint1/effort"
    //
    // push_back() = add element to end of vector
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  
  // Return the configuration to ros2_control
  // ros2_control will then "claim" these interfaces for this controller
  // Other controllers cannot use these interfaces while we hold them
  return config;
}

// =============================================================================
// SECTION 4: STATE INTERFACE CONFIGURATION (状态接口配置)
// =============================================================================
// WHAT: Tells ros2_control which hardware interfaces this controller will READ from
// WHEN: Called once during controller loading
// 
// This controller returns empty {} = we don't need to read ANY sensor data
// This is unusual! Most controllers need position/velocity feedback
// This works because we're just sending constant zero torque
//
controller_interface::InterfaceConfiguration
GravityCompensationExampleController::state_interface_configuration() const {
  // {} = empty initializer list
  // Creates an InterfaceConfiguration with default values:
  //   .type = NONE (no interfaces needed)
  //   .names = empty vector
  return {};
}

// =============================================================================
// SECTION 5: UPDATE FUNCTION - THE REAL-TIME CONTROL LOOP (实时控制循环)
// =============================================================================
// WHAT: Main control algorithm, executed every control cycle
// WHEN: Called at 1000 Hz (every 1 millisecond) by ros2_control
// 
// ⚠️ CRITICAL: This function runs in a REAL-TIME thread!
// Rules for real-time code:
//   ❌ NO memory allocation (new, malloc, vector.push_back)
//   ❌ NO blocking calls (mutex lock, sleep, I/O)
//   ❌ NO unbounded loops
//   ✅ Only simple math and pre-allocated data access
//
// Parameters explained:
//   time   - Current ROS timestamp (when this update() was called)
//   period - Duration since last update() call (typically 1ms = 0.001s)
//
// The /*time*/ syntax means: parameter exists but we don't use it
// This suppresses "unused parameter" compiler warnings
//
controller_interface::return_type GravityCompensationExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  // Range-based for loop (C++11 feature)
  // Equivalent to: for (int i = 0; i < command_interfaces_.size(); i++)
  //
  // auto& breakdown:
  //   auto  = compiler automatically deduces type (CommandInterface in this case)
  //   &     = reference (not a copy!) - IMPORTANT for performance
  //   
  // Without &: Would copy each interface object (slow, wastes memory)
  // With &: Directly accesses the original object (fast, no copy)
  //
  // command_interfaces_ is a member variable inherited from ControllerInterface
  // It's a vector containing the interfaces we requested in command_interface_configuration()
  // Size = 7 (one for each joint)
  //
  for (auto& command_interface : command_interfaces_) {
    // set_value(0) sends zero torque command to this joint
    // 
    // Why does zero torque = gravity compensation?
    // Because Franka's libfranka driver does this internally:
    //
    //   τ_actual = τ_command + τ_gravity(q)
    //
    // Where:
    //   τ_command = what we send (0 in this case)
    //   τ_gravity(q) = gravity torque at current joint positions q
    //   τ_actual = torque actually applied to motors
    //
    // So if we send τ_command = 0:
    //   τ_actual = 0 + τ_gravity(q) = τ_gravity(q)
    //
    // This exactly compensates gravity → robot "floats" in place!
    //
    command_interface.set_value(0);
  }
  
  // Return OK to indicate successful execution
  // Alternative: return_type::ERROR would stop the controller
  return controller_interface::return_type::OK;
}

// =============================================================================
// SECTION 6: ON_CONFIGURE - LIFECYCLE TRANSITION CALLBACK (生命周期回调)
// =============================================================================
// WHAT: Called when controller transitions from UNCONFIGURED → INACTIVE state
// WHEN: After on_init(), before on_activate()
// PURPOSE: Read parameters, create publishers/subscribers, allocate memory
//
// ✅ NOT real-time: Can do slow operations here (memory allocation, file I/O)
//
// Lifecycle state machine:
//   UNCONFIGURED --on_configure()--> INACTIVE --on_activate()--> ACTIVE
//                                              <--on_deactivate()--
//
CallbackReturn GravityCompensationExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Read the "arm_id" parameter from ROS2 parameter server
  //
  // Method chain breakdown:
  //   get_node()                    -> Returns pointer to ROS2 node (rclcpp::Node*)
  //   ->get_parameter("arm_id")     -> Returns rclcpp::Parameter object
  //   .as_string()                  -> Converts Parameter to std::string
  //
  // The parameter value comes from:
  //   1. YAML config file (controllers.yaml)
  //   2. Command line argument
  //   3. Default value set in on_init()
  //
  // Example YAML:
  //   gravity_compensation_controller:
  //     ros__parameters:
  //       arm_id: "fr3"
  //
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  
  // Return SUCCESS to proceed with lifecycle transition
  // Alternatives:
  //   FAILURE = abort transition, stay in current state
  //   ERROR = critical error, may shut down controller
  return CallbackReturn::SUCCESS;
}

// =============================================================================
// SECTION 7: ON_INIT - EARLIEST INITIALIZATION (最早的初始化)
// =============================================================================
// WHAT: Called when controller is first loaded into ros2_control
// WHEN: Before on_configure(), the very first lifecycle callback
// PURPOSE: Declare parameters with default values
//
// Why separate on_init() and on_configure()?
//   on_init(): Declare what parameters EXIST (with defaults)
//   on_configure(): Actually READ parameter values (may have changed)
//
CallbackReturn GravityCompensationExampleController::on_init() {
  // try-catch block for exception handling
  // If any code inside try{} throws an exception, execution jumps to catch{}
  try {
    // auto_declare<T>(name, default_value) is a ROS2 helper function
    //
    // Template syntax: <std::string> specifies the parameter type
    // Templates allow one function to work with different types:
    //   auto_declare<int>("count", 10)
    //   auto_declare<double>("gain", 1.5)
    //   auto_declare<std::string>("name", "robot")
    //
    // This does TWO things:
    //   1. Declares parameter "arm_id" exists (registers with ROS2)
    //   2. Sets default value "fr3" if not specified elsewhere
    //
    // Parameter will be readable in on_configure() via get_parameter()
    //
    auto_declare<std::string>("arm_id", "fr3");
    
  } catch (const std::exception& e) {
    // Exception handling:
    //   const std::exception& e = catch any standard exception by reference
    //   e.what() = returns error message string
    //
    // fprintf = C-style formatted print (faster than std::cout)
    // stderr = standard error stream (separate from stdout)
    //
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
// ↑ End of namespace block - everything above belongs to franka_example_controllers

// =============================================================================
// SECTION 8: PLUGIN REGISTRATION (插件注册)
// =============================================================================
// This section makes the controller discoverable by ros2_control at runtime

// Include pluginlib macros for registering ROS2 plugins
#include "pluginlib/class_list_macros.hpp"

// NOLINTNEXTLINE = tell static analysis tools to ignore the next line
// (The macro doesn't follow typical code style, but that's OK)

// PLUGINLIB_EXPORT_CLASS macro does the magic of plugin registration
//
// How ros2_control finds controllers:
//   1. Reads franka_example_controllers.xml (plugin manifest)
//   2. Finds class name: franka_example_controllers::GravityCompensationExampleController
//   3. Loads shared library (.so file) containing this code
//   4. Uses this macro to instantiate the controller object
//
// Arguments:
//   1st: Full class name with namespace
//   2nd: Base class that ros2_control expects
//
// Without this macro: ros2_control cannot create instances of this controller!
//
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::GravityCompensationExampleController,
                       controller_interface::ControllerInterface)
