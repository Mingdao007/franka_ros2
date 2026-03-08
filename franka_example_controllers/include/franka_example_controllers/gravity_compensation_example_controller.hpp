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
// FILE: gravity_compensation_example_controller.hpp
// TYPE: Header file (.hpp = C++ header, .h = C header)
//   - hpp 就是说明书，告诉别人“我能干什么、怎么调用”。
//   - cpp 就是真干活的地方，里面写“怎么算、怎么控制”。
//   - yaml 就是调参和开关，不改代码也能换参数、换控制器名字或通道。
// PURPOSE: Declares the class interface (what the class CAN do)
//          Implementation (HOW it does it) is in the .cpp file
// =============================================================================

// =============================================================================
// SECTION 1: INCLUDE GUARD
// =============================================================================
// #pragma once = modern include guard (compiler-specific but widely supported)
// 
// Purpose: Prevent multiple inclusion of this header file
// Problem without it: If A.cpp includes this header AND B.hpp which also 
//                     includes this header, you get "class already defined" error
//
// Alternative (older style):
//   #ifndef GRAVITY_COMPENSATION_EXAMPLE_CONTROLLER_HPP
//   #define GRAVITY_COMPENSATION_EXAMPLE_CONTROLLER_HPP
//   ... code ...
//   #endif
//
// #pragma once is cleaner and slightly faster (compiler can skip file entirely)
#pragma once

// =============================================================================
// SECTION 2: STANDARD LIBRARY INCLUDES
// =============================================================================
// <string> provides std::string class
// Always include what you use - don't rely on other headers including it
#include <string>

// =============================================================================
// SECTION 3: ROS2 / ros2_control INCLUDES
// =============================================================================
// Base class for all ros2_control controllers
// Provides: ControllerInterface class with lifecycle management
// Key inherited members:
//   - command_interfaces_ : vector of interfaces to write commands
//   - state_interfaces_   : vector of interfaces to read sensor data
//   - get_node()          : access to ROS2 node for parameters, logging
#include <controller_interface/controller_interface.hpp>
#include "franka_example_controllers/visibility_control.h"

// ROS2 time types
// rclcpp::Duration = time interval (e.g., 1ms between control loops)
// rclcpp::Time     = absolute timestamp (e.g., "2026-01-31 12:00:00.001")
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

// =============================================================================
// SECTION 4: TYPE ALIAS
// =============================================================================
// using = creates a type alias (like typedef but more readable)
// 
// The full type name is very long:
//   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//
// After this line, we can just write: CallbackReturn
//
// CallbackReturn is an enum with values:
//   SUCCESS = transition succeeded
//   FAILURE = transition failed, stay in current state
//   ERROR   = critical error, may require shutdown
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// =============================================================================
// SECTION 5: NAMESPACE DECLARATION
// =============================================================================
// All our controller classes live in this namespace
// Must match the namespace in the .cpp file
namespace franka_example_controllers {

// =============================================================================
// SECTION 6: CLASS DECLARATION
// =============================================================================
/**
 * @brief Gravity compensation controller - sends zero torques to all joints
 * 
 * When zero torque is commanded, Franka's internal driver automatically
 * applies gravity compensation, allowing the robot to "float" and be
 * hand-guided by the operator.
 * 
 * This is the simplest possible controller - good for learning the structure.
 */
// 
// Class declaration syntax:
//   class ClassName : public BaseClass { ... };
//
// ": public" = public inheritance
//   - All public members of BaseClass are accessible as public in this class
//   - This class IS-A ControllerInterface (polymorphism)
//
// controller_interface::ControllerInterface is the base class for all ros2_control controllers
// It provides:
//   - Lifecycle management (init, configure, activate, deactivate)
//   - Access to command/state interfaces
//   - Integration with ros2_control framework
//
class GravityCompensationExampleController : public controller_interface::ControllerInterface {
 public:
  // ===========================================================================
  // PUBLIC METHODS - Can be called from outside the class
  // ===========================================================================
  
  // ---------------------------------------------------------------------------
  // Lifecycle callback: Called when transitioning UNCONFIGURED -> INACTIVE
  // Purpose: Read parameters, set up publishers/subscribers
  // 
  // override = explicitly marks this as overriding a virtual function from base class
  //            Compiler will error if base class doesn't have this method (safety!)
  // ---------------------------------------------------------------------------
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  // ---------------------------------------------------------------------------
  // Lifecycle callback: Called first when controller is loaded
  // Purpose: Declare parameters with default values
  // ---------------------------------------------------------------------------
  CallbackReturn on_init() override;

  // ---------------------------------------------------------------------------
  // Interface configuration: Which hardware interfaces to WRITE to
  // 
  // [[nodiscard]] = C++17 attribute
  //   Compiler warns if caller ignores the return value
  //   Prevents bugs like: command_interface_configuration();  // oops, forgot to use result!
  //
  // const = this method doesn't modify any member variables
  //         allows calling on const objects: const Controller& c; c.command_interface_configuration();
  // ---------------------------------------------------------------------------
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  // ---------------------------------------------------------------------------
  // Interface configuration: Which hardware interfaces to READ from
  // Returns empty config for this controller (doesn't need sensor feedback)
  // ---------------------------------------------------------------------------
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  // ---------------------------------------------------------------------------
  // MAIN CONTROL LOOP - Called at 1kHz (every 1ms)
  // 
  // Parameters:
  //   time   - Current ROS timestamp
  //   period - Time since last update() call
  //
  // return_type::OK     = continue running
  // return_type::ERROR  = stop controller
  //
  // ⚠️ REAL-TIME CRITICAL: No memory allocation, no blocking, no I/O
  // ---------------------------------------------------------------------------
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  // ===========================================================================
  // PRIVATE MEMBERS - Only accessible within this class
  // ===========================================================================
  
  // Robot arm identifier, e.g., "fr3" or "panda"
  // Used to construct interface names like "fr3_joint1/effort"
  // Trailing underscore _ = Google style convention for member variables
  std::string arm_id_;
  
  // Number of joints in the robot arm
  // Franka has 7 DOF (degrees of freedom)
  // const = value cannot be changed after initialization
  // Initialized inline (C++11 feature) - no need to set in constructor
  const int num_joints = 7;
};

}  // namespace franka_example_controllers
