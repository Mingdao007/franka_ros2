# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =============================================================================
# Direct Force Control - JACOBIAN ESTIMATION VERSION (No F/T Sensor)
# =============================================================================
# Uses Jacobian-based force estimation instead of direct F/T sensor.
# Tuned parameters achieve: Mean error ≈ 0.04N, Std ≈ 0.05N with 5N target.
#
# ⚠️  CRITICAL: 实验结束后必须关闭 Gazebo！残留进程会导致下次实验失败！
#     关闭方法: Ctrl+C 或 pkill -9 -f "gz sim" && pkill -9 -f "ros2"
# =============================================================================

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        f"{arm_id_str}.urdf.xacro",
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            "arm_id": arm_id_str,
            "hand": load_gripper_str,
            "ros2_control": "true",
            "gazebo": "true",
            "ee_id": franka_hand_str,
            "gazebo_effort": "true",
        },
    )

    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Launch args
    load_gripper_name = "load_gripper"
    franka_hand_name = "franka_hand"
    arm_id_name = "arm_id"
    use_rviz_name = "use_rviz"

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    use_rviz = LaunchConfiguration(use_rviz_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value="true",
        description="true/false for activating the gripper",
    )
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value="franka_hand",
        description="Default value: franka_hand",
    )
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name,
        default_value="fr3",
        description="Available values: fr3, fp3 and fer",
    )
    use_rviz_launch_argument = DeclareLaunchArgument(
        use_rviz_name,
        default_value="false",
        description="Start RViz (true/false)",
    )

    # robot_description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand],
    )

    # Gazebo Sim world
    world = os.path.join(
        get_package_share_directory("franka_gazebo_bringup"),
        "worlds",
        "direct_force_world.sdf",
    )

    # Make sure Gazebo can find package resources / meshes
    desc_share_parent = os.path.dirname(get_package_share_directory("franka_description"))
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_path = (existing + os.pathsep if existing else "") + desc_share_parent
    set_gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path)

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"{world} -r"}.items(),
    )

    # Spawn robot from /robot_description
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            arm_id,
            "-topic",
            "/robot_description",
        ],
        output="screen",
    )

    # Optional RViz
    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file, "-f", "world"],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # Controller manager spawners (uses default yaml from URDF plugin)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    load_direct_force_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "direct_force_example_controller",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    # Delay to ensure Gazebo and controller_manager are fully started
    delayed_load_jsb = TimerAction(period=5.0, actions=[load_joint_state_broadcaster])
    delayed_load_force = TimerAction(period=6.0, actions=[load_direct_force_controller])

    return LaunchDescription(
        [
            load_gripper_launch_argument,
            franka_hand_launch_argument,
            arm_id_launch_argument,
            use_rviz_launch_argument,
            set_gz_sim_resource_path,
            gazebo,
            robot_state_publisher,
            rviz,
            spawn,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[delayed_load_jsb, delayed_load_force],
                )
            ),
        ]
    )
