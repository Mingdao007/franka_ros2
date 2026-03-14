# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)

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
            "hand": "false",        # No gripper — we add a fixed ball tip below.
            "ros2_control": "true",
            "gazebo": "true",
            "gazebo_effort": "true",
        },
    )

    # Inject a fixed ball-tip end-effector onto link8.
    # This replaces the Franka hand gripper with a simple rubber sphere
    # (no moving joints, no finger headaches).
    import xml.etree.ElementTree as ET
    tree = ET.fromstring(robot_description_config.toxml())

    ball_tip_xml = f"""
    <link name="{arm_id_str}_ball_tip">
      <inertial>
        <origin xyz="0 0 0.03" rpy="0 0 0"/>
        <mass value="0.1"/>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
      <visual name="stem_visual">
        <origin xyz="0 0 0.015" rpy="0 0 0"/>
        <geometry><cylinder radius="0.006" length="0.03"/></geometry>
        <material name="stem_metal">
          <color rgba="0.6 0.6 0.6 1"/>
        </material>
      </visual>
      <visual name="ball_visual">
        <origin xyz="0 0 0.03" rpy="0 0 0"/>
        <geometry><sphere radius="0.015"/></geometry>
        <material name="ball_rubber">
          <color rgba="0.2 0.2 0.2 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0.03" rpy="0 0 0"/>
        <geometry><sphere radius="0.015"/></geometry>
      </collision>
    </link>
    <joint name="{arm_id_str}_ball_tip_joint" type="fixed">
      <parent link="{arm_id_str}_link8"/>
      <child link="{arm_id_str}_ball_tip"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    <link name="{arm_id_str}_ball_tip_tcp"/>
    <joint name="{arm_id_str}_ball_tip_tcp_joint" type="fixed">
      <parent link="{arm_id_str}_ball_tip"/>
      <child link="{arm_id_str}_ball_tip_tcp"/>
      <origin xyz="0 0 0.045" rpy="0 0 0"/>
    </joint>
    """
    for elem in ET.fromstring(f"<wrapper>{ball_tip_xml}</wrapper>"):
        tree.append(elem)

    robot_description_xml = ET.tostring(tree, encoding="unicode")
    robot_description = {"robot_description": robot_description_xml}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    return [robot_state_publisher]


def get_gazebo_action(context: LaunchContext, headless):
    headless_str = context.perform_substitution(headless).lower()
    world = os.path.join(
        get_package_share_directory("franka_gazebo_bringup"),
        "worlds",
        "direct_force_world.sdf",
    )
    gui_config = os.path.join(
        get_package_share_directory("franka_gazebo_bringup"),
        "config",
        "gz_gui.config",
    )

    gz_args = f"{world} -r"
    if headless_str == "true":
        gz_args += " --headless-rendering"
    else:
        gz_args += f" --gui-config {gui_config}"

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    return [gazebo]


def generate_launch_description():
    load_gripper_name = "load_gripper"
    franka_hand_name = "franka_hand"
    arm_id_name = "arm_id"
    use_rviz_name = "use_rviz"
    headless_name = "headless"

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    use_rviz = LaunchConfiguration(use_rviz_name)
    headless = LaunchConfiguration(headless_name)

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
    headless_launch_argument = DeclareLaunchArgument(
        headless_name,
        default_value="false",
        description="Run Gazebo in server-only mode (true/false)",
    )

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand],
    )

    desc_share_parent = os.path.dirname(get_package_share_directory("franka_description"))
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_path = (existing + os.pathsep if existing else "") + desc_share_parent
    set_gz_sim_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path)
    gazebo = OpaqueFunction(function=get_gazebo_action, args=[headless])

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", arm_id, "-topic", "/robot_description"],
        output="screen",
    )

    rviz_file = os.path.join(
        get_package_share_directory("franka_gazebo_bringup"),
        "config",
        "hybrid_circle_force.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file, "-f", "world"],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
        output="screen",
    )

    load_hybrid_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hybrid_circle_force_controller", "--controller-manager-timeout", "60"],
        output="screen",
    )

    # Load controllers after spawn completes.
    # The spawner's --controller-manager-timeout handles waiting for CM readiness.
    delayed_load_jsb = TimerAction(period=0.5, actions=[load_joint_state_broadcaster])
    delayed_load_hybrid = TimerAction(period=1.5, actions=[load_hybrid_controller])

    return LaunchDescription(
        [
            load_gripper_launch_argument,
            franka_hand_launch_argument,
            arm_id_launch_argument,
            use_rviz_launch_argument,
            headless_launch_argument,
            set_gz_sim_resource_path,
            gazebo,
            robot_state_publisher,
            rviz,
            spawn,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[delayed_load_jsb, delayed_load_hybrid],
                )
            ),
        ]
    )
