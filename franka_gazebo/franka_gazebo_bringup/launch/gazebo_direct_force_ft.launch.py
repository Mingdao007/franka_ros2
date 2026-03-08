# Copyright (c) 2024 Franka Robotics GmbH
# Direct Force Control - F/T SENSOR VERSION
# Same as Jacobian version but with use_ft_sensor: true

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
                            IncludeLaunchDescription, TimerAction, SetEnvironmentVariable)
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
        get_package_share_directory("franka_description"), "robots", arm_id_str, f"{arm_id_str}.urdf.xacro")
    robot_description_config = xacro.process_file(franka_xacro_file, mappings={
        "arm_id": arm_id_str, "hand": load_gripper_str, "ros2_control": "true",
        "gazebo": "true", "ee_id": franka_hand_str, "gazebo_effort": "true"})
    robot_description = {"robot_description": robot_description_config.toxml()}
    return [Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher", output="both",
                 parameters=[robot_description, {"use_sim_time": True}])]

def generate_launch_description():
    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    franka_hand = LaunchConfiguration("franka_hand")
    
    franka_gazebo_bringup = get_package_share_directory("franka_gazebo_bringup")
    controller_config = os.path.join(franka_gazebo_bringup, "config", "franka_gazebo_controllers.yaml")
    ft_override_config = os.path.join(franka_gazebo_bringup, "config", "ft_sensor_override.yaml")
    world = os.path.join(franka_gazebo_bringup, "worlds", "direct_force_world.sdf")
    
    # Setup Gazebo resource path for meshes
    desc_share_parent = os.path.dirname(get_package_share_directory("franka_description"))
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_path = (existing + os.pathsep if existing else "") + desc_share_parent
    
    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_path),
        DeclareLaunchArgument("arm_id", default_value="fr3"),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("franka_hand", default_value="franka_hand"),
        
        # Gazebo with world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": f"{world} -r"}.items()),
        
        OpaqueFunction(function=get_robot_description, args=[arm_id, load_gripper, franka_hand]),
        
        Node(package="ros_gz_sim", executable="create",
             arguments=["-name", arm_id, "-topic", "/robot_description"], output="screen"),
        
        TimerAction(period=3.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "-c", "/controller_manager"])]),
        
        TimerAction(period=4.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["direct_force_example_controller", "-c", "/controller_manager",
                           "-p", controller_config, "-p", ft_override_config])]),
    ])
