import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    TimerAction,
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
        arm_id_str + ".urdf.xacro",
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
        parameters=[robot_description, {"use_sim_time": True}],
    )

    return [robot_state_publisher]


def generate_launch_description():
    load_gripper_name = "load_gripper"
    franka_hand_name = "franka_hand"
    arm_id_name = "arm_id"
    use_rviz_name = "use_rviz"
    switch_delay_s_name = "switch_delay_s"

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    use_rviz = LaunchConfiguration(use_rviz_name)
    switch_delay_s = LaunchConfiguration(switch_delay_s_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name, default_value="true",
        description="true/false for activating the gripper",
    )
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name, default_value="franka_hand",
        description="Default value: franka_hand",
    )
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name, default_value="fr3",
        description="Available values: fr3, fp3 and fer",
    )
    use_rviz_launch_argument = DeclareLaunchArgument(
        use_rviz_name, default_value="false",
        description="Start RViz for visualization (true/false).",
    )
    switch_delay_s_launch_argument = DeclareLaunchArgument(
        switch_delay_s_name, default_value="8.0",
        description="Delay (seconds) after move_to_start is loaded to switch to direct_force.",
    )

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand],
    )

    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(get_package_share_directory("franka_description"))
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = os.path.join(
        get_package_share_directory("franka_gazebo_bringup"),
        "worlds",
        "direct_force_world.sdf",
    )
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"{world} -r"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", arm_id, "-topic", "/robot_description"],
        output="screen",
    )

    rviz_file = os.path.join(get_package_share_directory("franka_description"), "rviz", "visualize_franka.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file, "-f", "world"],
        condition=IfCondition(use_rviz),
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )

    load_direct_force_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "inactive", "direct_force_example_controller"],
        output="screen",
    )

    load_move_to_start_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "move_to_start_example_controller"],
        output="screen",
    )

    switch_to_direct_force = ExecuteProcess(
        cmd=[
            "ros2", "control", "switch_controllers",
            "--deactivate", "move_to_start_example_controller",
            "--activate", "direct_force_example_controller",
            "--strict",
        ],
        output="screen",
    )

    delayed_switch = TimerAction(period=switch_delay_s, actions=[switch_to_direct_force])

    return LaunchDescription(
        [
            load_gripper_launch_argument,
            franka_hand_launch_argument,
            arm_id_launch_argument,
            use_rviz_launch_argument,
            switch_delay_s_launch_argument,

            gazebo_world,
            robot_state_publisher,
            rviz,
            spawn,

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_direct_force_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_direct_force_controller,
                    on_exit=[load_move_to_start_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_move_to_start_controller,
                    on_exit=[delayed_switch],
                )
            ),
        ]
    )
