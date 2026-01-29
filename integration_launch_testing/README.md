### Run integration tests

The [integration_launch_testing](./integration_launch_testing) package provides functional integration tests for Franka controllers and the gripper. These tests require **real hardware**: a Franka robot (e.g. FR3) on the network. The default `robot_ip` is `172.16.0.2`. Use `--skip-gripper` if no gripper is attached.

**Note:** To build this package and activate the smoke tests, you need to enable the `BUILD_TESTING` CMake flag when building the workspace:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

**Run all integration tests:**

```bash
ros2 run integration_launch_testing run_all_integration_tests --robot-ip <ROBOT_IP>
```

Options: `--skip-gripper`, `--timeout <SEC>`, `-v` / `--verbose`.

**Run a single controller test:**

```bash
ros2 run integration_launch_testing test_controller controller_name:=<CONTROLLER_NAME> robot_ip:=<ROBOT_IP>
```

Example controllers: `joint_impedance_example_controller`, `cartesian_pose_example_controller`, `move_to_start_example_controller`, and others from `controllers.yaml`.

**Run the gripper test only:**

```bash
launch_test install/integration_launch_testing/share/integration_launch_testing/test/test_franka_gripper_position.py robot_ip:=<ROBOT_IP>
```