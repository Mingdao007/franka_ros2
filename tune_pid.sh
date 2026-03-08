#!/bin/bash
# PID Tuning Helper Script for Force Control
# Usage: ./tune_pid.sh <kp> <ki> <kd>

if [ $# -ne 3 ]; then
    echo "Usage: $0 <kp> <ki> <kd>"
    echo "Example: $0 0.02 0.001 0.01"
    exit 1
fi

KP=$1
KI=$2
KD=$3

CONFIG_FILE="/home/andy/franka_ros2_ws/src/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"

echo "========================================="
echo "PID Tuning: kp=$KP, ki=$KI, kd=$KD"
echo "========================================="

# Backup original file
if [ ! -f "${CONFIG_FILE}.backup" ]; then
    cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"
    echo "Created backup: ${CONFIG_FILE}.backup"
fi

# Update PID parameters
sed -i "s/kp: [0-9.]\+/kp: $KP/" "$CONFIG_FILE"
sed -i "s/ki: [0-9.]\+/ki: $KI/" "$CONFIG_FILE"
sed -i "s/kd: [0-9.]\+/kd: $KD/" "$CONFIG_FILE"

echo "Updated parameters in $CONFIG_FILE"
echo ""
echo "Now launching Gazebo with new parameters..."
echo "Watch the commanded_wrench topic for steady-state error and oscillation."
echo ""
echo "Press Ctrl+C after collecting ~10 seconds of data."
echo ""

# Kill existing Gazebo
pkill -f gazebo 2>/dev/null
sleep 2

# Launch Gazebo
cd /home/andy/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch in background
ros2 launch franka_gazebo_bringup gazebo_direct_force_controller_example.launch.py use_rviz:=false > /tmp/tune_output.log 2>&1 &
LAUNCH_PID=$!

# Wait for startup
echo "Waiting for system to start (8 seconds)..."
sleep 8

# Collect data
echo ""
echo "Collecting data for 10 seconds..."
echo ""

timeout 10 ros2 topic echo /direct_force_example_controller/commanded_wrench --no-arr 2>&1 | tee /tmp/commanded_wrench_kp${KP}_ki${KI}_kd${KD}.log | grep -A 3 "force:" | grep "z:" | awk '{sum+=$2; sumsq+=$2*$2; count++} END {
    if (count > 0) {
        mean = sum/count;
        variance = (sumsq/count) - (mean*mean);
        stddev = sqrt(variance > 0 ? variance : 0);
        printf "\n========== Results ==========\n";
        printf "Samples: %d\n", count;
        printf "Mean: %.6f N (target: -2.0 N)\n", mean;
        printf "Std Dev: %.6f N (stability)\n", stddev;
        printf "Steady-state error: %.6f N (%.2f%%)\n", mean - (-2.0), abs((mean - (-2.0))/2.0)*100;
        printf "============================\n";
    }
}'

# Cleanup
echo ""
echo "Stopping Gazebo..."
pkill -f gazebo
wait $LAUNCH_PID 2>/dev/null

echo ""
echo "Log saved to: /tmp/commanded_wrench_kp${KP}_ki${KI}_kd${KD}.log"
echo ""
echo "To restore original config: cp ${CONFIG_FILE}.backup $CONFIG_FILE"
