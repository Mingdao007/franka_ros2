#!/usr/bin/env bash
set -euo pipefail

# OOD validation experiment: r=0.05m, everything else same as nominal.
# Uses the same experiment runner but overrides result directory and circle_radius.
# Keeps nominal script untouched.

WORKSPACE_ROOT="/home/andy/franka_ros2_ws"
SRC_ROOT="${WORKSPACE_ROOT}/src"
CONFIG_FILE="${SRC_ROOT}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
OOD_RESULT_BASE="${SRC_ROOT}/results/hybrid_circle_force_ood_r005"

# Backup original radius, set to 0.05
ORIG_RADIUS=$(grep 'circle_radius:' "${CONFIG_FILE}" | head -1 | awk '{print $2}')
sed -i "s/circle_radius: ${ORIG_RADIUS}/circle_radius: 0.05/" "${CONFIG_FILE}"
echo "[OOD] circle_radius: ${ORIG_RADIUS} -> 0.05"

STAMP="$(date +%Y%m%d_%H%M%S)"
export RESULT_DIR_OVERRIDE="${OOD_RESULT_BASE}/${STAMP}"

# Run the standard experiment script
bash "${SRC_ROOT}/run_hybrid_circle_force_experiment.sh"

# Restore original radius
sed -i "s/circle_radius: 0.05/circle_radius: ${ORIG_RADIUS}/" "${CONFIG_FILE}"
echo "[OOD] circle_radius restored to ${ORIG_RADIUS}"
