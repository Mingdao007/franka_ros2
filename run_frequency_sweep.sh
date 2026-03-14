#!/usr/bin/env bash
set -euo pipefail

# Frequency sweep wrapper: runs the baseline experiment at multiple circle frequencies.
# Backs up YAML before sweep and restores on exit.

WORKSPACE_ROOT="/home/andy/franka_ros2_ws"
SRC_ROOT="${WORKSPACE_ROOT}/src"
RESULT_BASE="${SRC_ROOT}/results/hybrid_circle_force"
CONFIG_FILE="${SRC_ROOT}/franka_gazebo/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"
EXPERIMENT_SCRIPT="${SRC_ROOT}/run_hybrid_circle_force_experiment.sh"

FREQUENCIES="${FREQUENCIES:-0.1 0.2 0.3}"
REPEATS="${REPEATS:-3}"
DURATION="${DURATION:-15}"
POLL_TIMEOUT="${POLL_TIMEOUT:-20}"
RUN_TIMEOUT="${RUN_TIMEOUT:-50}"

STAMP="$(date +%Y%m%d_%H%M%S)"
SWEEP_DIR="${RESULT_BASE}/sweep_${STAMP}"
mkdir -p "${SWEEP_DIR}"

# Backup YAML and ensure restoration on exit.
BACKUP_FILE="${CONFIG_FILE}.sweep_backup"
cp "${CONFIG_FILE}" "${BACKUP_FILE}"

restore_yaml() {
  if [[ -f "${BACKUP_FILE}" ]]; then
    cp "${BACKUP_FILE}" "${CONFIG_FILE}"
    rm -f "${BACKUP_FILE}"
    echo "[INFO] YAML config restored from backup."
  fi
}
trap restore_yaml EXIT INT TERM

echo "============================================================"
echo "[SWEEP] Frequencies: ${FREQUENCIES}"
echo "[SWEEP] Repeats per frequency: ${REPEATS}"
echo "[SWEEP] Duration: ${DURATION}s"
echo "[SWEEP] Results: ${SWEEP_DIR}"
echo "============================================================"
echo ""

freq_pass=0
freq_fail=0
freq_count=$(echo ${FREQUENCIES} | wc -w)
freq_idx=0

for freq in ${FREQUENCIES}; do
  freq_idx=$((freq_idx + 1))
  freq_dir="${SWEEP_DIR}/freq_${freq}"

  echo "============================================================"
  echo "[SWEEP] Starting frequency=${freq} Hz (${freq_idx}/${freq_count})"
  echo "============================================================"

  FREQUENCY="${freq}" \
  RESULT_DIR_OVERRIDE="${freq_dir}" \
  REPEATS="${REPEATS}" \
  DURATION="${DURATION}" \
  POLL_TIMEOUT="${POLL_TIMEOUT}" \
  RUN_TIMEOUT="${RUN_TIMEOUT}" \
  bash "${EXPERIMENT_SCRIPT}" && freq_pass=$((freq_pass + 1)) || freq_fail=$((freq_fail + 1))

  echo ""
done

echo "============================================================"
echo "[SWEEP SUMMARY] ${freq_pass}/$((freq_pass + freq_fail)) frequencies completed successfully"
echo "============================================================"

# Generate cross-frequency comparison.
if [[ "${freq_pass}" -gt 0 ]]; then
  python3 "${SRC_ROOT}/generate_sweep_comparison.py" --sweep-dir "${SWEEP_DIR}" || \
    echo "[ERROR] Sweep comparison generation failed."
fi

echo "[DONE] Frequency sweep complete: ${SWEEP_DIR}"
