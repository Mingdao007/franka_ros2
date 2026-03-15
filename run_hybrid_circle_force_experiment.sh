#!/usr/bin/env bash
set -euo pipefail

# Run baseline hybrid circle + constant-force experiments with automatic cleanup.
# Always launches Gazebo with GUI (no headless mode).

WORKSPACE_ROOT="/home/andy/franka_ros2_ws"
SRC_ROOT="${WORKSPACE_ROOT}/src"
RESULT_BASE="${SRC_ROOT}/results/hybrid_circle_force"
CONTROLLER_NAME="hybrid_circle_force_controller"
CONTROLLER_LOG_PATH="${WORKSPACE_ROOT}/hybrid_circle_force_log.csv"
# Top-level config — runtime file (symlink-install makes install/share point here).
CONFIG_FILE="${SRC_ROOT}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml"

REPEATS="${REPEATS:-1}"
DURATION="${DURATION:-15}"
POLL_TIMEOUT="${POLL_TIMEOUT:-20}"
RUN_TIMEOUT="${RUN_TIMEOUT:-50}"
FREQUENCY="${FREQUENCY:-}"
USE_RVIZ="${USE_RVIZ:-true}"
RESULT_DIR_OVERRIDE="${RESULT_DIR_OVERRIDE:-}"

STAMP="$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="${RESULT_DIR_OVERRIDE:-${RESULT_BASE}/${STAMP}}"
mkdir -p "${RESULT_DIR}"

# Override circle_frequency in YAML if FREQUENCY is set.
if [[ -n "${FREQUENCY}" ]]; then
  sed -i "/^    circle_frequency:/s/: [0-9.]\+/: ${FREQUENCY}/" "${CONFIG_FILE}"
  echo "[INFO] Overrode circle_frequency to ${FREQUENCY} Hz in ${CONFIG_FILE}"
fi

ts() { date +%s; }

cleanup_sim() {
  pkill -f "ruby.*gz|gz sim|ign|gazebo|parameter_bridge|robot_state_publisher|controller_manager|spawner" 2>/dev/null || true
  sleep 0.3
  pkill -9 -f "ruby.*gz|gz sim|ign|gazebo|parameter_bridge|robot_state_publisher|controller_manager|spawner" 2>/dev/null || true
  sleep 0.2
}

# Ensure cleanup on any exit (normal, Ctrl+C, or signal).
trap cleanup_sim EXIT INT TERM

wait_for_controller_active() {
  local timeout_s="$1"
  local poll_interval="${2:-0.5}"
  local start_ts
  start_ts="$(ts)"

  while true; do
    # Early exit if launch process has died (e.g. Gazebo crash).
    if [[ -n "${launch_pid:-}" ]] && ! kill -0 "${launch_pid}" 2>/dev/null; then
      echo "[ERROR] Launch process (pid=${launch_pid}) exited prematurely"
      return 1
    fi
    if ros2 control list_controllers 2>/dev/null | grep -qE "${CONTROLLER_NAME}.*active"; then
      return 0
    fi
    if (( $(ts) - start_ts > timeout_s )); then
      return 1
    fi
    sleep "${poll_interval}"
  done
}

echo "[INFO] Results: ${RESULT_DIR}"
echo "[INFO] Repeats=${REPEATS}, Duration=${DURATION}s, PollTimeout=${POLL_TIMEOUT}s, RunTimeout=${RUN_TIMEOUT}s"
[[ -n "${FREQUENCY}" ]] && echo "[INFO] Frequency=${FREQUENCY} Hz"
echo ""

run_pass=0
run_fail=0

for run_idx in $(seq 1 "${REPEATS}"); do
  run_dir="${RESULT_DIR}/run_${run_idx}"
  mkdir -p "${run_dir}"

  echo "============================================================"
  echo "[RUN ${run_idx}/${REPEATS}]"
  echo "============================================================"

  t0="$(ts)"
  cleanup_sim

  # Remove stale controller log so we get fresh data per run.
  rm -f "${CONTROLLER_LOG_PATH}"

  launch_log="${run_dir}/launch.log"
  timeout "${RUN_TIMEOUT}" \
    ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py \
    use_rviz:="${USE_RVIZ}" >"${launch_log}" 2>&1 &
  launch_pid=$!

  # Poll immediately — no blind settle sleep.
  t1="$(ts)"
  echo "[TIMING] Polling for controller activation (timeout=${POLL_TIMEOUT}s)..."

  if ! wait_for_controller_active "${POLL_TIMEOUT}"; then
    t2="$(ts)"
    echo "[ERROR] Controller did not become active (waited $((t2 - t1))s)" | tee -a "${run_dir}/run_status.txt"
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
    cleanup_sim
    run_fail=$((run_fail + 1))
    continue
  fi
  t2="$(ts)"
  echo "[TIMING] Controller active (+$((t2 - t0))s total, $((t2 - t1))s polling)"

  wrench_csv="${run_dir}/wrench.csv"
  collector_log="${run_dir}/collector.log"
  echo "[TIMING] Starting data collection for ${DURATION}s..."
  python3 "${SRC_ROOT}/collect_force_data.py" \
    --duration "${DURATION}" \
    --output "${wrench_csv}" \
    --controller-name "${CONTROLLER_NAME}" >"${collector_log}" 2>&1 || true
  t3="$(ts)"
  echo "[TIMING] Collection done (+$((t3 - t0))s total)"

  # Graceful stop, brief wait, then hard cleanup.
  kill -INT "${launch_pid}" 2>/dev/null || true
  sleep 2
  cleanup_sim
  wait "${launch_pid}" 2>/dev/null || true
  t4="$(ts)"
  echo "[TIMING] Shutdown complete (+$((t4 - t0))s total for run ${run_idx})"

  if [[ -f "${CONTROLLER_LOG_PATH}" ]]; then
    cp "${CONTROLLER_LOG_PATH}" "${run_dir}/internal.csv"
  else
    echo "[WARN] Missing controller internal log: ${CONTROLLER_LOG_PATH}" | tee -a "${run_dir}/run_status.txt"
  fi

  # Snapshot normalization parameters from the config actually used.
  meta_radius=$(grep 'circle_radius:' "${CONFIG_FILE}" | head -1 | awk '{print $2}')
  meta_force=$(grep 'force_desired:' "${CONFIG_FILE}" | head -1 | awk '{print $2}')

  cat >"${run_dir}/run_meta.txt" <<EOF
run_index=${run_idx}
duration=${DURATION}
controller=${CONTROLLER_NAME}
poll_timeout=${POLL_TIMEOUT}
poll_time=$((t2 - t1))
collection_time=$((t3 - t2))
total_wall_time=$((t4 - t0))
circle_frequency=${FREQUENCY:-default}
circle_radius=${meta_radius:-}
force_desired=${meta_force:-}
EOF

  if [[ -f "${wrench_csv}" ]] && [[ -s "${wrench_csv}" ]]; then
    run_pass=$((run_pass + 1))
    echo "[OK] Run ${run_idx} completed with data."
  else
    run_fail=$((run_fail + 1))
    echo "[WARN] Run ${run_idx} completed but wrench.csv is missing or empty." | tee -a "${run_dir}/run_status.txt"
  fi
  echo ""
done

echo "============================================================"
echo "[SUMMARY] ${run_pass}/${REPEATS} runs produced data, ${run_fail} failed/empty"
echo "============================================================"

if [[ "${run_pass}" -gt 0 ]]; then
  python3 "${SRC_ROOT}/generate_hybrid_experiment_report.py" --results-dir "${RESULT_DIR}" || \
    echo "[ERROR] Report generation failed; see Python traceback above."
else
  echo "[ERROR] No successful runs — skipping report generation."
fi

echo "[DONE] Experiment batch complete: ${RESULT_DIR}"
