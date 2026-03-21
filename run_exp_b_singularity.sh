#!/bin/bash
# Experiment B: Near-singularity Jacobian degradation test
# Spawns FR3 at cond(J)=30 config (vs baseline cond=9.4)
# q = [0.0, 0.4, 0.0, -0.9, 0.1, 0.5, 0.785]
# All links clear of table (min 19cm clearance)
#
# Runs: 1) Jacobian mode  2) FT sensor mode  (same config)
# Collects 20s of circle-phase data each, then compares IAE.

set -e

WORKSPACE=/home/andy/franka_ros2_ws
SRC=${WORKSPACE}/src
CONFIG=${SRC}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
LOG_FILE=${WORKSPACE}/hybrid_circle_force_log.csv
RESULT_DIR=${SRC}/results/exp_b_singularity
INIT_JOINTS="0.0,0.4,0.0,-0.9,0.1,0.5,0.785"
DURATION=20

mkdir -p "${RESULT_DIR}"

# ---- Build ----
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo "[BUILD] Building..."
colcon build \
  --build-base ${WORKSPACE}/build \
  --install-base ${WORKSPACE}/install \
  --base-paths ${SRC} \
  --packages-select franka_gazebo_bringup franka_example_controllers \
  > "${RESULT_DIR}/build.log" 2>&1
source ${WORKSPACE}/install/setup.bash
echo "[BUILD] Done"

# ---- Cleanup function ----
cleanup() {
  pkill -f "ruby.*gz\|gz sim\|ign\|gazebo\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" 2>/dev/null || true
  sleep 0.5
  pkill -9 -f "ruby.*gz\|gz sim\|ign\|gazebo\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" 2>/dev/null || true
  sleep 0.3
}
trap cleanup EXIT INT TERM

# ---- Run one experiment ----
run_one() {
  local use_ft="$1"
  local label="$2"
  local outfile="${RESULT_DIR}/${label}.csv"

  echo ""
  echo "======== ${label} (use_ft_sensor=${use_ft}) ========"

  # Set use_ft_sensor in config
  sed -i "s/use_ft_sensor: .*/use_ft_sensor: ${use_ft}/" "${CONFIG}"

  cleanup
  rm -f "${LOG_FILE}"

  # Launch with near-singular initial joints
  timeout 55 ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py \
    use_rviz:=false \
    "initial_joint_positions:=${INIT_JOINTS}" \
    > "${RESULT_DIR}/${label}_launch.log" 2>&1 &
  local lpid=$!

  # Wait for controller
  for i in $(seq 1 25); do
    sleep 1
    if ros2 control list_controllers 2>/dev/null | grep -qE "hybrid_circle_force_controller.*active"; then
      echo "  Controller active after ${i}s"
      sleep ${DURATION}
      echo "  Collection done (${DURATION}s)"
      break
    fi
    if [ $i -eq 25 ]; then
      echo "  TIMEOUT: controller never activated"
      kill $lpid 2>/dev/null || true
      cleanup
      return 1
    fi
  done

  # Save data
  if [[ -f "${LOG_FILE}" ]]; then
    cp "${LOG_FILE}" "${outfile}"
    local lines=$(wc -l < "${outfile}")
    local circle=$(grep -c ",1$" "${outfile}" 2>/dev/null || echo 0)
    echo "  Saved: ${lines} lines, ${circle} circle-phase samples"
  else
    echo "  ERROR: no log file"
  fi

  kill $lpid 2>/dev/null || true
  sleep 1
  cleanup
  wait $lpid 2>/dev/null || true
}

# ---- Run both ----
run_one "false" "jacobian_sing"
run_one "true"  "ft_sensor_sing"

# Restore config
sed -i "s/use_ft_sensor: .*/use_ft_sensor: false/" "${CONFIG}"

# ---- Analysis ----
echo ""
echo "======== Analysis (IAE, steady-state t>3s) ========"
python3 - "${RESULT_DIR}" <<'PYEOF'
import csv, math, sys, os

base = sys.argv[1]

def iae_analysis(csv_path, label):
    fz_iae, exy_iae, n = 0.0, 0.0, 0
    t_prev = None
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        t0 = None
        for row in reader:
            try: phase = int(row.get('phase', ''))
            except: continue
            if phase != 1: continue
            try:
                t = float(row['time']); fz_des = float(row['fz_des'])
                fz_meas = float(row['fz_meas']); ex = float(row['ex']); ey = float(row['ey'])
            except: continue
            if t0 is None: t0 = t
            if t - t0 < 3.0:
                t_prev = t
                continue
            dt = (t - t_prev) if t_prev is not None else 0.001
            t_prev = t
            fz_iae += abs(fz_des - fz_meas) * dt
            exy_iae += math.sqrt(ex**2 + ey**2) * dt
            n += 1
    if n == 0:
        print(f"  {label}: NO DATA")
        return
    dur = n * 0.001
    print(f"  {label}  ({dur:.1f}s)")
    print(f"    Fz  IAE/s: {fz_iae/dur:.4f} N")
    print(f"    eXY IAE/s: {exy_iae/dur*1000:.4f} mm")

for name, label in [("jacobian_sing", "Jacobian (near-singular)"),
                     ("ft_sensor_sing", "FT Sensor (near-singular)")]:
    p = os.path.join(base, f"{name}.csv")
    if os.path.exists(p):
        iae_analysis(p, label)

# Also compare with baseline if available
for name, label in [("../exp_d_ft_vs_jacobian/jacobian", "Jacobian (baseline)"),
                     ("../exp_d_ft_vs_jacobian/ft_sensor", "FT Sensor (baseline)")]:
    p = os.path.join(base, f"{name}.csv")
    if os.path.exists(p):
        iae_analysis(p, label)
PYEOF

echo ""
echo "[DONE] Results in: ${RESULT_DIR}"
