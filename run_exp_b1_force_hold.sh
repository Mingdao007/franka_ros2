#!/bin/bash
# Experiment B1: Fixed-point constant force hold — isolate sensing path
#
# circle_radius=0: no XY trajectory, pure Fz force control
# Compares Jacobian vs FT sensor at two postures:
#   - baseline:  q=[0, -0.3, 0, -2.0, 0, 1.7, 0.785]  cond(J)=9.4
#   - near-sing: q=[0, 0.4, 0, -0.9, 0.1, 0.5, 0.785]  cond(J)=30
# use_current_pose_as_center=true (each posture holds its own EE position)
#
# 4 runs total: {baseline, near-sing} × {Jacobian, FT sensor}

set -e

WORKSPACE=/home/andy/franka_ros2_ws
SRC=${WORKSPACE}/src
CONFIG=${SRC}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
LOG_FILE=${WORKSPACE}/hybrid_circle_force_log.csv
RESULT_DIR=${SRC}/results/exp_b1_force_hold
DURATION=20

Q_BASELINE=""
Q_NEARSING="0.0,0.4,0.0,-0.9,0.1,0.5,0.785"

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

# ---- Save original config ----
cp "${CONFIG}" "${RESULT_DIR}/config_backup.yaml"

# ---- Set circle_radius=0 for force-hold ----
sed -i 's/circle_radius: .*/circle_radius: 0.0/' "${CONFIG}"

# ---- Cleanup ----
cleanup() {
  pkill -f "ruby.*gz\|gz sim\|ign\|gazebo\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" 2>/dev/null || true
  sleep 0.5
  pkill -9 -f "ruby.*gz\|gz sim\|ign\|gazebo\|parameter_bridge\|robot_state_publisher\|controller_manager\|spawner" 2>/dev/null || true
  sleep 0.3
}
trap 'cp "${RESULT_DIR}/config_backup.yaml" "${CONFIG}"; cleanup' EXIT INT TERM

# ---- Run one experiment ----
run_one() {
  local init_joints="$1"
  local use_ft="$2"
  local label="$3"
  local outfile="${RESULT_DIR}/${label}.csv"

  echo ""
  echo "======== ${label} ========"

  sed -i "s/use_ft_sensor: .*/use_ft_sensor: ${use_ft}/" "${CONFIG}"

  cleanup
  rm -f "${LOG_FILE}"

  local launch_args="use_rviz:=false"
  if [[ -n "${init_joints}" ]]; then
    launch_args="${launch_args} initial_joint_positions:=${init_joints}"
  fi

  timeout 55 ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py \
    ${launch_args} \
    > "${RESULT_DIR}/${label}_launch.log" 2>&1 &
  local lpid=$!

  for i in $(seq 1 25); do
    sleep 1
    if ros2 control list_controllers 2>/dev/null | grep -qE "hybrid_circle_force_controller.*active"; then
      echo "  Active after ${i}s — collecting ${DURATION}s"
      sleep ${DURATION}
      echo "  Done"
      break
    fi
    if [ $i -eq 25 ]; then
      echo "  TIMEOUT"
      kill $lpid 2>/dev/null || true
      cleanup
      return 1
    fi
  done

  if [[ -f "${LOG_FILE}" ]]; then
    cp "${LOG_FILE}" "${outfile}"
    echo "  Saved: $(wc -l < "${outfile}") lines"
  else
    echo "  ERROR: no log file"
  fi

  kill $lpid 2>/dev/null || true
  sleep 1
  cleanup
  wait $lpid 2>/dev/null || true
}

# ---- Run all 4 combinations ----
run_one ""             "false" "baseline_jacobian"
run_one ""             "true"  "baseline_ft"
run_one "${Q_NEARSING}" "false" "nearsing_jacobian"
run_one "${Q_NEARSING}" "true"  "nearsing_ft"

# ---- Analysis ----
echo ""
echo "======== B1: Force Hold IAE Analysis (t>3s) ========"
python3 - "${RESULT_DIR}" <<'PYEOF'
import csv, math, sys, os

base = sys.argv[1]

def iae(path, label):
    fz_iae, n = 0.0, 0
    t_prev = None
    with open(path) as f:
        reader = csv.DictReader(f)
        t0 = None
        for row in reader:
            try: phase = int(row.get('phase', ''))
            except: continue
            if phase != 1: continue
            try:
                t = float(row['time'])
                fz_des = float(row['fz_des'])
                fz_meas = float(row['fz_meas'])
            except: continue
            if t0 is None: t0 = t
            if t - t0 < 3.0:
                t_prev = t
                continue
            dt = (t - t_prev) if t_prev is not None else 0.001
            t_prev = t
            fz_iae += abs(fz_des - fz_meas) * dt
            n += 1
    if n == 0:
        print(f"  {label}: NO DATA")
        return None
    dur = n * 0.001
    v = fz_iae / dur
    print(f"  {label:40s}  Fz IAE/s: {v:.4f} N  ({dur:.1f}s)")
    return v

print()
results = {}
for name, label in [
    ("baseline_jacobian", "Jacobian  / baseline (cond=9.4) "),
    ("baseline_ft",       "FT Sensor / baseline (cond=9.4) "),
    ("nearsing_jacobian", "Jacobian  / near-sing (cond=30) "),
    ("nearsing_ft",       "FT Sensor / near-sing (cond=30) "),
]:
    p = os.path.join(base, f"{name}.csv")
    if os.path.exists(p):
        results[name] = iae(p, label)

if all(v is not None for v in results.values()):
    print()
    jb, jn = results["baseline_jacobian"], results["nearsing_jacobian"]
    fb, fn = results["baseline_ft"], results["nearsing_ft"]
    print(f"  Jacobian  degradation: {jb:.4f} → {jn:.4f} N  ({(jn-jb)/jb*100:+.1f}%)")
    print(f"  FT Sensor degradation: {fb:.4f} → {fn:.4f} N  ({(fn-fb)/fb*100:+.1f}%)")
    print()
    print(f"  Baseline gap (FT-Jac): {(fb-jb)/jb*100:+.1f}%")
    print(f"  Near-sing gap (FT-Jac): {(fn-jn)/jn*100:+.1f}%")

PYEOF

echo ""
echo "[DONE] Results in: ${RESULT_DIR}"
