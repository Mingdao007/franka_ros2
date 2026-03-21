#!/bin/bash
# Experiment C1 Ablation: {Jacobian, FT} × {alpha=0.05, alpha=1.0}
#
# 2×2 factorial design to separate sensing path from filter strength.
# circle_radius=0, enable_contact_cycling=true (3s period, 1cm liftoff)
#
# Expected outcomes:
#   Jacobian + 1.0  → likely unstable (proves Jacobian NEEDS filtering)
#   Jacobian + 0.05 → stable but slow transient
#   FT + 1.0        → stable and fast transient
#   FT + 0.05       → stable but artificially slow (proves filtering hurts)

set -e

WORKSPACE=/home/andy/franka_ros2_ws
SRC=${WORKSPACE}/src
CONFIG=${SRC}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
LOG_FILE=${WORKSPACE}/hybrid_circle_force_log.csv
RESULT_DIR=${SRC}/results/exp_c1_ablation
DURATION=25

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

# ---- Set up contact cycling ----
sed -i 's/circle_radius: .*/circle_radius: 0.0/' "${CONFIG}"
if ! grep -q "enable_contact_cycling" "${CONFIG}"; then
  sed -i '/descent_contact_force:/a\      enable_contact_cycling: true\n      contact_cycle_period: 3.0\n      liftoff_height: 0.01\n      liftoff_speed: 0.03' "${CONFIG}"
else
  sed -i 's/enable_contact_cycling: .*/enable_contact_cycling: true/' "${CONFIG}"
fi

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
  local use_ft="$1"
  local alpha_jac="$2"
  local alpha_ft="$3"
  local label="$4"
  local outfile="${RESULT_DIR}/${label}.csv"

  echo ""
  echo "======== ${label} ========"

  sed -i "s/use_ft_sensor: .*/use_ft_sensor: ${use_ft}/" "${CONFIG}"
  sed -i "s/force_filter_alpha_jacobian: .*/force_filter_alpha_jacobian: ${alpha_jac}/" "${CONFIG}"
  sed -i "s/force_filter_alpha_ft: .*/force_filter_alpha_ft: ${alpha_ft}/" "${CONFIG}"

  cleanup
  rm -f "${LOG_FILE}"

  timeout 60 ros2 launch franka_gazebo_bringup gazebo_hybrid_circle_force.launch.py \
    use_rviz:=false \
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
    local lines=$(wc -l < "${outfile}")
    local liftoffs=$(grep -c ",2$" "${outfile}" 2>/dev/null || echo 0)
    echo "  Saved: ${lines} lines, liftoff samples: ${liftoffs}"
  else
    echo "  NO LOG (likely crashed)"
    echo "CRASHED" > "${outfile}"
  fi

  kill $lpid 2>/dev/null || true
  sleep 1
  cleanup
  wait $lpid 2>/dev/null || true
}

# ---- 2×2 ablation ----
run_one "false" "0.05" "1.0"  "jac_alpha005"
run_one "false" "1.0"  "1.0"  "jac_alpha100"
run_one "true"  "0.05" "0.05" "ft_alpha005"
run_one "true"  "0.05" "1.0"  "ft_alpha100"

# ---- Analysis ----
echo ""
echo "======== C1 Ablation: 2×2 Results ========"
python3 - "${RESULT_DIR}" <<'PYEOF'
import csv, math, sys, os

base = sys.argv[1]

def analyze(path, label):
    if not os.path.exists(path):
        print(f"  {label:35s}  MISSING")
        return None
    with open(path) as f:
        first = f.readline().strip()
    if first == "CRASHED":
        print(f"  {label:35s}  CRASHED / NO DATA")
        return {'crashed': True}

    data = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row['time'])
                phase = int(row.get('phase', ''))
                fz_des = float(row['fz_des'])
                fz_meas = float(row['fz_meas'])
            except:
                continue
            data.append((t, phase, fz_des, fz_meas))

    if not data:
        print(f"  {label:35s}  EMPTY")
        return None

    # Circle-phase IAE
    fz_iae = 0.0
    n = 0
    t_prev = None
    for t, phase, fz_des, fz_meas in data:
        if phase != 1:
            t_prev = t
            continue
        dt = (t - t_prev) if t_prev is not None else 0.001
        t_prev = t
        fz_iae += abs(fz_des - fz_meas) * dt
        n += 1

    # Contact transitions + settle time
    transitions = 0
    settle_times = []
    for i in range(1, len(data)):
        if data[i-1][1] == 0 and data[i][1] == 1:
            transitions += 1
            t_contact = data[i][0]
            for j in range(i, min(i+2000, len(data))):
                if data[j][0] - t_contact > 2.0:
                    break
                if abs(data[j][2] - data[j][3]) < 0.5:
                    settle_times.append(data[j][0] - t_contact)
                    break

    dur = n * 0.001 if n > 0 else 1.0
    avg_settle = sum(settle_times)/len(settle_times)*1000 if settle_times else float('nan')

    print(f"  {label:35s}  IAE/s={fz_iae/dur:.4f} N  settle={avg_settle:.0f}ms  contacts={transitions}")
    return {'iae': fz_iae/dur, 'settle': avg_settle, 'contacts': transitions}

print()
print(f"  {'Config':35s}  {'Fz IAE/s':>10}  {'Settle':>10}  {'Contacts':>10}")
print("  " + "-" * 70)
for name, label in [
    ("jac_alpha005", "Jacobian + alpha=0.05"),
    ("jac_alpha100", "Jacobian + alpha=1.0"),
    ("ft_alpha005",  "FT Sensor + alpha=0.05"),
    ("ft_alpha100",  "FT Sensor + alpha=1.0"),
]:
    analyze(os.path.join(base, f"{name}.csv"), label)

print()
print("  If Jacobian+1.0 crashed: proves Jacobian NEEDS heavy filtering")
print("  If FT+0.05 is slower than FT+1.0: proves filtering hurts transients")

PYEOF

echo ""
echo "[DONE] Results in: ${RESULT_DIR}"
