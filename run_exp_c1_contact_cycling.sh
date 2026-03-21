#!/bin/bash
# Experiment C1: Contact cycling — transient response comparison
#
# circle_radius=0 (fixed-point force hold)
# enable_contact_cycling=true: periodic liftoff/re-contact every 3s
# Jacobian uses alpha=0.05 (needs heavy filtering)
# FT sensor uses alpha=1.0 (clean signal, no filtering needed)
#
# Compares transient metrics: Fz IAE/s, force spikes at contact transitions

set -e

WORKSPACE=/home/andy/franka_ros2_ws
SRC=${WORKSPACE}/src
CONFIG=${SRC}/franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
LOG_FILE=${WORKSPACE}/hybrid_circle_force_log.csv
RESULT_DIR=${SRC}/results/exp_c1_contact_cycling
DURATION=25  # longer to capture multiple contact cycles

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

# ---- Enable contact cycling, force hold ----
sed -i 's/circle_radius: .*/circle_radius: 0.0/' "${CONFIG}"

# Add contact cycling params if not present
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
  local label="$2"
  local outfile="${RESULT_DIR}/${label}.csv"

  echo ""
  echo "======== ${label} ========"

  sed -i "s/use_ft_sensor: .*/use_ft_sensor: ${use_ft}/" "${CONFIG}"

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
    echo "  Saved: ${lines} lines"
    # Count phase transitions
    local liftoffs=$(grep -c ",2$" "${outfile}" 2>/dev/null || echo 0)
    echo "  Liftoff samples: ${liftoffs}"
  else
    echo "  ERROR: no log file"
  fi

  kill $lpid 2>/dev/null || true
  sleep 1
  cleanup
  wait $lpid 2>/dev/null || true
}

# ---- Run both ----
run_one "false" "jacobian_cycling"
run_one "true"  "ft_sensor_cycling"

# ---- Analysis ----
echo ""
echo "======== C1: Contact Cycling Analysis ========"
python3 - "${RESULT_DIR}" <<'PYEOF'
import csv, math, sys, os

base = sys.argv[1]

def analyze(path, label):
    # Collect all data points with phase info
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
            data.append({'t': t, 'phase': phase, 'fz_des': fz_des, 'fz_meas': fz_meas})

    if not data:
        print(f"  {label}: NO DATA")
        return

    # Overall Fz IAE/s (circle phase only, skip first 3s)
    fz_iae = 0.0
    n_circle = 0
    t0_circle = None
    t_prev = None
    for d in data:
        if d['phase'] == 1:
            if t0_circle is None:
                t0_circle = d['t']
            if d['t'] - t0_circle < 1.0:  # skip first 1s of EACH circle entry
                t_prev = d['t']
                continue
            dt = (d['t'] - t_prev) if t_prev is not None else 0.001
            t_prev = d['t']
            fz_iae += abs(d['fz_des'] - d['fz_meas']) * dt
            n_circle += 1

    # Count contact transitions
    transitions = 0
    max_spike = 0.0
    for i in range(1, len(data)):
        prev_phase = data[i-1]['phase']
        curr_phase = data[i]['phase']
        # Descent→Circle = contact event
        if prev_phase == 0 and curr_phase == 1:
            transitions += 1
            # Find max force spike in next 500ms
            t_contact = data[i]['t']
            for j in range(i, min(i+500, len(data))):
                if data[j]['t'] - t_contact > 0.5:
                    break
                spike = abs(data[j]['fz_des'] - data[j]['fz_meas'])
                max_spike = max(max_spike, spike)

    dur = n_circle * 0.001 if n_circle > 0 else 1.0
    print(f"  {label}")
    print(f"    Circle-phase Fz IAE/s: {fz_iae/dur:.4f} N")
    print(f"    Contact transitions:   {transitions}")
    print(f"    Max spike at contact:  {max_spike:.4f} N")
    print(f"    Total samples:         {len(data)}")

for name, label in [
    ("jacobian_cycling",   "Jacobian  (alpha=0.05)"),
    ("ft_sensor_cycling",  "FT Sensor (alpha=1.0) "),
]:
    p = os.path.join(base, f"{name}.csv")
    if os.path.exists(p):
        analyze(p, label)

PYEOF

echo ""
echo "[DONE] Results in: ${RESULT_DIR}"
