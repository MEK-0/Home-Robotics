#!/usr/bin/env bash

set +u

ROOT="$HOME/Home-Robotics"
ROS_WS="$ROOT/ros2_ws"

EXPECTED_CONTROLLERS=(
  "joint_state_broadcaster"
  "panda1_trajectory_controller"
  "panda2_trajectory_controller"
  "panda1_gripper_controller"
  "panda2_gripper_controller"
)

RUNS=10
READY_TIMEOUT=25

PASS_COUNT=0
FAIL_COUNT=0

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  LAUNCH_PID=""
  sleep 1
}

trap cleanup EXIT INT TERM

source "$ROOT/.ros_venv/bin/activate"
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"

check_controllers() {
  local output
  output="$(ros2 control list_controllers 2>/dev/null || true)"

  for controller in "${EXPECTED_CONTROLLERS[@]}"; do
    if ! echo "$output" | grep -qE "^${controller}[[:space:]].*[[:space:]]active([[:space:]]|$)"; then
      return 1
    fi
  done

  return 0
}

for run in $(seq 1 "$RUNS"); do
  echo
  echo "========================================"
  echo "Phase 2 startup test: $run/$RUNS"
  echo "========================================"

  LOG_FILE="/tmp/home_robotics_phase2_run_${run}.log"

  ros2 launch home_robotics_bringup \
    phase2_control.launch.py \
    use_viewer:=false \
    >"$LOG_FILE" 2>&1 &

  LAUNCH_PID=$!

  success=false
  start_time=$(date +%s)

  while true; do
    now=$(date +%s)
    elapsed=$((now - start_time))

    if check_controllers; then
      success=true
      break
    fi

    if (( elapsed >= READY_TIMEOUT )); then
      break
    fi

    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      break
    fi

    sleep 1
  done

  if [[ "$success" == true ]]; then
    echo "RUN $run: PASS"

    ros2 control list_controllers

    PASS_COUNT=$((PASS_COUNT + 1))
  else
    echo "RUN $run: FAIL"
    echo
    echo "Controller state:"
    ros2 control list_controllers 2>/dev/null || true

    echo
    echo "Last launch log lines:"
    tail -n 40 "$LOG_FILE"

    FAIL_COUNT=$((FAIL_COUNT + 1))
  fi

  cleanup
done

echo
echo "========================================"
echo "FINAL RESULT"
echo "========================================"
echo "Passed: $PASS_COUNT/$RUNS"
echo "Failed: $FAIL_COUNT/$RUNS"

if (( FAIL_COUNT == 0 )); then
  echo "PHASE 2 STARTUP DETERMINISM: PASS"
  exit 0
else
  echo "PHASE 2 STARTUP DETERMINISM: FAIL"
  exit 1
fi
