#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Bring up the orca4 simulation and run the default mission, in one terminal.
#
# Equivalent to the two-terminal dance in the README:
#   ros2 launch orca_bringup sim_launch.py
#   ros2 run orca_bringup mission_runner.py
#
# but it waits for the sim to actually be ready before sending the mission,
# instead of guessing with a sleep.
#
# Usage:
#   ./orca4_run.sh                 launch + mission (Gazebo and RViz UIs)
#   ./orca4_run.sh --headless      no Gazebo UI, no RViz (much faster)
#   ./orca4_run.sh --no-mission    bring the sim up and leave it running
#   ./orca4_run.sh --software-gl   force software rendering
#   ./orca4_run.sh --bag           also record a rosbag
#   ./orca4_run.sh --timeout 300   how long to wait for readiness (default 240)
#
# Ctrl+C at any point shuts the whole simulation down.
# ---------------------------------------------------------------------------
set -uo pipefail

WS="$HOME/colcon_ws"
RUN_DIR="$HOME/.orca4_run"
LOG="$RUN_DIR/sim_$(date +%Y%m%d_%H%M%S).log"

HEADLESS=0
RUN_MISSION=1
SOFTWARE_GL=0
BAG=0
TIMEOUT=240

while [ $# -gt 0 ]; do
  case "$1" in
    --headless)    HEADLESS=1 ;;
    --no-mission)  RUN_MISSION=0 ;;
    --software-gl) SOFTWARE_GL=1 ;;
    --bag)         BAG=1 ;;
    --timeout)     shift; TIMEOUT="${1:-240}" ;;
    -h|--help)     sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
  esac
  shift
done

c_info() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
c_ok()   { printf '\033[1;32m    %s\033[0m\n' "$*"; }
c_warn() { printf '\033[1;33m    %s\033[0m\n' "$*"; }
c_err()  { printf '\033[1;31m!!  %s\033[0m\n' "$*" >&2; }

mkdir -p "$RUN_DIR"

# WSL inherits the Windows PATH; Anaconda and friends shadow Linux libraries.
# Harmless if .bashrc already did this.
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | paste -sd: -)"
export PATH

if [ ! -f "$WS/src/orca4/setup.bash" ]; then
  c_err "$WS/src/orca4/setup.bash not found -- is the workspace built?"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source "$WS/src/orca4/setup.bash"
set -u

# DDS multicast discovery is unreliable on the WSL virtual NIC. Without this,
# nav2's lifecycle_manager never sees the servers it manages and stalls at
# "Creating and initializing lifecycle service clients", so /follow_waypoints
# is never advertised. Set it here rather than relying on .bashrc, which a
# non-interactive shell does not read.
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

[ "$SOFTWARE_GL" -eq 1 ] && export LIBGL_ALWAYS_SOFTWARE=1

LAUNCH_ARGS=()
if [ "$HEADLESS" -eq 1 ]; then
  LAUNCH_ARGS+=(gzclient:=False rviz:=False)
fi
[ "$BAG" -eq 1 ] && LAUNCH_ARGS+=(bag:=True)

SIM_PID=""

stale_check() {
  # A previous run that did not shut down cleanly leaves nodes holding the
  # MAVLink port and republishing topics; the next ardusub then dies with
  # exit code 1 and mavros loops on "Connection refused".
  local stale
  stale="$(pgrep -f "$WS/install/|gz sim .*sand.world|ardusub -S -w -M JSON" 2>/dev/null | tr '\n' ' ')"
  if [ -n "${stale// /}" ]; then
    c_warn "leftover orca4 processes from a previous run: $stale"
    c_warn "stopping them first"
    pkill -f "$WS/install/"           2>/dev/null
    pkill -f "gz sim .*sand.world"    2>/dev/null
    pkill -f "ardusub -S -w -M JSON"  2>/dev/null
    sleep 3
    pkill -9 -f "$WS/install/"          2>/dev/null
    pkill -9 -f "gz sim .*sand.world"   2>/dev/null
    pkill -9 -f "ardusub -S -w -M JSON" 2>/dev/null
    sleep 1
  fi
}

CLEANED=0
cleanup() {
  # The trap fires on both INT and EXIT; only tear down once.
  [ "$CLEANED" -eq 1 ] && return 0
  CLEANED=1
  # Ctrl+C reaches the mission runner directly; tear the sim down too.
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    echo
    c_info "Shutting down the simulation..."
    # Signal the whole process group. Signalling just the ros2 launch pid leaves
    # base_controller, manager and orb_slam2 running, which breaks the next run.
    kill -INT -"$SIM_PID" 2>/dev/null || kill -INT "$SIM_PID" 2>/dev/null
    for _ in $(seq 1 30); do
      kill -0 "$SIM_PID" 2>/dev/null || break
      sleep 0.5
    done
    if kill -0 "$SIM_PID" 2>/dev/null; then
      c_warn "clean shutdown timed out, terminating"
      kill -TERM -"$SIM_PID" 2>/dev/null || kill -TERM "$SIM_PID" 2>/dev/null
      sleep 3
    fi
  fi
  # Sweep anything that outlived the group kill. Patterns are tied to this
  # workspace and this world file, so nothing unrelated is touched.
  pkill -f "$WS/install/"           2>/dev/null
  pkill -f "gz sim .*sand.world"    2>/dev/null
  pkill -f "ardusub -S -w -M JSON"  2>/dev/null
  [ -n "$SIM_PID" ] && c_ok "stopped"
  return 0
}
trap cleanup EXIT INT TERM

# Wait for a regex to appear in the launch log, failing fast if the sim dies.
wait_for_log() {
  local pattern="$1" what="$2" deadline=$((SECONDS + TIMEOUT))
  while [ $SECONDS -lt $deadline ]; do
    if grep -qE "$pattern" "$LOG" 2>/dev/null; then return 0; fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
      c_err "the simulation exited while waiting for $what"
      c_err "last 30 lines of $LOG:"
      tail -30 "$LOG" >&2
      return 1
    fi
    sleep 2
  done
  c_err "timed out after ${TIMEOUT}s waiting for $what"
  c_err "last 30 lines of $LOG:"
  tail -30 "$LOG" >&2
  return 1
}

wait_for_action() {
  local action="$1" deadline=$((SECONDS + TIMEOUT))
  while [ $SECONDS -lt $deadline ]; do
    if ros2 action list 2>/dev/null | grep -qx "$action"; then return 0; fi
    kill -0 "$SIM_PID" 2>/dev/null || { c_err "simulation exited waiting for $action"; return 1; }
    sleep 2
  done
  c_err "timed out waiting for action server $action"
  return 1
}

# ORB_SLAM2 resolves its vocabulary through the relative path
# install/orb_slam2_ros/share/... so it only starts if the working directory is
# the colcon workspace. Launching from anywhere else kills the SLAM node.
cd "$WS" || { c_err "cannot cd to $WS"; exit 1; }

stale_check

c_info "Starting the simulation"
echo "    log: $LOG"
[ "$HEADLESS" -eq 1 ] && echo "    headless (no Gazebo UI, no RViz)"

# Enable job control so the launch gets its own process group, which lets
# cleanup() signal every node it spawns rather than just the launch process.
set -m
ros2 launch orca_bringup sim_launch.py "${LAUNCH_ARGS[@]}" > "$LOG" 2>&1 &
SIM_PID=$!
set +m

if [ "$RUN_MISSION" -eq 0 ]; then
  c_ok "simulation running (pid $SIM_PID). Ctrl+C to stop."
  wait "$SIM_PID"
  exit $?
fi

# base_controller refuses AUV mode until the EKF has converged, so waiting on
# the action servers alone is not enough -- the mission would be rejected.
c_info "Waiting for the EKF to converge (this takes a minute or two)"
wait_for_log "EKF is running" "the EKF" || exit 1
c_ok "EKF is running"

# Only wait for /set_target_mode. bringup.py launches nav2 with autostart:=False
# on purpose, and manager.cpp calls /lifecycle_manager_navigation/manage_nodes
# with STARTUP when the sub switches to AUV mode. So /follow_waypoints does not
# exist until the mission runner has already set AUV mode -- waiting for it here
# would deadlock. mission_runner.py calls wait_for_server() itself.
c_info "Waiting for the mode action server"
wait_for_action "/set_target_mode" || exit 1
c_ok "/set_target_mode is up (nav2 activates when the mission sets AUV mode)"

c_info "Running the mission (dive to -7m, two laps of the rectangle)"
echo
ros2 run orca_bringup mission_runner.py
MISSION_RC=$?
echo

if [ $MISSION_RC -eq 0 ]; then
  c_ok "mission finished"
else
  c_warn "mission exited with code $MISSION_RC"
fi

c_info "Simulation still running. Ctrl+C to stop, or watch: tail -f $LOG"
wait "$SIM_PID"
