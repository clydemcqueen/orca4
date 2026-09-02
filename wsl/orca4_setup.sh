#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# orca4 setup for WSL2 / Ubuntu 22.04  (ROS 2 Humble + Gazebo Harmonic)
#
# Mirrors docker/Dockerfile, which the README points to as the canonical
# install reference, adapted for a native WSL install (no NVIDIA, WSLg GUI).
#
# Resumable: each stage drops a stamp in ~/.orca4_setup_stamps, so re-running
# skips whatever already succeeded.
#   redo one stage:  rm ~/.orca4_setup_stamps/<stage>
#   start over:      rm -rf ~/.orca4_setup_stamps
# ---------------------------------------------------------------------------
set -euo pipefail

ORCA4_FORK="https://github.com/jensbremnes/orca4.git"
COLCON_WS="$HOME/colcon_ws"
AP_DIR="$HOME/ardupilot"
STAMPS="$HOME/.orca4_setup_stamps"

# 14 cores but only 15 GB RAM. ORB_SLAM2 / g2o translation units run well ove
# 1 GB each, so uncapped parallelism gets the build OOM-killed.
JOBS="${JOBS:-4}"

mkdir -p "$STAMPS"

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m    [done] %s\033[0m\n' "$*"; }
skip() { printf '\033[0;90m    [skip] %s (already done)\033[0m\n' "$*"; }

stage() {
  local name="$1"; shift
  if [ -f "$STAMPS/$name" ]; then skip "$name"; return 0; fi
  log "$name"
  if ! "$@"; then
    printf '\n\033[1;31m!! FAILED at stage: %s\033[0m\n' "$name" >&2
    printf '\033[1;31m   Fix the error above, then re-run this script.\033[0m\n' >&2
    printf '\033[1;31m   Completed stages will be skipped.\033[0m\n' >&2
    return 1
  fi
  touch "$STAMPS/$name"
  ok "$name"
}

codename() { . /etc/os-release && echo "$VERSION_CODENAME"; }

# WSL inherits the Windows PATH, and CMake derives search prefixes from PATH
# (strip a trailing bin/sbin, append include/lib). That makes Windows toolchains
# visible to Linux builds: Anaconda at /mnt/c/ProgramData/anaconda3/Library
# exposes protobuf 6.x, yaml-cpp, zlib, png and jpeg headers, and gz-msgs10
# needs the system protobuf 3.12. Drop every /mnt path for the build.
strip_windows_path() {
  local cleaned
  cleaned="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | paste -sd: -)"
  export PATH="$cleaned"
}

# Source the ROS underlay. Needed by rosdep (which reads ROS_DISTRO) and by
# colcon. The Dockerfile never does this because its osrf/ros base image bakes
# ROS_DISTRO in as an ENV; on a native install it only exists once sourced.
# set +u because the ROS setup scripts touch unbound variables.
ros_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
}

# --- 1. base apt packages --------------------------------------------------
s_apt_base() {
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl gnupg lsb-release ca-certificates \
    software-properties-common python3-pip python3-dev vim bash-completion \
    mesa-utils libgl1-mesa-dri
  sudo add-apt-repository -y universe
}

# --- 2. ROS 2 Humble -------------------------------------------------------
s_ros() {
  local cn; cn="$(codename)"
  if ! ls /etc/apt/sources.list.d/ros2* >/dev/null 2>&1; then
    # Preferred path: the ros2-apt-source .deb, which keeps the signing key
    # current. Fall back to the older keyring method if that is unreachable.
    local ver deb="/tmp/ros2-apt-source.deb"
    ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
           | grep -F '"tag_name"' | awk -F'"' '{print $4}')" || ver=""
    if [ -n "$ver" ] && curl -fsSL -o "$deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${cn}_all.deb"; then
      sudo apt-get install -y "$deb"
    else
      echo "ros2-apt-source unavailable; using the classic keyring method"
      sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${cn} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
  fi
  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-desktop ros-dev-tools \
    python3-colcon-common-extensions python3-vcstool python3-rosdep
}

# --- 3. Gazebo Harmonic ----------------------------------------------------
# Harmonic on Humble is a non-standard pairing, hence the extra rosdep rules.
s_gazebo() {
  sudo wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    https://packages.osrfoundation.org/gazebo.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic ros-humble-ros-gzharmonic

  # Teach rosdep the Gazebo Harmonic keys, or rosdep install fails later.
  sudo mkdir -p /etc/ros/rosdep/sources.list.d
  sudo wget -qO /etc/ros/rosdep/sources.list.d/00-gazebo.list \
    https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list
}

# --- 4. ArduPilot source + prereqs ----------------------------------------
s_ardupilot_prereqs() {
  sudo apt-get install -y \
    python3-wxgtk4.0 rapidjson-dev xterm libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  [ -d "$AP_DIR" ] || git clone --recurse-submodules \
    https://github.com/ArduPilot/ardupilot.git "$AP_DIR"
  cd "$AP_DIR"
  export SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
  Tools/environment_install/install-prereqs-ubuntu.sh -y
}

# --- 5. Build ArduSub for SITL --------------------------------------------
s_ardusub() {
  cd "$AP_DIR"
  export PATH="$HOME/.local/bin:$PATH"
  modules/waf/waf-light configure --board sitl
  modules/waf/waf-light build --target bin/ardusub -j"$JOBS"
}

# --- 6. colcon workspace: fork + workspace.repos + rosdep -----------------
s_workspace_fetch() {
  export PATH="$HOME/.local/bin:$PATH"
  ros_env
  export GZ_VERSION=harmonic
  mkdir -p "$COLCON_WS/src"
  cd "$COLCON_WS/src"
  [ -d orca4 ] || git clone "$ORCA4_FORK" orca4
  vcs import < orca4/workspace.repos

  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
  rosdep update

  cd "$COLCON_WS"
  rosdep install -y --from-paths src --ignore-src --rosdistro humble
}

# --- 7. GeographicLib datasets (mavros needs these) -----------------------
s_geographiclib() {
  cd /tmp
  wget -qO install_geographiclib_datasets.sh \
    https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  chmod +x install_geographiclib_datasets.sh
  sudo ./install_geographiclib_datasets.sh
}

# --- 8. Build the workspace ------------------------------------------------
s_build() {
  cd "$COLCON_WS"
  ros_env
  export GZ_VERSION=harmonic
  export MAKEFLAGS="-j$JOBS"
  colcon build --parallel-workers "$JOBS" --cmake-args -DCMAKE_BUILD_TYPE=Release
}

# --- 9. Shell environment --------------------------------------------------
s_shellenv() {
  if ! grep -qF "# >>> orca4 >>>" "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" <<'BASHRC'

# >>> orca4 >>>
# Keep DDS traffic off the WSL virtual NIC; multicast discovery is flaky there.
export ROS_LOCALHOST_ONLY=1

# WSL inherits the Windows PATH, and CMake turns PATH entries into search
# prefixes (strip bin/sbin, append include/lib). That exposes Windows headers
# to Linux builds -- Anaconda alone ships protobuf 6.x, yaml-cpp, zlib, png and
# jpeg, and gz-msgs10 needs the system protobuf 3.12. Git, StrawberryPerl and
# MiKTeX add more. Drop them so colcon builds resolve against Ubuntu only.
# Need a Windows tool (code, explorer.exe)? Run: restore_windows_path
export ORCA4_WINDOWS_PATH="$PATH"
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | paste -sd: -)"
export PATH
restore_windows_path() { export PATH="$ORCA4_WINDOWS_PATH"; }
# setup.bash puts ArduSub on PATH and sources colcon_ws/install/setup.bash,
# which chains /opt/ros/humble for us.
source "$HOME/colcon_ws/src/orca4/setup.bash"
# <<< orca4 <<<
BASHRC
  fi
}

main() {
  echo "orca4 setup -- ROS 2 Humble + Gazebo Harmonic on WSL2 Ubuntu 22.04"
  echo "  workspace:   $COLCON_WS"
  echo "  ardupilot:   $AP_DIR"
  echo "  build jobs:  $JOBS"
  echo "  expect roughly 1-2 hours and ~20 GB on the first run"

  strip_windows_path

  # Take the password once, then keep the sudo timestamp warm for the whole run
  # so no later stage stalls on a prompt.
  sudo -v
  ( while true; do sleep 50; sudo -n true 2>/dev/null || exit; kill -0 "$$" 2>/dev/null || exit; done ) &
  keepalive=$!
  trap 'kill "$keepalive" 2>/dev/null || true' EXIT

  stage apt_base           s_apt_base
  stage ros_humble         s_ros
  stage gazebo_harmonic    s_gazebo
  stage ardupilot_prereqs  s_ardupilot_prereqs
  stage ardusub_build      s_ardusub
  stage workspace_fetch    s_workspace_fetch
  stage geographiclib      s_geographiclib
  stage workspace_build    s_build
  stage shell_env          s_shellenv

  cat <<'DONE'

============================================================
 All stages complete.

 Open a NEW WSL shell so .bashrc takes effect, then:

   Terminal 1 -- Gazebo, RViz and all nodes:
     ros2 launch orca_bringup sim_launch.py

   Terminal 2 -- run the default mission:
     ros2 run orca_bringup mission_runner.py

 Your fork now lives at ~/colcon_ws/src/orca4
 From Windows:
   \\wsl.localhost\Ubuntu-22.04\home\jensbremnes\colcon_ws\src\orca4

 If Gazebo shows no window or renders black:
     export LIBGL_ALWAYS_SOFTWARE=1
============================================================
DONE
}

main "$@"
