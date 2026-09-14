#!/usr/bin/env bash
# Pre-relaunch cleanup for tank_sim_gazebo.
# Idempotent: OK if nothing is running. Does not require ROS.
#
# Fixes the common 2nd-start failure modes:
#   - leftover `gz sim` still owning the world / model `tank_sim`
#   - spawn then creating tank_sim_0 while bridges still talk to tank_sim
#   - teleop HTTP :8768 still bound by cmd_vel_web
#   - stale ros_gz parameter_bridge on tank_sim topics
set -u

log() { printf '[sim_cleanup] %s\n' "$*"; }

_kill_pat() {
  local pat="$1"
  if pgrep -f "$pat" >/dev/null 2>&1; then
    log "TERM $pat"
    pkill -TERM -f "$pat" >/dev/null 2>&1 || true
    sleep 0.4
    if pgrep -f "$pat" >/dev/null 2>&1; then
      log "KILL $pat"
      pkill -KILL -f "$pat" >/dev/null 2>&1 || true
    fi
  fi
}

# Teleop port from a previous cmd_vel_web (Address already in use → launch abort).
if command -v fuser >/dev/null 2>&1; then
  if fuser 8768/tcp >/dev/null 2>&1; then
    log "free :8768"
    fuser -k 8768/tcp >/dev/null 2>&1 || true
  fi
fi

# Gazebo Harmonic server / GUI (binary names vary by install).
_kill_pat '[/ ]gz sim'
_kill_pat 'gz-sim-server'
_kill_pat 'gz sim-server'
_kill_pat 'ruby /usr/.*/gz'

# Sim nodes that otherwise leak publishers on /cmd_vel /joint_states /clock.
_kill_pat 'cmd_vel_web.py'
_kill_pat 'pantilt_sim_adapter.py'
_kill_pat 'sim_nav_contract.py'
_kill_pat 'rewrite_frame_id.py'
_kill_pat 'tank_sim_bridge_'
_kill_pat 'tank_sim_gazebo/config/bridge.yaml'

# Narrow ros_gz bridge: only if the cmdline mentions tank_sim.
if pgrep -af 'parameter_bridge' 2>/dev/null | grep -E 'tank_sim|tank_sim_bridge_' >/dev/null; then
  _kill_pat 'parameter_bridge'
fi

sleep 0.2
log "done"
exit 0
