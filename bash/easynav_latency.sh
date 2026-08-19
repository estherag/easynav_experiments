#!/usr/bin/env bash
#
# EasyNav scan mode bridge latency experiment
# Measures time from BLOCKED LaserScan to robot stop
#

set -euo pipefail
sleep 10
MAX_WAIT_AFTER_TRIGGER=10
MOVE_TIME=10

GOAL_X="1.6925969123840332"
GOAL_Y="2.4261345863342285"
GOAL_QZ="0.8091"
GOAL_QW="0.5874"

# Home pose
ORIGIN_X="8.018"
ORIGIN_Y="-0.698"
ORIGIN_QZ="1.0000"
ORIGIN_QW="0.0004"

log() {
  echo "[$(date '+%H:%M:%S')] $*"
}

trigger_mode() {
  ros2 service call /trigger_mode std_srvs/srv/Trigger "{}" \
    >/dev/null 2>&1 || \
    log "WARNING: trigger_mode call failed"
}

send_goal() {
  local x="$1"
  local y="$2"
  local qz="$3"
  local qw="$4"

  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{
      header: {
        frame_id: map
      },
      pose: {
        position: {
          x: $x,
          y: $y,
          z: 0.0
        },
        orientation: {
          x: 0.0,
          y: 0.0,
          z: $qz,
          w: $qw
        }
      }
    }"
}

cancel_goal() {
  ros2 topic pub --once /easynav_control \
    easynav_interfaces/msg/NavigationControl \
    "{
      type: 6,
      user_id: nav_user_1
    }" >/dev/null 2>&1 || \
    log "WARNING: EasyNav cancel command failed"
}

# Check required interfaces
log "Checking /trigger_mode service..."

if ! ros2 service list | grep -q "/trigger_mode"; then
  echo "ERROR: /trigger_mode not found. Is scan_mode_bridge running?"
  exit 1
fi

log "Checking /goal_pose topic..."

if ! ros2 topic list | grep -q "^/goal_pose$"; then
  echo "ERROR: /goal_pose not found. Is EasyNav running?"
  exit 1
fi

# Send navigation goal
log "Sending EasyNav navigation goal (x=$GOAL_X, y=$GOAL_Y)..."

send_goal \
  "$GOAL_X" \
  "$GOAL_Y" \
  "$GOAL_QZ" \
  "$GOAL_QW"

log "Goal sent"

# Let robot move
log "Goal sent. Waiting ${MOVE_TIME}s..."

sleep "$MOVE_TIME"


# Trigger BLOCKED
log "Triggering BLOCKED mode"

trigger_mode


# Wait for scan_mode_bridge measurement
log "Waiting for stop detection (max ${MAX_WAIT_AFTER_TRIGGER}s)..."

sleep "$MAX_WAIT_AFTER_TRIGGER"

# Restore BRIDGE mode
log "Triggering BRIDGE mode"
trigger_mode


# # Cancel EasyNav navigation
# log "Canceling EasyNav goal"
# cancel_goal


# Return robot to origin
log "Returning robot to origin..."

send_goal \
  "$ORIGIN_X" \
  "$ORIGIN_Y" \
  "$ORIGIN_QZ" \
  "$ORIGIN_QW"

sleep 10

log "Experiment complete"
