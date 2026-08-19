#!/usr/bin/env bash
#
# Scan mode bridge latency experiment
# Measures time from BLOCKED mode to robot stop
#

set -euo pipefail

sleep 30

MAX_WAIT_AFTER_TRIGGER=10
MOVE_TIME=15

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
  ros2 service call /trigger_mode std_srvs/srv/Trigger {} 2>/dev/null || \
    log "WARNING: trigger_mode call failed"
}

# Check service availability
log "Checking /trigger_mode service..."
if ! ros2 service list | grep -q "/trigger_mode"; then
  echo "ERROR: /trigger_mode not found. Is scan_mode_bridge running?"
  exit 1
fi

# Send navigation goal
log "Sending navigation goal (x=$GOAL_X, y=$GOAL_Y)..."

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {z: $GOAL_QZ, w: $GOAL_QW}}}}" \
  >/dev/null 2>&1 &

# GOAL_PID=$!

# log "Goal sent (PID=$GOAL_PID)"

# Let the robot move
log "Goal sent. Waiting ${MOVE_TIME}s..."
sleep "$MOVE_TIME"

# Trigger BLOCKED
log "Triggering BLOCKED mode"
trigger_mode

# Wait for stop detection
log "Waiting for stop detection (max ${MAX_WAIT_AFTER_TRIGGER}s)..."
sleep "$MAX_WAIT_AFTER_TRIGGER"

# Restore BRIDGE mode
log "Triggering BRIDGE mode"
trigger_mode

# Cancel navigation goal
# log "Canceling navigation goal..."
# ros2 service call /navigate_to_pose/_action/cancel_goal \
#   action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1 || true

# # Kill background process if still running
# kill "$GOAL_PID" 2>/dev/null || true
# wait "$GOAL_PID" 2>/dev/null || true

# Return robot to origin
log "Returning robot to origin..."

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $ORIGIN_X, y: $ORIGIN_Y, z: 0.0}, orientation: {z: $ORIGIN_QZ, w: $ORIGIN_QW}}}}" \
  >/dev/null 2>&1 &

sleep 10
log "Experiment complete"
