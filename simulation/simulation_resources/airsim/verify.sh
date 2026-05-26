#!/usr/bin/env bash
# =============================================================================
# verify.sh -- end-to-end smoke-test for the AirSim backend.
#
# Run with ROS 2 sourced, in the drone's ROS_DOMAIN_ID (the script sets it from
# DRONE_ID). Typically run inside the aircraft or simulation container after a
#   SIM=airsim AUTOPILOT=px4 ./tools_and_docs/sim_run.sh
# launch. Prints a checkmark/cross per signal and exits non-zero on any failure.
#
#   DRONE_ID=1 AUTOPILOT=px4 ./verify.sh        # (defaults shown)
#
# NOTE: topic names for the bridge's camera/lidar are matched by pattern because
# they depend on the AirSim camera/sensor names in settings.json -- adjust the
# regexes if you renamed them.
# =============================================================================
set -u

DRONE_ID="${DRONE_ID:-1}"
AUTOPILOT="${AUTOPILOT:-px4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DRONE_ID}"

GREEN=$'\033[32m'; RED=$'\033[31m'; YEL=$'\033[33m'; RST=$'\033[0m'
PASS=0; FAIL=0

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found -- source /opt/ros/humble/setup.bash (and the AAS workspaces) first." >&2
  exit 2
fi

ok()   { echo "${GREEN}OK${RST}   $1"; PASS=$((PASS+1)); }
bad()  { echo "${RED}FAIL${RST} $1"; FAIL=$((FAIL+1)); }
warn() { echo "${YEL}WARN${RST} $1"; }

# True if a message arrives on $1 within $2 seconds.
has_msg()    { timeout "${2:-10}" ros2 topic echo --once "$1" >/dev/null 2>&1; }
# True if $1 is publishing (any rate) within $2 seconds.
is_live()    { timeout "${2:-5}" ros2 topic hz "$1" 2>/dev/null | grep -q "average rate"; }
# First topic matching regex $1 (empty if none).
first_topic(){ ros2 topic list 2>/dev/null | grep -E "$1" | head -n1; }

echo "== AirSim backend smoke-test (Drone${DRONE_ID}, ${AUTOPILOT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}) =="

# 1. Autopilot link (the de-risk gate)
if [ "$AUTOPILOT" = "px4" ]; then
  t=$(first_topic "/Drone${DRONE_ID}/fmu/out/vehicle_(local_position|status)")
  if [ -n "$t" ] && has_msg "$t" 10; then ok "Autopilot (PX4) publishing $t"
  else bad "Autopilot (PX4): /Drone${DRONE_ID}/fmu/out/* not live (none-target / XRCE agent?)"; fi
else
  if has_msg "/mavros/state" 10; then ok "Autopilot (ArduPilot) /mavros/state live"
  else bad "Autopilot (ArduPilot): /mavros/state not live"; fi
fi

# 2. Sim clock advancing (bridge owns /clock from AirSim)
if is_live "/clock" 5; then ok "/clock advancing (sim time)"; else bad "/clock not publishing"; fi

# 3. Bridge camera + lidar topics present
cam=$(first_topic "/Drone${DRONE_ID}/.*image")
if [ -n "$cam" ]; then ok "Bridge camera topic present ($cam)"; else bad "No bridge camera image topic under /Drone${DRONE_ID}/"; fi
lid=$(first_topic "/Drone${DRONE_ID}/.*(points|[Ll]idar)")
if [ -n "$lid" ]; then ok "Bridge lidar topic present ($lid)"; else bad "No bridge lidar topic under /Drone${DRONE_ID}/"; fi

# 4. LiDAR relay feeding KISS-ICP
if is_live "/lidar_points" 6; then ok "/lidar_points live (KISS-ICP input)"; else bad "/lidar_points not publishing (relay?)"; fi

# 5. Perception
if ros2 topic list 2>/dev/null | grep -qx "/detections"; then
  if is_live "/detections" 5; then ok "/detections live"
  else warn "/detections exists but idle (no target in camera view?)"; fi
else
  bad "/detections topic missing (yolo_py / camera shim?)"
fi

# 6. Bridge container (host-side check only)
if command -v docker >/dev/null 2>&1; then
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -q "airsim-bridge-container"; then ok "TEVV bridge container running"
  else bad "No airsim-bridge-container running"; fi
else
  warn "docker CLI not here -- skipping bridge-container check (run on host)"
fi

echo "== ${PASS} passed, ${FAIL} failed =="
[ "$FAIL" -eq 0 ]
