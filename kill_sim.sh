#!/usr/bin/env bash
# Kill every process from the sim/nav stack and restart the ROS daemon.
#
# WHY THIS EXISTS: if a previous launch is not fully killed, a leftover
# ros_gz_bridge keeps publishing /clock. Two /clock publishers make sim time
# jump around, which clears the TF buffer and produces planner errors like
# "Lookup would require extrapolation into the past" -> goals fail / robot
# gets stuck. Run this between runs to guarantee a clean slate.
#
# Usage:  ./kill_sim.sh

set -u

PATTERNS=(
  "ign gazebo"
  "ruby"
  "parameter_bridge"
  "image_bridge"
  "robot_state_publisher"
  "ros_gz_sim"
  "nav2_"
  "amcl"
  "map_server"
  "controller_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "velocity_smoother"
  "smoother_server"
  "lifecycle_manager"
  "async_slam_toolbox"
  "slam_toolbox"
  "gazebo.launch"
  "nav2.launch"
  "slam.launch"
  "object_goal_nav"
  "rviz2"
  "transform_listener"
)

echo "Killing sim/nav processes..."
for p in "${PATTERNS[@]}"; do
  # pkill returns non-zero when nothing matched; that's fine here.
  pkill -9 -f "$p" 2>/dev/null || true
done

# self-exclude: the line above can't match this script's own pattern list,
# but give children a moment to die.
sleep 2

REMAIN=$(pgrep -af "ign gazebo|parameter_bridge|nav2_|amcl|gazebo.launch|nav2.launch|rviz2|async_slam" | grep -v "kill_sim" | grep -v pgrep)
if [ -n "$REMAIN" ]; then
  echo "WARNING: some processes survived:"
  echo "$REMAIN"
else
  echo "All clean."
fi

# Restart the ROS 2 daemon so stale discovery state is dropped.
if [ -n "${ROS_DISTRO:-}" ]; then
  ros2 daemon stop  >/dev/null 2>&1
  ros2 daemon start >/dev/null 2>&1
  echo "ROS daemon restarted."
else
  echo "(ROS not sourced; skipped daemon restart)"
fi
