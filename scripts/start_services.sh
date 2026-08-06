#!/usr/bin/env bash
set -euo pipefail

if [ -n "${ROBOT_NAME:-}" ]; then
  resolved_robot_name="${ROBOT_NAME}"
elif [ -n "${HOSTNAME:-}" ]; then
  resolved_robot_name="${HOSTNAME}"
else
  resolved_robot_name="duckiebot"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${resolved_robot_name}.local:11311}"

if [ -z "${ROS_IP:-}" ]; then
  primary_ip="$(ip route get 1.1.1.1 | awk '{print $7; exit}')"
  export ROS_IP="${primary_ip}"
fi

if [ -z "${ROS_HOSTNAME:-}" ]; then
  export ROS_HOSTNAME="${resolved_robot_name}.local"
fi

echo "[startup] ROBOT_NAME=${resolved_robot_name}"
echo "[startup] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[startup] ROS_IP=${ROS_IP}"
echo "[startup] ROS_HOSTNAME=${ROS_HOSTNAME}"
echo "[startup] BRIDGE_PATH=${BRIDGE_PATH:-/ros}"
echo "[startup] FRONTEND_PORT=${FRONTEND_PORT}"
echo "[startup] FRONTEND_DIR=${FRONTEND_DIR}"

python3 -m http.server "${FRONTEND_PORT}" --bind 0.0.0.0 --directory "${FRONTEND_DIR}" >/tmp/frontend.log 2>&1 &
frontend_pid=$!

sleep 1
if ! kill -0 "${frontend_pid}" 2>/dev/null; then
  echo "[startup] frontend server failed to start"
  cat /tmp/frontend.log || true
  exit 1
fi
echo "[startup] frontend server started"

cleanup() {
  echo "[startup] stopping services"
  kill "${frontend_pid}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
cd ./Workshop/Duckietown_msg
catkin_make
source devel/setup.bash

jupyter lab \
  --ip=0.0.0.0 \
  --port="${JUPYTER_PORT}" \
  --no-browser \
  --notebook-dir="${NOTEBOOK_DIR}" \
  --ServerApp.token='' \
  --ServerApp.password='' \
  --ServerApp.allow_origin='*' \
  --ServerApp.disable_check_xsrf=True \
  --allow-root 
