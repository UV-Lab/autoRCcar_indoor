#!/bin/bash

set -eo pipefail

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo $SCRIPT_PATH

INSTALL_PATH="${SCRIPT_PATH}/ros2/install/setup.bash"
SCAN_PID_FILE="/tmp/.run_nav_scan.pid"


usage() {
  cat <<EOF
Usage: $0 {init|scan|savemap|post_process}

  init         Initialize navigation environment
  scan         Perform sensor scan
  savemap      Save generated map
  post_process Run post-processing steps

EOF
}

init_command() {
  echo "[$SCRIPT_PATH] init: initializing navigation environment"
  # TODO: add init implementation here
}

scan_command() {
  echo "[$SCRIPT_PATH] scan: performing sensor scan"

  if [[ -f "${SCAN_PID_FILE}" ]]; then
    local existing_pid
    existing_pid=$(<"${SCAN_PID_FILE}")
    if kill -0 "${existing_pid}" 2>/dev/null; then
      echo "Scan already running with PID ${existing_pid}."
      return 1
    fi
  fi

  setsid ros2 launch lio_sam run.launch.py >/dev/null 2>&1 &
  local scan_pid=$!
  echo "${scan_pid}" >"${SCAN_PID_FILE}"
  echo "Started scan process PID=${scan_pid}"
}

stop_scan_process() {
  if [[ ! -f "${SCAN_PID_FILE}" ]]; then
    return 0
  fi

  local scan_pid
  scan_pid=$(<"${SCAN_PID_FILE}")
  if [[ -z "${scan_pid}" ]]; then
    rm -f "${SCAN_PID_FILE}"
    return 0
  fi

  if kill -0 "${scan_pid}" 2>/dev/null; then
    echo "Stopping scan process PID=${scan_pid} and its subprocesses"
    kill -- -"${scan_pid}" 2>/dev/null || true
    sleep 1
    kill -0 "${scan_pid}" 2>/dev/null && kill -TERM -- -"${scan_pid}" 2>/dev/null || true
  fi

  rm -f "${SCAN_PID_FILE}"
}

savemap_command() {
  echo "[$SCRIPT_PATH] savemap: saving generated map"

  read -rp "Please enter save map path: " SaveMapPath

  if [[ -z "${SaveMapPath}" ]]; then
    echo "SaveMapPath cannot be empty and stop scan"
    stop_scan_process
    return 0
  fi

  if [[ ! -d "${SaveMapPath}" ]]; then
    mkdir -p "${SaveMapPath}"
  fi

  [[ "${SaveMapPath}" != */ ]] && SaveMapPath="${SaveMapPath}/"

  echo "Using save map destination: ${SaveMapPath}"
  ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '${SaveMapPath}'}"

  stop_scan_process
}

post_process_command() {
  echo "[$SCRIPT_PATH] post_process: running post-processing steps"
  
  read -rp "Please enter save map path: " SaveMapPath

  if [[ -z "${SaveMapPath}" ]]; then
    echo "Error: SaveMapPath cannot be empty"
    return 1
  fi

  # [[ "${SaveMapPath}" != */ ]] && SaveMapPath="${SaveMapPath}/"

  local transform_cmd="${SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/transform_global_map"
  local grid_builder_cmd="${SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/grid_map_builder"
  local cfg_file="${SCRIPT_PATH}/ros2/install/lio_sam/share/lio_sam/config/grid_map_builder_cfg.yaml"

  echo "Running transform_global_map..."
  "${transform_cmd}" \
    "${SaveMapPath}/pointCloud" \
    "${cfg_file}" \
    "${SaveMapPath}/tf_new_old_mat.txt" \
    "${SaveMapPath}/global_map_tf"
  if [[ $? -ne 0 ]]; then
    echo "Error: transform_global_map failed"
    return 1
  fi

  echo "Running grid_map_builder..."
  "${grid_builder_cmd}" \
    "${SaveMapPath}/global_map_tf/pointCloud" \
    "${cfg_file}" \
    "${SaveMapPath}/loc_map/grid_map" default
  if [[ $? -ne 0 ]]; then
    echo "Error: grid_map_builder failed"
    return 1
  fi
}

run_nav() {
  if [[ $# -ne 1 ]]; then
    echo "Error: Expected exactly one argument."
    usage
    return 1
  fi

  if [[ ! -f "${INSTALL_PATH}" ]]; then
    echo "Error: Required file not found: ${INSTALL_PATH}"
    return 1
  fi

  
  source ${INSTALL_PATH}

  local command="$1"
  case "$command" in
    init)
      init_command
      ;;
    scan)
      scan_command
      ;;
    savemap)
      savemap_command
      ;;
    post_process)
      post_process_command
      ;;
    *)
      echo "Error: Invalid argument '$command'."
      usage
      return 1
      ;;
  esac
}

run_nav "$@"
