#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent
print(SCRIPT_PATH)

INSTALL_PATH = f"{SCRIPT_PATH}/ros2/install/setup.bash"
SCAN_PID_FILE = Path("/tmp/.run_nav_scan.pid")
SAVE_MAP_PATH = ""


def print_usage() -> None:
    print(f"用法: {Path(sys.argv[0]).name} <用户数据文件夹路径>")


def validate_input_path(argv: list[str]) -> str:
    if len(argv) < 2:
        print_usage()
        raise ValueError("缺少用户数据文件夹路径参数")

    user_data_dir = argv[1]
    if not Path(user_data_dir).is_dir():
        raise FileNotFoundError(f"错误: 命令行路径不准确，目录不存在: {user_data_dir}")

    print(f"用户数据目录校验通过: {user_data_dir}")
    return user_data_dir


def _is_process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def run_shell_command(command: list[str]) -> None:
    try:
        subprocess.run(command, check=True)
    except (subprocess.SubprocessError, OSError):
        stop_scan_process()
        raise


def scan_command() -> None:
    print(f"[{SCRIPT_PATH}] scan: performing sensor scan")

    if SCAN_PID_FILE.exists():
        existing_text = SCAN_PID_FILE.read_text(encoding="utf-8").strip()
        if existing_text:
            existing_pid = int(existing_text)
            if _is_process_alive(existing_pid):
                raise RuntimeError(f"Scan k running with PID {existing_pid}.")

    try:
        proc = subprocess.Popen(
            ["ros2", "launch", "lio_sam", "run.launch.py"],
            preexec_fn=os.setsid,
        )
    except (subprocess.SubprocessError, OSError):
        stop_scan_process()
        raise

    SCAN_PID_FILE.write_text(str(proc.pid), encoding="utf-8")
    print(f"Started scan process PID={proc.pid}")


def function_two(user_data_dir: str) -> None:
    print("开始执行函数2...")
    run_shell_command(["ros2", "bag", "play", user_data_dir, "-r", "1"])
    print(f"函数2执行完成，目标目录: {user_data_dir}")


def stop_scan_process() -> None:
    if not SCAN_PID_FILE.exists():
        return

    scan_pid_text = SCAN_PID_FILE.read_text(encoding="utf-8").strip()
    if not scan_pid_text:
        SCAN_PID_FILE.unlink(missing_ok=True)
        return

    scan_pid = int(scan_pid_text)
    if _is_process_alive(scan_pid):
        print(f"Stopping scan process PID={scan_pid} and its subprocesses")
        try:
            os.killpg(scan_pid, signal.SIGINT)
        except ProcessLookupError:
            pass

        time.sleep(1)

        if _is_process_alive(scan_pid):
            try:
                os.killpg(scan_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    SCAN_PID_FILE.unlink(missing_ok=True)


def savemap_command(user_data_dir: str) -> None:
    global SAVE_MAP_PATH

    print(f"[{SCRIPT_PATH}] savemap: saving generated map")

    SAVE_MAP_PATH = f"{user_data_dir}_result"
    print(f"Please enter save map path: {SAVE_MAP_PATH}")

    if not SAVE_MAP_PATH:
        print("SAVE_MAP_PATH cannot be empty and stop scan")
        stop_scan_process()
        return

    save_map_dir = Path(SAVE_MAP_PATH)
    save_map_dir.mkdir(parents=True, exist_ok=True)

    if not SAVE_MAP_PATH.endswith("/"):
        SAVE_MAP_PATH = f"{SAVE_MAP_PATH}/"

    print(f"Using save map destination: {SAVE_MAP_PATH}")
    service_payload = (
        "{resolution: 0.2, destination: '" + SAVE_MAP_PATH + "'}"
    )
    run_shell_command(
        [
            "ros2",
            "service",
            "call",
            "/lio_sam/save_map",
            "lio_sam/srv/SaveMap",
            service_payload,
        ]
    )

    stop_scan_process()


def post_process_command() -> None:
    print(f"[{SCRIPT_PATH}] post_process: running post-processing steps")

    if not SAVE_MAP_PATH:
        raise RuntimeError("Error: SAVE_MAP_PATH cannot be empty")

    transform_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/transform_global_map"
    grid_builder_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/grid_map_builder"
    slam_map_post_processing_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/slam_map_post_processing"

    cfg_file = f"{SCRIPT_PATH}/ros2/install/lio_sam/share/lio_sam/config/grid_map_builder_cfg.yaml"
    loc_config_mid360_slope_file = f"{SCRIPT_PATH}/ros2/install/lio_sam/share/lio_sam/config/MsfLocConfig_mid360_slope.yaml"

    print("Running transform_global_map...")
    run_shell_command(
        [
            transform_cmd,
            f"{SAVE_MAP_PATH}/pointCloud",
            cfg_file,
            f"{SAVE_MAP_PATH}/tf_new_old_mat.txt",
            f"{SAVE_MAP_PATH}/global_map_tf",
        ]
    )

    print("Running grid_map_builder...")
    run_shell_command(
        [
            grid_builder_cmd,
            f"{SAVE_MAP_PATH}/global_map_tf/pointCloud",
            cfg_file,
            f"{SAVE_MAP_PATH}/loc_map/grid_map",
            "default",
        ]
    )

    print("Running slam_map_post_processing...")
    run_shell_command(
        [
            slam_map_post_processing_cmd,
            loc_config_mid360_slope_file,
            f"{SAVE_MAP_PATH}/global_map_tf",
            "50",
            f"{SAVE_MAP_PATH}/loc_map/split_map",
        ]
    )


def main() -> int:
    try:
        user_data_dir = validate_input_path(sys.argv)
        scan_command()
        function_two(user_data_dir)
        savemap_command(user_data_dir)
        post_process_command()
        print("全部步骤执行完成。")
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
