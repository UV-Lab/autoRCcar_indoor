#!/usr/bin/env python3

import sys
import os
import json
import subprocess
import time
import shlex

from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent
print(SCRIPT_PATH)

# 配置部分 - 请根据实际情况修改这些路径或命令
JSON_FILE_PATH = "/tmp/run_mapping.json"





def log(message):
    """简单的日志打印函数"""
    print(f"[INFO] {message}")


def run_shell_command(command: list) -> None:
    try:
        subprocess.run(command, check=True)
        log("save_map 执行完毕。")
    except subprocess.CalledProcessError as e:
        log(f"save_map 执行失败: {e}")

def write_pid_file(driver_pid, sam_pid):
    """Step 6: 记录进程ID到json文件"""
    data = {
        "driver_pid": driver_pid,
        "sam_pid": sam_pid,
        "timestamp": time.time()
    }
    try:
        with open(JSON_FILE_PATH, 'w') as f:
            json.dump(data, f)
        log(f"进程ID已记录到 {JSON_FILE_PATH}")
    except Exception as e:
        log(f"写入JSON文件失败: {e}")

def read_pid_file():
    """读取json文件中的PID"""
    try:
        with open(JSON_FILE_PATH, 'r') as f:
            data = json.load(f)
            return data.get("driver_pid"), data.get("sam_pid")
    except Exception as e:
        log(f"读取JSON文件失败: {e}")
        return None, None

def kill_process_tree(pid):
    """递归杀死进程及其子进程"""
    if not pid:
        return
    try:
        # 使用 pgrep 查找子进程
        # 注意：这里使用简单的 kill -TERM，如果需要强制杀死可以用 -9
        subprocess.run(["pgrep", "-P", str(pid)], capture_output=True)
        # 先杀子进程，再杀父进程
        subprocess.run(["pkill", "-TERM", "-P", str(pid)])
        time.sleep(0.5)
        subprocess.run(["kill", "-TERM", str(pid)])
        log(f"已终止进程: {pid}")
    except Exception as e:
        log(f"终止进程 {pid} 失败: {e}")

def execute_start():
    """Step 3 -> Start 分支逻辑"""
    log("执行启动逻辑...")

    # Step 4: 检查文件是否存在
    if os.path.exists(JSON_FILE_PATH):
        # Step 5: 提示已有程序在运行，直接结束
        log("检测到 /tmp/run_mapping.json 存在，提示：已经有程序正在运行。")
        sys.exit(1)
    else:
        # Step 6: 启动脚本并记录PID
        log("启动 livox_ros_driver2...")
        # 使用 Popen 启动后台进程
        # stdout=subprocess.DEVNULL 表示不输出日志到当前终端，你可以改为 PIPE 或文件
        p1 = subprocess.Popen(["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        log("启动 lio_sam_run...")
        p2 = subprocess.Popen(["ros2", "run", "lio_sam", "run.launch.py"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        log(f"启动成功，PID分别为: {p1.pid}, {p2.pid}")
        write_pid_file(p1.pid, p2.pid)

def execute_stop():
    """Step 3 -> End 分支逻辑"""
    log("执行停止逻辑...")

    transform_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/transform_global_map"
    grid_builder_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/grid_map_builder"
    slam_map_post_processing_cmd = f"{SCRIPT_PATH}/ros2/install/lio_sam/lib/lio_sam/slam_map_post_processing"

    cfg_file = f"{SCRIPT_PATH}/ros2/install/lio_sam/share/lio_sam/config/grid_map_builder_cfg.yaml"
    loc_config_mid360_slope_file = f"{SCRIPT_PATH}/ros2/install/lio_sam/share/lio_sam/config/MsfLocConfig_mid360_slope.yaml"

    # Step 4: 检查文件是否存在
    if not os.path.exists(JSON_FILE_PATH):
        # Step 11: 提示没有运行的程序，直接结束
        log("检测到 /tmp/run_mapping.json 不存在，提示：没有运行的程序。")
        sys.exit(1)
    else:
        # Step 7: 等待用户输入一个字符串
        user_input = input(">>> 请在此输入任意字符以继续执行保存地图操作: ")
        log(f"用户输入: {user_input}")

        # Step 8: 执行 save_map 命令行
        log("正在执行 save_map...")


        print(f"Using save map destination: {user_input}")
        service_payload = ("{resolution: 0.2, destination: '" + user_input + "'}")
        
        run_shell_command(["ros2", "service", "call",  "/lio_sam/save_map", "lio_sam/srv/SaveMap",
        f"{shlex.quote(service_payload)}"])
        # 根据需求决定失败是否继续，这里选择继续尝试清理

        # Step 9: 读取JSON，kill进程
        driver_pid, sam_pid = read_pid_file()

        log("正在终止相关进程...")
        kill_process_tree(driver_pid)
        kill_process_tree(sam_pid)

        # 清理 JSON 文件
        try:
            os.remove(JSON_FILE_PATH)
            log("已清理临时文件 /tmp/run_mapping.json")
        except OSError:
            log("清理临时文件失败")

        # 等待进程彻底关闭
        time.sleep(1)


        print("Running transform_global_map...")
        run_shell_command(
            [
                transform_cmd,
                f"{user_input}/pointCloud",
                cfg_file,
                f"{user_input}/tf_new_old_mat.txt",
                f"{user_input}/global_map_tf",
            ]
        )

        print("Running grid_map_builder...")
        run_shell_command(
            [
                grid_builder_cmd,
                f"{user_input}/global_map_tf/pointCloud",
                cfg_file,
                f"{user_input}/loc_map/grid_map",
                "default",
            ]
        )

        print("Running slam_map_post_processing...")
        run_shell_command(
            [
                slam_map_post_processing_cmd,
                loc_config_mid360_slope_file,
                f"{user_input}/global_map_tf",
                "50",
                f"{user_input}/loc_map/split_map",
            ]
        )

        

def main():
    """主函数逻辑"""

    # Step 1: 脚本接受一个参数 (action)
    # Step 2: 如果参数数量和值不对，则退出
    if len(sys.argv) != 2 or sys.argv[1] not in ['start', 'stop']:
        print("Usage: python3 manager.py [start|stop]")
        sys.exit(1)

    action = sys.argv[1]

    # Step 3: 参数值判断
    if action == 'start':
        execute_start()
    elif action == 'stop':
        execute_stop()

if __name__ == "__main__":
    main()