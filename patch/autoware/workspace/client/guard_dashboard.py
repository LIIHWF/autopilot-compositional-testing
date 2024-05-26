import subprocess
import time
import sys
import os
import signal

home = os.path.expanduser("~")

def write_status(status: str):
    with open(os.path.join(home, "config/dashboard_status.txt"), "w") as file:
        file.write(status)

def read_status():
    with open(os.path.join(home, "config/dashboard_status.txt"), "r") as file:
        return file.read()

def read_log():
    with open(os.path.join(home, "config/dashboard_log.txt"), "r") as file:
        return file.read()

def get_pid():
    command = """ps aux | grep "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch autoware_launch planning_simulator.launch.xml" | grep -v grep | awk '{print $2}'"""
    return int(subprocess.check_output(command, shell=True).decode("utf-8").strip())

def start_dashboard_and_wait_finished():
    write_status("starting")
    with open(os.path.join(home, "config/dashboard_log.txt"), "w") as log_file:
        process = subprocess.Popen(
            ['bash', os.path.join(home, 'workspace/scripts/start_dashboard.sh')],
            stdout = log_file,
            stderr = log_file,
        )
        
        while True:
            sys.stdout.flush()
            if read_log().count("waiting for initial pose") > 1:
                write_status("started")
                break
            time.sleep(1)
        while True:
            status = read_status().strip()
            if status == "finished":
                os.kill(get_pid(), signal.SIGINT)
                break
            time.sleep(1)
        process.wait()


while True:
    start_dashboard_and_wait_finished()
