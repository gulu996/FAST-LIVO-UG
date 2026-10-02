#!/usr/bin/env python3
"""Replay one real legacy bag through the actual ROS Shadow Graph; no tuning."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
import xmlrpc.client
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", choices=("20m", "40m"))
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--port", type=int, default=11632)
    parser.add_argument("--counterfactual-snapshots", action="store_true",
                        help="read-only event capture; no runtime optimizer changes")
    args = parser.parse_args()
    bag = Path("/home/gulu/data/qr_detect") / (args.dataset + ".bag")
    if not bag.is_file():
        raise RuntimeError("required bag not found: " + str(bag))
    root = args.output_root.resolve()
    with socket.socket() as check:
        check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        check.bind(("127.0.0.1", args.port))
    root.mkdir(parents=True, exist_ok=False)  # never overwrite earlier evidence
    env = os.environ.copy()
    env.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1",
               ROS_HOME=str(root / "ros_home"), ROS_LOG_DIR=str(root / "ros_logs"))
    env["LD_PRELOAD"] = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"
    repo = Path(__file__).resolve().parents[1]
    for source in ("config/mid360.yaml", "config/legacy_qr_replay/mapping.yaml",
                   "config/legacy_qr_replay/camera_1280x1024.yaml",
                   "launch/mapping_legacy_qr_replay.launch"):
        shutil.copy2(repo / source, root / Path(source).name)
    with (root / "bag_info.yaml").open("w") as info:
        subprocess.run(["rosbag", "info", "--yaml", str(bag)], env=env,
                       stdout=info, check=True)
    metadata = dict(dataset_path=str(bag), dataset_class="LEGACY_SAME_ID_REAL_REPLAY",
                    candidate="A", replay_rate=1.0, port=args.port,
                    counterfactual_snapshot_enable=args.counterfactual_snapshots)
    launch_path = repo / "launch/mapping_legacy_qr_replay.launch"
    if args.counterfactual_snapshots:
        wrapper = ET.Element("launch")
        include = ET.SubElement(wrapper, "include", file=str(launch_path))
        ET.SubElement(include, "arg", name="output_root", value=str(root))
        ET.SubElement(wrapper, "param", name="/aruco_landmarks/global_backend/counterfactual_snapshot_enable",
                      type="bool", value="true")
        launch_path = root / "counterfactual_capture.launch"
        ET.ElementTree(wrapper).write(launch_path, encoding="unicode")
    processes, logs = [], []

    def start(command, name):
        log = (root / (name + ".log")).open("w")
        logs.append(log)
        proc = subprocess.Popen(command, env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, proc))
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
                raise RuntimeError("owned process needed forced shutdown")

    started = time.monotonic()
    try:
        core = start(["roscore", "-p", str(args.port)], "roscore")
        master = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        deadline = time.monotonic() + 20
        while True:
            if core.poll() is not None:
                raise RuntimeError("isolated roscore exited")
            try:
                if master.getPid("reobservation_regression")[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("isolated master startup timeout")
            time.sleep(0.2)
        launch_command = ["roslaunch", str(launch_path)]
        if not args.counterfactual_snapshots:
            launch_command.append("output_root:=" + str(root))
        launch = start(launch_command, "roslaunch")
        deadline = time.monotonic() + 30
        while True:
            if launch.poll() is not None:
                raise RuntimeError("mapper launch exited before replay")
            state = master.getSystemState("reobservation_regression")[2]
            subscribers = dict(state[1])
            if all("/laserMapping" in subscribers.get(topic, []) for topic in
                   ("/left_camera/image", "/livox/lidar", "/livox/imu")):
                break
            if time.monotonic() > deadline:
                raise RuntimeError("required mapper subscriptions missing")
            time.sleep(0.2)
        subprocess.run(["rosparam", "dump", str(root / "effective_params.yaml")],
                       env=env, check=True)
        play = start(["rosbag", "play", "--clock", "--wait-for-subscribers",
                      "--delay=0.2", "--rate=1.0", str(bag)], "rosbag")
        while play.poll() is None:
            if launch.poll() is not None:
                raise RuntimeError("mapper launch exited during replay")
            # ROS launch can remain alive after its mapping child crashes.
            if master.lookupNode("reobservation_regression", "/laserMapping")[0] != 1:
                raise RuntimeError("mapping child disappeared during replay")
            if time.monotonic() - started > 210:
                raise RuntimeError("full replay timeout")
            time.sleep(1)
        if play.returncode != 0:
            raise RuntimeError("rosbag play failed")
        time.sleep(3)  # wall-time drain after final bag message
        stop(launch)
        stop(core)
        runs = list(root.glob("*/landmark_backend_final.txt"))
        if len(runs) != 1 or not runs[0].with_name(
                "landmark_reobservation_diagnostics.csv").is_file():
            raise RuntimeError("shutdown summary or reobservation CSV missing")
        metadata.update(run_directory=str(runs[0].parent),
                        wall_time_s=time.monotonic() - started, success=True)
        print(json.dumps(metadata), flush=True)
    finally:
        for name, proc in reversed(processes):
            stop(proc)
            metadata[name + "_exit"] = proc.returncode
        for log in logs:
            log.close()
        (root / "replay_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
