#!/usr/bin/env python3
"""Check the mapper's actual idle-rate declaration with the installed ROS library.

Run: python3 tools/imu_eof_callback_self_test.py
Requires the existing ROS roscpp development environment and a C++ compiler.
No ROS master, bag, sensor publishing, or estimator state is involved.
"""
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile


def main():
    source = (Path(sys.argv[1]) if len(sys.argv) > 1 else
              Path(__file__).resolve().parents[1] / "src/LIVMapper.cpp")
    run = source.read_text().split("void LIVMapper::run()", 1)[1].split(
        "void LIVMapper::prop_imu_once", 1)[0]
    declarations = re.findall(r"ros::(?:Wall)?Rate\s+rate\(5000\);", run)
    if len(declarations) != 1:
        raise RuntimeError("Expected one mapper idle-rate declaration; review this test")
    # ponytail: exercise the source declaration without linking the whole mapper;
    # real mapper/input/state regression remains a separate integration check.
    code = r'''
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <vector>

struct Callback : ros::CallbackInterface {
  std::vector<int>& seen;
  int identity;
  Callback(std::vector<int>& s, int id) : seen(s), identity(id) {}
  CallResult call() override { seen.push_back(identity); return Success; }
};

bool check(bool advance_clock) {
  ros::Time::setNow(ros::Time(10, 0));
  ros::CallbackQueue queue;
  std::vector<int> seen;
  std::atomic<bool> stop{false};
  std::atomic<int> count{0};
  std::promise<void> entering;
  auto ready = entering.get_future();
  auto worker = std::async(std::launch::async, [&] {
    RATE_DECLARATION
    queue.callAvailable(ros::WallDuration(0));
    entering.set_value();
    while (!stop) {
      rate.sleep();
      queue.callAvailable(ros::WallDuration(0));
      count = seen.size();
    }
  });
  ready.wait();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  for (int i = 0; i != 20; ++i) {
    queue.addCallback(boost::make_shared<Callback>(seen, i));
    if (advance_clock) ros::Time::setNow(ros::Time(10, (i + 1) * 1000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (count != 20 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const bool delivered_before_shutdown = count == 20;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  stop = true;
  // Unblock the old ROS-time implementation after a failed test; never hang CI.
  ros::Time::setNow(ros::Time(11, 0));
  if (worker.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    ros::Time::shutdown();
    worker.wait();
    return false;
  }
  worker.get();
  if (!delivered_before_shutdown || seen.size() != 20 || !queue.isEmpty()) return false;
  for (int i = 0; i != 20; ++i) if (seen[i] != i) return false;
  return true;
}

int main() {
  ros::Time::init();
  const bool frozen = check(false);
  const bool advancing = check(true);
  std::cout << "frozen_clock=" << frozen << " advancing_clock=" << advancing << '\n';
  return frozen && advancing ? 0 : 1;
}
'''.replace("RATE_DECLARATION", declarations[0])
    flags = shlex.split(subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "roscpp"], text=True))
    with tempfile.TemporaryDirectory(prefix="imu_eof_self_test_") as directory:
        cpp, binary = Path(directory) / "test.cpp", Path(directory) / "test"
        cpp.write_text(code)
        subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-O2",
                        str(cpp), "-o", str(binary), *flags], check=True)
        subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == "__main__":
    main()
