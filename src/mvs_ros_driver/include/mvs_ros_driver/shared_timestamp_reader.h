#ifndef MVS_ROS_DRIVER_SHARED_TIMESTAMP_READER_H_
#define MVS_ROS_DRIVER_SHARED_TIMESTAMP_READER_H_

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <ros/ros.h>

#include <livox_ros_driver2/shared_timestamp_state.h>

namespace mvs_ros_driver {

class SharedTimestampReader {
 public:
  SharedTimestampReader(std::string path, double retry_sec)
      : path_(std::move(path)),
        retry_ns_(static_cast<uint64_t>(
            std::max(0.1, retry_sec) * 1000000000.0)) {}

  ~SharedTimestampReader() { Disconnect(); }

  livox_ros::SharedTimestampReadResult Read(
      livox_ros::SharedTimestampSnapshot* snapshot, std::string* error) {
    const uint64_t now_ns = livox_ros::MonotonicNowNs();
    if (state_ == nullptr && !Connect(now_ns, error)) {
      return livox_ros::kSharedTimestampReadBadProtocol;
    }
    const auto result =
        livox_ros::ReadSharedTimestampState(state_, snapshot);
    if (result != livox_ros::kSharedTimestampReadOk && error != nullptr) {
      switch (result) {
        case livox_ros::kSharedTimestampReadInconsistent:
          *error = "shared_state_inconsistent";
          break;
        case livox_ros::kSharedTimestampReadBadProtocol:
          *error = "shared_protocol_magic_or_version_invalid";
          break;
        case livox_ros::kSharedTimestampReadNotReady:
          *error = "shared_writer_not_ready";
          break;
        case livox_ros::kSharedTimestampReadEmpty:
          *error = "shared_stamp_zero";
          break;
        default:
          *error = "shared_read_failed";
          break;
      }
    }
    return result;
  }

 private:
  bool Connect(uint64_t now_ns, std::string* error) {
    if (last_connect_attempt_ns_ != 0 &&
        now_ns - last_connect_attempt_ns_ < retry_ns_) {
      if (error != nullptr) {
        *error = "shared_writer_unavailable_retry_pending";
      }
      return false;
    }
    last_connect_attempt_ns_ = now_ns;

    const int fd = open(path_.c_str(), O_RDONLY);
    if (fd < 0) {
      if (error != nullptr) {
        *error = "open_failed: " + std::string(strerror(errno));
      }
      return false;
    }
    struct stat file_stat = {};
    if (fstat(fd, &file_stat) != 0) {
      if (error != nullptr) {
        *error = "fstat_failed: " + std::string(strerror(errno));
      }
      close(fd);
      return false;
    }
    if (file_stat.st_size !=
        static_cast<off_t>(sizeof(livox_ros::SharedTimestampState))) {
      if (error != nullptr) {
        *error = "shared_file_size_invalid";
      }
      close(fd);
      return false;
    }

    void* mapping =
        mmap(nullptr, sizeof(livox_ros::SharedTimestampState),
             PROT_READ, MAP_SHARED, fd, 0);
    const int mmap_errno = errno;
    close(fd);
    if (mapping == MAP_FAILED) {
      if (error != nullptr) {
        *error = "mmap_failed: " + std::string(strerror(mmap_errno));
      }
      return false;
    }
    state_ =
        static_cast<const livox_ros::SharedTimestampState*>(mapping);
    ROS_INFO("Connected shared timestamp reader");
    return true;
  }

  void Disconnect() {
    if (state_ == nullptr) {
      return;
    }
    if (munmap(const_cast<livox_ros::SharedTimestampState*>(state_),
               sizeof(livox_ros::SharedTimestampState)) != 0) {
      ROS_WARN("munmap shared timestamp failed: %s", strerror(errno));
    }
    state_ = nullptr;
  }

  std::string path_;
  uint64_t retry_ns_;
  uint64_t last_connect_attempt_ns_ = 0;
  const livox_ros::SharedTimestampState* state_ = nullptr;
};

}  // namespace mvs_ros_driver

#endif  // MVS_ROS_DRIVER_SHARED_TIMESTAMP_READER_H_
