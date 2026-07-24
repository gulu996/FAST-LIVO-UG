#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <image_transport/image_transport.h>
#include <algorithm>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <errno.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>

#include <livox_ros_driver2/shared_timestamp_state.h>
#include <mvs_ros_driver/timestamp_monitor.h>

using namespace std;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A,
  BayerBG8 = 0x0108000B
};
// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
int width, height;
image_transport::Publisher pub;
ros::Publisher timestamp_diagnostic_pub;
std::vector<PixelFormat> PIXEL_FORMAT = { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8, BayerBG8 };
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;
double shared_stamp_stale_timeout_sec = 0.5;
double max_header_step_sec = 0.5;
double performance_log_period_sec = 5.0;
std::string camera_hardware_id;

diagnostic_msgs::KeyValue DiagnosticValue(const std::string& key,
                                          const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

const char* ValidationResultName(
    mvs_ros_driver::StampValidationResult result) {
  switch (result) {
    case mvs_ros_driver::StampValidationResult::kAccepted:
      return "accepted";
    case mvs_ros_driver::StampValidationResult::kWriterEpochChanged:
      return "writer_epoch_changed";
    case mvs_ros_driver::StampValidationResult::kStale:
      return "stale";
    case mvs_ros_driver::StampValidationResult::kDuplicate:
      return "duplicate";
    case mvs_ros_driver::StampValidationResult::kNonMonotonic:
      return "non_monotonic";
    case mvs_ros_driver::StampValidationResult::kLargeStep:
      return "large_step";
  }
  return "unknown";
}

const char* GrabStrategyName(int strategy) {
  switch (strategy) {
    case MV_GrabStrategy_OneByOne:
      return "OneByOne";
    case MV_GrabStrategy_LatestImagesOnly:
      return "LatestImagesOnly";
    case MV_GrabStrategy_LatestImages:
      return "LatestImages";
    case MV_GrabStrategy_UpcomingImage:
      return "UpcomingImage";
    default:
      return "Invalid";
  }
}

class SharedTimestampReader {
 public:
  SharedTimestampReader(std::string path, double retry_sec)
      : path_(std::move(path)),
        retry_ns_(static_cast<uint64_t>(std::max(0.1, retry_sec) * 1e9)) {}

  ~SharedTimestampReader() { Disconnect(); }

  livox_ros::SharedTimestampReadResult Read(
      livox_ros::SharedTimestampSnapshot* snapshot, std::string* error) {
    const uint64_t now_ns = livox_ros::MonotonicNowNs();
    if (state_ == nullptr && !Connect(now_ns, error)) {
      return livox_ros::kSharedTimestampReadBadProtocol;
    }
    const auto result = livox_ros::ReadSharedTimestampState(state_, snapshot);
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

    void* mapping = mmap(nullptr, sizeof(livox_ros::SharedTimestampState),
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
    ROS_INFO("Connected shared timestamp reader: %s", path_.c_str());
    return true;
  }

  void Disconnect() {
    if (state_ != nullptr) {
      if (munmap(const_cast<livox_ros::SharedTimestampState*>(state_),
                 sizeof(livox_ros::SharedTimestampState)) != 0) {
        ROS_WARN("munmap shared timestamp failed: %s", strerror(errno));
      }
      state_ = nullptr;
    }
  }

  std::string path_;
  uint64_t retry_ns_;
  uint64_t last_connect_attempt_ns_ = 0;
  const livox_ros::SharedTimestampState* state_ = nullptr;
};

std::unique_ptr<SharedTimestampReader> shared_timestamp_reader;

struct PerformanceStats {
  uint64_t count = 0;
  double get_ms_sum = 0;
  double convert_ms_sum = 0;
  double resize_ms_sum = 0;
  double total_ms_sum = 0;
  double get_ms_max = 0;
  double convert_ms_max = 0;
  double resize_ms_max = 0;
  double total_ms_max = 0;
  std::chrono::steady_clock::time_point last_log =
      std::chrono::steady_clock::now();

  void Add(double get_ms, double convert_ms, double resize_ms,
           double total_ms) {
    ++count;
    get_ms_sum += get_ms;
    convert_ms_sum += convert_ms;
    resize_ms_sum += resize_ms;
    total_ms_sum += total_ms;
    get_ms_max = std::max(get_ms_max, get_ms);
    convert_ms_max = std::max(convert_ms_max, convert_ms);
    resize_ms_max = std::max(resize_ms_max, resize_ms);
    total_ms_max = std::max(total_ms_max, total_ms);
  }

  void MaybeLog() {
    const auto now = std::chrono::steady_clock::now();
    if (count == 0 ||
        std::chrono::duration<double>(now - last_log).count() <
            performance_log_period_sec) {
      return;
    }
    ROS_INFO(
        "Camera timing %lu frames: mean/max ms get=%.3f/%.3f "
        "convert=%.3f/%.3f resize=%.3f/%.3f total=%.3f/%.3f",
        count, get_ms_sum / count, get_ms_max,
        convert_ms_sum / count, convert_ms_max,
        resize_ms_sum / count, resize_ms_max,
        total_ms_sum / count, total_ms_max);
    *this = PerformanceStats();
  }
};


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
  {
    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  else
  {
    printf("Not support.\n");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];   
  if (image_scale < 0.1) image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  float AcquisitionFrameRate_ = Params["AcquisitionFrameRate"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
      std::string msg = "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO_STREAM(msg.c_str());
  } else {
    if(ExposureAutoMode == 2) {
      ROS_WARN_STREAM("Fail to set Exposure Auto Mode to Continues");
    }
    else {
      ROS_INFO_STREAM(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gamma");
  }

  nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", AcquisitionFrameRate_);
  if (MV_OK == nRet) {
    std::string msg = "Set AcquisitionFrameRate: " + std::to_string(AcquisitionFrameRate_);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set AcquisitionFrameRate");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {  // 捕捉 Ctrl + C 触发的 SIGINT 信号
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;    // 设置退出标志
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
  sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

struct FrameDiagnosticData {
  uint64_t ros_publish_seq = 0;
  uint64_t assigned_header_stamp_ns = 0;
  uint64_t shared_lidar_stamp_ns = 0;
  uint64_t camera_receive_ros_time_ns = 0;
  uint64_t publish_ros_time_ns = 0;
  uint64_t writer_epoch = 0;
  uint64_t shared_write_sequence = 0;
  uint32_t sdk_valid_image_count = 0;
  double get_frame_ms = 0;
  double pixel_convert_ms = 0;
  double resize_ms = 0;
  double total_processing_ms = 0;
  bool image_published = false;
  std::string timestamp_source;
  std::string status_message;
};

void PublishCameraDiagnostic(
    const MV_FRAME_OUT_INFO_EX& frame,
    const mvs_ros_driver::TimestampMonitor& monitor,
    const FrameDiagnosticData& data) {
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status;
  status.level = data.image_published
                     ? diagnostic_msgs::DiagnosticStatus::OK
                     : diagnostic_msgs::DiagnosticStatus::ERROR;
  status.name = "mvs_ros_driver/camera_timestamp";
  status.hardware_id = camera_hardware_id;
  status.message = data.status_message;

  const uint64_t camera_device_timestamp =
      (static_cast<uint64_t>(frame.nDevTimeStampHigh) << 32U) |
      static_cast<uint64_t>(frame.nDevTimeStampLow);
  status.values.push_back(DiagnosticValue(
      "ros_publish_seq", std::to_string(data.ros_publish_seq)));
  status.values.push_back(DiagnosticValue(
      "camera_frame_num", std::to_string(frame.nFrameNum)));
  status.values.push_back(DiagnosticValue(
      "camera_frame_counter", std::to_string(frame.nFrameCounter)));
  status.values.push_back(DiagnosticValue(
      "camera_trigger_index", std::to_string(frame.nTriggerIndex)));
  status.values.push_back(DiagnosticValue(
      "camera_device_timestamp_high",
      std::to_string(frame.nDevTimeStampHigh)));
  status.values.push_back(DiagnosticValue(
      "camera_device_timestamp_low",
      std::to_string(frame.nDevTimeStampLow)));
  status.values.push_back(DiagnosticValue(
      "camera_device_timestamp_combined",
      std::to_string(camera_device_timestamp)));
  status.values.push_back(DiagnosticValue(
      "camera_host_timestamp", std::to_string(frame.nHostTimeStamp)));
  status.values.push_back(DiagnosticValue(
      "camera_lost_packet", std::to_string(frame.nLostPacket)));
  status.values.push_back(DiagnosticValue(
      "assigned_header_stamp_ns",
      std::to_string(data.assigned_header_stamp_ns)));
  status.values.push_back(DiagnosticValue(
      "assigned_timestamp_source", data.timestamp_source));
  status.values.push_back(DiagnosticValue(
      "shared_lidar_stamp_ns",
      std::to_string(data.shared_lidar_stamp_ns)));
  status.values.push_back(DiagnosticValue(
      "camera_receive_ros_time_ns",
      std::to_string(data.camera_receive_ros_time_ns)));
  status.values.push_back(DiagnosticValue(
      "publish_ros_time_ns", std::to_string(data.publish_ros_time_ns)));
  status.values.push_back(DiagnosticValue(
      "frame_gap_count", std::to_string(monitor.frame_gap_count())));
  status.values.push_back(DiagnosticValue(
      "frame_counter_gap_count",
      std::to_string(monitor.frame_counter_gap_count())));
  status.values.push_back(DiagnosticValue(
      "trigger_gap_count", std::to_string(monitor.trigger_gap_count())));
  status.values.push_back(DiagnosticValue(
      "device_time_wrap_count",
      std::to_string(monitor.device_time_wrap_count())));
  status.values.push_back(DiagnosticValue(
      "duplicate_stamp_count",
      std::to_string(monitor.duplicate_stamp_count())));
  status.values.push_back(DiagnosticValue(
      "non_monotonic_stamp_count",
      std::to_string(monitor.non_monotonic_stamp_count())));
  status.values.push_back(DiagnosticValue(
      "invalid_shared_stamp_count",
      std::to_string(monitor.invalid_shared_stamp_count())));
  status.values.push_back(DiagnosticValue(
      "stale_stamp_count", std::to_string(monitor.stale_stamp_count())));
  status.values.push_back(DiagnosticValue(
      "large_step_count", std::to_string(monitor.large_step_count())));
  status.values.push_back(DiagnosticValue(
      "writer_restart_count",
      std::to_string(monitor.writer_restart_count())));
  status.values.push_back(DiagnosticValue(
      "writer_epoch", std::to_string(data.writer_epoch)));
  status.values.push_back(DiagnosticValue(
      "shared_write_sequence",
      std::to_string(data.shared_write_sequence)));
  status.values.push_back(DiagnosticValue(
      "sdk_valid_image_count",
      std::to_string(data.sdk_valid_image_count)));
  status.values.push_back(DiagnosticValue(
      "image_published", data.image_published ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "get_one_frame_ms", std::to_string(data.get_frame_ms)));
  status.values.push_back(DiagnosticValue(
      "pixel_conversion_ms", std::to_string(data.pixel_convert_ms)));
  status.values.push_back(
      DiagnosticValue("resize_ms", std::to_string(data.resize_ms)));
  status.values.push_back(DiagnosticValue(
      "total_processing_ms", std::to_string(data.total_processing_ms)));
  array.status.push_back(std::move(status));
  timestamp_diagnostic_pub.publish(array);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  
  unsigned char* pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);
  unsigned char* pDataForBGR = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

  if (pData == nullptr || pDataForBGR == nullptr) {
    printf("Memory allocation failed!\n");
    if (pData) free(pData);
    if (pDataForBGR) free(pDataForBGR);
    return nullptr;
  }

  mvs_ros_driver::TimestampMonitor timestamp_monitor(
      static_cast<uint64_t>(shared_stamp_stale_timeout_sec * 1e9),
      static_cast<uint64_t>(max_header_step_sec * 1e9));
  PerformanceStats performance;
  uint64_t published_sequence = 0;

  while (!exit_flag && ros::ok()) {
    const auto frame_start = std::chrono::steady_clock::now();
    const auto get_start = frame_start;
    nRet = MV_CC_GetOneFrameTimeout(
        pUser, pData, stParam.nCurValue * 3, &stImageInfo, 1000);
    const auto get_end = std::chrono::steady_clock::now();
    if (nRet != MV_OK) {
      ROS_WARN_THROTTLE(5.0,
                        "MV_CC_GetOneFrameTimeout failed: 0x%x", nRet);
      continue;
    }

    FrameDiagnosticData diagnostic;
    diagnostic.ros_publish_seq = published_sequence;
    diagnostic.camera_receive_ros_time_ns = ros::Time::now().toNSec();
    diagnostic.get_frame_ms =
        std::chrono::duration<double, std::milli>(get_end - get_start)
            .count();
    unsigned int valid_image_count = 0;
    if (MV_CC_GetValidImageNum(pUser, &valid_image_count) == MV_OK) {
      diagnostic.sdk_valid_image_count = valid_image_count;
    }

    const uint64_t device_timestamp =
        (static_cast<uint64_t>(stImageInfo.nDevTimeStampHigh) << 32U) |
        static_cast<uint64_t>(stImageInfo.nDevTimeStampLow);
    timestamp_monitor.ObserveCameraMetadata(
        stImageInfo.nFrameNum, stImageInfo.nFrameCounter,
        stImageInfo.nTriggerIndex, device_timestamp);

    ros::Time assigned_stamp;
    bool timestamp_valid = true;
    if (trigger_enable) {
      livox_ros::SharedTimestampSnapshot snapshot;
      std::string shared_error;
      const auto read_result =
          shared_timestamp_reader
              ? shared_timestamp_reader->Read(&snapshot, &shared_error)
              : livox_ros::kSharedTimestampReadBadProtocol;
      diagnostic.shared_lidar_stamp_ns = snapshot.stamp_ns;
      diagnostic.writer_epoch = snapshot.writer_epoch;
      diagnostic.shared_write_sequence = snapshot.write_sequence;
      if (read_result != livox_ros::kSharedTimestampReadOk) {
        timestamp_monitor.CountInvalidSharedStamp();
        timestamp_valid = false;
        diagnostic.timestamp_source = "INVALID_SHARED_TIMESTAMP";
        diagnostic.status_message =
            shared_error.empty() ? "shared_timestamp_unavailable"
                                 : shared_error;
      } else {
        const auto validation = timestamp_monitor.Validate(
            snapshot, livox_ros::MonotonicNowNs());
        timestamp_valid =
            validation ==
            mvs_ros_driver::StampValidationResult::kAccepted;
        diagnostic.timestamp_source =
            livox_ros::SharedTimestampClockSourceName(
                snapshot.clock_source);
        diagnostic.status_message = ValidationResultName(validation);
        if (timestamp_valid) {
          assigned_stamp.fromNSec(snapshot.stamp_ns);
          diagnostic.assigned_header_stamp_ns = snapshot.stamp_ns;
        }
      }
    } else {
      assigned_stamp = ros::Time::now();
      diagnostic.assigned_header_stamp_ns = assigned_stamp.toNSec();
      diagnostic.timestamp_source = "ROS_NOW_NON_TRIGGER";
      diagnostic.status_message = "non_trigger_mode";
    }

    if (!timestamp_valid) {
      diagnostic.publish_ros_time_ns = ros::Time::now().toNSec();
      diagnostic.total_processing_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - frame_start)
              .count();
      performance.Add(diagnostic.get_frame_ms, 0, 0,
                      diagnostic.total_processing_ms);
      performance.MaybeLog();
      PublishCameraDiagnostic(stImageInfo, timestamp_monitor, diagnostic);
      ROS_ERROR_THROTTLE(
          5.0,
          "Dropping triggered camera frame %u: %s. "
          "Trigger mode never falls back to ros::Time::now().",
          stImageInfo.nFrameNum, diagnostic.status_message.c_str());
      continue;
    }

    stConvertParam.nWidth = stImageInfo.nWidth;
    stConvertParam.nHeight = stImageInfo.nHeight;
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
    stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = pDataForBGR;
    stConvertParam.nDstBufferSize = stParam.nCurValue * 3;
    const auto convert_start = std::chrono::steady_clock::now();
    nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
    const auto convert_end = std::chrono::steady_clock::now();
    diagnostic.pixel_convert_ms =
        std::chrono::duration<double, std::milli>(
            convert_end - convert_start)
            .count();
    if (MV_OK != nRet) {
      diagnostic.status_message = "pixel_conversion_failed";
      diagnostic.publish_ros_time_ns = ros::Time::now().toNSec();
      diagnostic.total_processing_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - frame_start)
              .count();
      performance.Add(diagnostic.get_frame_ms,
                      diagnostic.pixel_convert_ms, 0,
                      diagnostic.total_processing_ms);
      performance.MaybeLog();
      PublishCameraDiagnostic(stImageInfo, timestamp_monitor, diagnostic);
      ROS_ERROR_THROTTLE(
          5.0, "MV_CC_ConvertPixelType failed: 0x%x", nRet);
      continue;
    }

    cv::Mat srcImage(stImageInfo.nHeight, stImageInfo.nWidth,
                     CV_8UC3, pDataForBGR);
    const auto resize_start = std::chrono::steady_clock::now();
    if (std::fabs(image_scale - 1.0F) > 1e-6F) {
      cv::resize(srcImage, srcImage,
                 cv::Size(srcImage.cols * image_scale,
                          srcImage.rows * image_scale),
                 cv::INTER_LINEAR);
    }
    const auto resize_end = std::chrono::steady_clock::now();
    diagnostic.resize_ms =
        std::chrono::duration<double, std::milli>(
            resize_end - resize_start)
            .count();

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
    msg->header.seq = static_cast<uint32_t>(published_sequence);
    msg->header.stamp = assigned_stamp;
    diagnostic.publish_ros_time_ns = ros::Time::now().toNSec();
    diagnostic.total_processing_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - frame_start)
            .count();
    diagnostic.image_published = true;
    diagnostic.status_message =
        trigger_enable ? "published_with_legacy_lidar_base_time"
                       : "published_with_ros_now_non_trigger";
    pub.publish(msg);
    ++published_sequence;
    performance.Add(diagnostic.get_frame_ms,
                    diagnostic.pixel_convert_ms, diagnostic.resize_ms,
                    diagnostic.total_processing_ms);
    performance.MaybeLog();
    PublishCameraDiagnostic(stImageInfo, timestamp_monitor, diagnostic);
  }

  if (pData) {
    free(pData);
    pData = nullptr;
  }

  if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = nullptr;
  }

  return 0;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mvs_trigger");
  if (argc < 2) {
    ROS_FATAL("Camera parameter file argument is required.");
    return -1;
  }
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  ros::NodeHandle private_node("~");
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;
  ros::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  camera_hardware_id = expect_serial_number;
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  pub = it.advertise(pub_topic, 1);
  timestamp_diagnostic_pub =
      private_node.advertise<diagnostic_msgs::DiagnosticArray>(
          "timestamp_diagnostics", 10);

  std::string shared_timestamp_path = "/home/gulu/timeshare";
  double shared_connect_retry_sec = 1.0;
  int image_node_num = 3;
  int grab_strategy = MV_GrabStrategy_LatestImagesOnly;
  int output_queue_size = 1;
  private_node.param<std::string>("shared_timestamp_path",
                                  shared_timestamp_path,
                                  "/home/gulu/timeshare");
  private_node.param("shared_timestamp_connect_retry_sec",
                     shared_connect_retry_sec, 1.0);
  private_node.param("shared_timestamp_stale_timeout_sec",
                     shared_stamp_stale_timeout_sec, 0.5);
  private_node.param("max_header_step_sec", max_header_step_sec, 0.5);
  private_node.param("performance_log_period_sec",
                     performance_log_period_sec, 5.0);
  private_node.param("image_node_num", image_node_num, 3);
  private_node.param("grab_strategy", grab_strategy,
                     static_cast<int>(MV_GrabStrategy_LatestImagesOnly));
  private_node.param("output_queue_size", output_queue_size, 1);

  shared_stamp_stale_timeout_sec =
      std::max(0.01, shared_stamp_stale_timeout_sec);
  max_header_step_sec = std::max(0.01, max_header_step_sec);
  performance_log_period_sec = std::max(0.1, performance_log_period_sec);
  image_node_num = std::max(1, image_node_num);
  output_queue_size =
      std::max(1, std::min(output_queue_size, image_node_num));
  if (grab_strategy < MV_GrabStrategy_OneByOne ||
      grab_strategy > MV_GrabStrategy_UpcomingImage) {
    ROS_FATAL("Invalid MVS grab_strategy=%d", grab_strategy);
    return -1;
  }
  if (trigger_enable) {
    shared_timestamp_reader.reset(
        new SharedTimestampReader(shared_timestamp_path,
                                  shared_connect_retry_sec));
  }

  SetupSignalHandler();
 
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0)
  {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL)
      {
        printf("Device info is NULL for device %d\n", i);
        return -1;
      } 
      PrintDeviceInfo(pDeviceInfo);            
    }  
  } 
  else
  {
    printf("Find No Devices!\n");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) 
  {
    if (expect_serial_number.empty()) 
    {
      ROS_ERROR("Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) 
    {
      if (stDeviceList.pDeviceInfo[i] == NULL) 
      {
        printf("Device info is NULL for device %d\n", i);
        continue;
      }
      
      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber);
      }
      else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
      }
      else
      {
        printf("Unknown device type!\n");
        continue;
      }
      if (serial_number.empty()) 
      {
        printf("Serial number is empty for device %d\n", i);
        continue;
      }
      if (expect_serial_number == serial_number) 
      {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) 
    {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      return -1;
    }
  }
  else
  {
    nIndex = 0;
  }
  
  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet)
  {
    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet)
  {
    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_SetImageNodeNum(handle,
                               static_cast<unsigned int>(image_node_num));
  if (MV_OK != nRet) {
    ROS_FATAL("MV_CC_SetImageNodeNum(%d) failed: 0x%x",
              image_node_num, nRet);
    return -1;
  }
  nRet = MV_CC_SetGrabStrategy(
      handle, static_cast<MV_GRAB_STRATEGY>(grab_strategy));
  if (MV_OK != nRet) {
    ROS_FATAL("MV_CC_SetGrabStrategy(%s) failed: 0x%x",
              GrabStrategyName(grab_strategy), nRet);
    return -1;
  }
  if (grab_strategy == MV_GrabStrategy_LatestImages) {
    nRet = MV_CC_SetOutputQueueSize(
        handle, static_cast<unsigned int>(output_queue_size));
    if (MV_OK != nRet) {
      ROS_FATAL("MV_CC_SetOutputQueueSize(%d) failed: 0x%x",
                output_queue_size, nRet);
      return -1;
    }
  }
  ROS_INFO("MVS buffering: strategy=%s image_node_num=%d "
           "output_queue_size=%d",
           GrabStrategyName(grab_strategy), image_node_num,
           output_queue_size);

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
  if (MV_OK != nRet) {
    printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
    return -1;
  }

  // MVCC_INTVALUE stParam;
  // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  // if (MV_OK != nRet) {
  //   printf("Get PayloadSize fail\n");
  //   return -1;
  // }
  // g_nPayloadSize = stParam.nCurValue * 3;

  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014 BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    printf("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    return -1;
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
    return -1;
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    printf("Start Grabbing fail.\n");
    return -1;
  }

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    printf("thread create failed.ret = %d\n", nRet);
    return -1;
  }

  while (!exit_flag && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO_STREAM("Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  shared_timestamp_reader.reset();

  return 0;
}
