/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.
*/

#include "uwb_manager.h"
#include "gnss_fusion_policy.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xmlrpcpp/XmlRpcValue.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <tuple>
#include <unistd.h>

bool correctUwbRangeValue(double raw_range_m, double range_bias_m,
                          double &corrected_range_m, UwbRejectReason &reject_reason)
{
  corrected_range_m = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(raw_range_m) || raw_range_m <= 0.0)
  {
    reject_reason = UwbRejectReason::INVALID_RAW_RANGE;
    return false;
  }
  if (!std::isfinite(range_bias_m))
  {
    reject_reason = UwbRejectReason::INVALID_RANGE_STATUS;
    return false;
  }

  corrected_range_m = raw_range_m - range_bias_m;
  if (!std::isfinite(corrected_range_m) || corrected_range_m <= 0.0)
  {
    reject_reason = UwbRejectReason::INVALID_CORRECTED_RANGE;
    return false;
  }
  reject_reason = UwbRejectReason::NONE;
  return true;
}

bool isUwbReplayCrossFormatDuplicate(const UwbRangeMeasurement &debug_measurement,
                                     const UwbRangeMeasurement &distance_measurement,
                                     double max_time_difference_s)
{
  if (debug_measurement.source_format != "uwbdbg" ||
      distance_measurement.source_format != "distance" ||
      debug_measurement.anchor_id != distance_measurement.anchor_id ||
      !std::isfinite(debug_measurement.raw_range_m) ||
      !std::isfinite(distance_measurement.raw_range_m))
    return false;
  const int64_t debug_range_mm =
      static_cast<int64_t>(std::llround(debug_measurement.raw_range_m * 1000.0));
  const int64_t distance_range_mm =
      static_cast<int64_t>(std::llround(distance_measurement.raw_range_m * 1000.0));
  return debug_range_mm == distance_range_mm &&
         std::fabs(debug_measurement.stamp - distance_measurement.stamp) <=
             std::max(0.0, max_time_difference_s);
}

namespace
{
bool xmlRpcHasMember(const XmlRpc::XmlRpcValue &value, const std::string &key)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeStruct && value.hasMember(key);
}

bool xmlRpcToDouble(const XmlRpc::XmlRpcValue &value, double &out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  return false;
}

bool xmlRpcToInt(const XmlRpc::XmlRpcValue &value, int &out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    const double d = static_cast<double>(value);
    out = static_cast<int>(std::llround(d));
    return std::fabs(d - static_cast<double>(out)) < 1e-6;
  }
  return false;
}

bool xmlRpcToBool(const XmlRpc::XmlRpcValue &value, bool &out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    out = static_cast<bool>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = (static_cast<int>(value) != 0);
    return true;
  }
  return false;
}

bool xmlRpcToString(const XmlRpc::XmlRpcValue &value, std::string &out)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  out = static_cast<std::string>(value);
  return true;
}

bool xmlRpcToVec3(const XmlRpc::XmlRpcValue &value, V3D &out)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) return false;
  double x = 0.0, y = 0.0, z = 0.0;
  if (!xmlRpcToDouble(value[0], x) || !xmlRpcToDouble(value[1], y) || !xmlRpcToDouble(value[2], z)) return false;
  out << x, y, z;
  return true;
}

speed_t baudrateToTermios(int baudrate)
{
  switch (baudrate)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}

std::string trimLine(const std::string &line)
{
  const size_t first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = line.find_last_not_of(" \t\r\n");
  return line.substr(first, last - first + 1);
}

bool isIntegerLike(double value)
{
  return std::isfinite(value) && std::fabs(value - std::round(value)) < 1e-6;
}

bool regexFindInt(const std::string &line, const std::regex &regex, int &value)
{
  std::smatch match;
  if (!std::regex_search(line, match, regex) || match.size() < 2) return false;
  try
  {
    value = std::stoi(match[1].str());
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}

bool regexFindDouble(const std::string &line, const std::regex &regex, double &value)
{
  std::smatch match;
  if (!std::regex_search(line, match, regex) || match.size() < 2) return false;
  try
  {
    value = std::stod(match[1].str());
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}

bool lineContainsErrorStatus(const std::string &line)
{
  return line.find("[ABORT]") != std::string::npos ||
         line.find("[ERROR]") != std::string::npos ||
         line.find("ABORT") != std::string::npos ||
         line.find("ERROR") != std::string::npos;
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string expandUserPath(const std::string &path)
{
  if (path.size() >= 2 && path[0] == '~' && path[1] == '/')
  {
    const char *home = std::getenv("HOME");
    if (home != nullptr) return std::string(home) + path.substr(1);
  }
  return path;
}

} // namespace

UwbManager::UwbManager() = default;

UwbManager::~UwbManager()
{
  shutdown();
}

bool UwbManager::correctUwbRange(int anchor_id, double raw_range_m,
                                 double &corrected_range_m, double &range_bias_m,
                                 UwbRejectReason &reject_reason) const
{
  range_bias_m = 0.0;
  const auto configured_it = configured_anchors_.find(anchor_id);
  if (configured_it != configured_anchors_.end())
  {
    range_bias_m = configured_it->second.range_bias_m;
  }
  else
  {
    const auto anchor_it = anchors_.find(anchor_id);
    if (anchor_it != anchors_.end()) range_bias_m = anchor_it->second.range_bias_m;
  }

  if (!correctUwbRangeValue(raw_range_m, range_bias_m, corrected_range_m, reject_reason))
    return false;
  if (corrected_range_m < min_range_m_ || corrected_range_m > max_range_m_)
  {
    reject_reason = UwbRejectReason::RANGE_LIMIT;
    return false;
  }
  return true;
}

bool UwbManager::initialize(ros::NodeHandle &nh, const std::string &save_path)
{
  if (!loadParameters(nh) || !en_ || input_mode_ == "disabled") return false;
  factor_backend_publisher_ = nh.advertise<uwb_serial_driver::UwbRangeArray>(factor_backend_topic_, 100);
  raw_log_file_.open(save_path + log_filename_, std::ios::out | std::ios::app);
  update_log_file_.open(save_path + update_log_filename_, std::ios::out | std::ios::app);
  if (input_mode_ == "external_topic") {
    input_source_ = "external_topic"; // One effective source for selection and timestamp rebasing.
    external_subscriber_ = nh.subscribe(external_topic_, 128, &UwbManager::externalRangeCallback, this);
    return true;
  }
  const std::string source = toLower(input_source_);
  if (source == "file" || source == "txt" || source == "replay") return loadReplayFile();
  if (!openSerial()) return false;
  running_.store(true);
  read_thread_ = std::thread(&UwbManager::readLoop, this);
  return true;
}

void UwbManager::shutdown()
{
  running_.store(false);
  if (read_thread_.joinable()) read_thread_.join();
  closeSerial();
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (raw_log_file_.is_open())
    {
      raw_log_file_.flush();
      raw_log_file_.close();
    }
    if (update_log_file_.is_open())
    {
      update_log_file_.flush();
      update_log_file_.close();
    }
  }
}

bool UwbManager::loadParameters(ros::NodeHandle &nh)
{
  GnssFusionPolicy policy;
  if (!loadGnssFusionPolicy(nh, policy)) throw std::invalid_argument("invalid fixed-lag fusion configuration");
  nh.param<bool>("rtk_backend/uwb_factor_backend_en", en_, false);
  factor_backend_output_en_ = en_;
  nh.param<std::string>("rtk_backend/uwb_range_topic", factor_backend_topic_, factor_backend_topic_);
  nh.param<std::string>("uwb/input_mode", input_mode_, "internal");
  input_mode_ = toLower(input_mode_);
  if (input_mode_ != "internal" && input_mode_ != "external_topic" && input_mode_ != "disabled")
    throw std::invalid_argument("uwb/input_mode must be internal, external_topic or disabled");
  nh.param<std::string>("uwb/external_topic", external_topic_, "/uwb/ranges");
  nh.param<std::string>("uwb/source", input_source_, "serial");
  nh.param<std::string>("uwb/serial_port", serial_port_, "/dev/ttyUSB0");
  nh.param<int>("uwb/baudrate", baudrate_, 115200);
  nh.param<bool>("uwb/dtr", dtr_high_, true);
  nh.param<bool>("uwb/rts", rts_high_, false);
  nh.param<std::string>("uwb/parser_mode", parser_mode_, "uwb");
  nh.param<std::string>("uwb/log_filename", log_filename_, "uwb_ranges.txt");
  nh.param<std::string>("uwb/update_log_filename", update_log_filename_, "uwb_updates.txt");
  nh.param<int>("uwb/log_flush_stride", log_flush_stride_, 1);
  nh.param<bool>("uwb/debug_log_en", debug_log_en_, true);
  nh.param<bool>("uwb/debug_to_console", debug_to_console_, false);
  nh.param<bool>("uwb/debug_to_file", debug_to_file_, true);
  nh.param<std::string>("uwb/replay_file", replay_file_, "");
  std::string replay_start_offset_param_path = "default";
  bool replay_start_offset_param_found = false;
  double replay_start_offset_param = 0.0;
  double replay_start_offset_primary = 0.0;
  double replay_start_offset_alias = 0.0;
  double replay_start_offset_short_alias = 0.0;
  double time_offset_primary = 0.0;
  const bool has_time_offset_s =
      nh.getParam("uwb/time_offset_s", time_offset_primary);
  const bool has_replay_start_offset_s =
      nh.getParam("uwb/replay_start_offset_s", replay_start_offset_primary);
  const bool has_replay_start_offset =
      nh.getParam("uwb/replay_start_offset", replay_start_offset_alias);
  const bool has_start_offset =
      nh.getParam("uwb/start_offset", replay_start_offset_short_alias);
  if (has_time_offset_s)
  {
    replay_start_offset_param = time_offset_primary;
    replay_start_offset_param_path = "uwb/time_offset_s";
    replay_start_offset_param_found = true;
  }
  if (has_replay_start_offset_s && !replay_start_offset_param_found)
  {
    replay_start_offset_param = replay_start_offset_primary;
    replay_start_offset_param_path = "uwb/replay_start_offset_s";
    replay_start_offset_param_found = true;
  }
  if (!has_time_offset_s && has_replay_start_offset &&
      (!replay_start_offset_param_found ||
       (std::fabs(replay_start_offset_param) < 1e-12 && std::fabs(replay_start_offset_alias) > 1e-12)))
  {
    replay_start_offset_param = replay_start_offset_alias;
    replay_start_offset_param_path = "uwb/replay_start_offset";
    replay_start_offset_param_found = true;
  }
  if (!has_time_offset_s && has_start_offset &&
      (!replay_start_offset_param_found ||
       (std::fabs(replay_start_offset_param) < 1e-12 && std::fabs(replay_start_offset_short_alias) > 1e-12)))
  {
    replay_start_offset_param = replay_start_offset_short_alias;
    replay_start_offset_param_path = "uwb/start_offset";
    replay_start_offset_param_found = true;
  }
  if ((has_replay_start_offset_s && has_replay_start_offset &&
       std::fabs(replay_start_offset_primary - replay_start_offset_alias) > 1e-9) ||
      (has_replay_start_offset_s && has_start_offset &&
       std::fabs(replay_start_offset_primary - replay_start_offset_short_alias) > 1e-9) ||
      (has_replay_start_offset && has_start_offset &&
       std::fabs(replay_start_offset_alias - replay_start_offset_short_alias) > 1e-9))
  {
    ROS_WARN("[UWB] Conflicting replay start offset params: replay_start_offset_s=%.6f(%d), replay_start_offset=%.6f(%d), start_offset=%.6f(%d). Use %s=%.6f.",
             replay_start_offset_primary, static_cast<int>(has_replay_start_offset_s),
             replay_start_offset_alias, static_cast<int>(has_replay_start_offset),
             replay_start_offset_short_alias, static_cast<int>(has_start_offset),
             replay_start_offset_param_path.c_str(), replay_start_offset_param);
  }
  replay_start_offset_s_ = replay_start_offset_param_found ? replay_start_offset_param : 0.0;
  if (has_time_offset_s &&
      ((has_replay_start_offset_s &&
        std::fabs(time_offset_primary - replay_start_offset_primary) > 1e-9) ||
       (has_replay_start_offset &&
        std::fabs(time_offset_primary - replay_start_offset_alias) > 1e-9) ||
       (has_start_offset &&
        std::fabs(time_offset_primary - replay_start_offset_short_alias) > 1e-9)))
  {
    ROS_WARN("[UWB] uwb/time_offset_s overrides conflicting deprecated replay offset aliases.");
  }
  nh.param<double>("uwb/range_scale", range_scale_, 1.0);
  nh.param<double>("uwb/min_range_m", min_range_m_, 0.05);
  nh.param<double>("uwb/max_range_m", max_range_m_, 250.0);
  nh.param<double>("uwb/max_age_s", max_age_s_, 0.5);
  double replay_match_threshold_param = -1.0;
  nh.param<double>("uwb/replay_match_threshold_s", replay_match_threshold_param, -1.0);
  replay_match_threshold_s_ = replay_match_threshold_param >= 0.0 ? replay_match_threshold_param : max_age_s_;
  nh.param<int>("uwb/max_queue_size", max_queue_size_, 512);
  if (!std::isfinite(range_scale_) || range_scale_ <= 0 || !std::isfinite(replay_start_offset_s_) ||
      !std::isfinite(min_range_m_) || !std::isfinite(max_range_m_) || min_range_m_ < 0 ||
      max_range_m_ < min_range_m_ || max_queue_size_ <= 0 || !std::isfinite(max_age_s_) ||
      !std::isfinite(replay_match_threshold_s_))
    throw std::invalid_argument("invalid UWB transport bounds");
  log_flush_stride_ = std::max(1, log_flush_stride_);
  XmlRpc::XmlRpcValue anchors_param;
  if (nh.getParam("uwb/anchors", anchors_param) && anchors_param.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < anchors_param.size(); ++i)
    {
      XmlRpc::XmlRpcValue anchor_param = anchors_param[i];
      if (anchor_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

      UwbAnchor anchor;
      anchor.id = i;
      if (xmlRpcHasMember(anchor_param, "id"))
      {
        int id = i;
        if (xmlRpcToInt(anchor_param["id"], id)) anchor.id = id;
      }

      if (xmlRpcHasMember(anchor_param, "role") &&
          !xmlRpcToString(anchor_param["role"], anchor.role))
      {
        ROS_WARN("[UWB] Anchor %d role must be a string. Use the role inferred from its id.", anchor.id);
        anchor.role.clear();
      }

      if (xmlRpcHasMember(anchor_param, "range_bias_m"))
      {
        if (!xmlRpcToDouble(anchor_param["range_bias_m"], anchor.range_bias_m) ||
            !std::isfinite(anchor.range_bias_m))
        {
          ROS_WARN("[UWB] Anchor %d has invalid range_bias_m. Use 0.0 m.", anchor.id);
          anchor.range_bias_m = 0.0;
        }
      }
      if (anchor.range_bias_m < 0.0)
      {
        ROS_WARN("[UWB] Anchor %d uses negative range_bias_m=%.6f m; corrected_range will be larger than raw_range.",
                 anchor.id, anchor.range_bias_m);
      }

      bool enabled = true;
      if (xmlRpcHasMember(anchor_param, "enable")) xmlRpcToBool(anchor_param["enable"], enabled);
      if (xmlRpcHasMember(anchor_param, "en")) xmlRpcToBool(anchor_param["en"], enabled);
      if (xmlRpcHasMember(anchor_param, "flag")) xmlRpcToBool(anchor_param["flag"], enabled);
      anchor.enabled = enabled;

      bool has_position = false;
      if (xmlRpcHasMember(anchor_param, "position"))
      {
        has_position = xmlRpcToVec3(anchor_param["position"], anchor.position_w);
      }
      if (!has_position)
      {
        ROS_WARN("[UWB] Anchor %d has no valid position. It can be parsed but will not be used for backend factor.", anchor.id);
        anchor.enabled = false;
      }


      configured_anchors_[anchor.id] = anchor;
      if (anchor.enabled) anchors_[anchor.id] = anchor;
    }
  }

  return true;
}

bool UwbManager::openSerial()
{
  ownership_conflict_ = false;
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    ROS_WARN("[UWB] Failed to open %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }
  if (::flock(serial_fd_, LOCK_EX | LOCK_NB) != 0)
  {
    ownership_conflict_ = true;
    ROS_ERROR("[UWB] Cannot acquire exclusive serial lock on %s: %s",
              serial_port_.c_str(), std::strerror(errno));
    closeSerial();
    return false;
  }

  if (!configureSerial())
  {
    closeSerial();
    return false;
  }
  return true;
}

bool UwbManager::configureSerial()
{
  if (serial_fd_ < 0) return false;

  termios tty;
  if (::tcgetattr(serial_fd_, &tty) != 0)
  {
    ROS_WARN("[UWB] tcgetattr failed on %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  const speed_t baud = baudrateToTermios(baudrate_);
  ::cfsetispeed(&tty, baud);
  ::cfsetospeed(&tty, baud);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    ROS_WARN("[UWB] tcsetattr failed on %s: %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }
  ::tcflush(serial_fd_, TCIOFLUSH);

  int modem_bits = 0;
  if (::ioctl(serial_fd_, TIOCMGET, &modem_bits) == 0)
  {
    if (dtr_high_) modem_bits |= TIOCM_DTR;
    else modem_bits &= ~TIOCM_DTR;
    if (rts_high_) modem_bits |= TIOCM_RTS;
    else modem_bits &= ~TIOCM_RTS;
    if (::ioctl(serial_fd_, TIOCMSET, &modem_bits) != 0)
    {
      ROS_WARN("[UWB] Failed to set DTR/RTS on %s: %s", serial_port_.c_str(), std::strerror(errno));
    }
  }
  else
  {
    ROS_WARN("[UWB] Failed to read modem bits on %s: %s", serial_port_.c_str(), std::strerror(errno));
  }

  return true;
}

void UwbManager::closeSerial()
{
  if (serial_fd_ >= 0)
  {
    ::flock(serial_fd_, LOCK_UN);
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void UwbManager::externalRangeCallback(
    const uwb_serial_driver::UwbRangeArrayConstPtr &message)
{
  ingestExternalRanges(*message);
}

void UwbManager::ingestExternalRanges(
    const uwb_serial_driver::UwbRangeArray &message)
{
  if (input_mode_ != "external_topic" || message.header.stamp.isZero() ||
      message.timestamp_source == uwb_serial_driver::UwbRangeArray::INVALID ||
      message.timestamp_uncertainty_ns == 0)
    return;

  std::vector<UwbRangeMeasurement> measurements;
  measurements.reserve(message.ranges.size());
  for (const auto &range : message.ranges)
  {
    UwbRangeMeasurement measurement;
    measurement.anchor_id = range.anchor_id;
    measurement.raw_range_m = range.raw_range_m;
    measurement.range_bias_m = range.range_bias_m;
    measurement.range_m = range.corrected_range_m;
    measurement.range_valid = range.valid;
    measurement.range_reject_reason =
        range.valid ? UwbRejectReason::NONE : UwbRejectReason::INVALID_RANGE_STATUS;
    measurement.diag = range.diag;
    measurement.measurement_uid =
        (message.round_sequence << 16) | static_cast<uint64_t>(range.anchor_id);
    measurement.source_line = static_cast<size_t>(message.round_sequence);
    measurement.source_format = "external_topic:" + range.source_format;
    measurement.stamp = message.header.stamp.toSec();
    measurements.push_back(measurement);
  }

  std::lock_guard<std::mutex> lock(measurement_mutex_);
  for (const auto &measurement : measurements)
    measurement_queue_.push_back(measurement);
  while (static_cast<int>(measurement_queue_.size()) > max_queue_size_)
    measurement_queue_.pop_front();
}

void UwbManager::readLoop()
{
  std::string line_buffer;
  line_buffer.reserve(512);
  char buffer[256];

  while (running_.load())
  {
    const ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));
    if (n > 0)
    {
      for (ssize_t i = 0; i < n; ++i)
      {
        const char c = buffer[i];
        if (c == '\n' || c == '\r')
        {
          const std::string line = trimLine(line_buffer);
          line_buffer.clear();
          if (!line.empty()) handleLine(line, ros::Time::now().toSec());
        }
        else
        {
          line_buffer.push_back(c);
          if (line_buffer.size() > 4096)
          {
            const std::string line = trimLine(line_buffer);
            line_buffer.clear();
            if (!line.empty()) handleLine(line, ros::Time::now().toSec());
          }
        }
      }
    }
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      ROS_WARN_THROTTLE(5.0, "[UWB] Serial read error on %s: %s", serial_port_.c_str(), std::strerror(errno));
      ros::Duration(0.02).sleep();
    }
    else
    {
      ros::Duration(0.005).sleep();
    }
  }
}

bool UwbManager::loadReplayFile()
{
  if (replay_file_.empty())
  {
    ROS_WARN("[UWB] uwb/replay_file is empty.");
    return false;
  }

  replay_file_ = expandUserPath(replay_file_);
  std::ifstream replay_file(replay_file_);
  if (!replay_file.is_open())
  {
    ROS_WARN("[UWB] Failed to open replay file: %s", replay_file_.c_str());
    return false;
  }

  replay_measurements_.clear();
  replay_index_ = 0;
  replay_started_ = false;
  replay_file_start_stamp_ = 0.0;
  replay_file_start_stamp_ready_ = false;
  replay_consumed_measurement_count_ = 0;
  replay_stale_measurement_count_ = 0;

  uint64_t total_parsed = 0;
  uint64_t invalid_zero_filtered = 0;
  uint64_t invalid_nonfinite_filtered = 0;
  uint64_t invalid_corrected_range_filtered = 0;
  uint64_t range_limit_filtered = 0;
  uint64_t invalid_other_filtered = 0;
  uint64_t duplicate_filtered = 0;
  uint64_t unknown_anchor_filtered = 0;
  size_t source_line = 0;
  std::vector<UwbRangeMeasurement> replay_candidates;
  std::string line;
  while (std::getline(replay_file, line))
  {
    source_line++;
    line = trimLine(line);
    if (line.empty() || line[0] == '#') continue;
    if (line.find(" UPDATE ") != std::string::npos || line.find("UPDATE used=") != std::string::npos) continue;

    std::istringstream iss(line);
    double stamp = 0.0;
    if (!(iss >> stamp)) continue;

    const size_t first_space = line.find_first_of(" \t");
    const std::string after_stamp = first_space == std::string::npos ? "" : trimLine(line.substr(first_space + 1));
    if (after_stamp.rfind("INFO ", 0) == 0 ||
        after_stamp.rfind("WARN ", 0) == 0 ||
        after_stamp.rfind("ERROR ", 0) == 0)
    {
      continue;
    }
    if (!replay_file_start_stamp_ready_)
    {
      replay_file_start_stamp_ = stamp;
      replay_file_start_stamp_ready_ = true;
    }

    std::string raw_line;
    const std::string raw_key = "raw=\"";
    const size_t raw_begin = line.find(raw_key);
    if (raw_begin != std::string::npos)
    {
      const size_t content_begin = raw_begin + raw_key.size();
      const size_t content_end = line.find('"', content_begin);
      if (content_end != std::string::npos)
      {
        raw_line = line.substr(content_begin, content_end - content_begin);
      }
    }

    if (raw_line.empty())
    {
      raw_line = after_stamp.empty() ? line : after_stamp;
    }

    // ponytail: replay de-duplication is timestamp-aware below; the range-only stale filter stays serial-only.
    auto measurements = parseLine(raw_line, stamp);
    total_parsed += measurements.size();
    for (auto &measurement : measurements)
    {
      measurement.source_line = source_line;
      measurement.stamp = stamp;
      if (!std::isfinite(measurement.raw_range_m))
      {
        invalid_nonfinite_filtered++;
        continue;
      }
      if (measurement.raw_range_m <= 0.0)
      {
        invalid_zero_filtered++;
        continue;
      }
      if (!configured_anchors_.empty() &&
          configured_anchors_.find(measurement.anchor_id) == configured_anchors_.end())
      {
        unknown_anchor_filtered++;
        continue;
      }
      if (!measurement.range_valid)
      {
        if (measurement.range_reject_reason == UwbRejectReason::INVALID_CORRECTED_RANGE)
          invalid_corrected_range_filtered++;
        else if (measurement.range_reject_reason == UwbRejectReason::RANGE_LIMIT)
          range_limit_filtered++;
        else
          invalid_other_filtered++;
        continue;
      }
      replay_candidates.push_back(measurement);
    }
  }

  std::sort(replay_candidates.begin(), replay_candidates.end(),
            [](const UwbRangeMeasurement &a, const UwbRangeMeasurement &b) {
              return a.stamp < b.stamp;
            });

  std::set<std::tuple<int, int64_t, int64_t>> seen_measurement_keys;
  std::set<std::pair<int, int64_t>> seen_diag_keys;
  constexpr double kCrossFormatDuplicateWindowS = 0.05;
  constexpr size_t kDuplicateDebugLimit = 5;
  size_t duplicate_debug_count = 0;
  for (const auto &measurement : replay_candidates)
  {
    const int64_t timestamp_us =
        static_cast<int64_t>(std::llround(measurement.stamp * 1e6));
    const int64_t range_mm =
        static_cast<int64_t>(std::llround(measurement.raw_range_m * 1000.0));
    const auto key = std::make_tuple(measurement.anchor_id, timestamp_us, range_mm);
    if (measurement.diag >= 0 &&
        !seen_diag_keys.insert({measurement.anchor_id, measurement.diag}).second)
    {
      duplicate_filtered++;
      continue;
    }
    bool cross_format_duplicate = false;
    const UwbRangeMeasurement *kept_distance_measurement = nullptr;
    if (measurement.source_format == "uwbdbg")
    {
      // ponytail: current replay files are small; upgrade this bounded scan to an indexed key if they grow large.
      for (const auto &candidate : replay_candidates)
      {
        if (!isUwbReplayCrossFormatDuplicate(
                measurement, candidate, kCrossFormatDuplicateWindowS))
          continue;
        cross_format_duplicate = true;
        kept_distance_measurement = &candidate;
        break;
      }
    }
    if (cross_format_duplicate)
    {
      duplicate_filtered++;
      if (debug_log_en_ && duplicate_debug_count < kDuplicateDebugLimit)
      {
        std::ostringstream oss;
        oss << "[UWB_REPLAY_DUPLICATE] anchor=" << measurement.anchor_id
            << " diag=" << measurement.diag
            << " timestamp_us=" << timestamp_us
            << " range_mm=" << range_mm
            << " filtered_source=" << measurement.source_format
            << " filtered_source_line=" << measurement.source_line
            << " kept_source=distance"
            << " kept_source_line="
            << (kept_distance_measurement != nullptr ?
                kept_distance_measurement->source_line : 0);
        emitUwbLine(measurement.stamp, "INFO", oss.str(),
                    debug_to_console_, debug_to_file_);
        duplicate_debug_count++;
      }
      continue;
    }
    if (!seen_measurement_keys.insert(key).second)
    {
      duplicate_filtered++;
      continue;
    }
    replay_measurements_.push_back(measurement);
  }

  for (size_t i = 0; i < replay_measurements_.size(); ++i)
    replay_measurements_[i].measurement_uid = static_cast<uint64_t>(i + 1);

  {
    std::ostringstream oss;
    oss << "[UWB_REPLAY_LOAD_STATS]"
        << " total_parsed=" << total_parsed
        << " valid_loaded=" << replay_measurements_.size()
        << " invalid_zero_filtered=" << invalid_zero_filtered
        << " invalid_nonfinite_filtered=" << invalid_nonfinite_filtered
        << " invalid_corrected_range_filtered=" << invalid_corrected_range_filtered
        << " range_limit_filtered=" << range_limit_filtered
        << " invalid_other_filtered=" << invalid_other_filtered
        << " duplicate_filtered=" << duplicate_filtered
        << " unknown_anchor_filtered=" << unknown_anchor_filtered;
    emitUwbLine(ros::Time::now().toSec(), "INFO", oss.str(), true, true);
  }

  if (!replay_measurements_.empty())
  {
    if (!replay_file_start_stamp_ready_)
    {
      replay_file_start_stamp_ = replay_measurements_.front().stamp;
      replay_file_start_stamp_ready_ = true;
    }
  }
  return !replay_measurements_.empty();
}

std::vector<UwbRangeMeasurement> UwbManager::takeReplayMeasurements(double current_lidar_stamp, double lidar_start_stamp)
{
  std::vector<UwbRangeMeasurement> measurements;
  if (replay_measurements_.empty() || replay_index_ >= replay_measurements_.size()) return measurements;
  if (!std::isfinite(current_lidar_stamp) || !std::isfinite(lidar_start_stamp) ||
      current_lidar_stamp <= 0.0 || lidar_start_stamp <= 0.0)
  {
    return measurements;
  }

  const double slam_relative_time = current_lidar_stamp - lidar_start_stamp;
  const double match_threshold = std::max(0.0, replay_match_threshold_s_);
  if (slam_relative_time < 0.0) return measurements;

  if (!replay_started_)
  {
    replay_started_ = true;
    if (!replay_file_start_stamp_ready_)
    {
      replay_file_start_stamp_ = replay_measurements_.front().stamp;
      replay_file_start_stamp_ready_ = true;
    }
    const double first_measurement_stamp = replay_measurements_.empty() ? 0.0 : replay_measurements_.front().stamp;
    std::ostringstream oss;
    oss << "REPLAY_TIMING_START time_mode=sensor_relative"
        << " file_start_stamp=" << replay_file_start_stamp_
        << " first_measurement_stamp=" << first_measurement_stamp
        << " replay_speed=1.000(ignored)"
        << " start_offset=" << replay_start_offset_s_
        << " lidar_start_stamp=" << lidar_start_stamp
        << " current_lidar_stamp=" << current_lidar_stamp
        << " slam_relative_time=" << slam_relative_time
        << " match_threshold=" << match_threshold
        << " first_relative_replay_time=" << (replay_start_offset_s_ + first_measurement_stamp - replay_file_start_stamp_)
        << " start_offset_applied=" << static_cast<int>(std::fabs(replay_start_offset_s_) > 1e-12)
        << " offset_semantics=positive_delay_negative_skip";
    logEvent(current_lidar_stamp, "INFO", oss.str());
    ROS_INFO("[UWB] Replay timing starts: time_mode=sensor_relative file_start_stamp=%.6f first_measurement_stamp=%.6f replay_speed=1.000(ignored) start_offset=%.3f lidar_start_stamp=%.6f current_lidar_stamp=%.6f slam_relative=%.3f match_threshold=%.3f applied=%d semantics=positive_delay_negative_skip",
             replay_file_start_stamp_, first_measurement_stamp, replay_start_offset_s_,
             lidar_start_stamp, current_lidar_stamp, slam_relative_time, match_threshold,
             static_cast<int>(std::fabs(replay_start_offset_s_) > 1e-12));
  }

  if (replay_last_slam_relative_time_ >= 0.0 &&
      slam_relative_time + match_threshold < replay_last_slam_relative_time_)
  {
    ROS_WARN("[UWB] Replay SLAM time moved backward: last=%.6f current=%.6f. Reset replay index.",
             replay_last_slam_relative_time_, slam_relative_time);
    std::ostringstream oss;
    oss << "REPLAY_SLAM_TIME_RESET last_slam_relative=" << replay_last_slam_relative_time_
        << " current_slam_relative=" << slam_relative_time
        << " match_threshold=" << match_threshold;
    logEvent(current_lidar_stamp, "WARN", oss.str());
    replay_index_ = 0;
  }
  replay_last_slam_relative_time_ = slam_relative_time;

  if (replay_start_offset_s_ > 0.0 && slam_relative_time + match_threshold < replay_start_offset_s_)
  {
    ROS_INFO_THROTTLE(3.0,
                      "[UWB] UWB_REPLAY_WAIT_START_OFFSET slam_relative=%.3f start_offset=%.3f threshold=%.3f",
                      slam_relative_time, replay_start_offset_s_, match_threshold);
    std::ostringstream oss;
    oss << "UWB_REPLAY_WAIT_START_OFFSET slam_relative=" << slam_relative_time
        << " start_offset=" << replay_start_offset_s_
        << " match_threshold=" << match_threshold
        << " lidar_start_stamp=" << lidar_start_stamp
        << " current_lidar_stamp=" << current_lidar_stamp
        << " file_start_stamp=" << replay_file_start_stamp_
        << " first_measurement_stamp=" << replay_measurements_.front().stamp
        << " start_offset_applied=1";
    logEventThrottled(current_lidar_stamp, "uwb_replay_wait_start_offset", 3.0, "INFO", oss.str());
    return measurements;
  }

  while (replay_index_ < replay_measurements_.size())
  {
    const auto &measurement = replay_measurements_[replay_index_];
    const double uwb_relative_time = replay_start_offset_s_ + (measurement.stamp - replay_file_start_stamp_);
    if (uwb_relative_time < -1e-9)
    {
      std::ostringstream oss;
      oss << "SKIP_REPLAY_BEFORE_ZERO anchor=" << measurement.anchor_id
          << " measurement_uid=" << measurement.measurement_uid
          << " measurement_stamp=" << measurement.stamp
          << " file_start_stamp=" << replay_file_start_stamp_
          << " measurement_relative=" << (measurement.stamp - replay_file_start_stamp_)
          << " start_offset=" << replay_start_offset_s_
          << " uwb_relative_time=" << uwb_relative_time;
      logEventThrottled(current_lidar_stamp, "skip_replay_before_zero", 3.0, "INFO", oss.str());
      replay_stale_measurement_count_++;
      replay_index_++;
      continue;
    }

    const double dt = uwb_relative_time - slam_relative_time;
    if (dt > match_threshold) break;
    if (dt < -match_threshold)
    {
      ROS_WARN_THROTTLE(3.0,
                        "[UWB] Drop stale replay range: anchor=%d, slam_relative=%.6f, uwb_relative=%.6f, dt=%.3f s > threshold=%.3f.",
                        measurement.anchor_id, slam_relative_time, uwb_relative_time, -dt, match_threshold);
      {
        std::ostringstream oss;
        oss << "DROP_STALE_REPLAY anchor=" << measurement.anchor_id
            << " measurement_uid=" << measurement.measurement_uid
            << " slam_relative_time=" << slam_relative_time
            << " uwb_relative_time=" << uwb_relative_time
            << " dt=" << dt
            << " abs_dt=" << std::fabs(dt)
            << " match_threshold=" << match_threshold;
        logEventThrottled(current_lidar_stamp, "drop_stale_replay", 3.0, "WARN", oss.str());
      }
      replay_stale_measurement_count_++;
      replay_index_++;
      continue;
    }
    UwbRangeMeasurement matched_measurement = measurement;
    matched_measurement.time_diff_s = dt;
    measurements.push_back(matched_measurement);
    replay_consumed_measurement_count_++;
    replay_index_++;
  }
  return measurements;
}

void UwbManager::publishBackendMeasurements(
    const std::vector<UwbRangeMeasurement> &measurements,
    double lidar_start_stamp)
{
  if (!factor_backend_output_en_ || !factor_backend_publisher_ ||
      measurements.empty())
    return;

  const std::string source = toLower(input_source_);
  const bool replay_source =
      source == "file" || source == "txt" || source == "replay";
  for (const UwbRangeMeasurement &measurement : measurements)
  {
    double measurement_stamp = measurement.stamp;
    if (replay_source)
    {
      if (!replay_file_start_stamp_ready_ || !std::isfinite(lidar_start_stamp))
        continue;
      // The fixed offset is deliberately applied by the backend. This header
      // preserves the rebased txt measurement time before that correction.
      measurement_stamp =
          lidar_start_stamp + measurement.stamp - replay_file_start_stamp_;
    }
    if (!std::isfinite(measurement_stamp) || measurement_stamp <= 0.0) continue;

    uwb_serial_driver::UwbRangeArray message;
    message.header.stamp.fromSec(measurement_stamp);
    message.header.frame_id = "uwb_measurement_time";
    message.session_id = 0;
    message.round_sequence =
        measurement.measurement_uid != 0 ? measurement.measurement_uid
                                         : ++factor_backend_sequence_;
    message.tag_id = 0;
    message.timestamp_source =
        replay_source ? uwb_serial_driver::UwbRangeArray::REPLAY_REBASED
                      : uwb_serial_driver::UwbRangeArray::HOST_RECEIVE_LOCAL;
    message.timestamp_uncertainty_ns = 1;
    message.host_receive_stamp = ros::Time::now();

    uwb_serial_driver::UwbRange range;
    range.anchor_id = static_cast<std::uint16_t>(measurement.anchor_id);
    range.raw_range_m = measurement.raw_range_m;
    range.range_bias_m = measurement.range_bias_m;
    range.corrected_range_m = measurement.range_m;
    range.diag = measurement.diag;
    range.valid = measurement.range_valid;
    range.reject_reason = range.valid
                              ? uwb_serial_driver::UwbRange::ACCEPTED
                              : uwb_serial_driver::UwbRange::INVALID_STATUS;
    range.source_format = measurement.source_format;
    message.ranges.push_back(range);
    factor_backend_publisher_.publish(message);
  }
}

void UwbManager::handleLine(const std::string &line, double stamp)
{
  const auto measurements = parseLine(line, stamp);
  logRawLine(stamp, line, measurements);
  if (measurements.empty()) return;

  std::lock_guard<std::mutex> lock(measurement_mutex_);
  for (const auto &measurement : measurements)
  {
    measurement_queue_.push_back(measurement);
  }
  while (static_cast<int>(measurement_queue_.size()) > max_queue_size_)
  {
    measurement_queue_.pop_front();
  }
}

std::vector<UwbRangeMeasurement> UwbManager::parseLine(const std::string &line, double stamp) const
{
  std::vector<UwbRangeMeasurement> measurements;

  auto appendMeasurement = [&](int anchor_id, double raw_range,
                               const std::string &source_format, int64_t diag)
  {
    UwbRangeMeasurement measurement;
    measurement.anchor_id = anchor_id;
    measurement.raw_range_m = raw_range * range_scale_;
    measurement.range_valid =
        correctUwbRange(anchor_id, measurement.raw_range_m, measurement.range_m,
                        measurement.range_bias_m, measurement.range_reject_reason);
    measurement.source_format = source_format;
    measurement.diag = diag;
    measurement.stamp = stamp;
    measurement.raw_line = line;
    measurements.push_back(measurement);
  };

  static const std::regex distance_line_regex(
      R"(distance\s*\[\s*([-+]?\d+)\s*\]\s*,\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?))",
      std::regex::icase);
  std::smatch distance_match;
  if (std::regex_search(line, distance_match, distance_line_regex))
  {
    if (!lineContainsErrorStatus(line))
    {
      appendMeasurement(std::stoi(distance_match[1].str()), std::stod(distance_match[2].str()),
                        "distance", -1);
    }
    return measurements;
  }
  if (parser_mode_ == "distance") return measurements;

  static const std::regex target_regex(R"(\btarget\s*=\s*([-+]?\d+))", std::regex::icase);
  static const std::regex ok_regex(R"(\bok\s*=\s*([-+]?\d+))", std::regex::icase);
  static const std::regex diag_regex(R"(\bdiag\s*=\s*([-+]?\d+))", std::regex::icase);
  static const std::regex dist_regex(R"(\bdist\s*=\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?))", std::regex::icase);
  const bool parse_debug_distance = parser_mode_ == "uwb" || parser_mode_ == "auto";
  if (parse_debug_distance &&
      (line.find("[UWBDBG]") != std::string::npos || line.find("dist=") != std::string::npos))
  {
    int target_id = -1;
    int ok = 0;
    int diag = -1;
    double dist_m = 0.0;
    if (regexFindInt(line, target_regex, target_id) &&
        regexFindInt(line, ok_regex, ok) &&
        regexFindDouble(line, dist_regex, dist_m) &&
        ok == 1 &&
        !lineContainsErrorStatus(line))
    {
      regexFindInt(line, diag_regex, diag);
      appendMeasurement(target_id, dist_m,
                        line.find("[UWBDBG]") != std::string::npos ? "uwbdbg" : "debug_distance",
                        diag);
    }
    return measurements;
  }

  if (parser_mode_ == "uwb") return measurements;

  std::vector<double> values;
  static const std::regex number_regex(R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
  auto begin = std::sregex_iterator(line.begin(), line.end(), number_regex);
  auto end = std::sregex_iterator();
  for (auto it = begin; it != end; ++it)
  {
    try
    {
      values.push_back(std::stod(it->str()));
    }
    catch (const std::exception &)
    {
    }
  }
  if (values.empty()) return measurements;

  bool parse_as_pairs = (parser_mode_ == "pairs");
  if (parser_mode_ == "auto" && values.size() >= 2 && values.size() % 2 == 0)
  {
    parse_as_pairs = true;
    for (size_t i = 0; i < values.size(); i += 2)
    {
      if (!isIntegerLike(values[i]))
      {
        parse_as_pairs = false;
        break;
      }
      const int id = static_cast<int>(std::llround(values[i]));
      if (!anchor_order_.empty() &&
          std::find(anchor_order_.begin(), anchor_order_.end(), id) == anchor_order_.end())
      {
        parse_as_pairs = false;
        break;
      }
    }
  }

  if (parse_as_pairs)
  {
    for (size_t i = 0; i + 1 < values.size(); i += 2)
    {
      if (!isIntegerLike(values[i])) continue;
      appendMeasurement(static_cast<int>(std::llround(values[i])), values[i + 1], "pairs", -1);
    }
    return measurements;
  }

  for (size_t i = 0; i < values.size(); ++i)
  {
    const int anchor_id = (i < anchor_order_.size()) ? anchor_order_[i] : static_cast<int>(i);
    appendMeasurement(anchor_id, values[i], "values", -1);
  }
  return measurements;
}

std::vector<UwbRangeMeasurement> UwbManager::takeRecentMeasurements(double now)
{
  std::vector<UwbRangeMeasurement> measurements;
  std::lock_guard<std::mutex> lock(measurement_mutex_);

  while (!measurement_queue_.empty())
  {
    UwbRangeMeasurement measurement = measurement_queue_.front();
    measurement_queue_.pop_front();
    if (max_age_s_ > 0.0 && now - measurement.stamp > max_age_s_) continue;
    measurement.time_diff_s = measurement.stamp - now;
    measurements.push_back(measurement);
  }
  return measurements;
}

void UwbManager::logRawLine(double stamp, const std::string &line, const std::vector<UwbRangeMeasurement> &measurements)
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (!raw_log_file_.is_open()) return;

  raw_log_file_ << std::fixed << std::setprecision(6) << stamp << " raw=\"" << line << "\" parsed=";
  if (measurements.empty())
  {
    raw_log_file_ << "none";
  }
  else
  {
    for (const auto &measurement : measurements)
    {
      raw_log_file_ << measurement.anchor_id
                    << ":raw=" << std::setprecision(4) << measurement.raw_range_m << "m"
                    << ",bias=" << measurement.range_bias_m << "m"
                    << ",corrected=" << measurement.range_m << "m"
                    << ",valid=" << static_cast<int>(measurement.range_valid) << " ";
    }
  }
  raw_log_file_ << '\n';
  raw_log_pending_lines_++;
  if (raw_log_pending_lines_ >= log_flush_stride_)
  {
    raw_log_file_.flush();
    raw_log_pending_lines_ = 0;
  }
}

void UwbManager::logEvent(double stamp, const std::string &level, const std::string &message)
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (!update_log_file_.is_open()) return;

  update_log_file_ << std::fixed << std::setprecision(6)
                   << stamp << " " << level << " " << message << '\n';
  update_log_file_.flush();
}

void UwbManager::logEventThrottled(double stamp, const std::string &key, double period_s,
                                   const std::string &level, const std::string &message)
{
  std::lock_guard<std::mutex> lock(log_mutex_);
  if (!update_log_file_.is_open()) return;

  const auto it = event_log_last_stamp_.find(key);
  if (period_s > 0.0 && it != event_log_last_stamp_.end() && stamp - it->second < period_s)
  {
    return;
  }
  event_log_last_stamp_[key] = stamp;

  update_log_file_ << std::fixed << std::setprecision(6)
                   << stamp << " " << level << " " << message << '\n';
  update_log_file_.flush();
}

// Immutable sensor observations are independent of the frontend ESIKF state.
void UwbManager::forwardMeasurementsAt(double current_lidar_stamp, double lidar_start_stamp)
{
  if (!en_) return;
  const std::string source = toLower(input_source_);
  const bool replay = source == "file" || source == "txt" || source == "replay";
  const double now = (replay || input_mode_ == "external_topic") ? current_lidar_stamp : ros::Time::now().toSec();
  const auto measurements = replay ? takeReplayMeasurements(current_lidar_stamp, lidar_start_stamp) : takeRecentMeasurements(now);
  publishBackendMeasurements(measurements, lidar_start_stamp);
}
void UwbManager::emitUwbLine(double stamp, const std::string &level, const std::string &line,
                             bool console, bool file)
{
  if (console)
  {
    if (level == "WARN" || level == "ERROR") ROS_WARN_STREAM(line);
    else ROS_INFO_STREAM(line);
  }
  if (file) logEvent(stamp, level, line);
}
