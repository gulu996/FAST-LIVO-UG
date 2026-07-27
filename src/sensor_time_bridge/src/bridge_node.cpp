#include "sensor_time_bridge/local_time.h"
#include "sensor_time_bridge/time_mapping.h"
#include "sensor_time_bridge/wire_protocol.h"

#include <sensor_time_msgs/ConvertTime.h>
#include <sensor_time_msgs/HostMonotonicToLocal.h>
#include <sensor_time_msgs/McuEvent.h>
#include <sensor_time_msgs/TimeMapping.h>
#include <sensor_time_msgs/TimeStatus.h>
#include <sensor_time_msgs/UtcObservation.h>
#include <std_msgs/UInt8MultiArray.h>

#include <ros/ros.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace stb = sensor_time_bridge;

namespace
{
uint64_t monotonicRawNs()
{
  timespec value{};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &value);
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}

uint64_t fnv1a(uint64_t value, uint64_t seed = 1469598103934665603ULL)
{
  uint64_t hash = seed;
  for (int i = 0; i < 8; ++i)
  {
    hash ^= static_cast<uint8_t>(value >> (i * 8));
    hash *= 1099511628211ULL;
  }
  return hash == 0 ? 1 : hash;
}

speed_t baudConstant(int baud)
{
  switch (baud)
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
    default: return 0;
  }
}

ros::Time rosTime(uint64_t ns)
{
  ros::Time stamp;
  stamp.fromNSec(ns);
  return stamp;
}
} // namespace

class BridgeNode
{
public:
  BridgeNode() : private_nh_("~"), mapping_(loadMappingParameters())
  {
    private_nh_.param<std::string>("input_mode", input_mode_, "serial");
    private_nh_.param<std::string>("serial_port", serial_port_, "/dev/sensor_mcu");
    private_nh_.param<int>("serial_baud", serial_baud_, 115200);
    private_nh_.param<std::string>("input_file", input_file_, "");
    int queue_capacity = 256;
    private_nh_.param<int>("queue_capacity", queue_capacity, 256);
    queue_capacity_ = static_cast<size_t>(std::max(1, queue_capacity));
    queue_.setCapacity(queue_capacity_);

    event_pub_ = nh_.advertise<sensor_time_msgs::McuEvent>("/sensor_time/events", 64);
    trigger_pub_ = nh_.advertise<sensor_time_msgs::McuEvent>("/sensor_time/mcu_trigger", 32);
    pps_pub_ = nh_.advertise<sensor_time_msgs::McuEvent>("/sensor_time/mcu_pps", 8);
    gnss_pps_pub_ =
        nh_.advertise<sensor_time_msgs::McuEvent>("/sensor_time/gnss_pps_capture", 8);
    mapping_pub_ =
        nh_.advertise<sensor_time_msgs::TimeMapping>("/sensor_time/mapping", 1, true);
    status_pub_ =
        nh_.advertise<sensor_time_msgs::TimeStatus>("/sensor_time/status", 1, true);
    utc_sub_ = nh_.subscribe("/sensor_time/utc_observation", 64,
                            &BridgeNode::utcObservationCallback, this);
    wire_sub_ = nh_.subscribe("/sensor_time/mcu_wire", 128,
                             &BridgeNode::wireCallback, this);
    host_service_ = nh_.advertiseService(
        "/sensor_time/host_monotonic_to_local",
        &BridgeNode::hostTimeService, this);
    conversion_service_ = nh_.advertiseService(
        "/sensor_time/convert",
        &BridgeNode::conversionService, this);
    drain_timer_ = nh_.createTimer(ros::Duration(0.002), &BridgeNode::drain, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0), &BridgeNode::publishStatus, this);

    writer_epoch_ = fnv1a(monotonicRawNs() ^ static_cast<uint64_t>(::getpid()));
    if (input_mode_ == "serial" || input_mode_ == "file")
    {
      running_.store(true);
      input_thread_ = std::thread(&BridgeNode::inputLoop, this);
    }
    else if (input_mode_ != "simulation")
    {
      throw std::runtime_error("input_mode must be serial, simulation, or file");
    }
    ROS_INFO("[SENSOR_TIME] input_mode=%s writer_epoch=%lu queue_capacity=%zu",
             input_mode_.c_str(), static_cast<unsigned long>(writer_epoch_),
             queue_capacity_);
    if (input_mode_ == "simulation")
      ROS_INFO("[SENSOR_TIME] mcu_input_mode=simulation recorder_gate_only=true");
  }

  ~BridgeNode()
  {
    running_.store(false);
    if (input_thread_.joinable()) input_thread_.join();
    closeInput();
  }

private:
  stb::MappingParameters loadMappingParameters()
  {
    stb::MappingParameters parameters;
    int mapping_window_size = 120;
    int min_pps_pairs = 10;
    int relock_confirm_count = 10;
    int mapping_history_size = 64;
    double min_fit_span_s = 9.0;
    double pps_timeout_s = 3.5;
    double holdover_invalid_timeout_s = 3600.0;
    double max_relock_step_ns = 100000000.0;
    private_nh_.param<int>("mapping_window_size", mapping_window_size, 120);
    private_nh_.param<int>("min_pps_pairs", min_pps_pairs, 10);
    private_nh_.param<double>("min_fit_span_s", min_fit_span_s, 9.0);
    private_nh_.param<double>("max_pps_residual_ns",
                             parameters.max_pps_residual_ns, 2000000.0);
    private_nh_.param<double>("max_scale_ppm", parameters.max_scale_ppm, 10000.0);
    private_nh_.param<double>("pps_timeout_s", pps_timeout_s, 3.5);
    private_nh_.param<double>("holdover_invalid_timeout_s",
                             holdover_invalid_timeout_s, 3600.0);
    private_nh_.param<int>("relock_confirm_count", relock_confirm_count, 10);
    private_nh_.param<int>("mapping_history_size", mapping_history_size, 64);
    private_nh_.param<double>("max_relock_step_ns",
                             max_relock_step_ns, 100000000.0);
    private_nh_.param<double>("holdover_uncertainty_growth_ns_per_s",
                             parameters.holdover_uncertainty_growth_ns_per_s, 1000.0);
    parameters.mapping_window_size = static_cast<size_t>(std::max(2, mapping_window_size));
    parameters.min_pps_pairs = static_cast<size_t>(std::max(2, min_pps_pairs));
    parameters.relock_confirm_count =
        static_cast<size_t>(std::max(2, relock_confirm_count));
    parameters.mapping_history_size =
        static_cast<size_t>(std::max(1, mapping_history_size));
    parameters.max_relock_step_ns =
        static_cast<uint64_t>(std::max(0.0, max_relock_step_ns));
    parameters.min_fit_span_ns =
        static_cast<uint64_t>(std::max(0.0, min_fit_span_s) * 1.0e9);
    parameters.pps_timeout_ns =
        static_cast<uint64_t>(std::max(0.1, pps_timeout_s) * 1.0e9);
    parameters.holdover_invalid_timeout_ns =
        static_cast<uint64_t>(std::max(1.0, holdover_invalid_timeout_s) * 1.0e9);
    return parameters;
  }

  void wireCallback(const std_msgs::UInt8MultiArrayConstPtr &message)
  {
    feedBytes(message->data.data(), message->data.size());
  }

  void feedBytes(const uint8_t *data, size_t size)
  {
    std::vector<stb::WireEvent> events;
    std::vector<stb::DecodeError> errors;
    {
      std::lock_guard<std::mutex> lock(decoder_mutex_);
      decoder_.feed(data, size, events, errors);
    }
    const uint64_t receive_ns = monotonicRawNs();
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (stb::DecodeError error : errors)
    {
      if (error == stb::DecodeError::CRC) ++crc_error_count_;
      else ++frame_error_count_;
      last_detail_ = std::string("wire decode error: ") + stb::decodeErrorName(error);
    }
    for (stb::WireEvent &event : events)
    {
      if (queue_.push(stb::QueuedWireEvent{std::move(event), receive_ns}))
        ++queue_overflow_count_;
    }
  }

  void drain(const ros::TimerEvent &)
  {
    for (int count = 0; count < 64; ++count)
    {
      stb::QueuedWireEvent received;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!queue_.pop(received)) break;
      }
      processEvent(received);
    }
  }

  void processEvent(const stb::QueuedWireEvent &received)
  {
    const stb::WireEvent &wire = received.event;
    if (have_boot_ && wire.mcu_boot_id != mcu_boot_id_)
    {
      if (wire.message_type != stb::MessageType::BOOT ||
          retired_boot_ids_.count(wire.mcu_boot_id) != 0)
      {
        ++duplicate_event_count_;
        last_detail_ = "rejected event from old or unannounced boot";
        return;
      }
      retired_boot_ids_.insert(mcu_boot_id_);
      beginSession(wire.mcu_boot_id);
    }
    else if (!have_boot_)
    {
      if (wire.message_type != stb::MessageType::BOOT)
      {
        ++frame_error_count_;
        last_detail_ = "waiting for BOOT";
        return;
      }
      beginSession(wire.mcu_boot_id);
    }

    uint64_t calculated_ns = 0;
    std::string error;
    if (!stb::tickToLocalNs(wire.local_tick, wire.local_tick_hz,
                            calculated_ns, &error) ||
        calculated_ns != wire.local_stamp_ns)
    {
      ++frame_error_count_;
      last_detail_ = error.empty() ? "wire local_stamp_ns mismatch" : error;
      return;
    }
    const stb::SequenceResult sequence =
        sequence_tracker_.observe(wire.mcu_boot_id, wire.event_sequence);
    if (!sequence.accept)
    {
      ++duplicate_event_count_;
      last_detail_ = sequence.old ? "old event sequence" : "duplicate event sequence";
      return;
    }
    event_gap_count_ += sequence.gap;
    if (!monotonic_gate_.accept(session_id_,
                                static_cast<uint32_t>(wire.message_type),
                                calculated_ns))
    {
      ++frame_error_count_;
      last_detail_ = "non-monotonic stream timestamp";
      return;
    }

    latest_local_ns_ = calculated_ns;
    last_event_sequence_ = wire.event_sequence;
    mcu_dropped_event_count_ =
        std::max<uint64_t>(mcu_dropped_event_count_, wire.mcu_dropped_event_count);
    host_mapper_.add(received.host_monotonic_ns, calculated_ns);
    if (continuous_events_ < 3) ++continuous_events_;
    local_ready_ = continuous_events_ >= 3;

    sensor_time_msgs::McuEvent message;
    message.header.stamp = rosTime(calculated_ns);
    message.header.frame_id = "local_sensor_time";
    message.session_id = session_id_;
    message.mcu_boot_id = mcu_boot_id_;
    message.writer_epoch = writer_epoch_;
    message.protocol_version = wire.protocol_version;
    message.event_sequence = wire.event_sequence;
    message.message_type = static_cast<uint8_t>(wire.message_type);
    message.source_sequence = wire.source_sequence;
    message.local_tick = wire.local_tick;
    message.local_tick_hz = wire.local_tick_hz;
    message.local_stamp_ns = calculated_ns;
    message.host_receive_monotonic_ns = received.host_monotonic_ns;
    message.time_uncertainty_ns = 1000;
    message.flags = wire.flags;
    event_pub_.publish(message);
    if (wire.message_type == stb::MessageType::CAMERA_TRIGGER)
      trigger_pub_.publish(message);
    else if (wire.message_type == stb::MessageType::LOCAL_PPS_OUTPUT)
      pps_pub_.publish(message);
    else if (wire.message_type == stb::MessageType::GNSS_PPS_CAPTURE)
      gnss_pps_pub_.publish(message);
  }

  void beginSession(uint64_t boot_id)
  {
    have_boot_ = true;
    mcu_boot_id_ = boot_id;
    session_id_ = fnv1a(stb::kSyntheticEpochNs,
                        fnv1a(stb::kProtocolVersion, fnv1a(boot_id)));
    continuous_events_ = 0;
    local_ready_ = false;
    latest_local_ns_ = 0;
    sequence_tracker_.reset();
    monotonic_gate_.reset();
    host_mapper_.reset();
    mapping_.reset();
    last_detail_ = "new MCU session";
    publishMapping();
  }

  void utcObservationCallback(
      const sensor_time_msgs::UtcObservationConstPtr &observation)
  {
    if (!observation->valid || observation->session_id != session_id_) return;
    if (observation->association_type !=
            sensor_time_msgs::UtcObservation::HARDWARE_PPS_ASSOCIATED &&
        observation->association_type !=
            sensor_time_msgs::UtcObservation::REPLAY_PRESERVED)
    {
      ++rejected_observation_count_;
      last_detail_ = "UTC observation is not hardware-associated";
      return;
    }
    stb::TimePair pair{observation->local_stamp_ns, observation->utc_stamp_ns,
                       observation->time_uncertainty_ns};
    std::string reason;
    if (!mapping_.addPair(pair, &reason))
      last_detail_ = "UTC pair rejected: " + reason;
    else
      last_detail_ = "UTC pair accepted";
    publishMapping();
  }

  void publishMapping()
  {
    sensor_time_msgs::TimeMapping message;
    message.header.stamp = latest_local_ns_ == 0 ? ros::Time(0) : rosTime(latest_local_ns_);
    message.header.frame_id = "local_sensor_time";
    message.session_id = session_id_;
    message.mcu_boot_id = mcu_boot_id_;
    message.writer_epoch = writer_epoch_;
    message.mapping_version = mapping_.mappingVersion();
    message.time_state = static_cast<uint8_t>(mapping_.state());
    const stb::AffineMapping &fit = mapping_.mapping();
    message.local_reference_ns = fit.local_reference_ns;
    message.utc_reference_ns = fit.utc_reference_ns;
    message.utc_ns_per_local_ns = fit.utc_ns_per_local_ns;
    message.slope_ppm = fit.slope_ppm;
    message.valid_from_local_ns = fit.valid_from_local_ns;
    message.fit_span_ns = fit.fit_span_ns;
    message.sample_count = fit.sample_count;
    message.residual_rms_ns = fit.residual_rms_ns;
    message.residual_max_ns = fit.residual_max_ns;
    message.time_uncertainty_ns = mapping_.uncertainty(latest_local_ns_);
    message.accepted_pair_count = mapping_.acceptedPairCount();
    message.rejected_pair_count =
        mapping_.rejectedPairCount() + rejected_observation_count_;
    mapping_pub_.publish(message);
  }

  void publishStatus(const ros::TimerEvent &)
  {
    if (latest_local_ns_ != 0) mapping_.update(latest_local_ns_);
    sensor_time_msgs::TimeStatus status;
    status.header.stamp =
        latest_local_ns_ == 0 ? ros::Time(0) : rosTime(latest_local_ns_);
    status.header.frame_id = "local_sensor_time";
    status.session_id = session_id_;
    status.mcu_boot_id = mcu_boot_id_;
    status.writer_epoch = writer_epoch_;
    status.mapping_version = mapping_.mappingVersion();
    status.time_state = static_cast<uint8_t>(mapping_.state());
    status.clock_source = have_boot_ ? sensor_time_msgs::TimeStatus::MCU_HSE_FREE_RUN
                                    : sensor_time_msgs::TimeStatus::CLOCK_INVALID;
    status.local_ready = local_ready_;
    status.utc_mapping_valid = mapping_.mappingUsable();
    status.last_event_sequence = last_event_sequence_;
    status.event_gap_count = event_gap_count_;
    status.crc_error_count = crc_error_count_;
    status.frame_error_count = frame_error_count_;
    status.duplicate_event_count = duplicate_event_count_;
    status.mcu_dropped_event_count =
        mcu_dropped_event_count_ + queue_overflow_count_;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      status.bridge_queue_depth = queue_.size();
    }
    status.seconds_since_last_gnss_pps =
        mapping_.secondsSinceLastPair(latest_local_ns_);
    status.time_uncertainty_ns = mapping_.uncertainty(latest_local_ns_);
    status.detail = last_detail_;
    status_pub_.publish(status);
    publishMapping();
  }

  bool hostTimeService(
      sensor_time_msgs::HostMonotonicToLocal::Request &request,
      sensor_time_msgs::HostMonotonicToLocal::Response &response)
  {
    response.session_id = session_id_;
    response.writer_epoch = writer_epoch_;
    response.success = local_ready_ &&
                       host_mapper_.toLocal(request.host_monotonic_ns,
                                           response.local_stamp_ns,
                                           response.time_uncertainty_ns);
    response.detail = response.success ? "HOST_RECEIVE_LOCAL"
                                       : "host monotonic mapping not ready";
    return true;
  }

  bool conversionService(sensor_time_msgs::ConvertTime::Request &request,
                         sensor_time_msgs::ConvertTime::Response &response)
  {
    response.mapping_version = mapping_.mappingVersion();
    response.time_state = static_cast<uint8_t>(mapping_.state());
    response.time_uncertainty_ns = mapping_.uncertainty(latest_local_ns_);
    if (!mapping_.mappingUsable())
    {
      response.detail = "UTC mapping not usable";
      return true;
    }
    if (request.direction == sensor_time_msgs::ConvertTime::Request::LOCAL_TO_UTC)
      response.success = mapping_.mapping().localToUtc(request.input_ns, response.output_ns);
    else if (request.direction ==
             sensor_time_msgs::ConvertTime::Request::UTC_TO_LOCAL)
      response.success = mapping_.mapping().utcToLocal(request.input_ns, response.output_ns);
    else
      response.detail = "invalid conversion direction";
    if (response.success) response.detail = "ok";
    return true;
  }

  void inputLoop()
  {
    if (input_mode_ == "file")
    {
      const int fd = ::open(input_file_.c_str(), O_RDONLY);
      if (fd < 0)
      {
        ROS_ERROR("[SENSOR_TIME] Cannot open input_file=%s: %s",
                  input_file_.c_str(), std::strerror(errno));
        return;
      }
      uint8_t buffer[256];
      ssize_t count = 0;
      while (running_.load() && (count = ::read(fd, buffer, sizeof(buffer))) > 0)
      {
        feedBytes(buffer, static_cast<size_t>(count));
        ros::Duration(0.001).sleep();
      }
      ::close(fd);
      return;
    }

    uint8_t buffer[256];
    while (running_.load())
    {
      if (input_fd_ < 0 && !openSerial())
      {
        ros::Duration(1.0).sleep();
        continue;
      }
      const ssize_t count = ::read(input_fd_, buffer, sizeof(buffer));
      if (count > 0) feedBytes(buffer, static_cast<size_t>(count));
      else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
               errno != EINTR)
      {
        ROS_ERROR("[SENSOR_TIME] Serial read failed: %s", std::strerror(errno));
        closeInput();
      }
      else
      {
        ros::Duration(0.002).sleep();
      }
    }
  }

  bool openSerial()
  {
    closeInput();
    input_fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (input_fd_ < 0)
    {
      ROS_ERROR_THROTTLE(5.0, "[SENSOR_TIME] Cannot open %s: %s",
                         serial_port_.c_str(), std::strerror(errno));
      return false;
    }
    if (::flock(input_fd_, LOCK_EX | LOCK_NB) != 0)
    {
      ROS_ERROR("[SENSOR_TIME] Serial ownership conflict on %s", serial_port_.c_str());
      closeInput();
      return false;
    }
    termios tty{};
    const speed_t baud = baudConstant(serial_baud_);
    if (baud == 0 || ::tcgetattr(input_fd_, &tty) != 0)
    {
      ROS_ERROR("[SENSOR_TIME] Unsupported baud or tcgetattr failure");
      closeInput();
      return false;
    }
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, baud);
    ::cfsetospeed(&tty, baud);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (::tcsetattr(input_fd_, TCSANOW, &tty) != 0)
    {
      ROS_ERROR("[SENSOR_TIME] tcsetattr failed: %s", std::strerror(errno));
      closeInput();
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(decoder_mutex_);
      decoder_.reset();
    }
    return true;
  }

  void closeInput()
  {
    if (input_fd_ >= 0)
    {
      ::flock(input_fd_, LOCK_UN);
      ::close(input_fd_);
      input_fd_ = -1;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher event_pub_, trigger_pub_, pps_pub_, gnss_pps_pub_;
  ros::Publisher mapping_pub_, status_pub_;
  ros::Subscriber utc_sub_, wire_sub_;
  ros::ServiceServer host_service_, conversion_service_;
  ros::Timer drain_timer_, status_timer_;

  std::string input_mode_, serial_port_, input_file_;
  int serial_baud_ = 115200;
  size_t queue_capacity_ = 256;
  int input_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread input_thread_;
  std::mutex decoder_mutex_;
  stb::IncrementalDecoder decoder_;
  std::mutex queue_mutex_;
  stb::BoundedWireEventQueue queue_;

  stb::SequenceTracker sequence_tracker_;
  stb::StreamMonotonicGate monotonic_gate_;
  stb::MappingStateMachine mapping_;
  stb::HostMonotonicMapper host_mapper_;
  std::set<uint64_t> retired_boot_ids_;
  uint64_t writer_epoch_ = 0;
  uint64_t mcu_boot_id_ = 0;
  uint64_t session_id_ = 0;
  uint64_t latest_local_ns_ = 0;
  uint64_t event_gap_count_ = 0;
  uint64_t crc_error_count_ = 0;
  uint64_t frame_error_count_ = 0;
  uint64_t duplicate_event_count_ = 0;
  uint64_t mcu_dropped_event_count_ = 0;
  uint64_t queue_overflow_count_ = 0;
  uint32_t rejected_observation_count_ = 0;
  uint32_t last_event_sequence_ = 0;
  int continuous_events_ = 0;
  bool have_boot_ = false;
  bool local_ready_ = false;
  std::string last_detail_ = "waiting for MCU BOOT";
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_time_bridge");
  try
  {
    BridgeNode node;
    ros::spin();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL("[SENSOR_TIME] %s", error.what());
    return 2;
  }
  return 0;
}
