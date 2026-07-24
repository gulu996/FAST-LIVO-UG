#include "sensor_time_bridge/local_time.h"
#include "sensor_time_bridge/wire_protocol.h"

#include <sensor_time_msgs/UtcObservation.h>
#include <std_msgs/UInt8MultiArray.h>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>

namespace stb = sensor_time_bridge;

namespace
{
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

void put32(std::vector<uint8_t> &output, uint32_t value)
{
  for (int i = 0; i < 4; ++i)
    output.push_back(static_cast<uint8_t>(value >> (i * 8)));
}

ros::Time rosTime(uint64_t ns)
{
  ros::Time stamp;
  stamp.fromNSec(ns);
  return stamp;
}
} // namespace

class Simulator
{
public:
  Simulator() : private_nh_("~")
  {
    private_nh_.param<int>("local_tick_hz", local_tick_hz_, 1000000);
    private_nh_.param<double>("clock_error_ppm", clock_error_ppm_, 50.0);
    private_nh_.param<double>("camera_rate_hz", camera_rate_hz_, 10.0);
    private_nh_.param<double>("local_pps_rate_hz", local_pps_rate_hz_, 1.0);
    private_nh_.param<double>("start_without_gnss_s", start_without_gnss_s_, 300.0);
    private_nh_.param<double>("gnss_lock_duration_s", gnss_lock_duration_s_, 300.0);
    private_nh_.param<double>("gnss_outage_duration_s", gnss_outage_duration_s_, 300.0);
    private_nh_.param<double>("relock_delay_s", relock_delay_s_, 10.0);
    private_nh_.param<int>("mcu_boot_id", boot_id_param_, 1);
    if (local_tick_hz_ <= 0 || camera_rate_hz_ <= 0.0 ||
        local_pps_rate_hz_ <= 0.0)
      throw std::runtime_error("simulator rates must be positive");

    boot_id_ = static_cast<uint64_t>(std::max(1, boot_id_param_));
    session_id_ =
        fnv1a(stb::kSyntheticEpochNs,
              fnv1a(stb::kProtocolVersion, fnv1a(boot_id_)));
    wire_pub_ =
        nh_.advertise<std_msgs::UInt8MultiArray>("/sensor_time/mcu_wire", 128, true);
    utc_pub_ =
        nh_.advertise<sensor_time_msgs::UtcObservation>(
            "/sensor_time/utc_observation", 32);
    start_wall_ = ros::WallTime::now();
    utc_start_ns_ = ros::Time::now().toNSec();
    timer_ = nh_.createWallTimer(ros::WallDuration(0.0025),
                                 &Simulator::timerCallback, this);
    ROS_INFO("[MCU_SIM] tick_hz=%d error_ppm=%.3f camera=%.3fHz pps=%.3fHz session=%lu",
             local_tick_hz_, clock_error_ppm_, camera_rate_hz_,
             local_pps_rate_hz_, static_cast<unsigned long>(session_id_));
  }

private:
  struct PendingObservation
  {
    uint64_t local_ns;
    uint64_t utc_ns;
    uint32_t sequence;
  };

  uint64_t tickAt(double elapsed_s) const
  {
    const long double scaled =
        static_cast<long double>(elapsed_s) * local_tick_hz_ *
        (1.0L + static_cast<long double>(clock_error_ppm_) / 1.0e6L);
    return static_cast<uint64_t>(std::max<long double>(0.0L, std::floor(scaled)));
  }

  void publishWire(stb::MessageType type, double elapsed_s, uint32_t source_sequence)
  {
    stb::WireEvent event;
    event.message_type = type;
    event.mcu_boot_id = boot_id_;
    event.event_sequence = event_sequence_++;
    event.local_tick = tickAt(elapsed_s);
    event.local_tick_hz = static_cast<uint32_t>(local_tick_hz_);
    if (!stb::tickToLocalNs(event.local_tick, event.local_tick_hz,
                            event.local_stamp_ns))
      return;
    put32(event.payload, source_sequence);
    if (type == stb::MessageType::STATUS) put32(event.payload, 0);
    const std::vector<uint8_t> bytes = stb::encodeWireEvent(event);
    std_msgs::UInt8MultiArray message;
    message.data = bytes;
    wire_pub_.publish(message);
  }

  bool gnssAvailable(double elapsed_s) const
  {
    if (elapsed_s < start_without_gnss_s_) return false;
    const double outage_start = start_without_gnss_s_ + gnss_lock_duration_s_;
    const double restore = outage_start + gnss_outage_duration_s_ + relock_delay_s_;
    return elapsed_s < outage_start || elapsed_s >= restore;
  }

  void timerCallback(const ros::WallTimerEvent &)
  {
    const double elapsed = (ros::WallTime::now() - start_wall_).toSec();
    if (!boot_published_ && wire_pub_.getNumSubscribers() == 0)
      return;
    if (!boot_published_)
    {
      publishWire(stb::MessageType::BOOT, 0.0, 0);
      boot_published_ = true;
      boot_publish_wall_ = ros::WallTime::now();
      return;
    }
    if ((ros::WallTime::now() - boot_publish_wall_).toSec() < 0.25)
      return;
    while (next_camera_s_ <= elapsed)
    {
      publishWire(stb::MessageType::CAMERA_TRIGGER, next_camera_s_,
                  camera_sequence_++);
      next_camera_s_ += 1.0 / camera_rate_hz_;
    }
    while (next_pps_s_ <= elapsed)
    {
      publishWire(stb::MessageType::LOCAL_PPS_OUTPUT, next_pps_s_,
                  pps_sequence_);
      if (gnssAvailable(next_pps_s_))
      {
        publishWire(stb::MessageType::GNSS_PPS_CAPTURE, next_pps_s_,
                    gnss_sequence_);
        const uint64_t tick = tickAt(next_pps_s_);
        uint64_t local_ns = 0;
        stb::tickToLocalNs(tick, static_cast<uint32_t>(local_tick_hz_), local_ns);
        pending_observations_.push_back(
            PendingObservation{local_ns,
                               utc_start_ns_ +
                                   static_cast<uint64_t>(std::llround(next_pps_s_ * 1.0e9)),
                               gnss_sequence_++});
      }
      ++pps_sequence_;
      next_pps_s_ += 1.0 / local_pps_rate_hz_;
    }
    while (next_status_s_ <= elapsed)
    {
      publishWire(stb::MessageType::STATUS, next_status_s_, status_sequence_++);
      next_status_s_ += 1.0;
    }
    while (!pending_observations_.empty())
    {
      const PendingObservation pending = pending_observations_.front();
      pending_observations_.pop_front();
      sensor_time_msgs::UtcObservation observation;
      observation.header.stamp = rosTime(pending.local_ns);
      observation.header.frame_id = "local_sensor_time";
      observation.session_id = session_id_;
      observation.source_sequence = pending.sequence;
      observation.local_stamp_ns = pending.local_ns;
      observation.utc_stamp_ns = pending.utc_ns;
      observation.time_uncertainty_ns = 1000;
      observation.association_type =
          sensor_time_msgs::UtcObservation::REPLAY_PRESERVED;
      observation.valid = true;
      observation.detail = "software simulator exact PPS pair";
      utc_pub_.publish(observation);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher wire_pub_, utc_pub_;
  ros::WallTimer timer_;
  ros::WallTime start_wall_;
  ros::WallTime boot_publish_wall_;
  uint64_t utc_start_ns_ = 0;
  uint64_t boot_id_ = 1;
  uint64_t session_id_ = 0;
  int boot_id_param_ = 1;
  int local_tick_hz_ = 1000000;
  double clock_error_ppm_ = 0.0;
  double camera_rate_hz_ = 10.0;
  double local_pps_rate_hz_ = 1.0;
  double start_without_gnss_s_ = 300.0;
  double gnss_lock_duration_s_ = 300.0;
  double gnss_outage_duration_s_ = 300.0;
  double relock_delay_s_ = 10.0;
  double next_camera_s_ = 0.1;
  double next_pps_s_ = 1.0;
  double next_status_s_ = 1.0;
  uint32_t event_sequence_ = 0;
  uint32_t camera_sequence_ = 0;
  uint32_t pps_sequence_ = 0;
  uint32_t gnss_sequence_ = 0;
  uint32_t status_sequence_ = 0;
  bool boot_published_ = false;
  std::deque<PendingObservation> pending_observations_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mcu_event_simulator");
  try
  {
    Simulator simulator;
    ros::spin();
  }
  catch (const std::exception &error)
  {
    ROS_FATAL("[MCU_SIM] %s", error.what());
    return 2;
  }
  return 0;
}
