#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace sensor_time_bridge
{

constexpr uint32_t kProtocolMagic = 0x31544c53U;
constexpr uint8_t kProtocolVersion = 1;
constexpr uint16_t kWireHeaderLength = 44;
constexpr size_t kMaxDecodedFrameSize = 512;

enum class MessageType : uint8_t
{
  BOOT = 1,
  LOCAL_PPS_OUTPUT = 2,
  CAMERA_TRIGGER = 3,
  GNSS_PPS_CAPTURE = 4,
  STATUS = 5,
  ERROR = 6
};

enum class DecodeError
{
  NONE,
  COBS,
  TOO_LONG,
  TOO_SHORT,
  MAGIC,
  VERSION,
  TYPE,
  LENGTH,
  CRC
};

struct WireEvent
{
  uint8_t protocol_version = kProtocolVersion;
  MessageType message_type = MessageType::STATUS;
  uint16_t flags = 0;
  uint64_t mcu_boot_id = 0;
  uint32_t event_sequence = 0;
  uint32_t source_sequence = 0;
  uint64_t local_tick = 0;
  uint64_t local_stamp_ns = 0;
  uint32_t local_tick_hz = 0;
  uint32_t mcu_dropped_event_count = 0;
  std::vector<uint8_t> payload;
};

uint32_t crc32c(const uint8_t *data, size_t size);
std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> &input);
bool cobsDecode(const uint8_t *data, size_t size, std::vector<uint8_t> &output);
std::vector<uint8_t> encodeWireEvent(const WireEvent &event);
DecodeError decodeWireEvent(const uint8_t *data, size_t size, WireEvent &event);
const char *decodeErrorName(DecodeError error);

class IncrementalDecoder
{
public:
  explicit IncrementalDecoder(size_t max_encoded_size = 768);
  void feed(const uint8_t *data, size_t size,
            std::vector<WireEvent> &events,
            std::vector<DecodeError> &errors);
  void reset();

private:
  size_t max_encoded_size_;
  bool overflowed_ = false;
  std::vector<uint8_t> encoded_;
};

struct QueuedWireEvent
{
  WireEvent event;
  uint64_t host_monotonic_ns = 0;
};

class BoundedWireEventQueue
{
public:
  explicit BoundedWireEventQueue(size_t capacity = 256);
  void setCapacity(size_t capacity);
  bool push(QueuedWireEvent event);
  bool pop(QueuedWireEvent &event);
  size_t size() const { return events_.size(); }

private:
  size_t capacity_;
  std::deque<QueuedWireEvent> events_;
};

struct SequenceResult
{
  bool accept = false;
  bool duplicate = false;
  bool old = false;
  uint64_t gap = 0;
};

class SequenceTracker
{
public:
  SequenceResult observe(uint64_t boot_id, uint32_t sequence);
  void reset();

private:
  bool valid_ = false;
  uint64_t boot_id_ = 0;
  uint32_t sequence_ = 0;
};

} // namespace sensor_time_bridge
