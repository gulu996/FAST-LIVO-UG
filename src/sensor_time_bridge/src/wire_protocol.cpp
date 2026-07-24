#include "sensor_time_bridge/wire_protocol.h"

#include <algorithm>

namespace sensor_time_bridge
{
namespace
{
void put16(std::vector<uint8_t> &out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value));
  out.push_back(static_cast<uint8_t>(value >> 8));
}

void put32(std::vector<uint8_t> &out, uint32_t value)
{
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

void put64(std::vector<uint8_t> &out, uint64_t value)
{
  for (int i = 0; i < 8; ++i) out.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

uint16_t get16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

uint32_t get32(const uint8_t *p)
{
  uint32_t value = 0;
  for (int i = 0; i < 4; ++i) value |= static_cast<uint32_t>(p[i]) << (8 * i);
  return value;
}

uint64_t get64(const uint8_t *p)
{
  uint64_t value = 0;
  for (int i = 0; i < 8; ++i) value |= static_cast<uint64_t>(p[i]) << (8 * i);
  return value;
}

bool knownType(uint8_t type)
{
  return type >= static_cast<uint8_t>(MessageType::BOOT) &&
         type <= static_cast<uint8_t>(MessageType::ERROR);
}
} // namespace

uint32_t crc32c(const uint8_t *data, size_t size)
{
  uint32_t crc = 0xffffffffU;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0x82f63b78U & (0U - (crc & 1U)));
  }
  return ~crc;
}

std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> &input)
{
  std::vector<uint8_t> output;
  output.reserve(input.size() + input.size() / 254 + 1);
  size_t code_index = 0;
  output.push_back(0);
  uint8_t code = 1;
  for (uint8_t byte : input)
  {
    if (byte == 0)
    {
      output[code_index] = code;
      code_index = output.size();
      output.push_back(0);
      code = 1;
    }
    else
    {
      output.push_back(byte);
      ++code;
      if (code == 0xff)
      {
        output[code_index] = code;
        code_index = output.size();
        output.push_back(0);
        code = 1;
      }
    }
  }
  output[code_index] = code;
  return output;
}

bool cobsDecode(const uint8_t *data, size_t size, std::vector<uint8_t> &output)
{
  output.clear();
  size_t index = 0;
  while (index < size)
  {
    const uint8_t code = data[index++];
    if (code == 0) return false;
    const size_t count = static_cast<size_t>(code - 1);
    if (index + count > size) return false;
    output.insert(output.end(), data + index, data + index + count);
    index += count;
    if (code != 0xff && index < size) output.push_back(0);
  }
  return true;
}

std::vector<uint8_t> encodeWireEvent(const WireEvent &event)
{
  std::vector<uint8_t> decoded;
  decoded.reserve(kWireHeaderLength + event.payload.size() + 4);
  put32(decoded, kProtocolMagic);
  decoded.push_back(event.protocol_version);
  decoded.push_back(static_cast<uint8_t>(event.message_type));
  put16(decoded, kWireHeaderLength);
  put16(decoded, static_cast<uint16_t>(event.payload.size()));
  put16(decoded, event.flags);
  put64(decoded, event.mcu_boot_id);
  put32(decoded, event.event_sequence);
  put64(decoded, event.local_tick);
  put64(decoded, event.local_stamp_ns);
  put32(decoded, event.local_tick_hz);
  decoded.insert(decoded.end(), event.payload.begin(), event.payload.end());
  put32(decoded, crc32c(decoded.data(), decoded.size()));
  std::vector<uint8_t> encoded = cobsEncode(decoded);
  encoded.push_back(0);
  return encoded;
}

DecodeError decodeWireEvent(const uint8_t *data, size_t size, WireEvent &event)
{
  std::vector<uint8_t> decoded;
  if (!cobsDecode(data, size, decoded)) return DecodeError::COBS;
  if (decoded.size() > kMaxDecodedFrameSize) return DecodeError::TOO_LONG;
  if (decoded.size() < kWireHeaderLength + 4) return DecodeError::TOO_SHORT;
  if (get32(decoded.data()) != kProtocolMagic) return DecodeError::MAGIC;
  if (decoded[4] != kProtocolVersion) return DecodeError::VERSION;
  if (!knownType(decoded[5])) return DecodeError::TYPE;
  const uint16_t header_length = get16(decoded.data() + 6);
  const uint16_t payload_length = get16(decoded.data() + 8);
  if (header_length != kWireHeaderLength ||
      static_cast<size_t>(header_length) + payload_length + 4 != decoded.size())
    return DecodeError::LENGTH;
  const uint32_t expected_crc = get32(decoded.data() + decoded.size() - 4);
  if (crc32c(decoded.data(), decoded.size() - 4) != expected_crc)
    return DecodeError::CRC;

  event.protocol_version = decoded[4];
  event.message_type = static_cast<MessageType>(decoded[5]);
  event.flags = get16(decoded.data() + 10);
  event.mcu_boot_id = get64(decoded.data() + 12);
  event.event_sequence = get32(decoded.data() + 20);
  event.local_tick = get64(decoded.data() + 24);
  event.local_stamp_ns = get64(decoded.data() + 32);
  event.local_tick_hz = get32(decoded.data() + 40);
  event.payload.assign(decoded.begin() + header_length,
                       decoded.begin() + header_length + payload_length);
  event.source_sequence = payload_length >= 4 ? get32(event.payload.data()) : 0;
  event.mcu_dropped_event_count =
      event.message_type == MessageType::STATUS && payload_length >= 8
          ? get32(event.payload.data() + 4)
          : 0;
  return DecodeError::NONE;
}

const char *decodeErrorName(DecodeError error)
{
  switch (error)
  {
    case DecodeError::NONE: return "none";
    case DecodeError::COBS: return "cobs";
    case DecodeError::TOO_LONG: return "too_long";
    case DecodeError::TOO_SHORT: return "too_short";
    case DecodeError::MAGIC: return "magic";
    case DecodeError::VERSION: return "version";
    case DecodeError::TYPE: return "message_type";
    case DecodeError::LENGTH: return "length";
    case DecodeError::CRC: return "crc";
  }
  return "unknown";
}

IncrementalDecoder::IncrementalDecoder(size_t max_encoded_size)
    : max_encoded_size_(max_encoded_size)
{
  encoded_.reserve(std::min(max_encoded_size_, size_t{768}));
}

void IncrementalDecoder::feed(const uint8_t *data, size_t size,
                              std::vector<WireEvent> &events,
                              std::vector<DecodeError> &errors)
{
  for (size_t i = 0; i < size; ++i)
  {
    if (data[i] != 0)
    {
      if (!overflowed_)
      {
        if (encoded_.size() < max_encoded_size_) encoded_.push_back(data[i]);
        else overflowed_ = true;
      }
      continue;
    }
    if (overflowed_)
    {
      errors.push_back(DecodeError::TOO_LONG);
    }
    else if (!encoded_.empty())
    {
      WireEvent event;
      const DecodeError error = decodeWireEvent(encoded_.data(), encoded_.size(), event);
      if (error == DecodeError::NONE) events.push_back(std::move(event));
      else errors.push_back(error);
    }
    encoded_.clear();
    overflowed_ = false;
  }
}

void IncrementalDecoder::reset()
{
  encoded_.clear();
  overflowed_ = false;
}

BoundedWireEventQueue::BoundedWireEventQueue(size_t capacity)
    : capacity_(std::max<size_t>(1, capacity))
{
}

void BoundedWireEventQueue::setCapacity(size_t capacity)
{
  capacity_ = std::max<size_t>(1, capacity);
  while (events_.size() > capacity_) events_.pop_front();
}

bool BoundedWireEventQueue::push(QueuedWireEvent event)
{
  const bool dropped = events_.size() >= capacity_;
  if (dropped) events_.pop_front();
  events_.push_back(std::move(event));
  return dropped;
}

bool BoundedWireEventQueue::pop(QueuedWireEvent &event)
{
  if (events_.empty()) return false;
  event = std::move(events_.front());
  events_.pop_front();
  return true;
}

SequenceResult SequenceTracker::observe(uint64_t boot_id, uint32_t sequence)
{
  if (!valid_ || boot_id != boot_id_)
  {
    valid_ = true;
    boot_id_ = boot_id;
    sequence_ = sequence;
    return SequenceResult{true, false, false, 0};
  }
  const uint32_t delta = sequence - sequence_;
  if (delta == 0) return SequenceResult{false, true, false, 0};
  if (delta >= 0x80000000U) return SequenceResult{false, false, true, 0};
  sequence_ = sequence;
  return SequenceResult{true, false, false, static_cast<uint64_t>(delta - 1)};
}

void SequenceTracker::reset()
{
  valid_ = false;
  boot_id_ = 0;
  sequence_ = 0;
}

} // namespace sensor_time_bridge
