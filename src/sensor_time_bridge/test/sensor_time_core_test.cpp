#include "sensor_time_bridge/local_time.h"
#include "sensor_time_bridge/time_mapping.h"
#include "sensor_time_bridge/wire_protocol.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace stb = sensor_time_bridge;

TEST(WireProtocol, Crc32cGolden)
{
  const char text[] = "123456789";
  EXPECT_EQ(0xe3069283U,
            stb::crc32c(reinterpret_cast<const uint8_t *>(text), 9));
}

TEST(WireProtocol, CobsGoldenAndIncrementalHalfFrame)
{
  const std::vector<uint8_t> input{0x11, 0x22, 0x00, 0x33};
  const std::vector<uint8_t> expected{0x03, 0x11, 0x22, 0x02, 0x33};
  EXPECT_EQ(expected, stb::cobsEncode(input));
  std::vector<uint8_t> decoded;
  ASSERT_TRUE(stb::cobsDecode(expected.data(), expected.size(), decoded));
  EXPECT_EQ(input, decoded);

  stb::WireEvent event;
  event.message_type = stb::MessageType::CAMERA_TRIGGER;
  event.mcu_boot_id = 42;
  event.event_sequence = 7;
  event.local_tick = 100000;
  event.local_tick_hz = 1000000;
  ASSERT_TRUE(stb::tickToLocalNs(event.local_tick, event.local_tick_hz,
                                 event.local_stamp_ns));
  event.payload = {9, 0, 0, 0};
  const std::vector<uint8_t> frame = stb::encodeWireEvent(event);
  stb::IncrementalDecoder parser;
  std::vector<stb::WireEvent> events;
  std::vector<stb::DecodeError> errors;
  parser.feed(frame.data(), frame.size() / 2, events, errors);
  EXPECT_TRUE(events.empty());
  parser.feed(frame.data() + frame.size() / 2,
              frame.size() - frame.size() / 2, events, errors);
  ASSERT_EQ(1U, events.size());
  EXPECT_EQ(42U, events.front().mcu_boot_id);
  EXPECT_EQ(9U, events.front().source_sequence);
  EXPECT_TRUE(errors.empty());
}

TEST(WireProtocol, RejectsCrcMagicVersionTypeAndResynchronizes)
{
  stb::WireEvent event;
  event.message_type = stb::MessageType::STATUS;
  event.mcu_boot_id = 1;
  event.local_tick_hz = 1000000;
  stb::tickToLocalNs(0, event.local_tick_hz, event.local_stamp_ns);
  std::vector<uint8_t> good = stb::encodeWireEvent(event);
  std::vector<uint8_t> bad = good;
  bad[std::min<size_t>(5, bad.size() - 2)] ^= 0x5a;
  std::vector<uint8_t> stream = bad;
  stream.insert(stream.end(), good.begin(), good.end());
  stb::IncrementalDecoder parser;
  std::vector<stb::WireEvent> events;
  std::vector<stb::DecodeError> errors;
  parser.feed(stream.data(), stream.size(), events, errors);
  EXPECT_EQ(1U, events.size());
  EXPECT_EQ(1U, errors.size());

  std::vector<uint8_t> decoded;
  ASSERT_TRUE(stb::cobsDecode(good.data(), good.size() - 1, decoded));
  decoded[0] ^= 1;
  std::vector<uint8_t> encoded = stb::cobsEncode(decoded);
  stb::WireEvent output;
  EXPECT_EQ(stb::DecodeError::MAGIC,
            stb::decodeWireEvent(encoded.data(), encoded.size(), output));
  decoded[0] ^= 1;
  decoded[4] = 99;
  encoded = stb::cobsEncode(decoded);
  EXPECT_EQ(stb::DecodeError::VERSION,
            stb::decodeWireEvent(encoded.data(), encoded.size(), output));
  decoded[4] = stb::kProtocolVersion;
  decoded[5] = 99;
  encoded = stb::cobsEncode(decoded);
  EXPECT_EQ(stb::DecodeError::TYPE,
            stb::decodeWireEvent(encoded.data(), encoded.size(), output));
}

TEST(WireProtocol, RandomLossAndBoundedFrameRecover)
{
  stb::WireEvent event;
  event.message_type = stb::MessageType::LOCAL_PPS_OUTPUT;
  event.mcu_boot_id = 9;
  event.local_tick_hz = 1000000;
  stb::tickToLocalNs(1000000, event.local_tick_hz, event.local_stamp_ns);
  std::vector<uint8_t> frame = stb::encodeWireEvent(event);
  std::vector<uint8_t> damaged = frame;
  damaged.erase(damaged.begin() + damaged.size() / 2);
  damaged.insert(damaged.end(), frame.begin(), frame.end());
  stb::IncrementalDecoder parser(64);
  std::vector<stb::WireEvent> events;
  std::vector<stb::DecodeError> errors;
  parser.feed(damaged.data(), damaged.size(), events, errors);
  ASSERT_EQ(1U, events.size());
  EXPECT_FALSE(errors.empty());

  std::vector<uint8_t> oversized(100, 0x7f);
  oversized.push_back(0);
  parser.feed(oversized.data(), oversized.size(), events, errors);
  EXPECT_NE(errors.end(),
            std::find(errors.begin(), errors.end(), stb::DecodeError::TOO_LONG));

  std::mt19937 generator(20260723U);
  for (int trial = 0; trial < 32; ++trial)
  {
    std::vector<uint8_t> random_damage = frame;
    std::uniform_int_distribution<size_t> location(
        0, random_damage.size() - 2);
    random_damage.erase(random_damage.begin() + location(generator));
    random_damage.insert(random_damage.end(), frame.begin(), frame.end());
    stb::IncrementalDecoder trial_parser;
    std::vector<stb::WireEvent> trial_events;
    std::vector<stb::DecodeError> trial_errors;
    trial_parser.feed(random_damage.data(), random_damage.size(),
                      trial_events, trial_errors);
    ASSERT_FALSE(trial_events.empty());
    EXPECT_EQ(event.mcu_boot_id, trial_events.back().mcu_boot_id);
  }
}

TEST(WireProtocol, BoundedQueueDropsOldestAndReportsOverflow)
{
  stb::BoundedWireEventQueue queue(2);
  for (uint32_t sequence = 1; sequence <= 3; ++sequence)
  {
    stb::QueuedWireEvent item;
    item.event.event_sequence = sequence;
    EXPECT_EQ(sequence == 3, queue.push(std::move(item)));
  }
  EXPECT_EQ(2U, queue.size());
  stb::QueuedWireEvent item;
  ASSERT_TRUE(queue.pop(item));
  EXPECT_EQ(2U, item.event.event_sequence);
  ASSERT_TRUE(queue.pop(item));
  EXPECT_EQ(3U, item.event.event_sequence);
  EXPECT_FALSE(queue.pop(item));
}

TEST(SequenceTracker, DuplicateGapWrapAndBoot)
{
  stb::SequenceTracker tracker;
  EXPECT_TRUE(tracker.observe(1, 0xfffffffeU).accept);
  EXPECT_TRUE(tracker.observe(1, 0xffffffffU).accept);
  EXPECT_TRUE(tracker.observe(1, 0U).accept);
  const stb::SequenceResult duplicate = tracker.observe(1, 0U);
  EXPECT_TRUE(duplicate.duplicate);
  const stb::SequenceResult gap = tracker.observe(1, 3U);
  EXPECT_TRUE(gap.accept);
  EXPECT_EQ(2U, gap.gap);
  EXPECT_TRUE(tracker.observe(2, 1U).accept);
}

TEST(LocalTime, IntegerEpochLargeTickAndSessionMonotonic)
{
  uint64_t stamp = 0;
  ASSERT_TRUE(stb::tickToLocalNs(0, 1000000, stamp));
  EXPECT_EQ(stb::kSyntheticEpochNs, stamp);
  ASSERT_TRUE(stb::tickToLocalNs(1234567890123ULL, 1000000, stamp));
  EXPECT_EQ(stb::kSyntheticEpochNs + 1234567890123000ULL, stamp);
  EXPECT_FALSE(stb::tickToLocalNs(UINT64_MAX, 1, stamp));

  stb::StreamMonotonicGate gate;
  EXPECT_TRUE(gate.accept(1, 3, 10));
  EXPECT_FALSE(gate.accept(1, 3, 10));
  EXPECT_FALSE(gate.accept(1, 3, 9));
  EXPECT_TRUE(gate.accept(1, 4, 9));
  EXPECT_TRUE(gate.accept(2, 3, 1));
}

TEST(LocalTime, Stm32Counter16PendingOverflowReference)
{
  uint64_t tick = 0;
  ASSERT_TRUE(stb::extendCounter16Snapshot(7, 0xfffeU, false, 7, tick));
  EXPECT_EQ((7ULL << 16U) | 0xfffeU, tick);
  ASSERT_TRUE(stb::extendCounter16Snapshot(7, 2U, true, 7, tick));
  EXPECT_EQ((8ULL << 16U) | 2U, tick);
  ASSERT_TRUE(stb::extendCounter16Snapshot(7, 0xfffeU, true, 7, tick));
  EXPECT_EQ((7ULL << 16U) | 0xfffeU, tick);
  EXPECT_FALSE(stb::extendCounter16Snapshot(7, 1U, false, 8, tick));
}

TEST(TimeMapping, RobustFitRoundTripOutlier)
{
  std::vector<stb::TimePair> pairs;
  const uint64_t local0 = stb::kSyntheticEpochNs;
  const uint64_t utc0 = 1700000000000000000ULL;
  for (uint64_t i = 0; i < 30; ++i)
  {
    uint64_t utc = utc0 + i * 1000000050ULL;
    if (i == 14) utc += 50000000ULL;
    pairs.push_back({local0 + i * 1000000000ULL, utc, 1000});
  }
  stb::AffineMapping mapping;
  ASSERT_TRUE(stb::robustAffineFit(pairs, 1000000.0, mapping));
  EXPECT_NEAR(0.05, mapping.slope_ppm, 0.01);
  uint64_t utc = 0, local = 0;
  ASSERT_TRUE(mapping.localToUtc(local0 + 1234567890ULL, utc));
  ASSERT_TRUE(mapping.utcToLocal(utc, local));
  EXPECT_LE(local > local0 + 1234567890ULL
                ? local - (local0 + 1234567890ULL)
                : (local0 + 1234567890ULL) - local,
            1U);
}

TEST(TimeMapping, FullStateSequenceKeepsLocalContinuous)
{
  stb::MappingParameters parameters;
  parameters.min_pps_pairs = 10;
  parameters.min_fit_span_ns = 9000000000ULL;
  parameters.relock_confirm_count = 10;
  parameters.pps_timeout_ns = 1000000000ULL;
  parameters.max_pps_residual_ns = 10000.0;
  parameters.holdover_uncertainty_growth_ns_per_s = 1000.0;
  stb::MappingStateMachine mapping(parameters);
  const uint64_t local0 = stb::kSyntheticEpochNs;
  const uint64_t utc0 = 1700000000000000000ULL;
  uint64_t previous_camera = 0, previous_lidar = 0, previous_imu = 0;
  bool saw_acquiring = false, saw_locked = false, saw_holdover = false;
  bool saw_relocking = false, saw_relocked = false;
  uint32_t first_lock_version = 0;
  uint64_t holdover_uncertainty = 0;
  uint64_t relocked_uncertainty = 0;

  for (uint64_t tenth = 0; tenth <= 12000; ++tenth)
  {
    const uint64_t local = local0 + tenth * 100000000ULL;
    if (tenth % 10 == 0)
    {
      const uint64_t second = tenth / 10;
      const bool first_window = second >= 300 && second <= 600;
      const bool second_window = second >= 900;
      if (first_window || second_window)
      {
        const uint64_t utc = utc0 + second * 1000000020ULL;
        mapping.addPair({local, utc, 1000});
      }
      mapping.update(local);
      saw_acquiring |= mapping.state() == stb::TimeState::ACQUIRING;
      if (mapping.state() == stb::TimeState::LOCKED)
      {
        if (!saw_locked)
        {
          saw_locked = true;
          first_lock_version = mapping.mappingVersion();
        }
        else if (saw_relocking && mapping.mappingVersion() > first_lock_version)
        {
          saw_relocked = true;
          relocked_uncertainty = mapping.uncertainty(local);
        }
      }
      if (mapping.state() == stb::TimeState::HOLDOVER)
      {
        saw_holdover = true;
        holdover_uncertainty = mapping.uncertainty(local);
      }
      saw_relocking |= mapping.state() == stb::TimeState::RELOCKING;
    }
    if (tenth % 1 == 0)
    {
      EXPECT_GT(local, previous_camera);
      EXPECT_GT(local, previous_lidar);
      previous_camera = previous_lidar = local;
    }
    for (int imu = 0; imu < 20; ++imu)
    {
      const uint64_t imu_stamp = local + static_cast<uint64_t>(imu) * 5000000ULL;
      EXPECT_GT(imu_stamp, previous_imu);
      previous_imu = imu_stamp;
    }
  }
  EXPECT_TRUE(saw_acquiring);
  EXPECT_TRUE(saw_locked);
  EXPECT_TRUE(saw_holdover);
  EXPECT_TRUE(saw_relocking);
  EXPECT_TRUE(saw_relocked);
  EXPECT_GT(holdover_uncertainty, 1000U);
  EXPECT_LT(relocked_uncertainty, holdover_uncertainty);
}

TEST(TimeMapping, RelockStepIsRejectedAndHistoryIsBounded)
{
  stb::MappingParameters parameters;
  parameters.min_pps_pairs = 3;
  parameters.relock_confirm_count = 3;
  parameters.min_fit_span_ns = 2000000000ULL;
  parameters.pps_timeout_ns = 1000000000ULL;
  parameters.max_pps_residual_ns = 1000.0;
  parameters.max_relock_step_ns = 10000000ULL;
  parameters.mapping_history_size = 2;
  stb::MappingStateMachine mapping(parameters);
  const uint64_t local0 = stb::kSyntheticEpochNs;
  const uint64_t utc0 = 1700000000000000000ULL;
  for (uint64_t second = 0; second < 3; ++second)
    ASSERT_TRUE(mapping.addPair(
        {local0 + second * 1000000000ULL,
         utc0 + second * 1000000000ULL, 1000}));
  ASSERT_EQ(stb::TimeState::LOCKED, mapping.state());
  for (uint64_t second = 3; second < 5; ++second)
    ASSERT_TRUE(mapping.addPair(
        {local0 + second * 1000000000ULL,
         utc0 + second * 1000000000ULL, 1000}));
  const uint32_t locked_version = mapping.mappingVersion();
  ASSERT_EQ(2U, mapping.history().size());

  mapping.update(local0 + 6000000000ULL);
  ASSERT_EQ(stb::TimeState::HOLDOVER, mapping.state());
  for (uint64_t second = 6; second < 9; ++second)
    mapping.addPair({local0 + second * 1000000000ULL,
                     utc0 + second * 1000000000ULL + 1000000000ULL, 1000});
  EXPECT_EQ(stb::TimeState::RELOCKING, mapping.state());
  EXPECT_EQ(locked_version, mapping.mappingVersion());
  EXPECT_EQ(2U, mapping.history().size());
  EXPECT_TRUE(mapping.mappingUsable());
}

TEST(HostMonotonicMapper, ReceiveTimeIsMappedWithUncertainty)
{
  stb::HostMonotonicMapper mapper;
  for (uint64_t i = 0; i < 10; ++i)
    EXPECT_TRUE(mapper.add(1000000000ULL + i * 100000000ULL,
                           stb::kSyntheticEpochNs + i * 100000010ULL));
  uint64_t local = 0, uncertainty = 0;
  ASSERT_TRUE(mapper.toLocal(1500000000ULL, local, uncertainty));
  EXPECT_NEAR(static_cast<double>(stb::kSyntheticEpochNs + 500000050ULL),
              static_cast<double>(local), 2.0);
  EXPECT_GT(uncertainty, 0U);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
