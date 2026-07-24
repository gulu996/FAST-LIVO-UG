#include <gtest/gtest.h>

#include "mvs_ros_driver/timestamp_monitor.h"

namespace {

livox_ros::SharedTimestampSnapshot Snapshot(uint64_t epoch,
                                            uint64_t stamp_ns,
                                            uint64_t monotonic_ns) {
  livox_ros::SharedTimestampSnapshot snapshot;
  snapshot.magic = livox_ros::kSharedTimestampMagic;
  snapshot.version = livox_ros::kSharedTimestampVersion;
  snapshot.writer_epoch = epoch;
  snapshot.stamp_ns = stamp_ns;
  snapshot.clock_source = livox_ros::kLidarBaseTimeLegacyGps;
  snapshot.ready = true;
  snapshot.last_update_monotonic_ns = monotonic_ns;
  return snapshot;
}

TEST(TimestampMonitor, DetectsDuplicateNonMonotonicAndStale) {
  mvs_ros_driver::TimestampMonitor monitor(500, 500);
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kAccepted,
            monitor.Validate(Snapshot(1, 1000, 100), 200));
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kDuplicate,
            monitor.Validate(Snapshot(1, 1000, 200), 250));
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kNonMonotonic,
            monitor.Validate(Snapshot(1, 900, 250), 300));
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kStale,
            monitor.Validate(Snapshot(1, 1100, 100), 1000));
  EXPECT_EQ(1U, monitor.duplicate_stamp_count());
  EXPECT_EQ(1U, monitor.non_monotonic_stamp_count());
  EXPECT_EQ(1U, monitor.stale_stamp_count());
}

TEST(TimestampMonitor, DropsEpochChangeAndLargeStep) {
  mvs_ros_driver::TimestampMonitor monitor(1000, 200);
  ASSERT_EQ(mvs_ros_driver::StampValidationResult::kAccepted,
            monitor.Validate(Snapshot(1, 1000, 100), 110));
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kWriterEpochChanged,
            monitor.Validate(Snapshot(2, 1100, 200), 210));
  ASSERT_EQ(mvs_ros_driver::StampValidationResult::kAccepted,
            monitor.Validate(Snapshot(2, 1100, 250), 260));
  EXPECT_EQ(mvs_ros_driver::StampValidationResult::kLargeStep,
            monitor.Validate(Snapshot(2, 2000, 300), 310));
  EXPECT_EQ(1U, monitor.writer_restart_count());
  EXPECT_EQ(1U, monitor.large_step_count());
}

TEST(TimestampMonitor, CountsFrameAndTriggerGaps) {
  mvs_ros_driver::TimestampMonitor monitor(1000, 1000);
  monitor.ObserveCameraMetadata(10, 20, 30, 1000);
  monitor.ObserveCameraMetadata(13, 22, 34, 1100);
  EXPECT_EQ(2U, monitor.frame_gap_count());
  EXPECT_EQ(1U, monitor.frame_counter_gap_count());
  EXPECT_EQ(3U, monitor.trigger_gap_count());
  monitor.ObserveCameraMetadata(14, 23, 35, 5);
  EXPECT_EQ(1U, monitor.device_time_wrap_count());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
