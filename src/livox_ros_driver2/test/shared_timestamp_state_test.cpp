#include <gtest/gtest.h>

#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ros/time.h>

#include "livox_ros_driver2/shared_timestamp_state.h"

namespace {

using livox_ros::SharedTimestampSnapshot;
using livox_ros::SharedTimestampState;

TEST(SharedTimestampState, ReadsCommittedState) {
  SharedTimestampState state = {};
  livox_ros::WriteSharedTimestampState(
      &state, 42, 123456789ULL, livox_ros::kLidarBaseTimeLegacyGps,
      true, 900);

  SharedTimestampSnapshot snapshot;
  EXPECT_EQ(livox_ros::kSharedTimestampReadOk,
            livox_ros::ReadSharedTimestampState(&state, &snapshot));
  EXPECT_EQ(42U, snapshot.writer_epoch);
  EXPECT_EQ(123456789U, snapshot.stamp_ns);
  EXPECT_TRUE(snapshot.ready);
  EXPECT_EQ(0U, snapshot.write_sequence & 1U);
}

TEST(SharedTimestampState, RejectsWriterNotReady) {
  SharedTimestampState state = {};
  livox_ros::WriteSharedTimestampState(
      &state, 7, 100, livox_ros::kLidarBaseTimeLegacyPtp, false, 50);
  SharedTimestampSnapshot snapshot;
  EXPECT_EQ(livox_ros::kSharedTimestampReadNotReady,
            livox_ros::ReadSharedTimestampState(&state, &snapshot));
  EXPECT_EQ(7U, snapshot.writer_epoch);
}

TEST(SharedTimestampState, ExposesWriterRestartEpoch) {
  SharedTimestampState state = {};
  SharedTimestampSnapshot first;
  SharedTimestampSnapshot second;
  livox_ros::WriteSharedTimestampState(
      &state, 100, 1000, livox_ros::kLidarBaseTimeLegacyGps, true, 10);
  ASSERT_EQ(livox_ros::kSharedTimestampReadOk,
            livox_ros::ReadSharedTimestampState(&state, &first));
  livox_ros::WriteSharedTimestampState(
      &state, 101, 1100, livox_ros::kLidarBaseTimeLegacyGps, true, 20);
  ASSERT_EQ(livox_ros::kSharedTimestampReadOk,
            livox_ros::ReadSharedTimestampState(&state, &second));
  EXPECT_NE(first.writer_epoch, second.writer_epoch);
}

TEST(SharedTimestampState, RejectsStartupBurstUntilRealtimeCadence) {
  livox_ros::LegacyTimestampReadyGate gate(
      3, 300000000ULL, 25000000ULL, 300000000ULL);
  EXPECT_FALSE(gate.Observe(100000000ULL,
                            livox_ros::kLidarBaseTimeLegacyGps,
                            1000000000ULL));
  EXPECT_FALSE(gate.Observe(200000000ULL,
                            livox_ros::kLidarBaseTimeLegacyGps,
                            1001000000ULL));
  EXPECT_FALSE(gate.Observe(300000000ULL,
                            livox_ros::kLidarBaseTimeLegacyGps,
                            1002000000ULL));
  EXPECT_FALSE(gate.Observe(400000000ULL,
                            livox_ros::kLidarBaseTimeLegacyGps,
                            1102000000ULL));
  EXPECT_TRUE(gate.Observe(500000000ULL,
                           livox_ros::kLidarBaseTimeLegacyGps,
                           1202000000ULL));
}

TEST(SharedTimestampState, CrossProcessSeqlockNeverReturnsTornState) {
  auto* state = static_cast<SharedTimestampState*>(
      mmap(nullptr, sizeof(SharedTimestampState), PROT_READ | PROT_WRITE,
           MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  ASSERT_NE(MAP_FAILED, state);
  *state = {};
  livox_ros::WriteSharedTimestampState(
      state, 55, 1, livox_ros::kLidarBaseTimeLegacyGps, true, 1);

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    for (uint64_t value = 2; value <= 20000; ++value) {
      livox_ros::WriteSharedTimestampState(
          state, 55, value, livox_ros::kLidarBaseTimeLegacyGps,
          true, value);
    }
    _exit(0);
  }

  int child_status = 0;
  uint64_t successful_reads = 0;
  while (waitpid(child, &child_status, WNOHANG) == 0) {
    SharedTimestampSnapshot snapshot;
    if (livox_ros::ReadSharedTimestampState(state, &snapshot) ==
        livox_ros::kSharedTimestampReadOk) {
      ++successful_reads;
      EXPECT_EQ(55U, snapshot.writer_epoch);
      EXPECT_EQ(snapshot.stamp_ns, snapshot.last_update_monotonic_ns);
      EXPECT_EQ(0U, snapshot.write_sequence & 1U);
    }
  }
  EXPECT_TRUE(WIFEXITED(child_status));
  EXPECT_EQ(0, WEXITSTATUS(child_status));
  EXPECT_GT(successful_reads, 0U);
  EXPECT_EQ(0, munmap(state, sizeof(SharedTimestampState)));
}

TEST(RosTimeConversion, FromNSecIsIntegerExact) {
  const uint64_t timestamp_ns = 1590192198700268745ULL;
  ros::Time stamp;
  stamp.fromNSec(timestamp_ns);
  EXPECT_EQ(timestamp_ns, stamp.toNSec());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
