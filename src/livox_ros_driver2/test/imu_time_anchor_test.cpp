#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "livox_ros_driver2/imu_time_anchor.h"

namespace {

using livox_ros::ImuTimeAnchorSnapshot;
using livox_ros::ImuTimeAnchorState;

TEST(ImuTimeAnchorProtocol, LayoutAndGoldenBytesMatchPython) {
  EXPECT_EQ(64U, sizeof(ImuTimeAnchorState));
  EXPECT_EQ(8U, alignof(ImuTimeAnchorState));
  EXPECT_EQ(0U, offsetof(ImuTimeAnchorState, magic));
  EXPECT_EQ(4U, offsetof(ImuTimeAnchorState, version));
  EXPECT_EQ(8U, offsetof(ImuTimeAnchorState, writer_epoch));
  EXPECT_EQ(16U, offsetof(ImuTimeAnchorState, write_sequence));
  EXPECT_EQ(24U, offsetof(ImuTimeAnchorState, imu_stamp_ns));
  EXPECT_EQ(32U, offsetof(ImuTimeAnchorState, host_monotonic_ns));
  EXPECT_EQ(40U, offsetof(ImuTimeAnchorState, update_monotonic_ns));
  EXPECT_EQ(48U, offsetof(ImuTimeAnchorState, clock_source));
  EXPECT_EQ(52U, offsetof(ImuTimeAnchorState, ready));
  EXPECT_EQ(56U, offsetof(ImuTimeAnchorState, uncertainty_ns));

  ImuTimeAnchorState state = {};
  state.magic = livox_ros::kImuTimeAnchorMagic;
  state.version = livox_ros::kImuTimeAnchorVersion;
  state.writer_epoch = 0x0102030405060708ULL;
  state.write_sequence = 0x1112131415161718ULL;
  state.imu_stamp_ns = 0x2122232425262728ULL;
  state.host_monotonic_ns = 0x3132333435363738ULL;
  state.update_monotonic_ns = 0x4142434445464748ULL;
  state.clock_source = livox_ros::kImuAnchorDeviceGps;
  state.ready = 1;
  state.uncertainty_ns = 0x5152535455565758ULL;

  const std::array<uint8_t, 64> golden = {{
      0x49, 0x4d, 0x41, 0x31, 0x01, 0x00, 0x00, 0x00,
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
      0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,
      0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,
      0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31,
      0x48, 0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41,
      0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
      0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51,
  }};
  EXPECT_EQ(0, std::memcmp(&state, golden.data(), golden.size()));

  ImuTimeAnchorSnapshot parsed;
  EXPECT_EQ(livox_ros::kImuTimeAnchorReadOk,
            livox_ros::ReadImuTimeAnchorState(&state, &parsed));
  EXPECT_EQ(state.writer_epoch, parsed.writer_epoch);
  EXPECT_EQ(state.imu_stamp_ns, parsed.imu_stamp_ns);
  EXPECT_EQ(state.host_monotonic_ns, parsed.host_monotonic_ns);
}

TEST(ImuTimeAnchorProtocol, RejectsOddAndBadProtocolState) {
  ImuTimeAnchorState state = {};
  livox_ros::WriteImuTimeAnchorState(
      &state, 9, 100, 200, 300, livox_ros::kImuAnchorDeviceGps,
      true, 10);
  ImuTimeAnchorSnapshot snapshot;
  state.write_sequence |= 1U;
  EXPECT_EQ(livox_ros::kImuTimeAnchorReadInconsistent,
            livox_ros::ReadImuTimeAnchorState(&state, &snapshot, 1));
  state.write_sequence += 1U;
  state.magic = 0;
  EXPECT_EQ(livox_ros::kImuTimeAnchorReadBadProtocol,
            livox_ros::ReadImuTimeAnchorState(&state, &snapshot));
}

TEST(ImuTimeAnchorProtocol, CrossProcessSeqlockNeverTears) {
  auto* state = static_cast<ImuTimeAnchorState*>(
      mmap(nullptr, sizeof(ImuTimeAnchorState), PROT_READ | PROT_WRITE,
           MAP_SHARED | MAP_ANONYMOUS, -1, 0));
  ASSERT_NE(MAP_FAILED, state);
  std::memset(state, 0, sizeof(*state));
  livox_ros::WriteImuTimeAnchorState(
      state, 55, 1, 1001, 2001, livox_ros::kImuAnchorDeviceGps,
      true, 10);

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    for (uint64_t value = 2; value <= 20000; ++value) {
      livox_ros::WriteImuTimeAnchorState(
          state, 55, value, value + 1000, value + 2000,
          livox_ros::kImuAnchorDeviceGps, true, 10);
    }
    _exit(0);
  }

  int child_status = 0;
  uint64_t successful_reads = 0;
  while (waitpid(child, &child_status, WNOHANG) == 0) {
    ImuTimeAnchorSnapshot snapshot;
    if (livox_ros::ReadImuTimeAnchorState(state, &snapshot) ==
        livox_ros::kImuTimeAnchorReadOk) {
      ++successful_reads;
      EXPECT_EQ(snapshot.imu_stamp_ns + 1000,
                snapshot.host_monotonic_ns);
      EXPECT_EQ(snapshot.imu_stamp_ns + 2000,
                snapshot.update_monotonic_ns);
      EXPECT_EQ(0U, snapshot.write_sequence & 1U);
    }
  }
  EXPECT_TRUE(WIFEXITED(child_status));
  EXPECT_EQ(0, WEXITSTATUS(child_status));
  EXPECT_GT(successful_reads, 0U);
  EXPECT_EQ(0, munmap(state, sizeof(ImuTimeAnchorState)));
}

TEST(ImuTimeAnchorWriter, ReadyGateResetAndShutdownInvalidation) {
  char path[] = "/tmp/livox_imu_anchor_writer_XXXXXX";
  const int temporary_fd = mkstemp(path);
  ASSERT_GE(temporary_fd, 0);
  ASSERT_EQ(0, close(temporary_fd));

  livox_ros::ImuTimeAnchorWriterConfig config;
  config.min_ready_samples = 3;
  config.max_stamp_step_ns = 100000000ULL;
  config.uncertainty_ns = 10000000ULL;
  livox_ros::ImuTimeAnchorWriter writer(config);
  std::string error;
  ASSERT_TRUE(writer.Open(path, 123, &error)) << error;

  EXPECT_FALSE(writer.Observe(
      1000000000ULL, 5000000000ULL,
      livox_ros::kImuAnchorDeviceGps, 5000000100ULL));
  EXPECT_FALSE(writer.Observe(
      1005000000ULL, 5005000000ULL,
      livox_ros::kImuAnchorDeviceGps, 5005000100ULL));
  EXPECT_TRUE(writer.Observe(
      1010000000ULL, 5010000000ULL,
      livox_ros::kImuAnchorDeviceGps, 5010000100ULL));
  EXPECT_TRUE(writer.ready());

  EXPECT_FALSE(writer.Observe(
      1010000000ULL, 5015000000ULL,
      livox_ros::kImuAnchorDeviceGps, 5015000100ULL));
  EXPECT_FALSE(writer.ready());
  EXPECT_EQ(1U, writer.stats().duplicate);
  writer.Close();

  const int fd = open(path, O_RDONLY);
  ASSERT_GE(fd, 0);
  struct stat file_stat = {};
  ASSERT_EQ(0, fstat(fd, &file_stat));
  EXPECT_EQ(64, file_stat.st_size);
  void* mapping = mmap(nullptr, 64, PROT_READ, MAP_SHARED, fd, 0);
  ASSERT_NE(MAP_FAILED, mapping);
  ImuTimeAnchorSnapshot closed;
  EXPECT_EQ(livox_ros::kImuTimeAnchorReadNotReady,
            livox_ros::ReadImuTimeAnchorState(
                static_cast<const ImuTimeAnchorState*>(mapping), &closed));
  EXPECT_FALSE(closed.ready);
  EXPECT_EQ(0U, closed.imu_stamp_ns);
  EXPECT_EQ(0U, closed.host_monotonic_ns);
  EXPECT_EQ(0, munmap(mapping, 64));
  EXPECT_EQ(0, close(fd));
  EXPECT_EQ(0, unlink(path));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
