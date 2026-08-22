/*
Small dependency-free regression check for the GNSS adapter core.
*/

#include "gnss_adapter.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void check(bool condition, const std::string &message)
{
  if (!condition) throw std::runtime_error(message);
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
  return std::fabs(actual - expected) <= tolerance;
}

GnssAdapterConfig manualConfig()
{
  GnssAdapterConfig config;
  config.origin_mode = "manual";
  config.origin_lla << 22.5299731, 113.9331154, 39.324;
  config.fixed_confirm_count = 2;
  config.fixed_lost_count = 2;
  config.recovery_confirm_count = 3;
  return config;
}

GnssAdapterConfig averageConfig(int sample_count = 10)
{
  GnssAdapterConfig config = manualConfig();
  config.origin_mode = "average_fixed";
  config.fixed_confirm_count = 1;
  config.origin_average_count = sample_count;
  config.origin_average_max_gap_s = 2.0;
  config.max_time_gap_s = 10.0;
  return config;
}

gnss_comm::GnssPVTSolnMsg fixedMessage(double tow = 186157.8)
{
  gnss_comm::GnssPVTSolnMsg message;
  message.time.week = 2393;
  message.time.tow = tow;
  message.fix_type = 3;
  message.valid_fix = true;
  message.diff_soln = true;
  message.carr_soln = 2;
  message.num_sv = 25;
  message.latitude = 22.5299731;
  message.longitude = 113.9331154;
  message.altitude = 39.324;
  message.height_msl = 30.0;
  message.h_acc = 0.014;
  message.v_acc = 0.016;
  message.p_dop = 1.63;
  message.vel_n = 1.0;
  message.vel_e = 2.0;
  message.vel_d = -0.5;
  message.vel_acc = 0.02;
  return message;
}

gnss_serial_driver::GnssPvtStamped stampedFixed(std::uint64_t stamp_ns)
{
  gnss_serial_driver::GnssPvtStamped message;
  message.header.stamp.fromNSec(stamp_ns);
  message.session_id = 1;
  message.writer_epoch = 1;
  message.local_measurement_time_valid = true;
  message.valid_for_fusion = true;
  message.pvt = fixedMessage();
  message.pvt.time.week = 0;
  message.pvt.time.tow = 0.0;
  return message;
}

void testStampedLocalTimeAndMetadataGates()
{
  GnssAdapter adapter(manualConfig());
  const ros::Time callback_time(999, 123);
  const std::uint64_t first_stamp_ns = 100123456789ULL;
  const std::uint64_t second_stamp_ns = first_stamp_ns + 100000000ULL;

  const GnssAdapterResult first =
      adapter.process(stampedFixed(first_stamp_ns), callback_time);
  check(first.status.header.stamp.toNSec() == first_stamp_ns,
        "stamped-local status must preserve wrapper header stamp exactly");
  check(first.status.reject_reason == "RTK_NOT_CONFIRMED",
        "stamped-local fixed confirmation must use the shared state machine");

  const GnssAdapterResult second =
      adapter.process(stampedFixed(second_stamp_ns), callback_time);
  check(second.status.accepted && second.publish_odometry,
        "stamped-local confirmed fixed sample must publish");
  check(second.status.header.stamp.toNSec() == second_stamp_ns &&
        second.odometry.header.stamp.toNSec() == second_stamp_ns,
        "stamped-local odometry and status stamps must match at nanosecond precision");
  check(second.status.header.stamp != callback_time,
        "callback time must not replace stamped-local measurement time");

  GnssAdapter zero_adapter(manualConfig());
  auto zero = stampedFixed(0);
  check(zero_adapter.process(zero, callback_time).status.reject_reason ==
            "ZERO_MEASUREMENT_TIMESTAMP",
        "zero stamped-local header must be rejected");

  GnssAdapter local_time_adapter(manualConfig());
  auto invalid_local_time = stampedFixed(first_stamp_ns);
  invalid_local_time.local_measurement_time_valid = false;
  check(local_time_adapter.process(invalid_local_time, callback_time)
            .status.reject_reason == "LOCAL_MEASUREMENT_TIME_INVALID",
        "invalid local measurement time flag must be rejected");

  GnssAdapter fusion_flag_adapter(manualConfig());
  auto invalid_for_fusion = stampedFixed(first_stamp_ns);
  invalid_for_fusion.valid_for_fusion = false;
  check(fusion_flag_adapter.process(invalid_for_fusion, callback_time)
            .status.reject_reason == "INVALID_FOR_FUSION",
        "invalid-for-fusion wrapper must be rejected");
}

void testStampedLocalLossAndRecovery()
{
  GnssAdapter adapter(manualConfig());
  std::uint64_t stamp_ns = 200000000000ULL;
  adapter.process(stampedFixed(stamp_ns), ros::Time(999));
  stamp_ns += 100000000ULL;
  check(adapter.process(stampedFixed(stamp_ns), ros::Time(999)).status.accepted,
        "stamped-local fixed confirmation count must activate");

  auto lost = stampedFixed(stamp_ns + 100000000ULL);
  lost.pvt.valid_fix = false;
  adapter.process(lost, ros::Time(999));
  lost.header.stamp.fromNSec(stamp_ns + 200000000ULL);
  check(adapter.process(lost, ros::Time(999)).status.filtered_quality ==
            fast_livo::GnssStatus::INVALID,
        "stamped-local fixed loss count must enter LOST");

  stamp_ns += 300000000ULL;
  check(adapter.process(stampedFixed(stamp_ns), ros::Time(999))
            .status.filtered_quality == fast_livo::GnssStatus::RECOVERING,
        "stamped-local first fixed after loss must recover, not publish");
  stamp_ns += 100000000ULL;
  adapter.process(stampedFixed(stamp_ns), ros::Time(999));
  stamp_ns += 100000000ULL;
  check(adapter.process(stampedFixed(stamp_ns), ros::Time(999)).status.accepted,
        "stamped-local recovery confirmation count must reactivate");
}

void testFixedStateIgnoresUnavailableAccuracyRecord()
{
  GnssAdapter adapter(manualConfig());
  std::uint64_t stamp_ns = 300000000000ULL;
  adapter.process(stampedFixed(stamp_ns), ros::Time(999));

  auto unavailable_accuracy = stampedFixed(stamp_ns + 100000000ULL);
  unavailable_accuracy.pvt.h_acc = 999.0;
  const GnssAdapterResult filtered =
      adapter.process(unavailable_accuracy, ros::Time(999));
  check(filtered.status.filtered_quality == fast_livo::GnssStatus::RTK_FIXED &&
        !filtered.status.accepted &&
        filtered.status.reject_reason == "H_ACC_TOO_LARGE",
        "a fixed carrier solution with unavailable accuracy must be filtered, not count as fixed loss");

  stamp_ns += 200000000ULL;
  check(adapter.process(stampedFixed(stamp_ns), ros::Time(999)).status.accepted,
        "the next usable fixed record must retain the confirmed carrier state");
}

void testTimeStateEnuAndCovariance()
{
  GnssAdapter adapter(manualConfig());
  const ros::Time callback_time(1763437340, 0);

  const GnssAdapterResult first = adapter.process(fixedMessage(), callback_time);
  check(near(first.status.header.stamp.toSec(), 1763437339.8, 1e-6),
        "GPST to UTC conversion omitted/duplicated the leap-second correction");
  check(first.status.raw_quality == fast_livo::GnssStatus::RTK_FIXED,
        "sample must classify as raw RTK_FIXED");
  check(first.status.filtered_quality == fast_livo::GnssStatus::INVALID,
        "first fixed sample must await confirmation");
  check(!first.status.accepted && first.status.reject_reason == "RTK_NOT_CONFIRMED",
        "unconfirmed fixed sample must be rejected explicitly");

  const GnssAdapterResult second = adapter.process(fixedMessage(186157.9), callback_time);
  check(second.status.filtered_quality == fast_livo::GnssStatus::RTK_FIXED,
        "confirmed fixed sample must become active");
  check(second.status.accepted && second.publish_odometry,
        "confirmed fixed sample with manual origin must publish");
  check(std::fabs(second.odometry.pose.pose.position.x) < 1e-6 &&
        std::fabs(second.odometry.pose.pose.position.y) < 1e-6 &&
        std::fabs(second.odometry.pose.pose.position.z) < 1e-6,
        "manual origin equal to measurement must produce zero ENU");
  check(near(second.odometry.twist.twist.linear.x, 2.0) &&
        near(second.odometry.twist.twist.linear.y, 1.0) &&
        near(second.odometry.twist.twist.linear.z, 0.5),
        "NED velocity must map to ENU");
  check(near(second.odometry.pose.covariance[0], 0.03 * 0.03) &&
        near(second.odometry.pose.covariance[7], 0.03 * 0.03) &&
        near(second.odometry.pose.covariance[14], 0.05 * 0.05),
        "position standard deviations must be clamped then squared");
  check(near(second.odometry.twist.covariance[0], 0.05 * 0.05),
        "velocity standard deviation floor must be squared");
  check(second.odometry.pose.covariance[21] >= 1e6 &&
        second.odometry.pose.covariance[28] >= 1e6 &&
        second.odometry.pose.covariance[35] >= 1e6,
        "placeholder orientation must have unknown/large covariance");
}

void testInvalidInputs()
{
  const GnssAdapterConfig config = manualConfig();

  gnss_comm::GnssPVTSolnMsg message = fixedMessage();
  message.latitude = std::numeric_limits<double>::quiet_NaN();
  GnssAdapter nan_adapter(config);
  check(nan_adapter.process(message, ros::Time(1)).status.reject_reason == "INVALID_LATITUDE",
        "NaN latitude must be rejected");

  message = fixedMessage();
  message.valid_fix = false;
  GnssAdapter invalid_fix_adapter(config);
  check(invalid_fix_adapter.process(message, ros::Time(1)).status.reject_reason == "INVALID_FIX",
        "invalid fix must be rejected");

  message = fixedMessage();
  message.carr_soln = 1;
  GnssAdapter float_adapter(config);
  const GnssAdapterResult float_result = float_adapter.process(message, ros::Time(1));
  check(float_result.status.raw_quality == fast_livo::GnssStatus::RTK_FLOAT &&
        !float_result.status.accepted,
        "RTK Float must classify correctly and obey the default reject policy");

  message = fixedMessage();
  message.num_sv = 5;
  GnssAdapter satellite_adapter(config);
  check(satellite_adapter.process(message, ros::Time(1)).status.reject_reason ==
            "NUM_SV_TOO_LOW",
        "too few satellites must be rejected");

  message = fixedMessage();
  message.h_acc = 1.42;
  GnssAdapter accuracy_adapter(config);
  check(accuracy_adapter.process(message, ros::Time(1)).status.reject_reason ==
            "H_ACC_TOO_LARGE",
        "large horizontal accuracy estimate must be rejected");
}

void testTimeOrderingAndMissingOrigin()
{
  GnssAdapterConfig config = manualConfig();
  config.fixed_confirm_count = 1;
  GnssAdapter adapter(config);
  check(adapter.process(fixedMessage(), ros::Time(1)).status.accepted,
        "single-frame test configuration must activate fixed state");
  check(adapter.process(fixedMessage(), ros::Time(1)).status.reject_reason ==
            "DUPLICATE_GNSS_TIME",
        "duplicate GNSS time must not be processed twice");
  check(adapter.process(fixedMessage(186157.7), ros::Time(1)).status.reject_reason ==
            "NON_MONOTONIC_TIME",
        "backward GNSS time must be rejected");

  config.origin_mode = "first_fixed";
  config.accept_rtk_float = true;
  GnssAdapter no_origin_adapter(config);
  gnss_comm::GnssPVTSolnMsg float_message = fixedMessage();
  float_message.carr_soln = 1;
  const GnssAdapterResult no_origin = no_origin_adapter.process(float_message, ros::Time(1));
  check(!no_origin.status.origin_initialized && !no_origin.status.accepted &&
        no_origin.status.reject_reason == "ORIGIN_NOT_INITIALIZED",
        "enabled non-fixed quality must still wait for a confirmed-fixed origin");
}

void testLossAndRecovery()
{
  GnssAdapter adapter(manualConfig());
  adapter.process(fixedMessage(), ros::Time(1));
  check(adapter.process(fixedMessage(186157.9), ros::Time(1)).status.accepted,
        "state must become active after initial confirmation");

  gnss_comm::GnssPVTSolnMsg invalid = fixedMessage(186158.0);
  invalid.valid_fix = false;
  const GnssAdapterResult first_loss = adapter.process(invalid, ros::Time(1));
  check(first_loss.status.filtered_quality == fast_livo::GnssStatus::RTK_FIXED &&
        !first_loss.status.accepted && first_loss.status.consecutive_lost_count == 1,
        "ACTIVE_FIXED must hold filtered state, but reject the current bad sample");
  invalid.time.tow = 186158.1;
  adapter.process(invalid, ros::Time(1));

  GnssAdapterResult recovery = adapter.process(fixedMessage(186158.2), ros::Time(1));
  check(recovery.status.filtered_quality == fast_livo::GnssStatus::RECOVERING &&
        !recovery.status.accepted,
        "first fixed sample after loss must enter RECOVERING");
  recovery = adapter.process(fixedMessage(186158.3), ros::Time(1));
  check(!recovery.status.accepted, "recovery must require consecutive fixed samples");
  recovery = adapter.process(fixedMessage(186158.4), ros::Time(1));
  check(recovery.status.filtered_quality == fast_livo::GnssStatus::RTK_FIXED &&
        recovery.status.accepted,
        "recovery confirmation count must restore active fixed state");

}

void testAverageOriginContinuousFixed()
{
  GnssAdapter adapter(averageConfig());
  const std::uint64_t base_ns = 100000000000ULL;
  GnssAdapterResult result;
  for (int index = 0; index < 10; ++index)
  {
    result = adapter.process(
        stampedFixed(base_ns + static_cast<std::uint64_t>(index) * 100000000ULL),
        ros::Time(999));
    check(result.origin_average_accepted,
          "every usable Fixed candidate must be accepted into the average");
    check(result.origin_average_count == static_cast<uint32_t>(index + 1),
          "average candidate count must increase exactly once per epoch");
    if (index < 9)
      check(!result.status.origin_initialized,
            "average origin must wait for all configured samples");
  }
  check(result.origin_initialized_now && result.status.origin_initialized &&
        result.status.accepted,
        "the tenth continuous Fixed candidate must initialize the origin");
}

void testAverageOriginSkipsRejectedRecords()
{
  GnssAdapter adapter(averageConfig());
  const std::uint64_t base_ns = 200000000000ULL;
  GnssAdapterResult result;
  for (int index = 0; index < 10; ++index)
  {
    const std::uint64_t good_stamp =
        base_ns + static_cast<std::uint64_t>(index) * 200000000ULL;
    result = adapter.process(stampedFixed(good_stamp), ros::Time(999));
    check(result.origin_average_count == static_cast<uint32_t>(index + 1),
          "usable Fixed records must accumulate across rejected records");

    if (index == 0)
    {
      auto invalid_for_fusion = stampedFixed(good_stamp + 25000000ULL);
      invalid_for_fusion.valid_for_fusion = false;
      const GnssAdapterResult skipped =
          adapter.process(invalid_for_fusion, ros::Time(999));
      check(skipped.origin_average_skipped &&
            skipped.origin_average_count == 1 &&
            skipped.origin_average_skip_reason == "INVALID_FOR_FUSION",
            "invalid_for_fusion must preserve collected origin candidates");
    }

    if (index < 9)
    {
      auto rejected_accuracy = stampedFixed(good_stamp + 50000000ULL);
      rejected_accuracy.pvt.h_acc = 999.0;
      rejected_accuracy.pvt.v_acc = 999.0;
      const GnssAdapterResult skipped =
          adapter.process(rejected_accuracy, ros::Time(999));
      check(skipped.status.reject_reason == "H_ACC_TOO_LARGE" &&
            skipped.origin_average_skipped &&
            !skipped.origin_average_reset &&
            skipped.origin_average_count == static_cast<uint32_t>(index + 1),
            "quality-gate rejection must skip without clearing the average");
    }

    if (index == 0)
    {
      auto non_candidate_quality = stampedFixed(good_stamp + 75000000ULL);
      non_candidate_quality.pvt.carr_soln = 1;
      const GnssAdapterResult skipped =
          adapter.process(non_candidate_quality, ros::Time(999));
      check(skipped.origin_average_skipped &&
            !skipped.origin_average_reset &&
            skipped.origin_average_count == 1,
            "a non-candidate RTK Float record must preserve the average");
    }
  }
  check(result.origin_initialized_now && result.origin_average_count == 10,
        "the tenth usable Fixed record must initialize despite alternating rejects");
}

void testAverageOriginCandidateGapReset()
{
  GnssAdapter adapter(averageConfig());
  const std::uint64_t base_ns = 300000000000ULL;
  for (int index = 0; index < 9; ++index)
  {
    adapter.process(
        stampedFixed(base_ns + static_cast<std::uint64_t>(index) * 100000000ULL),
        ros::Time(999));
  }

  const GnssAdapterResult after_gap =
      adapter.process(stampedFixed(base_ns + 3000000000ULL), ros::Time(999));
  check(after_gap.origin_average_reset &&
        after_gap.origin_average_reset_reason == "VALID_FIXED_GAP" &&
        after_gap.origin_average_old_count == 9 &&
        after_gap.origin_average_accepted &&
        after_gap.origin_average_count == 1 &&
        !after_gap.status.origin_initialized,
        "a long candidate-to-candidate gap must restart the average at one");
}

void testAverageOriginTimeRegressionReset()
{
  GnssAdapter adapter(averageConfig());
  const std::uint64_t base_ns = 400000000000ULL;
  for (int index = 0; index < 3; ++index)
  {
    adapter.process(
        stampedFixed(base_ns + static_cast<std::uint64_t>(index) * 100000000ULL),
        ros::Time(999));
  }

  const GnssAdapterResult regressed =
      adapter.process(stampedFixed(390000000000ULL), ros::Time(999));
  check(regressed.origin_average_reset &&
        regressed.origin_average_reset_reason == "TIME_REGRESSION" &&
        regressed.origin_average_old_count == 3 &&
        regressed.origin_average_accepted &&
        regressed.origin_average_count == 1 &&
        !regressed.status.origin_initialized,
        "time regression must reset collected origin candidates");

  const GnssAdapterResult restarted =
      adapter.process(stampedFixed(390100000000ULL), ros::Time(999));
  check(restarted.origin_average_accepted &&
        restarted.origin_average_count == 2 &&
        !restarted.status.origin_initialized,
        "the post-regression stream must continue from the new epoch");
}

void testAverageOriginDuplicateEpochSkipped()
{
  GnssAdapter adapter(averageConfig(2));
  const std::uint64_t first_ns = 500000000000ULL;
  adapter.process(stampedFixed(first_ns), ros::Time(999));

  const GnssAdapterResult duplicate =
      adapter.process(stampedFixed(first_ns), ros::Time(999));
  check(duplicate.status.reject_reason == "DUPLICATE_GNSS_TIME" &&
        duplicate.origin_average_skipped &&
        duplicate.origin_average_count == 1 &&
        !duplicate.status.origin_initialized,
        "a duplicate GNSS epoch must not count twice");

  const GnssAdapterResult second =
      adapter.process(stampedFixed(first_ns + 100000000ULL), ros::Time(999));
  check(second.origin_initialized_now && second.origin_average_count == 2,
        "one later unique epoch must complete the two-sample average");
}

void testAverageOriginSourceEpochReset()
{
  GnssAdapter adapter(averageConfig(2));
  auto first = stampedFixed(600000000000ULL);
  adapter.process(first, ros::Time(999));

  auto new_source_epoch = stampedFixed(100000000000ULL);
  new_source_epoch.session_id = 2;
  const GnssAdapterResult reset =
      adapter.process(new_source_epoch, ros::Time(999));
  check(reset.origin_average_reset &&
        reset.origin_average_reset_reason == "SOURCE_EPOCH_CHANGED" &&
        reset.origin_average_old_count == 1 &&
        reset.origin_average_accepted &&
        reset.origin_average_count == 1 &&
        !reset.status.origin_initialized,
        "session/writer epoch change must restart the average with the new sample");

  auto second_new_source_sample = stampedFixed(100100000000ULL);
  second_new_source_sample.session_id = 2;
  const GnssAdapterResult completed =
      adapter.process(second_new_source_sample, ros::Time(999));
  check(completed.origin_initialized_now && completed.origin_average_count == 2,
        "the new source epoch must be able to complete a fresh average");
}

void testFirstFixedModeUnchanged()
{
  GnssAdapterConfig config = manualConfig();
  config.origin_mode = "first_fixed";
  config.fixed_confirm_count = 1;
  GnssAdapter adapter(config);
  const GnssAdapterResult result = adapter.process(fixedMessage(), ros::Time(1));
  check(result.origin_initialized_now && result.status.origin_initialized &&
        result.status.accepted && !result.origin_average_accepted &&
        !result.origin_average_reset,
        "first_fixed must retain its single confirmed-Fixed initialization behavior");
}

void testStatusOnlyModeDoesNotCreateLocalEnu()
{
  GnssAdapterConfig config = manualConfig();
  config.origin_mode = "first_fixed";
  config.fixed_confirm_count = 1;
  config.publish_local_enu_odometry = false;
  GnssAdapter adapter(config);
  const GnssAdapterResult result = adapter.process(fixedMessage(), ros::Time(1));
  check(result.status.accepted && result.status.origin_initialized &&
        !result.publish_odometry && !result.origin_initialized_now,
        "status-only mode must preserve quality acceptance without creating local ENU odometry");
}
} // namespace

int main()
{
  try
  {
    testStampedLocalTimeAndMetadataGates();
    testStampedLocalLossAndRecovery();
    testFixedStateIgnoresUnavailableAccuracyRecord();
    testTimeStateEnuAndCovariance();
    testInvalidInputs();
    testTimeOrderingAndMissingOrigin();
    testLossAndRecovery();
    testAverageOriginContinuousFixed();
    testAverageOriginSkipsRejectedRecords();
    testAverageOriginCandidateGapReset();
    testAverageOriginTimeRegressionReset();
    testAverageOriginDuplicateEpochSkipped();
    testAverageOriginSourceEpochReset();
    testFirstFixedModeUnchanged();
    testStatusOnlyModeDoesNotCreateLocalEnu();
  }
  catch (const std::exception &error)
  {
    std::cerr << "gnss_adapter_self_test: FAIL: " << error.what() << std::endl;
    return 1;
  }
  std::cout << "gnss_adapter_self_test: PASS" << std::endl;
  return 0;
}
