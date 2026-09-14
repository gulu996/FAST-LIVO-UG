#include "rtk_fixed_lag_backend.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fast_livo_backend {

struct RtkFixedLagBackendSelfTestAccess {
  struct GnssWeightResult {
    double fixed_quality_scale = 0.0;
    double fixed_satellite_scale = 0.0;
    double float_quality_scale = 0.0;
    double float_satellite_scale = 0.0;
    double recovery_initial_scale = 0.0;
    double recovery_rejected_later_scale = 0.0;
    double recovery_float_hold_scale = 0.0;
    double recovery_before_fixed_stable_scale = 0.0;
    double recovery_midpoint_scale = 0.0;
    double recovery_finished_scale = 0.0;
    bool recovery_started_before_accept = false;
    bool recovery_fade_started_before_fixed_stable = false;
    bool recovery_float_rate_limited_before_period = false;
    bool recovery_float_rate_allowed_at_period = false;
    bool recovery_float_rate_applied_to_fixed = false;
    bool recovery_float_rate_applied_after_fade = false;
  };

  struct AuthoritativeCovarianceResult {
    gtsam::Vector3 legacy_sigmas = gtsam::Vector3::Zero();
    gtsam::Vector3 fixed_like_sigmas = gtsam::Vector3::Zero();
    gtsam::Vector3 float_like_sigmas = gtsam::Vector3::Zero();
    gtsam::Vector3 clamped_sigmas = gtsam::Vector3::Zero();
    gtsam::Vector3 recovery_sigmas = gtsam::Vector3::Zero();
    double authoritative_quality_scale = 0.0;
    double authoritative_satellite_scale = 0.0;
    double recovery_scale = 0.0;
  };

  struct BoundaryResult {
    bool alignment_ready = false;
    std::set<std::int64_t> factor_stamps;
    std::size_t alignment_pending = 0;
    std::size_t graph_pending = 0;
    std::size_t alignment_pair_count = 0;
    std::size_t graph_gnss_factor_count = 0;
    std::uint64_t moved_to_graph_pending = 0;
    std::uint64_t transition_rejected = 0;
    std::uint64_t transition_waiting = 0;
    std::uint64_t terminal_rejected = 0;
    std::uint64_t silent_drop_count = 0;
    std::uint64_t duplicate_factor_count = 0;
    std::int64_t alignment_cutoff_stamp_ns = -1;
    std::int64_t conservation_delta = 0;
  };

  struct EndOfStreamResult {
    std::size_t graph_pending = 0;
    std::size_t processable_graph_pending = 0;
    std::uint64_t live_rejected = 0;
    std::uint64_t rejected = 0;
    std::uint64_t time_rejected = 0;
    std::uint64_t factors = 0;
    std::int64_t last_processed_stamp_ns = -1;
    std::int64_t conservation_delta = 0;
    std::int64_t processable_conservation_delta = 0;
    std::uint64_t silent_drop_count = 0;
    std::uint64_t processable_silent_drop_count = 0;
    std::string last_reject_reason;
  };

  static bool rejectsNonFiniteLeverArm() {
    RtkFixedLagBackend backend;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.antenna_lever_arm_body_m =
        gtsam::Point3(std::nan(""), 0.0, 0.0);
    try {
      backend.validateParameters();
    } catch (const std::invalid_argument &) {
      return true;
    }
    return false;
  }

  static bool runGravityConsistencyCheck() {
    const auto check = [](bool condition, const char *message) {
      if (!condition) throw std::runtime_error(message);
    };
    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.livo_gravity_consistency_en = true;
    bool rejected = false;
    try { backend.validateParameters(); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "gravity consistency accepted an undeclared raw frame");
    backend.config_.raw_odom_gravity_aligned = true;
    backend.validateParameters();
    backend.config_.prior_roll_pitch_sigma_rad = std::nan("");
    rejected = false;
    try { backend.validateParameters(); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "gravity consistency accepted a non-finite sigma");
    backend.config_.prior_roll_pitch_sigma_rad = 0.05;

    const gtsam::Key key = gtsam::Symbol('x', 0);
    const gtsam::Pose3 raw(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.5),
                           gtsam::Point3(1.0, 2.0, 3.0));
    gtsam::NonlinearFactorGraph factors;
    rejected = false;
    try {
      backend.appendGravityConsistencyFactor(
          key, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(std::nan(""), 0, 0)),
          factors);
    } catch (const std::invalid_argument &) { rejected = true; }
    check(rejected && factors.empty(), "invalid raw pose inserted a gravity factor");
    backend.appendGravityConsistencyFactor(key, raw, factors);
    const auto factor = boost::dynamic_pointer_cast<gtsam::Pose3AttitudeFactor>(
        factors.at(0));
    check(bool(factor), "gravity consistency did not use the native attitude factor");
    const gtsam::Pose3 yaw_and_translation(
        gtsam::Rot3::Rz(1.1).compose(raw.rotation()),
        gtsam::Point3(-10.0, 20.0, 9.0));
    gtsam::Matrix analytical;
    check(factor->evaluateError(yaw_and_translation, analytical).norm() < 1e-12,
          "gravity factor constrained yaw or actual height");
    const auto evaluate = [&](const gtsam::Pose3 &pose) -> gtsam::Vector2 {
      return factor->evaluateError(pose);
    };
    const gtsam::Matrix numerical =
        gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
            evaluate, yaw_and_translation, 1e-6);
    check((analytical - numerical).norm() < 1e-6 &&
              analytical.rightCols(3).norm() < 1e-12,
          "gravity factor Jacobian or zero translation sensitivity is wrong");
    const gtsam::Pose3 tilted(
        gtsam::Rot3::Rx(0.15).compose(yaw_and_translation.rotation()),
        yaw_and_translation.translation());
    check(factor->evaluateError(tilted).norm() > 0.1,
          "gravity factor did not observe roll/pitch inconsistency");

    // Real graph insertion paths, with changing body tilt and non-flat height.
    for (const bool enabled : {false, true}) {
      RtkFixedLagBackend graph_backend;
      graph_backend.config_.enable = false;
      graph_backend.config_.save_results = false;
      graph_backend.config_.save_text_log = false;
      graph_backend.config_.livo_gravity_consistency_en = enabled;
      graph_backend.config_.raw_odom_gravity_aligned = true;
      graph_backend.initial_map_to_odom_ = gtsam::Pose3(
          gtsam::Rot3::Rz(0.4), gtsam::Point3(10.0, -3.0, 5.0));
      check(graph_backend.initializeGraph({ros::Time(1, 0), raw}),
            "actual initial graph insertion failed");
      const std::vector<std::string> triggers{"motion", "gnss", "uwb"};
      for (std::size_t i = 0; i < triggers.size(); ++i) {
        const gtsam::Pose3 next(
            gtsam::Rot3::RzRyRx(0.2 + 0.01 * i, -0.1, 0.6 + 0.1 * i),
            gtsam::Point3(2.0 + i, 2.0, 3.0 + std::sin(i + 1.0)));
        check(graph_backend.createGraphNode(
                  {ros::Time(static_cast<uint32_t>(i + 2), 0), next},
                  triggers[i], nullptr),
              "actual shared motion/GNSS/UWB node insertion failed");
        const auto estimated = graph_backend.keyframes_.back().optimized_pose;
        check(estimated.equals(graph_backend.initial_map_to_odom_.compose(next), 1e-8),
              "gravity consistency flattened height or changed an exact raw motion");
      }
      check(graph_backend.gravity_factor_count_ == (enabled ? 4u : 0u) &&
                graph_backend.active_gravity_factors_ == (enabled ? 4u : 0u),
            "gravity factor accounting differs from actual node insertions");
    }

    // A soft inconsistent attitude prior must not overwhelm observed vertical.
    gtsam::NonlinearFactorGraph graph;
    const gtsam::Pose3 identity_raw(gtsam::Rot3(), gtsam::Point3(0, 0, 3));
    backend.appendGravityConsistencyFactor(key, identity_raw, graph);
    const gtsam::Pose3 initial(gtsam::Rot3::RzRyRx(0.2, 0.1, 0.5),
                               gtsam::Point3(4, -2, 3));
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        key, initial, gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
    gtsam::Values values;
    values.insert(key, initial);
    const auto result = gtsam::LevenbergMarquardtOptimizer(graph, values)
                            .optimize().at<gtsam::Pose3>(key);
    check(std::acos(result.rotation().matrix()(2, 2)) < 0.01 &&
              (result.translation() - initial.translation()).norm() < 1e-8,
          "soft gravity consistency failed to suppress tilt or constrained height");
    return true;
  }

  static void initializeTestGraph(
      RtkFixedLagBackend &backend,
      const RtkFixedLagBackend::RawOdomSample &sample,
      double prior_sigma = 0.1) {
    gtsam::ISAM2Params parameters;
    parameters.findUnusedFactorSlots = true;
    backend.smoother_.reset(new gtsam::IncrementalFixedLagSmoother(
        backend.config_.lag_seconds, parameters));

    const gtsam::Key key = gtsam::Symbol('x', 0);
    const gtsam::Pose3 map_pose =
        backend.initial_map_to_odom_.compose(sample.pose);
    gtsam::Vector6 sigmas;
    sigmas.setConstant(prior_sigma);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    gtsam::NonlinearFactorGraph factors;
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, map_pose, noise));
    gtsam::Values values;
    values.insert(key, map_pose);
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
    timestamps[key] = sample.stamp.toSec();
    if (!backend.updateSmoother(factors, values, timestamps))
      throw std::runtime_error("test graph initialization failed");

    backend.keyframes_.push_back(RtkFixedLagBackend::Keyframe{
        0, key, sample.stamp, sample.pose, map_pose, true});
    backend.next_keyframe_id_ = 1;
    backend.total_nodes_created_ = 1;
    backend.initialized_ = true;
  }

  static bool runGnssReacquisitionCheck() {
    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.gnss_recovery_gap_threshold_s = 5.0;
    backend.config_.gnss_recovery_fixed_confirm_factors = 3;
    backend.config_.gnss_recovery_fixed_max_gap_s = 0.5;
    backend.config_.gnss_reacquisition_consistency_m = 0.5;
    backend.last_added_gnss_factor_stamp_ns_ = 1'000'000'000LL;
    RtkFixedLagBackend::GnssMeasurement measurement;
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    const auto candidate = [&](double stamp, double offset, double nis = 1.0) {
      measurement.stamp.fromSec(stamp);
      return backend.confirmGnssReacquisition(
          measurement, gtsam::Vector3(offset, 0.0, 0.0), nis);
    };
    const auto check = [](bool condition, const char *message) {
      if (!condition) throw std::runtime_error(message);
    };
    check(!candidate(10.0, 6.0) && !candidate(10.2, 6.0) &&
              !candidate(10.4, 6.0),
          "disabled reacquisition changed the old absolute gate");
    backend.config_.gnss_reacquisition_en = true;
    check(!candidate(4.0, 6.0), "reacquisition started without an outage");
    check(!candidate(10.0, 6.0) && !candidate(10.2, 6.1) &&
              candidate(10.4, 6.2),
          "consistent Fixed return did not confirm");
    check(!candidate(10.6, 20.0) && !candidate(10.8, 6.0) &&
              !candidate(11.0, 6.0) && candidate(11.2, 6.0),
          "isolated Fixed spike was accepted or failed to reset candidates");
    check(!candidate(11.4, 6.0, 20.0) && !candidate(11.6, 6.0) &&
              !candidate(11.8, 6.0) && candidate(12.0, 6.0),
          "NIS failure bypassed the gate or failed to reset candidates");
    check(!candidate(13.0, 6.0) && !candidate(13.2, 6.0) &&
              candidate(13.4, 6.0),
          "a long candidate gap failed to reset confirmation");
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    check(!candidate(13.6, 6.0), "Float triggered Fixed reacquisition");
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    check(!candidate(13.8, 6.0) && !candidate(14.0, 6.0) &&
              candidate(14.2, 6.0),
          "Float did not break the Fixed candidate streak");
    check(!candidate(14.2, 6.0) && !candidate(14.4, 6.0),
          "duplicate candidate time did not reset confirmation");

    // Exercise the real factor insertion, not only the candidate helper.
    backend.gnss_reacquisition_candidate_count_ = 0;
    initializeTestGraph(backend,
                        {ros::Time(20, 0), gtsam::Pose3()}, 10.0);
    measurement.position = gtsam::Point3(6.0, 0.0, 0.0);
    measurement.sigmas = gtsam::Vector3::Constant(0.1);
    for (int i = 0; i < 3; ++i) {
      measurement.stamp.fromSec(20.0 + 0.2 * i);
      const bool added = backend.addGnssFactor(measurement,
                                               backend.keyframes_.front());
      check(added == (i == 2),
            "the real graph accepted an unconfirmed return or rejected a confirmed one");
    }
    const auto pose = backend.smoother_->calculateEstimate<gtsam::Pose3>(
        backend.keyframes_.front().key);
    check(backend.gnss_factor_count_ == 1 &&
              backend.gnss_reacquisition_candidate_count_ == 0 &&
              (pose.translation() - measurement.position).norm() < 0.1,
          "confirmed reacquisition did not correct the graph or clear candidates");
    return true;
  }

  static BoundaryResult runAlignmentBoundaryOrder(bool gnss_first) {
    constexpr std::int64_t kSecondNs = 1000000000LL;
    const std::int64_t first_stamp_ns = 10 * kSecondNs;
    const std::int64_t cutoff_stamp_ns =
        first_stamp_ns + 200000000LL;
    const std::int64_t boundary_stamp_ns =
        first_stamp_ns + 300000000LL;
    const auto stamp = [](std::int64_t nanoseconds) {
      ros::Time value;
      value.fromNSec(static_cast<std::uint64_t>(nanoseconds));
      return value;
    };

    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.alignment_min_pairs = 3;
    backend.config_.alignment_min_baseline_m = 2.0;
    backend.config_.alignment_max_rmse_m = 0.01;
    for (int index = 0; index < 3; ++index) {
      const std::int64_t pair_stamp_ns =
          first_stamp_ns + index * 100000000LL;
      const gtsam::Point3 position(static_cast<double>(index), 0.0, 0.0);
      backend.alignment_pairs_.push_back(
          AlignmentPair{position, position, pair_stamp_ns});
      backend.raw_odom_buffer_.push_back(RtkFixedLagBackend::RawOdomSample{
          stamp(pair_stamp_ns), gtsam::Pose3(gtsam::Rot3(), position)});
    }
    backend.alignment_gnss_used_ = backend.alignment_pairs_.size();
    backend.gnss_received_ = backend.alignment_pairs_.size();

    gtsam::Vector3 sigmas;
    sigmas.setConstant(0.1);
    const RtkFixedLagBackend::GnssMeasurement boundary{
        stamp(boundary_stamp_ns), gtsam::Point3(3.0, 0.0, 0.0), sigmas};
    if (gnss_first) {
      backend.pending_alignment_gnss_.push_back(boundary);
      ++backend.gnss_received_;
    } else {
      backend.raw_odom_buffer_.push_back(RtkFixedLagBackend::RawOdomSample{
          stamp(boundary_stamp_ns),
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0))});
    }

    BoundaryResult result;
    result.alignment_ready = backend.tryFinishAlignment();
    if (!result.alignment_ready)
      throw std::runtime_error("test alignment did not finish");
    if (gnss_first) {
      backend.raw_odom_buffer_.push_back(RtkFixedLagBackend::RawOdomSample{
          stamp(boundary_stamp_ns),
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0))});
    } else {
      backend.insertPendingGnss(boundary);
      ++backend.gnss_received_;
    }

    gtsam::Pose3 boundary_pose;
    double interval_s = 0.0;
    std::string interpolation_reason;
    if (!backend.interpolateRawPose(boundary.stamp, &boundary_pose, &interval_s,
                                    &interpolation_reason)) {
      throw std::runtime_error("boundary raw interpolation failed: " +
                               interpolation_reason);
    }
    initializeTestGraph(
        backend, RtkFixedLagBackend::RawOdomSample{boundary.stamp,
                                                   boundary_pose});
    if (backend.pending_factor_gnss_.size() != 1)
      throw std::runtime_error("boundary GNSS was not uniquely graph-ready");

    const std::uint64_t rejected_before = backend.gnss_rejected_;
    const bool factor_added = backend.addGnssFactor(
        backend.pending_factor_gnss_.front(), backend.keyframes_.front());
    if (factor_added) {
      result.factor_stamps.insert(backend.last_added_gnss_factor_stamp_ns_);
    } else {
      result.terminal_rejected = backend.gnss_rejected_ - rejected_before;
    }
    backend.last_processed_gnss_stamp_ns_ = boundary_stamp_ns;
    backend.pending_factor_gnss_.pop_front();
    for (const auto &factor : backend.smoother_->getFactors()) {
      if (factor && boost::dynamic_pointer_cast<GnssPositionArmFactor>(factor))
        ++result.graph_gnss_factor_count;
    }

    result.alignment_pending = backend.pending_alignment_gnss_.size();
    result.graph_pending = backend.pending_factor_gnss_.size();
    result.alignment_pair_count = backend.alignment_.pair_count;
    result.moved_to_graph_pending =
        backend.alignment_transition_to_graph_pending_;
    result.transition_rejected = backend.alignment_transition_rejected_;
    result.transition_waiting = backend.alignment_transition_waiting_;
    result.silent_drop_count = backend.gnssSilentDropCount();
    result.duplicate_factor_count = backend.gnss_duplicate_factor_count_;
    result.alignment_cutoff_stamp_ns =
        backend.alignment_last_used_gnss_stamp_ns_;
    result.conservation_delta = backend.gnssConservationDelta();
    return result;
  }

  static EndOfStreamResult runEndOfStreamGnssCheck() {
    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.alignment_.valid = true;
    gtsam::Vector3 sigmas;
    sigmas.setConstant(0.1);
    backend.raw_odom_buffer_.push_back(RtkFixedLagBackend::RawOdomSample{
        ros::Time(10, 252913237), gtsam::Pose3()});
    backend.pending_factor_gnss_.push_back(
        RtkFixedLagBackend::GnssMeasurement{
            ros::Time(10, 300000000), gtsam::Point3(), sigmas});
    backend.gnss_received_ = 1;
    backend.processPendingGnss();
    const std::uint64_t live_rejected = backend.gnss_rejected_;
    backend.finalizePendingGraphGnss();

    RtkFixedLagBackend processable;
    processable.config_.enable = false;
    processable.config_.save_results = false;
    processable.config_.save_text_log = false;
    processable.raw_odom_buffer_.push_back(RtkFixedLagBackend::RawOdomSample{
        ros::Time(10, 400000000), gtsam::Pose3()});
    processable.pending_factor_gnss_.push_back(
        RtkFixedLagBackend::GnssMeasurement{
            ros::Time(10, 300000000), gtsam::Point3(), sigmas});
    processable.gnss_received_ = 1;
    processable.finalizePendingGraphGnss();
    return EndOfStreamResult{
        backend.pending_factor_gnss_.size(),
        processable.pending_factor_gnss_.size(), live_rejected,
        backend.gnss_rejected_, backend.gnss_time_rejected_,
        backend.gnss_factor_count_,
        backend.last_processed_gnss_stamp_ns_, backend.gnssConservationDelta(),
        processable.gnssConservationDelta(), backend.gnssSilentDropCount(),
        processable.gnssSilentDropCount(),
        backend.last_reject_reason_};
  }

  static std::pair<bool, bool> runFilteredFixedAlignmentResetCheck() {
    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.alignment_pairs_.push_back(
        AlignmentPair{gtsam::Point3(), gtsam::Point3(), 1000000000LL});
    gtsam::Vector3 sigmas;
    sigmas.setConstant(0.1);
    backend.pending_alignment_gnss_.push_back(
        RtkFixedLagBackend::GnssMeasurement{
            ros::Time(1, 0), gtsam::Point3(), sigmas});

    fast_livo::GnssStatusPtr filtered_fixed(new fast_livo::GnssStatus());
    filtered_fixed->header.stamp = ros::Time(1, 100000000);
    filtered_fixed->filtered_quality = fast_livo::GnssStatus::RTK_FIXED;
    filtered_fixed->accepted = false;
    filtered_fixed->reject_reason = "H_ACC_TOO_LARGE";
    backend.gnssStatusCallback(filtered_fixed);
    const bool quality_gate_preserved =
        backend.alignment_pairs_.size() == 1 &&
        backend.pending_alignment_gnss_.size() == 1;

    fast_livo::GnssStatusPtr fixed_lost(new fast_livo::GnssStatus());
    fixed_lost->header.stamp = ros::Time(1, 200000000);
    fixed_lost->filtered_quality = fast_livo::GnssStatus::INVALID;
    fixed_lost->accepted = false;
    fixed_lost->reject_reason = "INVALID_FIX";
    backend.gnssStatusCallback(fixed_lost);
    const bool true_loss_reset = backend.alignment_pairs_.empty() &&
                                 backend.pending_alignment_gnss_.empty();
    return {quality_gate_preserved, true_loss_reset};
  }

  static bool runGnssQualityPolicyCheck() {
    RtkFixedLagBackend backend;
    backend.config_.accept_rtk_fixed = true;
    backend.config_.accept_rtk_float = true;
    backend.config_.accept_differential = false;
    backend.config_.accept_single = false;
    return backend.gnssQualityAccepted(fast_livo::GnssStatus::RTK_FIXED) &&
           backend.gnssQualityAccepted(fast_livo::GnssStatus::RTK_FLOAT) &&
           !backend.gnssQualityAccepted(fast_livo::GnssStatus::DIFFERENTIAL) &&
           !backend.gnssQualityAccepted(fast_livo::GnssStatus::SINGLE) &&
           !backend.gnssQualityAccepted(fast_livo::GnssStatus::INVALID);
  }

  static GnssWeightResult runGnssWeightCheck() {
    RtkFixedLagBackend backend;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.rtk_float_sigma_scale = 3.0;
    backend.config_.rtk_float_reference_satellites = 10;
    backend.config_.rtk_float_satellite_sigma_scale_max = 1.5;
    backend.config_.gnss_recovery_gap_threshold_s = 5.0;
    backend.config_.gnss_recovery_ramp_duration_s = 10.0;
    backend.config_.gnss_recovery_initial_sigma_scale = 4.0;
    backend.config_.gnss_recovery_fixed_confirm_factors = 3;
    backend.config_.gnss_recovery_fixed_max_gap_s = 0.5;
    backend.config_.gnss_recovery_float_factor_rate_hz = 1.0;

    fast_livo::GnssStatus status;
    status.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    status.num_sv = 7;
    GnssWeightResult result;
    result.fixed_quality_scale = backend.gnssQualitySigmaScale(
        status, &result.fixed_satellite_scale);

    status.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    result.float_quality_scale = backend.gnssQualitySigmaScale(
        status, &result.float_satellite_scale);

    backend.last_added_gnss_factor_stamp_ns_ = 1'000'000'000LL;
    result.recovery_initial_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(11, 0));
    result.recovery_rejected_later_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(16, 0));
    result.recovery_started_before_accept =
        backend.gnss_recovery_start_stamp_ns_ >= 0;
    // Simulate the first successful Float factor insertion at t=16 s.
    RtkFixedLagBackend::GnssMeasurement measurement;
    backend.gnss_recovery_start_stamp_ns_ = 16'000'000'000LL;
    backend.last_added_gnss_factor_stamp_ns_ = 16'000'000'000LL;
    measurement.stamp = ros::Time(16, 0);
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    backend.commitGnssRecoveryAcceptedFactor(measurement);
    backend.last_added_recovery_float_factor_stamp_ns_ = 16'000'000'000LL;
    measurement.stamp = ros::Time(16, 900'000'000);
    result.recovery_float_rate_limited_before_period =
        backend.gnssRecoveryFloatFactorRateLimited(measurement);
    measurement.stamp = ros::Time(17, 0);
    result.recovery_float_rate_allowed_at_period =
        !backend.gnssRecoveryFloatFactorRateLimited(measurement);
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    measurement.stamp = ros::Time(16, 100'000'000);
    result.recovery_float_rate_applied_to_fixed =
        backend.gnssRecoveryFloatFactorRateLimited(measurement);
    backend.gnss_recovery_fade_start_stamp_ns_ = 16'000'000'000LL;
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    result.recovery_float_rate_applied_after_fade =
        backend.gnssRecoveryFloatFactorRateLimited(measurement);
    backend.gnss_recovery_fade_start_stamp_ns_ = -1;
    backend.last_added_gnss_factor_stamp_ns_ = 26'000'000'000LL;
    result.recovery_float_hold_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(26, 0));

    // A Float between Fixed factors breaks continuity, so three new accepted
    // Fixed factors are required before the time-based fade can start.
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    measurement.stamp = ros::Time(26, 200'000'000);
    backend.commitGnssRecoveryAcceptedFactor(measurement);
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    measurement.stamp = ros::Time(26, 400'000'000);
    backend.commitGnssRecoveryAcceptedFactor(measurement);
    for (int i = 0; i < 2; ++i) {
      measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
      measurement.stamp = ros::Time(26, 600'000'000 + i * 200'000'000);
      backend.commitGnssRecoveryAcceptedFactor(measurement);
    }
    result.recovery_fade_started_before_fixed_stable =
        backend.gnss_recovery_fade_start_stamp_ns_ >= 0;
    backend.last_added_gnss_factor_stamp_ns_ = 27'000'000'000LL;
    result.recovery_before_fixed_stable_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(27, 0));
    measurement.raw_quality = fast_livo::GnssStatus::RTK_FIXED;
    measurement.stamp = ros::Time(27, 0);
    backend.commitGnssRecoveryAcceptedFactor(measurement);

    backend.last_added_gnss_factor_stamp_ns_ = 32'000'000'000LL;
    result.recovery_midpoint_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(32, 0));
    backend.last_added_gnss_factor_stamp_ns_ = 37'000'000'000LL;
    result.recovery_finished_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(37, 0));
    return result;
  }

  static AuthoritativeCovarianceResult runAuthoritativeCovarianceCheck() {
    RtkFixedLagBackend backend;
    backend.config_.min_gnss_sigma_xy_m = 0.03;
    backend.config_.min_gnss_sigma_z_m = 0.05;
    backend.config_.max_gnss_sigma_xy_m = 2.0;
    backend.config_.max_gnss_sigma_z_m = 3.0;
    backend.config_.rtk_float_sigma_scale = 3.0;
    backend.config_.rtk_float_reference_satellites = 10;
    backend.config_.rtk_float_satellite_sigma_scale_max = 1.5;
    backend.config_.gnss_recovery_gap_threshold_s = 5.0;
    backend.config_.gnss_recovery_ramp_duration_s = 10.0;
    backend.config_.gnss_recovery_initial_sigma_scale = 4.0;

    fast_livo::GnssStatus status;
    status.raw_quality = fast_livo::GnssStatus::RTK_FLOAT;
    status.num_sv = 7;
    double quality_scale = 0.0;
    double satellite_scale = 0.0;
    AuthoritativeCovarianceResult result;
    result.legacy_sigmas = backend.gnssBaseSigmas(
        gtsam::Vector3(0.25, 0.25, 0.5), status, &quality_scale,
        &satellite_scale);

    status.covariance_authoritative = true;
    result.fixed_like_sigmas = backend.gnssBaseSigmas(
        gtsam::Vector3(0.25, 0.25, 0.5), status,
        &result.authoritative_quality_scale,
        &result.authoritative_satellite_scale);
    result.float_like_sigmas = backend.gnssBaseSigmas(
        gtsam::Vector3(1.5, 1.5, 2.5), status, &quality_scale,
        &satellite_scale);
    result.clamped_sigmas = backend.gnssBaseSigmas(
        gtsam::Vector3(5.0, 4.0, 8.0), status, &quality_scale,
        &satellite_scale);

    backend.last_added_gnss_factor_stamp_ns_ = 1'000'000'000LL;
    result.recovery_scale =
        backend.updateGnssRecoverySigmaScale(ros::Time(11, 0));
    result.recovery_sigmas = backend.clampGnssSigmas(
        result.fixed_like_sigmas, result.recovery_scale);
    return result;
  }

  static gtsam::Pose3 runResultReferenceCheck() {
    const gtsam::Pose3 body_pose(gtsam::Rot3::Rz(M_PI_2),
                                 gtsam::Point3(1.0, 2.0, 3.0));
    return RtkFixedLagBackend::resultReferencePose(
        body_pose, gtsam::Point3(0.180, 0.010, -1.192));
  }

  static bool runStandardFixedLagCheck() {
    RtkFixedLagBackend backend;
    backend.config_.enable = false;
    backend.config_.save_results = false;
    backend.config_.save_text_log = false;
    backend.config_.lag_seconds = 2.0;
    backend.config_.uwb_factor_backend_en = true;
    initializeTestGraph(
        backend, RtkFixedLagBackend::RawOdomSample{ros::Time(1, 0),
                                                   gtsam::Pose3()});
    gtsam::Vector6 sigmas;
    sigmas.setConstant(0.1);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    for (int index = 1; index <= 10; ++index) {
      const gtsam::Key previous = gtsam::Symbol('x', index - 1);
      const gtsam::Key current = gtsam::Symbol('x', index);
      const gtsam::Pose3 pose(
          gtsam::Rot3(), gtsam::Point3(static_cast<double>(index), 0.0, 0.0));
      gtsam::NonlinearFactorGraph factors;
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          previous, current,
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)), noise));
      gtsam::Values values;
      values.insert(current, pose);
      gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
      timestamps[current] = index + 1;
      if (!backend.updateSmoother(factors, values, timestamps)) return false;
    }

    const auto &timestamps = backend.smoother_->timestamps();
    const gtsam::Values estimate = backend.smoother_->calculateEstimate();
    return timestamps.count(gtsam::Symbol('x', 0)) == 0 &&
           timestamps.size() <= 3 &&
           estimate.exists(gtsam::Symbol('x', 10)) &&
           (estimate.at<gtsam::Pose3>(gtsam::Symbol('x', 10)).translation() -
            gtsam::Point3(10.0, 0.0, 0.0)).norm() < 1e-8;
  }
};

}  // namespace fast_livo_backend

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

void testAlignment() {
  constexpr double yaw = 0.4;
  const gtsam::Rot3 rotation = gtsam::Rot3::Rz(yaw);
  const gtsam::Point3 translation(12.0, -3.0, 1.5);
  std::vector<fast_livo_backend::AlignmentPair> pairs;
  for (int i = 0; i < 30; ++i) {
    const gtsam::Point3 odom(0.5 * i, std::sin(0.2 * i), 0.05 * i);
    pairs.push_back({odom, rotation.rotate(odom) + translation});
  }
  const auto result =
      fast_livo_backend::RtkFixedLagBackend::estimateSe2Alignment(pairs);
  require(result.valid, "alignment must be observable");
  require(std::abs(result.yaw_rad - yaw) < 1e-10,
          "alignment yaw is wrong");
  require((result.translation - translation).norm() < 1e-10,
          "alignment translation is wrong");
  require(result.rmse_m < 1e-10, "alignment RMSE is wrong");
  require(result.baseline_m > 10.0, "alignment baseline is wrong");
}

void testKeyframeSelection() {
  fast_livo_backend::BackendConfig config;
  const gtsam::Pose3 origin;
  require(!fast_livo_backend::RtkFixedLagBackend::shouldCreateKeyframe(
              origin, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0, 0)),
              0.1, config),
          "small motion must not create a keyframe");
  require(fast_livo_backend::RtkFixedLagBackend::shouldCreateKeyframe(
              origin, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.8, 0, 0)),
              0.1, config),
          "translation threshold must create a keyframe");
  require(fast_livo_backend::RtkFixedLagBackend::shouldCreateKeyframe(
              origin,
              gtsam::Pose3(
                  gtsam::Rot3::Rz(9.0 / 180.0 * 3.14159265358979323846),
                  gtsam::Point3()),
              0.1, config),
          "rotation threshold must create a keyframe");
  require(fast_livo_backend::RtkFixedLagBackend::shouldCreateKeyframe(
              origin, origin, 1.0, config),
          "time threshold must create a keyframe");
}

void testLeverArmFactor() {
  const gtsam::Pose3 pose(
      gtsam::Rot3::Rz(3.14159265358979323846 / 2.0),
      gtsam::Point3(1.0, 2.0, 3.0));
  const gtsam::Point3 lever_arm(1.0, 0.0, 0.0);
  const gtsam::Point3 antenna(1.0, 3.0, 3.0);
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  fast_livo_backend::GnssPositionArmFactor factor(
      gtsam::Symbol('x', 0), antenna, lever_arm, noise);
  gtsam::Matrix jacobian;
  require(factor.evaluateError(pose, jacobian).norm() < 1e-12,
          "lever arm prediction is wrong");
  require(jacobian.rows() == 3 && jacobian.cols() == 6,
          "lever arm Jacobian dimensions are wrong");
  require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
              rejectsNonFiniteLeverArm(),
          "non-finite antenna lever arm was not rejected");
}

void testUwbRangeFactorJacobian() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);
  const std::vector<gtsam::Pose3> poses{
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.4),
                   gtsam::Point3(1.0, 2.0, 0.5)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.4, 0.3, -0.2),
                   gtsam::Point3(-3.0, 0.7, 2.1)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.8, 0.1, -0.5),
                   gtsam::Point3(8.0, -4.0, 1.2))};
  const std::vector<gtsam::Point3> anchors{
      gtsam::Point3(4.0, -1.0, 0.2), gtsam::Point3(-2.0, 5.0, 1.4),
      gtsam::Point3(0.5, 0.3, -2.0)};
  const std::vector<gtsam::Point3> lever_arms{
      gtsam::Point3(0.15, -0.08, 0.11),
      gtsam::Point3(-0.12, 0.04, 0.20),
      gtsam::Point3(0.05, 0.18, -0.09)};

  for (std::size_t index = 0; index < poses.size(); ++index) {
    const double measured =
        (poses[index].transformFrom(lever_arms[index]) - anchors[index])
            .norm() -
        0.25;
    fast_livo_backend::UwbRangeFactor factor(
        gtsam::Symbol('x', index), anchors[index], measured,
        lever_arms[index], noise);
    gtsam::Matrix analytical;
    const gtsam::Vector1 error = factor.evaluateError(poses[index], analytical);
    const std::function<gtsam::Vector1(const gtsam::Pose3 &)> evaluate =
        [&](const gtsam::Pose3 &pose) {
          return factor.evaluateError(pose);
        };
    const gtsam::Matrix16 numerical =
        gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
            evaluate, poses[index], 1e-6);
    require(std::abs(error(0) - 0.25) < 1e-10,
            "UWB range residual sign is wrong");
    require((analytical - numerical).cwiseAbs().maxCoeff() < 1e-6,
            "UWB range analytical Jacobian disagrees with numerical derivative");
  }
}

void testRawPoseInterpolation() {
  const ros::Time stamp0(10, 0);
  const ros::Time stamp1(10, 100000000);
  const ros::Time target(10, 50000000);
  const gtsam::Pose3 pose0(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Pose3 pose1(
      gtsam::Rot3::Rz(3.14159265358979323846 / 2.0),
      gtsam::Point3(2.0, 4.0, 6.0));
  gtsam::Pose3 interpolated;
  double interval_s = 0.0;
  std::string reason;
  require(fast_livo_backend::RtkFixedLagBackend::interpolatePose(
              stamp0, pose0, stamp1, pose1, target, 0.15, &interpolated,
              &interval_s, &reason),
          "valid raw pose interpolation was rejected");
  require((interpolated.translation() - gtsam::Point3(1.0, 2.0, 3.0))
                  .norm() < 1e-12,
          "raw pose translation interpolation is wrong");
  require(std::abs(gtsam::Rot3::Logmap(interpolated.rotation()).z() -
                   3.14159265358979323846 / 4.0) < 1e-12,
          "raw pose SLERP is wrong");
  require(std::abs(interval_s - 0.1) < 1e-12,
          "raw pose interpolation interval is wrong");
  require(!fast_livo_backend::RtkFixedLagBackend::interpolatePose(
              stamp0, pose0, stamp1, pose1, target, 0.05, &interpolated,
              &interval_s, &reason) &&
              reason == "RAW_ODOM_INTERPOLATION_GAP_TOO_LARGE",
          "oversized interpolation gap was not rejected precisely");
}

void testAlignmentBoundaryTransition() {
  constexpr std::int64_t kSecondNs = 1000000000LL;
  const std::int64_t cutoff_stamp_ns =
      10 * kSecondNs + 200000000LL;
  const std::int64_t boundary_stamp_ns =
      10 * kSecondNs + 300000000LL;
  const auto gnss_first = fast_livo_backend::
      RtkFixedLagBackendSelfTestAccess::runAlignmentBoundaryOrder(true);
  const auto raw_first = fast_livo_backend::
      RtkFixedLagBackendSelfTestAccess::runAlignmentBoundaryOrder(false);

  require(gnss_first.alignment_ready && raw_first.alignment_ready,
          "both callback orders must finish alignment");
  require(gnss_first.alignment_pair_count == 3 &&
              raw_first.alignment_pair_count == 3,
          "alignment must use the same three timestamped pairs");
  require(gnss_first.alignment_cutoff_stamp_ns == cutoff_stamp_ns &&
              raw_first.alignment_cutoff_stamp_ns == cutoff_stamp_ns,
          "alignment cutoff must come from the last actually used pair");
  require(gnss_first.alignment_pending == 0 &&
              raw_first.alignment_pending == 0 &&
              gnss_first.graph_pending == 0 && raw_first.graph_pending == 0,
          "both pending queues must be empty after terminal processing");
  require(gnss_first.factor_stamps == raw_first.factor_stamps &&
              gnss_first.factor_stamps ==
                  std::set<std::int64_t>{boundary_stamp_ns},
          "factor stamp sets must match across callback orders");
  require(gnss_first.graph_gnss_factor_count == 1 &&
              raw_first.graph_gnss_factor_count == 1 &&
              gnss_first.terminal_rejected == 0 &&
              raw_first.terminal_rejected == 0,
          "boundary GNSS must have exactly one factor terminal state");
  require(gnss_first.moved_to_graph_pending == 1 &&
              raw_first.moved_to_graph_pending == 0 &&
              gnss_first.transition_waiting == 1 &&
              raw_first.transition_waiting == 0,
          "alignment transition counters are wrong");
  require(gnss_first.transition_rejected == 0 &&
              raw_first.transition_rejected == 0 &&
              gnss_first.silent_drop_count == 0 &&
              raw_first.silent_drop_count == 0 &&
              gnss_first.conservation_delta == 0 &&
              raw_first.conservation_delta == 0,
          "silent_drop_count must be zero in both callback orders");
  require(gnss_first.duplicate_factor_count == 0 &&
              raw_first.duplicate_factor_count == 0,
          "duplicate_factor_count must be zero");
}

void testEndOfStreamGnssRejection() {
  const auto result = fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
      runEndOfStreamGnssCheck();
  require(result.live_rejected == 0 && result.graph_pending == 0 &&
              result.rejected == 1 && result.time_rejected == 1 &&
              result.factors == 0,
          "future GNSS must wait live, then be rejected at end of stream");
  require(result.processable_graph_pending == 1,
          "end-of-stream handling must not hide a processable graph backlog");
  require(result.last_processed_stamp_ns == 10300000000LL &&
              result.conservation_delta == 0 &&
              result.processable_conservation_delta == 0 &&
              result.silent_drop_count == 0 &&
              result.processable_silent_drop_count == 0 &&
              result.last_reject_reason ==
                  "GNSS_NO_RAW_ODOM_BRACKET_AT_END_OF_STREAM",
          "end-of-stream GNSS rejection lost its stamp, reason, or accounting");
}

void testFilteredFixedDoesNotResetAlignment() {
  const auto result = fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
      runFilteredFixedAlignmentResetCheck();
  require(result.first,
          "a filtered RTK_FIXED record must not reset collected alignment");
  require(result.second,
          "a confirmed RTK fixed loss must reset collected alignment");
}

void testGnssQualityPolicy() {
  require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
              runGnssQualityPolicyCheck(),
          "backend GNSS quality policy did not honor configured classes");
}

void testGnssWeightingAndRecoveryRamp() {
  const auto result = fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
      runGnssWeightCheck();
  require(std::abs(result.fixed_quality_scale - 1.0) < 1e-12 &&
              std::abs(result.fixed_satellite_scale - 1.0) < 1e-12,
          "Fixed covariance must remain unchanged");
  require(std::abs(result.float_quality_scale - 3.0) < 1e-12 &&
              std::abs(result.float_satellite_scale -
                       std::sqrt(10.0 / 7.0)) < 1e-12,
          "Float quality/satellite covariance inflation is wrong");
  require(std::abs(result.recovery_initial_scale - 4.0) < 1e-12 &&
              std::abs(result.recovery_rejected_later_scale - 4.0) < 1e-12 &&
              !result.recovery_started_before_accept &&
              std::abs(result.recovery_float_hold_scale - 4.0) < 1e-12 &&
              std::abs(result.recovery_before_fixed_stable_scale - 4.0) <
                  1e-12 &&
              !result.recovery_fade_started_before_fixed_stable &&
              result.recovery_float_rate_limited_before_period &&
              result.recovery_float_rate_allowed_at_period &&
              !result.recovery_float_rate_applied_to_fixed &&
              !result.recovery_float_rate_applied_after_fade &&
              std::abs(result.recovery_midpoint_scale - 2.5) < 1e-12 &&
              std::abs(result.recovery_finished_scale - 1.0) < 1e-12,
          "GNSS recovery must hold Float weak until stable Fixed factors");
}

void testAuthoritativeGnssCovariance() {
  const auto result = fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
      runAuthoritativeCovarianceCheck();
  const double legacy_scale = 3.0 * std::sqrt(10.0 / 7.0);
  require((result.legacy_sigmas -
           gtsam::Vector3(0.25 * legacy_scale, 0.25 * legacy_scale,
                          0.5 * legacy_scale))
                  .norm() < 1e-12,
          "non-authoritative Pose/legacy covariance behavior changed");
  require((result.fixed_like_sigmas - gtsam::Vector3(0.25, 0.25, 0.5))
                  .norm() < 1e-12 &&
              std::abs(result.authoritative_quality_scale - 1.0) < 1e-12 &&
              std::abs(result.authoritative_satellite_scale - 1.0) < 1e-12,
          "authoritative Fixed-like covariance was inflated again");
  require((result.float_like_sigmas - gtsam::Vector3(1.5, 1.5, 2.5))
                  .norm() < 1e-12,
          "authoritative Float-like covariance was inflated again");
  require((result.clamped_sigmas - gtsam::Vector3(2.0, 2.0, 3.0)).norm() <
              1e-12,
          "authoritative covariance did not retain configured max clamps");
  require(std::abs(result.recovery_scale - 4.0) < 1e-12 &&
              (result.recovery_sigmas - gtsam::Vector3(1.0, 1.0, 2.0))
                      .norm() < 1e-12,
          "authoritative covariance bypassed the existing recovery scale");
}

void testResultReferenceLeverArm() {
  const gtsam::Pose3 result =
      fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
          runResultReferenceCheck();
  require((result.translation() - gtsam::Point3(0.990, 2.180, 1.808)).norm() <
              1e-12 &&
              result.rotation().equals(gtsam::Rot3::Rz(M_PI_2), 1e-12),
          "saved result pose did not rotate the body-frame scoring lever arm");
}

void testTrueFixedLagMarginalization() {
  gtsam::IncrementalFixedLagSmoother smoother(2.0);
  gtsam::Vector6 sigmas;
  sigmas.setConstant(0.1);
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  for (int i = 0; i <= 10; ++i) {
    const gtsam::Key key = gtsam::Symbol('x', i);
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(i, 0.0, 0.0));
    gtsam::NonlinearFactorGraph factors;
    if (i == 0) {
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, pose, noise));
    } else {
      factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          gtsam::Symbol('x', i - 1), key,
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)),
          noise));
    }
    gtsam::Values values;
    values.insert(key, pose);
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
    timestamps[key] = i;
    smoother.update(factors, values, timestamps);
  }

  require(smoother.timestamps().size() <= 3,
          "fixed-lag state count grew beyond the 2-second window");
  for (const auto &entry : smoother.timestamps())
    require(entry.second >= 8.0,
            "fixed-lag smoother retained an expired variable");
}

void testStandardFixedLagMarginalization() {
  require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
              runStandardFixedLagCheck(),
          "fixed-lag graph did not marginalize the initial key correctly");
}

}  // namespace

int main() {
  ros::Time::init();
  try {
    testAlignment();
    testKeyframeSelection();
    testLeverArmFactor();
    testUwbRangeFactorJacobian();
    testRawPoseInterpolation();
    testAlignmentBoundaryTransition();
    testEndOfStreamGnssRejection();
    testFilteredFixedDoesNotResetAlignment();
    testGnssQualityPolicy();
    testGnssWeightingAndRecoveryRamp();
    require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
                runGnssReacquisitionCheck(),
            "consistent Fixed reacquisition checks failed");
    testAuthoritativeGnssCovariance();
    require(fast_livo_backend::RtkFixedLagBackendSelfTestAccess::
                runGravityConsistencyCheck(),
            "gravity consistency checks failed");
    testResultReferenceLeverArm();
    testTrueFixedLagMarginalization();
    testStandardFixedLagMarginalization();
  } catch (const std::exception &error) {
    std::cerr << "rtk_fixed_lag_backend_self_test: FAIL: " << error.what()
              << std::endl;
    return 1;
  }
  std::cout << "rtk_fixed_lag_backend_self_test: PASS" << std::endl;
  return 0;
}
