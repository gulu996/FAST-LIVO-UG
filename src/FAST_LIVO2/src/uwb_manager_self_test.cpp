#include "uwb_manager.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
  double corrected_range_m = 0.0;
  UwbRejectReason range_reject_reason = UwbRejectReason::NONE;
  assert(correctUwbRangeValue(8.476, 0.316, corrected_range_m, range_reject_reason));
  assert(std::fabs(corrected_range_m - 8.160) < 1e-12);
  assert(range_reject_reason == UwbRejectReason::NONE);
  assert(correctUwbRangeValue(8.476, 0.0, corrected_range_m, range_reject_reason));
  assert(corrected_range_m == 8.476);
  assert(correctUwbRangeValue(8.476, -0.1, corrected_range_m, range_reject_reason));
  assert(std::fabs(corrected_range_m - 8.576) < 1e-12);
  assert(!correctUwbRangeValue(0.0, 0.0, corrected_range_m, range_reject_reason));
  assert(range_reject_reason == UwbRejectReason::INVALID_RAW_RANGE);
  assert(!correctUwbRangeValue(0.2, 0.3, corrected_range_m, range_reject_reason));
  assert(range_reject_reason == UwbRejectReason::INVALID_CORRECTED_RANGE);
  assert(!correctUwbRangeValue(std::numeric_limits<double>::infinity(), 0.0,
                               corrected_range_m, range_reject_reason));
  assert(range_reject_reason == UwbRejectReason::INVALID_RAW_RANGE);

  UwbRangeMeasurement debug_measurement;
  debug_measurement.anchor_id = 1;
  debug_measurement.raw_range_m = 8.476;
  debug_measurement.stamp = 10.0;
  debug_measurement.source_format = "uwbdbg";
  UwbRangeMeasurement distance_measurement = debug_measurement;
  distance_measurement.stamp = 10.012;
  distance_measurement.source_format = "distance";
  assert(isUwbReplayCrossFormatDuplicate(debug_measurement, distance_measurement, 0.05));
  distance_measurement.stamp = 10.2;
  assert(!isUwbReplayCrossFormatDuplicate(debug_measurement, distance_measurement, 0.05));

  UwbManager external_manager;
  external_manager.setInputMode("external_topic");
  uwb_serial_driver::UwbRangeArray external;
  external.header.stamp.fromSec(946684801.5);
  external.round_sequence = 12;
  external.timestamp_source = uwb_serial_driver::UwbRangeArray::HOST_RECEIVE_LOCAL;
  external.timestamp_uncertainty_ns = 1000000;
  uwb_serial_driver::UwbRange range;
  range.anchor_id = 3;
  range.raw_range_m = 8.5;
  range.range_bias_m = 0.1;
  range.corrected_range_m = 8.4;
  range.valid = true;
  range.source_format = "pairs";
  external.ranges.push_back(range);
  external_manager.ingestExternalRanges(external);
  assert(external_manager.pendingMeasurementCount() == 1);
  external.timestamp_uncertainty_ns = 0;
  external_manager.ingestExternalRanges(external);
  assert(external_manager.pendingMeasurementCount() == 1);

  std::cout << "uwb_manager_self_test: PASS (range validation, replay deduplication and external timestamp ingress)\n";
}
