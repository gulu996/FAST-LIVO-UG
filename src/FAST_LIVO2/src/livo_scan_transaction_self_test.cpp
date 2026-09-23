#include "livo_point_time_split.h"
#include "livo_scan_lifecycle.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

struct Point { float curvature; int id; };

int main()
{
  constexpr double scan_begin = 10.0;
  constexpr double scan_end = 10.2;
  std::vector<Point> raw;
  for (int index = 0; index <= 20; ++index)
    raw.push_back(Point{static_cast<float>(index * 10), index});

  const std::vector<double> images{10.0, 10.033333333, 10.066666667,
                                   10.1, 10.133333333, 10.166666667, 10.2};
  std::vector<Point> pending = raw;
  std::vector<int> assignment_count(raw.size(), 0);
  double interval_begin = scan_begin;
  std::uint64_t vio_dispatches = 0;
  fast_livo::LivoScanLifecycle lifecycle;
  lifecycle.begin(1, scan_begin, scan_end, raw.size(), true, scan_begin);

  for (double image_time : images)
  {
    if (image_time >= scan_end - 1e-9) break;  // endpoint LIO precedes an equal-time image
    if (image_time > interval_begin + 1e-9)
    {
      std::vector<Point> current, future;
      fast_livo::appendLivoTimeSplit(pending, interval_begin,
                                     interval_begin, image_time,
                                     current, future);
      for (const Point &point : current) ++assignment_count[point.id];
      pending = std::move(future);
      interval_begin = image_time;
    }
    lifecycle.noteImage(image_time);
    ++vio_dispatches;
    assert(lifecycle.snapshot().lio_transactions == 0);
  }

  std::vector<Point> final_segment, future;
  fast_livo::appendLivoTimeSplit(pending, interval_begin,
                                 interval_begin, scan_end,
                                 final_segment, future);
  for (const Point &point : final_segment) ++assignment_count[point.id];
  // A point exactly on the endpoint belongs to the following half-open interval.
  assert(future.size() == 1 && future.front().id == 20);
  ++assignment_count[future.front().id];

  lifecycle.noteLioTransaction(scan_end);
  lifecycle.noteMapInsertion();
  lifecycle.noteImage(scan_end);
  lifecycle.validateCompleted();
  assert(vio_dispatches == 6);
  assert(lifecycle.snapshot().image_events == 7);
  assert(lifecycle.snapshot().lio_transactions == 1);
  assert(lifecycle.snapshot().map_insertions == 1);
  for (int count : assignment_count) assert(count == 1);

  // No-image scans still produce exactly one complete-scan transaction.
  lifecycle.begin(2, 10.2, 10.4, raw.size(), true, 10.2);
  lifecycle.noteLioTransaction(10.4);
  lifecycle.noteMapInsertion();
  lifecycle.validateCompleted();

  // Startup scans do not require a measurement transaction.
  lifecycle.begin(3, 10.4, 10.6, raw.size(), false, 10.4);
  lifecycle.begin(4, 10.6, 10.8, raw.size(), true, 10.6);
  lifecycle.noteImage(10.633333333);
  lifecycle.noteImage(10.666666667);
  lifecycle.noteLioTransaction(10.8);
  lifecycle.validateCompleted();

  bool rejected = false;
  try { lifecycle.noteLioTransaction(10.8); }
  catch (const std::logic_error &) { rejected = true; }
  assert(rejected);

  std::cout << "livo_scan_transaction_self_test: PASS\n";
}
