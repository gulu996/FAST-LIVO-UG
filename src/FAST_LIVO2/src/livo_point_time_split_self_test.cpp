#include "livo_point_time_split.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

struct Point { float curvature; int id; };

int main() {
  const std::vector<Point> scan{{10, 1}, {40, 2}, {50, 3}, {75, 4}, {95, 5}};
  std::vector<Point> current, pending;
  fast_livo::appendLivoTimeSplit(scan, 1.0, 1.0, 1.025, current, pending);
  assert(current.size() == 1 && pending.size() == 4);
  auto carried = pending;
  current.clear(); pending.clear();
  fast_livo::appendLivoTimeSplit(carried, 1.025, 1.025, 1.060, current, pending);
  assert(current.size() == 2 && current[0].id == 2 && current[1].id == 3);
  assert(pending.size() == 2 && pending[0].id == 4);
  for (const auto &p : current) assert(p.curvature < 35.001f);
  carried = pending;
  current.clear(); pending.clear();
  fast_livo::appendLivoTimeSplit(carried, 1.060, 1.060, 1.110, current, pending);
  assert(current.size() == 2 && pending.empty());
  assert(current[0].id == 4 && current[1].id == 5);
  assert(std::fabs(current[0].curvature - 15.0f) < 1e-4f);
  bool rejected = false;
  try { fast_livo::appendLivoTimeSplit(scan, 0, 1, 1, current, pending); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  rejected = false;
  try { fast_livo::appendLivoTimeSplit(current, 0, 0, 1, current, pending); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  const std::vector<Point> invalid{{std::numeric_limits<float>::quiet_NaN(), 9}};
  rejected = false;
  try { fast_livo::appendLivoTimeSplit(invalid, 0, 0, 1, current, pending); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  std::cout << "PASS: repeated image-time splitting preserves every point and rejects future-point consumption\n";
}
